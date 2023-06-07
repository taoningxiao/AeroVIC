#include "aero.h"
#include "args.h"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/SparseCore/SparseUtil.h"
#include "kernel.h"
#include "parallel.h"
#include "timer.h"
#include <algorithm>
#include <complex>
#include <random>
#include <vector>

Aero::Aero(Vec2d _origin, Vec2i _size, double _spacing):
    velocityX(_origin + Vec2d(0, 0.5) * _spacing, _size + Vec2i(1, 0), _spacing),
    velocityY(_origin + Vec2d(0.5, 0) * _spacing, _size + Vec2i(0, 1), _spacing),
    vortexNode(_origin, _size + Vec2i(1, 1), _spacing),
    origin(_origin), size(_size), spacing(_spacing) {
    vortex_particles.clear();
    Particle sample;
    sample.vortex = 0;

    std::random_device rd;
    // fix random seed
    std::mt19937 gen(rd());
    gen.seed(12345);
    std::uniform_real_distribution<double> dist(0, 1);
    vortex_particles.resize(size.x() * size.y() * config["particles-per-cell"].as<int>());
    for (int t = 0; t < size.x() * size.y(); ++t) {
        int  i       = t % size.x();
        int  j       = t / size.x();
        auto ori_pos = vortexNode.tri2pos(Vec2i(i, j));
        for (int k = 0; k < config["particles-per-cell"].as<int>(); k++) {
            // sample.position = ori_pos + Vec2d(dist(rd), dist(rd)) * spacing;
            sample.position                                                  = ori_pos + Vec2d(dist(gen), dist(gen)) * spacing;
            sample.color                                                     = 2 * std::abs(sample.position.y()) / (spacing * size.y()) * 0.8;
            vortex_particles[t * config["particles-per-cell"].as<int>() + k] = sample;
        }
    }
}

void Aero::simulate(double dt) {
    myClock["simulate"].start();
    advect(dt);
    buildGridTable();
    vortexP2G(dt);
    calVelocity(dt);
    boundaryPenalty(dt);
    calVortex(dt);
    diffusion(dt);
    vortexG2P(dt);
    myClock["simulate"].stop();
}

void Aero::advect(double dt) {
    myClock["simulate/advect"].start();
    double                                 r = config["radius"].as<double>();
    std::random_device                     rd;
    std::uniform_real_distribution<double> dist(0, 1);
    for (auto & particle : vortex_particles) {
        // if (particle.vortex != 0) continue;
        Vec2d vec1(velocityX.interpolate(particle.position), velocityY.interpolate(particle.position));
        Vec2d vec2(velocityX.interpolate(particle.position + vec1 * dt / 2), velocityY.interpolate(particle.position + vec1 * dt / 2));
        Vec2d vec3(velocityX.interpolate(particle.position + vec2 * dt / 2), velocityY.interpolate(particle.position + vec2 * dt / 2));
        Vec2d vec4(velocityX.interpolate(particle.position + vec3 * dt), velocityY.interpolate(particle.position + vec3 * dt));
        auto  new_position = particle.position + dt * (vec1 + 2 * vec2 + 3 * vec3 + vec4) / 6;
        if (! inside(new_position)) {
            particle.position.x() = origin.x();
            particle.position.y() = origin.y() + dist(rd) * size.y() * spacing;
            particle.vortex       = 0;
            particle.color        = 2 * std::abs(particle.position.y()) / (spacing * size.y()) * 0.8;
            // particle.color = std::abs(particle.position.y()) / (2 * spacing * size.y());
        } else particle.position = new_position;
    }
    myClock["simulate/advect"].stop();
}

void Aero::buildGridTable() {
    myClock["simulate/buildGridTable"].start();
    grid_particles.resize(vortexNode.size.x() * vortexNode.size.y());
    for (auto it = grid_particles.begin(); it != grid_particles.end(); ++it) it->clear();
    particle_grids.resize(vortex_particles.size());
    for (auto it = particle_grids.begin(); it != particle_grids.end(); ++it) it->clear();

    for (int particleIdx = 0; particleIdx < vortex_particles.size(); particleIdx++) {
        Vec2i tri = vortexNode.pos2tri(vortex_particles[particleIdx].position);
        for (int i = std::max(0, tri.x() - 2); i <= std::min(tri.x() + 2, vortexNode.size.x() - 1); i++) {
            for (int j = std::max(0, tri.y() - 2); j <= std::min(tri.y() + 2, vortexNode.size.y() - 1); j++) {
                int idx = vortexNode.getIdx(Vec2i(i, j));
                grid_particles[idx].push_back(particleIdx);
                particle_grids[particleIdx].push_back(Vec2i(i, j));
            }
        }
    }
    myClock["simulate/buildGridTable"].stop();
}

void Aero::vortexP2G(double dt) {
    myClock["simulate/vortexP2G"].start();
    M4Kernel kernel(config["kernel-radius"].as<double>());
    double   radius = config["radius"].as<double>();
    parallelFor(0, int(vortexNode.size.x() * vortexNode.size.y()), [&](int t) {
        int  i       = t % vortexNode.size.x();
        int  j       = t / vortexNode.size.x();
        int  idx     = vortexNode.getIdx(Vec2i(i, j));
        auto cur_pos = vortexNode.tri2pos(Vec2i(i, j));

        vortexNode.scalarV[idx] = 0;

        double w = 0;

        for (const auto & particleIdx : grid_particles[idx]) {
            auto & particle = vortex_particles[particleIdx];
            vortexNode.scalarV[idx] += kernel((particle.position - cur_pos).norm()) * particle.vortex;
            vortexNode.color[idx] += kernel((particle.position - cur_pos).norm()) * particle.color;
            w += kernel((particle.position - cur_pos).norm());
        }

        if (w != 0) {
            vortexNode.scalarV[idx] /= w;
            vortexNode.color[idx] /= w;
            if (cur_pos.norm() <= radius) vortexNode.color[idx] = 0;
        }
        if (std::abs(vortexNode.scalarV[idx]) < EPS) vortexNode.scalarV[idx] = 0;
    });
    myClock["simulate/vortexP2G"].stop();
}

void Aero::diffusion(double dt) {
    myClock["simulate/diffusion"].start();
    double coe = config["viscous-coe"].as<double>();

    int num = vortexNode.scalarV.size();

    Eigen::VectorXd rhs(num);
    for (int i = 0; i < num; i++) rhs[i] = vortexNode.scalarV[i];

    Eigen::SparseMatrix<double>         laplacian(num, num);
    std::vector<Eigen::Triplet<double>> coefficients;
    coefficients.clear();

    double r            = config["radius"].as<double>();
    double inv_spacing2 = 1 / (spacing * spacing);

    for (int i = 0; i < vortexNode.size.x(); i++) {
        for (int j = 0; j < vortexNode.size.y(); j++) {
            int   idx = vortexNode.getIdx(Vec2i(i, j));
            Vec2d pos = vortexNode.tri2pos(Vec2i(i, j));
            coefficients.push_back(Eigen::Triplet<double>(idx, idx, 1));
            if (pos.norm() < r) continue;

            double w = 0;
            if (i != vortexNode.size.x() - 1) {
                Vec2d pos_right = vortexNode.tri2pos(Vec2i(i + 1, j));
                if (pos_right.norm() >= r) {
                    int idx_right = vortexNode.getIdx(Vec2i(i + 1, j));
                    coefficients.push_back(Eigen::Triplet<double>(idx, idx_right, -coe * dt * inv_spacing2));
                    w += coe * dt * inv_spacing2;
                }
            }
            if (i != 0) {
                Vec2d pos_left = vortexNode.tri2pos(Vec2i(i - 1, j));
                if (pos_left.norm() >= r) {
                    int idx_left = vortexNode.getIdx(Vec2i(i - 1, j));
                    coefficients.push_back(Eigen::Triplet<double>(idx, idx_left, -coe * dt * inv_spacing2));
                    w += coe * dt * inv_spacing2;
                }
            }
            if (j != vortexNode.size.y() - 1) {
                Vec2d pos_up = vortexNode.tri2pos(Vec2i(i, j + 1));
                if (pos_up.norm() >= r) {
                    int idx_up = vortexNode.getIdx(Vec2i(i, j + 1));
                    coefficients.push_back(Eigen::Triplet<double>(idx, idx_up, -coe * dt * inv_spacing2));
                    w += coe * dt * inv_spacing2;
                }
            }
            if (j != 0) {
                Vec2d pos_down = vortexNode.tri2pos(Vec2i(i, j - 1));
                if (pos_down.norm() >= r) {
                    int idx_down = vortexNode.getIdx(Vec2i(i, j - 1));
                    coefficients.push_back(Eigen::Triplet<double>(idx, idx_down, -coe * dt * inv_spacing2));
                    w += coe * dt * inv_spacing2;
                }
            }
            coefficients.push_back(Eigen::Triplet<double>(idx, idx, w));
        }
    }

    laplacian.setFromTriplets(coefficients.begin(), coefficients.end());

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper, Eigen::IncompleteCholesky<double>> cg(laplacian);

    const double tolerance = std::max(1e-10, std::numeric_limits<double>::epsilon() / rhs.norm());
    cg.setTolerance(tolerance);

    Eigen::VectorXd ans(num);
    ans.setZero();
    ans = cg.solveWithGuess(rhs, ans);
    if (cg.error() > tolerance * 2) {
        std::cout << "#iterations:     " << cg.iterations() << std::endl;
        std::cout << "estimated error: " << cg.error() << std::endl;
        bool isSymmetric = laplacian.isApprox(laplacian.transpose());
        std::cout << fmt::format("sym condition: {}\n", isSymmetric);

        Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> eigensolver(laplacian);
        if (eigensolver.eigenvalues().minCoeff() > -1e-10)
            std::cout << "laplacian is positive definite." << std::endl;
        else
            std::cout << "laplacian is not positive definite." << std::endl;
    }

    for (int i = 0; i < num; i++) vortexNode.scalarV[i] = ans[i];
    myClock["simulate/diffusion"].stop();
}

void Aero::calVelocity(double dt) {
    myClock["simulate/calVelocity"].start();
    int num = vortexNode.scalarV.size();

    psi.resize(num);
    Eigen::VectorXd rhs(num);
    for (int i = 0; i < num; i++) rhs[i] = vortexNode.scalarV[i];

    Eigen::SparseMatrix<double>         laplacian(num, num);
    std::vector<Eigen::Triplet<double>> coefficients;
    coefficients.clear();

    double r        = config["radius"].as<double>();
    double u_far    = config["u-far"].as<double>();
    double spacing2 = spacing * spacing;

    for (int i = 0; i < vortexNode.size.x(); i++) {
        int j = 0;
        coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j)), 1.0));
        auto nodePos                        = vortexNode.tri2pos(Vec2i(i, j));
        rhs[vortexNode.getIdx(Vec2i(i, j))] = u_far * nodePos.y() * (1 - r * r / (nodePos.squaredNorm()));
        j                                   = vortexNode.size.y() - 1;
        coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j)), 1.0));
        nodePos                             = vortexNode.tri2pos(Vec2i(i, j));
        rhs[vortexNode.getIdx(Vec2i(i, j))] = u_far * nodePos.y() * (1 - r * r / (nodePos.squaredNorm()));
    }

    for (int i = 0; i < vortexNode.size.x(); i++) {
        for (int j = 0; j < vortexNode.size.y(); j++) {
            /* handling boundary at top and bottom */
            if ((j == 0) || (j == vortexNode.size.y() - 1)) {
                // rhs[vortexNode.getIdx(Vec2i(i, j))] = u_far * nodePos.y();
                continue;
            }

            double w = 4 / spacing2;
            if (i == vortexNode.size.x() - 1) {
                auto nodePos = vortexNode.tri2pos(Vec2i(i, j));
                rhs[vortexNode.getIdx(Vec2i(i, j))] += (2 * u_far * r * r * nodePos.x() * nodePos.y() / (nodePos.squaredNorm() * nodePos.squaredNorm())) / spacing;
                w -= 1 / spacing2;
            } else {
                coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i + 1, j)), -1.0 / spacing2));
            }

            if (i == 0) {
                auto nodePos = vortexNode.tri2pos(Vec2i(i, j));
                rhs[vortexNode.getIdx(Vec2i(i, j))] -= (2 * u_far * r * r * nodePos.x() * nodePos.y() / (nodePos.squaredNorm() * nodePos.squaredNorm())) / spacing;
                w -= 1 / spacing2;
            } else {
                coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i - 1, j)), -1.0 / spacing2));
            }

            if ((j + 1 == vortexNode.size.y() - 1)) {
                rhs[vortexNode.getIdx(Vec2i(i, j))] += rhs[vortexNode.getIdx(Vec2i(i, j + 1))] / spacing2;
            } else {
                coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j + 1)), -1.0 / spacing2));
            }
            if ((j - 1 == 0)) {
                rhs[vortexNode.getIdx(Vec2i(i, j))] += rhs[vortexNode.getIdx(Vec2i(i, j - 1))] / spacing2;
            } else {
                coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j - 1)), -1.0 / spacing2));
            }
            coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j)), w));
        }
    }

    laplacian.setFromTriplets(coefficients.begin(), coefficients.end());
    // std::cout << "laplacian\n"
    //           << laplacian << "\n";
    // std::cout << "rhs\n"
    //           << rhs << "\n";

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper, Eigen::IncompleteCholesky<double>> cg(laplacian);

    const double tolerance = std::max(1e-10, std::numeric_limits<double>::epsilon() / rhs.norm());
    cg.setTolerance(tolerance);

    psi = cg.solveWithGuess(rhs, psi);
    if (cg.error() > tolerance * 2) {
        std::cout << "#iterations:     " << cg.iterations() << std::endl;
        std::cout << "estimated error: " << cg.error() << std::endl;
        bool isSymmetric = laplacian.isApprox(laplacian.transpose());
        std::cout << fmt::format("sym condition: {}\n", isSymmetric);

        Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> eigensolver(laplacian);
        if (eigensolver.eigenvalues().minCoeff() > -1e-10)
            std::cout << "laplacian is positive definite." << std::endl;
        else
            std::cout << "laplacian is not positive definite." << std::endl;
    }

    for (int i = 0; i < velocityX.size.x(); i++) {
        for (int j = 0; j < velocityX.size.y(); j++) {
            int idx_down                                     = velocityX.getIdx(Vec2i(i, j));
            int idx_up                                       = idx_down + (size.x() + 1);
            velocityX.scalarV[velocityX.getIdx(Vec2i(i, j))] = (psi[idx_up] - psi[idx_down]) / spacing;
            if (std::abs(velocityX.scalarV[velocityX.getIdx(Vec2i(i, j))]) < EPS) velocityX.scalarV[velocityX.getIdx(Vec2i(i, j))] = 0;
        }
    }

    for (int i = 0; i < velocityY.size.x(); i++) {
        for (int j = 0; j < velocityY.size.y(); j++) {
            int idx_left                                     = vortexNode.getIdx(Vec2i(i, j));
            int idx_right                                    = idx_left + 1;
            velocityY.scalarV[velocityY.getIdx(Vec2i(i, j))] = -(psi[idx_right] - psi[idx_left]) / spacing;
            if (std::abs(velocityY.scalarV[velocityY.getIdx(Vec2i(i, j))]) < EPS) velocityY.scalarV[velocityY.getIdx(Vec2i(i, j))] = 0;
        }
    }

    myClock["simulate/calVelocity"].stop();
}

void Aero::boundaryPenalty(double dt) {
    myClock["simulate/boundaryPenalty"].start();
    double radius = config["radius"].as<double>();
    for (int i = 0; i < velocityX.size.x(); i++) {
        for (int j = 0; j < velocityX.size.y(); j++) {
            auto vel_pos = (velocityX.tri2pos(Vec2i(i, j)));
            if (vel_pos.norm() <= radius) velocityX.scalarV[velocityX.getIdx(Vec2i(i, j))] = 0;
        }
    }

    for (int i = 0; i < velocityY.size.x(); i++) {
        for (int j = 0; j < velocityY.size.y(); j++) {
            auto vel_pos = (velocityY.tri2pos(Vec2i(i, j)));
            if (vel_pos.norm() <= radius) velocityY.scalarV[velocityY.getIdx(Vec2i(i, j))] = 0;
        }
    }
    myClock["simulate/boundaryPenalty"].stop();
}

void Aero::calVortex(double dt) {
    myClock["simulate/calVortex"].start();
    vortex_change.resize(vortexNode.size.x() * vortexNode.size.y());
    std::fill(vortex_change.begin(), vortex_change.end(), 0);
    for (int i = 1; i < vortexNode.size.x() - 1; i++) {
        for (int j = 1; j < vortexNode.size.y() - 1; j++) {
            Vec2i  tri_left  = Vec2i(i - 1, j);
            Vec2i  tri_right = Vec2i(i, j);
            double gradY     = (velocityY[tri_right] - velocityY[tri_left]) / spacing;

            Vec2i  tri_down = Vec2i(i, j - 1);
            Vec2i  tri_up   = Vec2i(i, j);
            double gradX    = (velocityX[tri_up] - velocityX[tri_down]) / spacing;

            vortex_change[vortexNode.getIdx(Vec2i(i, j))]      = (gradY - gradX) - vortexNode.scalarV[vortexNode.getIdx(Vec2i(i, j))];
            vortexNode.scalarV[vortexNode.getIdx(Vec2i(i, j))] = gradY - gradX;
            if (std::abs(vortexNode.scalarV[vortexNode.getIdx(Vec2i(i, j))]) < EPS) vortexNode.scalarV[vortexNode.getIdx(Vec2i(i, j))] = 0;
        }
    }
    myClock["simulate/calVortex"].stop();
}

void Aero::vortexG2P(double dt) {
    myClock["simulate/vortexG2P"].start();
    M4Kernel kernel(config["kernel-radius"].as<double>());
    parallelFor(0, int(vortex_particles.size()), [&](int idx) {
        auto & particle = vortex_particles[idx];
        double w        = 0;
        // particle.vortex = 0;
        double change = 0;
        for (const auto tri : particle_grids[idx]) {
            int cur_idx = vortexNode.getIdx(tri);
            change += kernel((particle.position - vortexNode.tri2pos(tri)).norm()) * vortex_change[cur_idx];
            w += kernel((particle.position - vortexNode.tri2pos(tri)).norm());
        }
        if (w != 0) change /= w;
        if (std::abs(change) < EPS) change = 0;
        particle.vortex += change;
    });
    myClock["simulate/vortexG2P"].stop();
}