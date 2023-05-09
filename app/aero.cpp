#include "aero.h"
#include "Eigen/src/SparseCore/SparseUtil.h"
#include "args.h"
#include <random>

Aero::Aero(Vec2d _origin, Vec2i _size, double _spacing):
    velocityX(_origin + Vec2d(0, 0.5) * _spacing, _size + Vec2i(1, 0), _spacing),
    velocityY(_origin + Vec2d(0.5, 0) * _spacing, _size + Vec2i(0, 1), _spacing),
    vortexNode(_origin, _size + Vec2i(1, 1), _spacing),
    origin(_origin), size(_size), spacing(_spacing) {
    vortex_particles.clear();
    Particle sample;
    sample.vortex = 0;
    for (int i = 0; i < size.x() + 1; i++) {
        for (int j = 0; j < size.y() + 1; j++) {
            sample.position = vortexNode.tri2pos(Vec2i(i, j));
            vortex_particles.push_back(sample);
        }
    }
}

void Aero::simulate(double dt) {
    advect(dt);
    // diffusion(dt);
    calVelocity(dt);
    // boundaryPenalty(dt);
    // calVortex(dt);
}

void Aero::advect(double dt) {
    for (auto & particle : vortex_particles) {
        double vecX         = velocityX.interpolate(particle.position);
        double vecY         = velocityY.interpolate(particle.position);
        auto   new_position = particle.position + Vec2d(vecX, vecY) * dt;
        if (! inside(new_position)) {
            particle.position *= -1;
            particle.vortex = 0;
        } else particle.position = new_position;
    }
}

void Aero::calVelocity(double dt) {
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
        j = vortexNode.size.y() - 1;
        coefficients.push_back(Eigen::Triplet<double>(vortexNode.getIdx(Vec2i(i, j)), vortexNode.getIdx(Vec2i(i, j)), 1.0));
        nodePos                        = vortexNode.tri2pos(Vec2i(i, j));
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
            int idx_down = velocityX.getIdx(Vec2i(i, j));
            int idx_up = idx_down + (size.x() + 1);
            velocityX.scalarV[velocityX.getIdx(Vec2i(i, j))] = (psi[idx_up] - psi[idx_down]) / spacing;
        }
    }

    for (int i = 0; i < velocityY.size.x(); i++) {
        for (int j = 0; j < velocityY.size.y(); j++) {
            int idx_left = vortexNode.getIdx(Vec2i(i, j));
            int idx_right = idx_left + 1;
            velocityY.scalarV[velocityY.getIdx(Vec2i(i, j))] = -(psi[idx_right] - psi[idx_left]) / spacing;
        }
    }
}