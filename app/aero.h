#pragma once
#include "particle2.h"
#include "grid2.h"
#include <vector>
#include <random>

class Aero {
public:
    std::vector<Particle> vortex_particles;
    Grid2 velocityX;
    Grid2 velocityY;
    Grid2 vortexNode;
    Eigen::VectorXd psi;
    Vec2d origin;
    double spacing;
    Vec2i size;
    Aero(Vec2d _origin, Vec2i _size, double _spacing);
    inline bool inside(Vec2d pos) {
        Vec2d bias = pos - origin;
        return ((bias.x() >= 0) && (bias.y() >= 0) && (bias.x() <= size.x() * spacing) && (bias.y() <= size.y() * spacing));
    }

    void simulate(double dt);
    void advect(double dt);
    void vortexP2G(double dt);
    void diffusion(double dt);
    void calVelocity(double dt);
    void boundaryPenalty(double dt);
    void calVortex(double dt);
    void vortexG2P(double dt);

    /* for space hashing */
    std::vector<std::vector<int>> grid_particles;
    std::vector<std::vector<Vec2i>> particle_grids;
    void buildGridTable();
};