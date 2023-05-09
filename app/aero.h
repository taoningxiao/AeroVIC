#pragma once
#include "particle2.h"
#include "grid2.h"
#include <vector>
#include <unordered_set>

class Aero {
public:
    std::vector<Particle> vortex_particles;
    Grid2 velocityX;
    Grid2 velocityY;
    Grid2 vortexNode;
    Vec2d origin;
    double spacing;
    Vec2i size;
    Aero(Vec2d _origin, Vec2i _size, double _spacing);

    void simulate(double dt);
    void advect(double dt);
    void diffusion(double dt);
    void calVelocity(double dt);
    void boundaryPenalty(double dt);
    void calVortex(double dt);
};