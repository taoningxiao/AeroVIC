#include "aero.h"

Aero::Aero(Vec2d _origin, Vec2i _size, double _spacing):
    velocityX(_origin + Vec2d(0, 0.5) * _spacing, _size + Vec2i(1, 0), _spacing), 
    velocityY(_origin + Vec2d(0.5, 0) * _spacing, _size + Vec2i(0, 1), _spacing), 
    vortexNode(_origin, _size + Vec2i(1, 1), _spacing),
    origin(_origin), size(_size + Vec2i(1, 1)), spacing(_spacing) {
        vortex_particles.clear();
        Particle sample;
        sample.vortex = 0;
        for (int i = 0; i < size.x(); i++) {
            for (int j = 0; j < size.y(); j++) {
                sample.position = vortexNode.tri2pos(Vec2i(i, j));
                vortex_particles.push_back(sample);
            }
        }
}

void Aero::simulate(double dt) {
    // advect(dt);
    // diffusion(dt);
    // calVelocity(dt);
    // boundaryPenalty(dt);
    // calVortex(dt);
}