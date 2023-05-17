#pragma once

#include <cmath>
#define M_PI 3.1415926
class M4Kernel {
public:
    double radius;
    double h2, h3;
    M4Kernel(double _radius):
        radius(_radius), h2(_radius * _radius), h3(_radius * _radius * _radius) {};
    double operator()(double dist) const {
        if (dist > radius) return 0;
        else {
            double x = 1 - dist * dist / h2;
            return 315 / (64 * M_PI * h3) * x * x * x;
        }
    }
};