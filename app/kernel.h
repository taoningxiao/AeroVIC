#pragma once

#include <cmath>
class M4Kernel {
public:
    double radius;
    M4Kernel(double _radius): radius(_radius) {};
    double operator ()(double dist) const {
        double v = std::abs(dist) / radius;
        if (v > 2) return 0;
        else if (v >= 1) return (2 - v)*(2 - v)*(1 - v)/2;
        else return 1 - 5*v*v/2 + 3*v*v*v/2;
    }
};