#pragma once
#include "args.h"
#include "fmt/format.h"
#include <vector>
#include <iostream>

class Grid2 {
public:
    Vec2i size;
    Vec2d origin;
    double spacing;
    Grid2(Vec2d _origin, Vec2i _size, double _spacing);
    std::vector<double> scalarV;
    double operator[] (Vec2i tri) {
        if (inside(tri)) return scalarV[tri.y() * size.x() + tri.x()];
        else {
            std::cout << fmt::format("out of boundary ({}, {})\n", tri.x(), tri.y());
            exit(1);
        }
    }
    inline bool inside(Vec2i tri) const{return ((tri.x() < size.x()) && (tri.x() >= 0) && (tri.y() < size.y()) && (tri.y() >= 0));}
    
    Vec2i pos2tri(const Vec2d& pos) const;
    Vec2d tri2pos(const Vec2i& tri) const;
};