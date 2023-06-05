#include "grid2.h"
#include <complex>
#include <cstdlib>

Grid2::Grid2(Vec2d _origin, Vec2i _size, double _spacing):
    origin(_origin), size(_size), spacing(_spacing) {
    scalarV.resize(size.x() * size.y(), 0);
    color.resize(size.x()*size.y(), 0);
}

Vec2i Grid2::pos2tri(const Vec2d& pos) const{
    Vec2i ans = Vec2i::Zero();
    for (int i = 0; i < 2; i++)
        ans[i] = static_cast<int>(std::floor((pos[i] - origin[i]) / spacing));
    return ans;
}

Vec2d Grid2::tri2pos(const Vec2i& tri) const{
    Vec2d ans = origin;
    for (int i = 0; i < 2; i++) ans[i] += spacing * tri[i];
    return ans;
}

double Grid2::interpolate(const Eigen::Vector2d &pos) {
    Vec2i tri = pos2tri(pos);
    Vec2d weight = (pos - tri2pos(tri)) / spacing;
    double ans = 0;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            Vec2d area_vec = Vec2d::Ones() - Vec2d(i, j) - weight;
            double area_weight = std::abs(area_vec.x() * area_vec.y());
            ans += (*this)[tri + Vec2i(i, j)] * area_weight;
        }
    }
    return ans;
}