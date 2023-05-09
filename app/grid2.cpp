#include "grid2.h"
#include <cstdlib>

Grid2::Grid2(Vec2d _origin, Vec2i _size, double _spacing):
    origin(_origin), size(_size), spacing(_spacing) {
    scalarV.resize(size.x() * size.y(), 0);
}

Vec2i Grid2::pos2tri(const Vec2d& pos) const{
    Vec2i ans = Vec2i::Zero();
    for (int i = 0; i < 2; i++)
        ans[i] = int((pos[i] - origin[i]) / spacing);
    if (inside(ans))
        return ans;
    else {
        std::cout << fmt::format("out of boundary ({}, {})\n", pos.x(), pos.y());
        exit(2);
    }
}

Vec2d Grid2::tri2pos(const Vec2i& tri) const{
    if (!inside(tri)) {
        std::cout << fmt::format("out of boundary ({}, {})\n", tri.x(), tri.y());
        exit(3);
    }
    Vec2d ans = origin;
    for (int i = 0; i < 2; i++) ans[i] += spacing * tri[i];
    return ans;
}