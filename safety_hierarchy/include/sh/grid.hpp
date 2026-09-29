// Uniform 2-D grid over the world. Cell (i, j) covers
// [x0 + i*res, x0 + (i+1)*res) x [y0 + j*res, y0 + (j+1)*res).
#pragma once

#include <cmath>
#include <vector>

#include "sh/common.hpp"

namespace sh {

struct Cell {
    int i = 0, j = 0;
};

struct GridSpec {
    double x0 = 0.0, y0 = 0.0, res = 0.1;
    int nx = 0, ny = 0;

    bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < nx && j < ny; }
    int index(int i, int j) const { return j * nx + i; }
    int size() const { return nx * ny; }
    Cell cell_of(double x, double y) const {
        return {static_cast<int>(std::floor((x - x0) / res)),
                static_cast<int>(std::floor((y - y0) / res))};
    }
    Cell cell_of(const Vec2& p) const { return cell_of(p.x, p.y); }
    Vec2 center(int i, int j) const { return {x0 + (i + 0.5) * res, y0 + (j + 0.5) * res}; }
};

template <class T>
struct Grid {
    GridSpec spec;
    std::vector<T> data;

    Grid() = default;
    Grid(const GridSpec& s, T init) : spec(s), data(static_cast<size_t>(s.size()), init) {}

    T& at(int i, int j) { return data[static_cast<size_t>(spec.index(i, j))]; }
    const T& at(int i, int j) const { return data[static_cast<size_t>(spec.index(i, j))]; }
    // Value outside the grid is `outside`.
    T get(int i, int j, T outside) const { return spec.inside(i, j) ? at(i, j) : outside; }
    void fill(T v) { std::fill(data.begin(), data.end(), v); }
};

}  // namespace sh
