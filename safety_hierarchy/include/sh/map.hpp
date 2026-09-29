// Static environment: map files, rasterization, inflation, disk collision tests.
//
// Map file format (one statement per line, '#' starts a comment):
//   world    xmin ymin xmax ymax
//   rect     x0 y0 x1 y1          static obstacle (axis-aligned)
//   periodic ax ay bx by          path of a periodic dynamic obstacle (set 2)
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sh/grid.hpp"

namespace sh {

struct Rect {
    double x0, y0, x1, y1;
};

struct Segment {
    Vec2 a, b;
};

struct MapDef {
    std::string name;
    double xmin = -15, ymin = -7.5, xmax = 15, ymax = 7.5;
    std::vector<Rect> rects;
    std::vector<Segment> periodic;
};

MapDef load_map(const std::string& path);  // throws std::runtime_error on bad input

// Ground-truth occupancy (1 = static). A cell is occupied if its centre lies in
// a rectangle; the outermost ring of cells is the boundary wall.
Grid<uint8_t> rasterize(const MapDef& map, double res);

// Offsets (di, dj) of every cell whose square is closer than r to the square
// of the centre cell. Stamping them around each occupied cell gives the set of
// cells in which *any* disk centre of radius r would overlap that cell.
std::vector<Cell> inflation_offsets(double r, double res);
void stamp(Grid<uint8_t>& g, int i, int j, const std::vector<Cell>& offsets);
Grid<uint8_t> inflate(const Grid<uint8_t>& occ, double r);

// Exact test: does the disk (c, r) overlap an occupied cell square?
// Cells outside the grid count as occupied.
bool disk_hits(const Grid<uint8_t>& occ, const Vec2& c, double r);

}  // namespace sh
