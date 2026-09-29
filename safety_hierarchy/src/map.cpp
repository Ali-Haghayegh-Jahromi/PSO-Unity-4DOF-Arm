#include "sh/map.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sh {

MapDef load_map(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open map file: " + path);
    MapDef map;
    map.name = path.substr(path.find_last_of("/\\") + 1);
    std::string line;
    int lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ss(line);
        std::string kw;
        if (!(ss >> kw)) continue;
        double a, b, c, d;
        if (!(ss >> a >> b >> c >> d))
            throw std::runtime_error(path + ":" + std::to_string(lineno) + ": expected 4 numbers");
        if (kw == "world") {
            map.xmin = a, map.ymin = b, map.xmax = c, map.ymax = d;
        } else if (kw == "rect") {
            map.rects.push_back({std::min(a, c), std::min(b, d), std::max(a, c), std::max(b, d)});
        } else if (kw == "periodic") {
            map.periodic.push_back({{a, b}, {c, d}});
        } else {
            throw std::runtime_error(path + ":" + std::to_string(lineno) + ": unknown keyword " + kw);
        }
    }
    return map;
}

Grid<uint8_t> rasterize(const MapDef& map, double res) {
    GridSpec s;
    s.x0 = map.xmin;
    s.y0 = map.ymin;
    s.res = res;
    s.nx = static_cast<int>(std::lround((map.xmax - map.xmin) / res));
    s.ny = static_cast<int>(std::lround((map.ymax - map.ymin) / res));
    Grid<uint8_t> g(s, 0);
    for (int j = 0; j < s.ny; ++j) {
        for (int i = 0; i < s.nx; ++i) {
            if (i == 0 || j == 0 || i == s.nx - 1 || j == s.ny - 1) {
                g.at(i, j) = 1;
                continue;
            }
            const Vec2 c = s.center(i, j);
            for (const Rect& r : map.rects) {
                if (c.x >= r.x0 && c.x <= r.x1 && c.y >= r.y0 && c.y <= r.y1) {
                    g.at(i, j) = 1;
                    break;
                }
            }
        }
    }
    return g;
}

std::vector<Cell> inflation_offsets(double r, double res) {
    std::vector<Cell> off;
    const int n = static_cast<int>(std::ceil(r / res)) + 1;
    for (int dj = -n; dj <= n; ++dj) {
        for (int di = -n; di <= n; ++di) {
            // Distance between the two cell squares.
            const double gx = std::max(std::abs(di) - 1, 0) * res;
            const double gy = std::max(std::abs(dj) - 1, 0) * res;
            if (std::hypot(gx, gy) < r) off.push_back({di, dj});
        }
    }
    return off;
}

void stamp(Grid<uint8_t>& g, int i, int j, const std::vector<Cell>& offsets) {
    for (const Cell& o : offsets) {
        const int a = i + o.i, b = j + o.j;
        if (g.spec.inside(a, b)) g.at(a, b) = 1;
    }
}

Grid<uint8_t> inflate(const Grid<uint8_t>& occ, double r) {
    Grid<uint8_t> out(occ.spec, 0);
    const auto off = inflation_offsets(r, occ.spec.res);
    for (int j = 0; j < occ.spec.ny; ++j)
        for (int i = 0; i < occ.spec.nx; ++i)
            if (occ.at(i, j)) stamp(out, i, j, off);
    return out;
}

bool disk_hits(const Grid<uint8_t>& occ, const Vec2& c, double r) {
    const GridSpec& s = occ.spec;
    const Cell lo = s.cell_of(c.x - r, c.y - r);
    const Cell hi = s.cell_of(c.x + r, c.y + r);
    for (int j = lo.j; j <= hi.j; ++j) {
        for (int i = lo.i; i <= hi.i; ++i) {
            if (!occ.get(i, j, 1)) continue;
            const double cx0 = s.x0 + i * s.res, cy0 = s.y0 + j * s.res;
            const double px = std::clamp(c.x, cx0, cx0 + s.res);
            const double py = std::clamp(c.y, cy0, cy0 + s.res);
            if (std::hypot(c.x - px, c.y - py) < r) return true;
        }
    }
    return false;
}

}  // namespace sh
