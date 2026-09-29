// Minimal RGB image + binary PPM writer, used by sh_run to render costmaps.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sh {

struct Rgb {
    uint8_t r = 0, g = 0, b = 0;
};

struct Image {
    int w = 0, h = 0;
    std::vector<Rgb> px;
    Image(int w_, int h_, Rgb fill = {255, 255, 255}) : w(w_), h(h_), px(static_cast<size_t>(w_) * h_, fill) {}
    // (x, y) with y pointing up, as in the world frame.
    void set(int x, int y, Rgb c) {
        if (x >= 0 && y >= 0 && x < w && y < h) px[static_cast<size_t>(h - 1 - y) * w + x] = c;
    }
    void disk(double cx, double cy, double r, Rgb c);
    bool write_ppm(const std::string& path) const;
};

}  // namespace sh
