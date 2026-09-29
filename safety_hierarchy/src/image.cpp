#include "sh/image.hpp"

#include <cmath>
#include <cstdio>

namespace sh {

void Image::disk(double cx, double cy, double r, Rgb c) {
    for (int y = static_cast<int>(std::floor(cy - r)); y <= static_cast<int>(std::ceil(cy + r)); ++y)
        for (int x = static_cast<int>(std::floor(cx - r)); x <= static_cast<int>(std::ceil(cx + r)); ++x)
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r) set(x, y, c);
}

bool Image::write_ppm(const std::string& path) const {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (const Rgb& p : px) {
        const unsigned char b[3] = {p.r, p.g, p.b};
        std::fwrite(b, 1, 3, f);
    }
    return std::fclose(f) == 0;
}

}  // namespace sh
