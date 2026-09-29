#include "sh/distance_field.hpp"

#include <limits>
#include <queue>

namespace sh {

void propagate_distance(const GridSpec& spec, const std::vector<uint8_t>& blocked,
                        const std::vector<FieldSeed>& seeds, double max_dist, std::vector<float>* dist) {
    const int n = spec.size();
    const float inf = std::numeric_limits<float>::infinity();
    dist->assign(static_cast<size_t>(n), inf);
    std::vector<int> src(static_cast<size_t>(n), -1);

    using Item = std::pair<float, int>;  // (distance, cell)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

    auto dist_to = [&](int cell, int s) {
        const Vec2 c = spec.center(cell % spec.nx, cell / spec.nx);
        return static_cast<float>(sh::dist(c, seeds[static_cast<size_t>(s)].src) - seeds[static_cast<size_t>(s)].offset);
    };

    for (int s = 0; s < static_cast<int>(seeds.size()); ++s) {
        const int c = seeds[static_cast<size_t>(s)].cell;
        const float d = dist_to(c, s);
        if (d < (*dist)[static_cast<size_t>(c)]) {
            (*dist)[static_cast<size_t>(c)] = d;
            src[static_cast<size_t>(c)] = s;
            pq.push({d, c});
        }
    }

    const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    auto is_blocked = [&](int i, int j) { return !spec.inside(i, j) || blocked[static_cast<size_t>(spec.index(i, j))]; };

    while (!pq.empty()) {
        const auto [d, c] = pq.top();
        pq.pop();
        if (d > (*dist)[static_cast<size_t>(c)] || d > max_dist) continue;
        const int i = c % spec.nx, j = c / spec.nx;
        const int s = src[static_cast<size_t>(c)];
        for (int k = 0; k < 8; ++k) {
            const int a = i + di[k], b = j + dj[k];
            if (is_blocked(a, b)) continue;
            if (k >= 4 && (is_blocked(a, j) || is_blocked(i, b))) continue;  // no corner cutting
            const int nb = spec.index(a, b);
            const float nd = dist_to(nb, s);
            if (nd < (*dist)[static_cast<size_t>(nb)]) {
                (*dist)[static_cast<size_t>(nb)] = nd;
                src[static_cast<size_t>(nb)] = s;
                pq.push({nd, nb});
            }
        }
    }
}

}  // namespace sh
