#include "sh/nf1.hpp"

#include <algorithm>
#include <vector>

namespace sh {

Nf1 compute_nf1(const Grid<uint8_t>& inflated_static, const Cell& goal) {
    const GridSpec& s = inflated_static.spec;
    Nf1 out;
    out.value = Grid<int32_t>(s, kNf1Unreached);
    for (size_t k = 0; k < out.value.data.size(); ++k)
        if (inflated_static.data[k]) out.value.data[k] = kNf1Obstacle;

    std::vector<int> queue;
    queue.reserve(static_cast<size_t>(s.size()));
    if (s.inside(goal.i, goal.j)) {
        out.value.at(goal.i, goal.j) = 0;  // the goal is seeded even if it touches an inflated cell
        queue.push_back(s.index(goal.i, goal.j));
    }
    int32_t max_v = 0;
    const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
    for (size_t head = 0; head < queue.size(); ++head) {
        const int idx = queue[head];
        const int i = idx % s.nx, j = idx / s.nx;
        const int32_t v = out.value.data[static_cast<size_t>(idx)];
        max_v = std::max(max_v, v);
        for (int n = 0; n < 4; ++n) {
            const int a = i + di[n], b = j + dj[n];
            if (!s.inside(a, b)) continue;
            int32_t& w = out.value.at(a, b);
            if (w != kNf1Unreached) continue;  // obstacle or already assigned
            w = v + 1;
            queue.push_back(s.index(a, b));
        }
    }
    out.min_value = 0;
    out.max_value = max_v;
    return out;
}

}  // namespace sh
