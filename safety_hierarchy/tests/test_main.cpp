#include "test.hpp"

namespace test {
std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
int failures = 0;
}  // namespace test

int main() {
    for (const auto& c : test::registry()) {
        const int before = test::failures;
        c.fn();
        std::printf("%s %s\n", test::failures == before ? "[ ok ]" : "[FAIL]", c.name);
    }
    std::printf("%zu tests, %d failed checks\n", test::registry().size(), test::failures);
    return test::failures == 0 ? 0 : 1;
}
