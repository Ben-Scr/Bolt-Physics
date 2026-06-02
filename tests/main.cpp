#include "TestRunner.hpp"

#include <cstdio>

int main()
{
    auto& registry = IndexPhys::Tests::Registry();
    std::printf("Running %zu tests...\n", registry.size());

    for (const auto& test : registry) {
        IndexPhys::Tests::CurrentTest() = test.name;
        const int failsBefore = IndexPhys::Tests::FailCount();
        test.fn();
        const int failsAfter = IndexPhys::Tests::FailCount();
        std::printf("  %s %s\n",
                    (failsAfter == failsBefore) ? "PASS" : "FAIL",
                    test.name);
    }

    const int failures = IndexPhys::Tests::FailCount();
    const int assertions = IndexPhys::Tests::AssertionCount();
    std::printf("\n%d / %d assertions passed across %zu tests. (%d failed)\n",
                assertions - failures, assertions, registry.size(), failures);

    return failures == 0 ? 0 : 1;
}
