#pragma once
#include "Export.hpp"
#include "Vec2.hpp"

namespace IndexPhys {
    struct INDEX_PHYS_API WorldSettings
    {
        Vec2 gravity{ 0.0f, -9.81f };
        Vec2 worldMin{ -1000.0f, -1000.0f };
        Vec2 worldMax{ 1000.0f,  1000.0f };
        float broadphaseCellSize = 5.0f;
        // How many velocity-solver passes to run per Step. Higher values
        // reduce jitter in stacks at the cost of CPU. Clamped to >= 1.
        int solverIterations = 8;
        bool enableWorldBounds = false;
    };
}
