#pragma once
#include "Vec2.hpp"

namespace BoltPhys {
    struct AABB
    {
        Vec2 min;
        Vec2 max;

        bool Intersects(const AABB& other) const;
        Vec2 GetSize() const;
    };
}