#include "Vec2.hpp"

#include <cmath>
#include <limits>

namespace BoltPhys {
    float Length(const Vec2& v) noexcept
    {
        return std::sqrt(LengthSq(v));
    }

    Vec2 Normalize(const Vec2& v) noexcept
    {
        const float length = Length(v);
        if (length <= std::numeric_limits<float>::epsilon()) {
            return {};
        }

        return v / length;
    }
}