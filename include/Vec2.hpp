#pragma once
#include <glm/vec2.hpp>

namespace IndexPhys {
    using Vec2 = glm::vec2;

    constexpr float Dot(const Vec2& a, const Vec2& b) noexcept
    {
        return a.x * b.x + a.y * b.y;
    }

    constexpr float LengthSq(const Vec2& v) noexcept
    {
        return Dot(v, v);
    }

    constexpr float DistanceSq(const Vec2& a, const Vec2& b) noexcept
    {
        const Vec2 d{ a.x - b.x, a.y - b.y };
        return Dot(d, d);
    }

    float Length(const Vec2& v) noexcept;
    float Distance(const Vec2& a, const Vec2& b) noexcept;
    Vec2 Normalize(const Vec2& v) noexcept;
}
