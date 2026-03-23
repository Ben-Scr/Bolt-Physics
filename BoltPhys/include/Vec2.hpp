#pragma once

namespace BoltPhys {
    struct Vec2
    {
        float x = 0.0f;
        float y = 0.0f;

        Vec2 operator+(const Vec2&) const;
        Vec2 operator-(const Vec2&) const;
        Vec2 operator*(float s) const;
        Vec2& operator+=(const Vec2&);
        Vec2& operator-=(const Vec2&);
    };

    float Dot(const Vec2& a, const Vec2& b);
    float LengthSq(const Vec2& v);
    float Length(const Vec2& v);
    Vec2 Normalize(const Vec2& v);
}
