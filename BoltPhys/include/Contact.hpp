#pragma once
#include "Vec2.hpp"
#include "Body.hpp"

namespace BoltPhys {
    struct Contact
    {
        Body* BodyA = nullptr;
        Body* BodyB = nullptr;
        Vec2 Normal{};
        float Penetration = 0.0f;
    };
}