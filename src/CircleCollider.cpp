#include "CircleCollider.hpp"
#include "Body.hpp"

namespace IndexPhys {
    namespace {
        constexpr float kDefaultRadius = 0.5f;
    }

    CircleCollider::CircleCollider(float radius)
        : Collider(ColliderType::Circle),
        m_Radius(radius > 0.0f ? radius : kDefaultRadius)
    {}

    float CircleCollider::GetRadius() const noexcept
    {
        return m_Radius;
    }

    void CircleCollider::SetRadius(float radius)
    {
        m_Radius = radius > 0.0f ? radius : kDefaultRadius;
    }

    AABB CircleCollider::ComputeAABB() const noexcept
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        const Vec2 extents{ m_Radius, m_Radius };
        return { center - extents, center + extents };
    }
}