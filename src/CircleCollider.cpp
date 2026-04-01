#include "CircleCollider.hpp"
#include "Body2D.hpp"

namespace BoltPhys {
    namespace {
        constexpr float kDefaultRadius = 0.5f;
    }

    CircleCollider::CircleCollider(float radius)
        : Collider(ColliderType::Circle),
        m_radius(radius > 0.0f ? radius : kDefaultRadius)
    {}

    float CircleCollider::GetRadius() const noexcept
    {
        return m_radius;
    }

    void CircleCollider::SetRadius(float radius)
    {
        m_radius = radius > 0.0f ? radius : kDefaultRadius;
    }

    AABB CircleCollider::ComputeAABB() const noexcept
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        const Vec2 extents{ m_radius, m_radius };
        return { center - extents, center + extents };
    }
}