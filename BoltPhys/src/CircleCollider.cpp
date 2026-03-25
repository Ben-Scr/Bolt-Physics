#include "CircleCollider.hpp"

#include "Body2D.hpp"

namespace BoltPhys {
    CircleCollider::CircleCollider(float radius)
        : Collider2D(ColliderType::Circle),
        m_radius(radius > 0.0f ? radius : 0.5f)
    {
    }

    float CircleCollider::GetRadius() const noexcept
    {
        return m_radius;
    }

    void CircleCollider::SetRadius(float radius)
    {
        m_radius = radius > 0.0f ? radius : 0.5f;
    }

    AABB CircleCollider::ComputeAABB() const
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        const Vec2 extents{ m_radius, m_radius };
        return { center - extents, center + extents };
    }
}