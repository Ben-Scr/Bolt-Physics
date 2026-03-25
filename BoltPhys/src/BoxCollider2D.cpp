#include "BoxCollider2D.hpp"

#include "Body2D.hpp"

namespace BoltPhys {
    BoxCollider::BoxCollider(const Vec2& halfExtents)
        : Collider2D(ColliderType::Box),
        m_halfExtents(halfExtents)
    {
    }

    const Vec2& BoxCollider::GetHalfExtents() const noexcept
    {
        return m_halfExtents;
    }

    void BoxCollider::SetHalfExtents(const Vec2& halfExtents) noexcept
    {
        m_halfExtents = halfExtents;
    }

    AABB BoxCollider::ComputeAABB() const
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        return { center - m_halfExtents, center + m_halfExtents };
    }
}