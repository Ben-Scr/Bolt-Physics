#include "BoxCollider2D.hpp"

#include "Body2D.hpp"

namespace BoltPhys {
    BoxCollider2D::BoxCollider2D() noexcept
        : Collider2D(ColliderType::Box), m_halfExtents({ 0.5f, 0.5f }) {

    }

    BoxCollider2D::BoxCollider2D(const Vec2& halfExtents)
        : Collider2D(ColliderType::Box),
        m_halfExtents(halfExtents)
    {
    }

    const Vec2& BoxCollider2D::GetHalfExtents() const noexcept
    {
        return m_halfExtents;
    }

    void BoxCollider2D::SetHalfExtents(const Vec2& halfExtents) noexcept
    {
        m_halfExtents = halfExtents;
    }

    AABB BoxCollider2D::ComputeAABB() const
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        return { center - m_halfExtents, center + m_halfExtents };
    }
}