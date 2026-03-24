#include "BoxCollider.hpp"

#include "Body.hpp"

namespace BoltPhys {
    BoxCollider::BoxCollider(const Vec2& scale)
        : Collider(ColliderType::Box),
        m_HalfExtends(scale)
    {
    }

    const Vec2& BoxCollider::GetScale() const noexcept
    {
        return m_HalfExtends;
    }

    void BoxCollider::SetScale(const Vec2& scale) noexcept
    {
        m_HalfExtends = scale;
    }

    AABB BoxCollider::ComputeAABB() const
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        return { center - m_HalfExtends, center + m_HalfExtends };
    }
}