#include "BoxCollider.hpp"
#include "Body.hpp"

#include <algorithm>
#include <cmath>

namespace Index::Physics {
    namespace {
        Vec2 SanitizeHalfExtents(const Vec2& halfExtents) noexcept
        {
            constexpr float kMinExtent = 1e-4f;
            return {
                std::max(std::fabs(halfExtents.x), kMinExtent),
                std::max(std::fabs(halfExtents.y), kMinExtent)
            };
        }
    }

    BoxCollider::BoxCollider() noexcept
        : Collider(ColliderType::Box), m_HalfExtents({ 0.5f, 0.5f }) {

    }

    BoxCollider::BoxCollider(const Vec2& halfExtents)
        : Collider(ColliderType::Box),
        m_HalfExtents(SanitizeHalfExtents(halfExtents))
    {}

    const Vec2& BoxCollider::GetHalfExtents() const noexcept
    {
        return m_HalfExtents;
    }

    void BoxCollider::SetHalfExtents(const Vec2& halfExtents) noexcept
    {
        m_HalfExtents = SanitizeHalfExtents(halfExtents);
    }

    AABB BoxCollider::ComputeAABB() const noexcept
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        return { center - m_HalfExtents, center + m_HalfExtents };
    }
}