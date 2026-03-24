#pragma once
#include "Export.hpp"
#include "Collider.hpp"

namespace BoltPhys {
    class BOLT_PHYS_API BoxCollider final : public Collider
    {
    public:
        explicit BoxCollider(const Vec2& scale);

        const Vec2& GetScale() const noexcept;
        void SetScale(const Vec2& scale) noexcept;

        AABB ComputeAABB() const override;

    private:
        Vec2 m_HalfExtends;
    };
}