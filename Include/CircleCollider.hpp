#pragma once
#include "Export.hpp"
#include "Collider.hpp"

namespace IndexPhys {
    class INDEX_PHYS_API CircleCollider final : public Collider
    {
    public:
        explicit CircleCollider(float radius);

        float GetRadius() const noexcept;
        void SetRadius(float radius);

        AABB ComputeAABB() const noexcept override;

    private:
        float m_Radius = 0.5f;
    };
}