#pragma once
#include "Export.hpp"
#include "Collider.hpp"

#include <vector>

namespace IndexPhys {
    class INDEX_PHYS_API PolygonCollider final : public Collider
    {
    public:
        PolygonCollider();

        // Vertices are interpreted as a local-space convex polygon. Collision
        // queries use SAT and do not support concave polygons.
        void SetVertices(const Vec2* vertices, std::size_t count);
        void SetVertices(std::vector<Vec2> vertices);
        std::size_t GetVertexCount() const noexcept;
        const Vec2* GetVertices() const noexcept;

        AABB ComputeAABB() const noexcept override;

    private:
        std::vector<Vec2> m_Vertices;
    };
}
