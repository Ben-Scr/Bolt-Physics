#include "PolygonCollider.hpp"

#include "Body.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace Index::Physics {
    namespace {
        constexpr std::size_t kMinPolygonVertices = 3;
    }

    PolygonCollider::PolygonCollider()
        : Collider(ColliderType::Polygon)
    {}

    void PolygonCollider::SetVertices(const Vec2* vertices, std::size_t count)
    {
        m_Vertices.clear();
        if (vertices == nullptr || count < kMinPolygonVertices) {
            return;
        }

        m_Vertices.assign(vertices, vertices + count);
    }

    void PolygonCollider::SetVertices(std::vector<Vec2> vertices)
    {
        if (vertices.size() < kMinPolygonVertices) {
            m_Vertices.clear();
            return;
        }

        m_Vertices = std::move(vertices);
    }

    std::size_t PolygonCollider::GetVertexCount() const noexcept
    {
        return m_Vertices.size();
    }

    const Vec2* PolygonCollider::GetVertices() const noexcept
    {
        return m_Vertices.empty() ? nullptr : m_Vertices.data();
    }

    AABB PolygonCollider::ComputeAABB() const noexcept
    {
        const Vec2 bodyPosition = GetBody() ? GetBody()->GetPosition() : Vec2{};
        if (m_Vertices.empty()) {
            return { bodyPosition, bodyPosition };
        }

        Vec2 minValue{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        Vec2 maxValue{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

        for (const Vec2& vertex : m_Vertices) {
            const Vec2 worldVertex = bodyPosition + vertex;
            minValue.x = std::min(minValue.x, worldVertex.x);
            minValue.y = std::min(minValue.y, worldVertex.y);
            maxValue.x = std::max(maxValue.x, worldVertex.x);
            maxValue.y = std::max(maxValue.y, worldVertex.y);
        }

        return { minValue, maxValue };
    }
}