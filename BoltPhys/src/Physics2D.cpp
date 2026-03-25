#include "Physics2D.hpp"

#include "Body2D.hpp"
#include "BoxCollider2D.hpp"
#include "CircleCollider.hpp"
#include "Collider2D.hpp"
#include "PhysicsWorld.hpp"
#include "PolygonCollider2D.hpp"

#include <algorithm>
#include <cmath>

namespace BoltPhys {
    PhysicsWorld2D* Physics2D::s_context = nullptr;

    namespace {
        bool ContainsPointInAABB(const AABB& aabb, const Vec2& point) noexcept
        {
            return point.x >= aabb.min.x && point.x <= aabb.max.x &&
                point.y >= aabb.min.y && point.y <= aabb.max.y;
        }

        Vec2 GetColliderWorldPosition(Collider2D& collider) noexcept
        {
            const Body2D* body = collider.GetBody();
            return body != nullptr ? body->GetPosition() : Vec2{};
        }

        bool ContainsPointCircle(CircleCollider& circle, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(circle);
            const Vec2 delta = point - center;
            const float radius = circle.GetRadius();
            return Dot(delta, delta) <= radius * radius;
        }

        bool ContainsPointBox(BoxCollider2D& box, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(box);
            const Vec2 half = box.GetHalfExtents();
            const AABB aabb{ center - half, center + half };
            return ContainsPointInAABB(aabb, point);
        }

        bool ContainsPointPolygon(PolygonCollider2D& polygon, const Vec2& point) noexcept
        {
            const std::size_t vertexCount = polygon.GetVertexCount();
            const Vec2* vertices = polygon.GetVertices();
            if (vertices == nullptr || vertexCount < 3) {
                return false;
            }

            const Vec2 offset = GetColliderWorldPosition(polygon);
            bool inside = false;

            for (std::size_t i = 0, j = vertexCount - 1; i < vertexCount; j = i++) {
                const Vec2 vi = vertices[i] + offset;
                const Vec2 vj = vertices[j] + offset;

                const bool intersectsY = (vi.y > point.y) != (vj.y > point.y);
                if (!intersectsY) {
                    continue;
                }

                const float denominator = (vj.y - vi.y);
                if (std::fabs(denominator) <= 1e-6f) {
                    continue;
                }

                const float xCross = ((vj.x - vi.x) * (point.y - vi.y) / denominator) + vi.x;
                if (point.x < xCross) {
                    inside = !inside;
                }
            }

            return inside;
        }

        bool ColliderContainsPoint(Collider2D& collider, const Vec2& point) noexcept
        {
            switch (collider.GetType()) {
            case ColliderType::Circle:
                return ContainsPointCircle(static_cast<CircleCollider&>(collider), point);
            case ColliderType::Box:
                return ContainsPointBox(static_cast<BoxCollider2D&>(collider), point);
            case ColliderType::Polygon:
                return ContainsPointPolygon(static_cast<PolygonCollider2D&>(collider), point);
            default:
                return ContainsPointInAABB(collider.ComputeAABB(), point);
            }
        }

        Contact BuildAabbContact(Collider2D& colliderA, Collider2D& colliderB) noexcept
        {
            const AABB aabbA = colliderA.ComputeAABB();
            const AABB aabbB = colliderB.ComputeAABB();

            const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
            const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);

            const Vec2 centerA = (aabbA.min + aabbA.max) * 0.5f;
            const Vec2 centerB = (aabbB.min + aabbB.max) * 0.5f;
            const Vec2 delta = centerB - centerA;

            Contact contact{};
            contact.bodyA = colliderA.GetBody();
            contact.bodyB = colliderB.GetBody();
            contact.penetration = std::min(overlapX, overlapY);

            if (overlapX < overlapY) {
                contact.normal = { delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f };
            }
            else {
                contact.normal = { 0.0f, delta.y >= 0.0f ? 1.0f : -1.0f };
            }

            return contact;
        }
    }

    void Physics2D::SetContext(PhysicsWorld2D& world) noexcept
    {
        s_context = &world;
    }

    void Physics2D::ClearContext() noexcept
    {
        s_context = nullptr;
    }

    Contact* Physics2D::OverlapsWith(Collider2D& collider)
    {
        if (s_context == nullptr) {
            return nullptr;
        }

        for (Collider2D* other : s_context->GetColliders()) {
            if (other == nullptr || other == &collider) {
                continue;
            }

            Contact* contact = OverlapsWith(collider, *other);
            if (contact != nullptr) {
                return contact;
            }
        }

        return nullptr;
    }

    Contact* Physics2D::OverlapsWith(Collider2D& colliderA, Collider2D& colliderB)
    {
        const AABB aabbA = colliderA.ComputeAABB();
        const AABB aabbB = colliderB.ComputeAABB();
        if (!aabbA.Intersects(aabbB)) {
            return nullptr;
        }

        thread_local Contact s_contact{};
        s_contact = BuildAabbContact(colliderA, colliderB);
        return &s_contact;
    }

    const Collider2D* Physics2D::ContainsPoint(const Vec2& point)
    {
        if (s_context == nullptr) {
            return nullptr;
        }

        for (Collider2D* collider : s_context->GetColliders()) {
            if (collider == nullptr) {
                continue;
            }

            if (ContainsPoint(*collider, point)) {
                return collider;
            }
        }

        return nullptr;
    }

    bool Physics2D::ContainsPoint(Collider2D& collider, const Vec2& point)
    {
        return ColliderContainsPoint(collider, point);
    }
}