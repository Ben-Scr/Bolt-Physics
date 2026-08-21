#include "Physics2D.hpp"

#include "Body.hpp"
#include "BoxCollider.hpp"
#include "CircleCollider.hpp"
#include "Collider.hpp"
#include "PhysicsWorld.hpp"
#include "PolygonCollider.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace Index::Physics {
    PhysicsWorld* Physics2D::s_Context = nullptr;

    namespace {
        Vec2 GetColliderWorldPosition(const Collider& collider) noexcept
        {
            const Body* body = collider.GetBody();
            return body != nullptr ? body->GetPosition() : Vec2{ 0.0f, 0.0f };
        }

        bool ContainsPointCircle(const CircleCollider& circle, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(circle);
            const float radius = circle.GetRadius();
            return DistanceSq(point, center) <= radius * radius;
        }

        bool ContainsPointBox(const BoxCollider& box, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(box);
            const Vec2 half = box.GetHalfExtents();
            const AABB aabb{ center - half, center + half };
            return aabb.Contains(point);
        }

        bool ContainsPointPolygon(const PolygonCollider& polygon, const Vec2& point) noexcept
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

                const float denominator = vj.y - vi.y;
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

        bool ColliderContainsPoint(Collider& collider, const Vec2& point) noexcept
        {
            switch (collider.GetType()) {
            case ColliderType::Circle:
                return ContainsPointCircle(static_cast<CircleCollider&>(collider), point);
            case ColliderType::Box:
                return ContainsPointBox(static_cast<BoxCollider&>(collider), point);
            case ColliderType::Polygon:
                return ContainsPointPolygon(static_cast<PolygonCollider&>(collider), point);
            }
            return collider.ComputeAABB().Contains(point);
        }

        Contact MakeContact(Collider& a, Collider& b, const Vec2& normal, float penetration) noexcept
        {
            Contact contact{};
            contact.bodyA = a.GetBody();
            contact.bodyB = b.GetBody();
            contact.colliderA = &a;
            contact.colliderB = &b;
            contact.normal = normal;
            contact.penetration = penetration;
            return contact;
        }

        std::optional<Contact> NarrowphaseAABB(Collider& a, Collider& b) noexcept
        {
            const AABB aabbA = a.ComputeAABB();
            const AABB aabbB = b.ComputeAABB();
            if (!aabbA.Intersects(aabbB)) {
                return std::nullopt;
            }

            const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
            const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);
            if (overlapX <= 0.0f || overlapY <= 0.0f) {
                return std::nullopt;
            }

            const Vec2 delta = aabbB.GetCenter() - aabbA.GetCenter();
            Vec2 normal{ 0.0f, 0.0f };
            float penetration = 0.0f;
            if (overlapX < overlapY) {
                normal = { delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f };
                penetration = overlapX;
            }
            else {
                normal = { 0.0f, delta.y >= 0.0f ? 1.0f : -1.0f };
                penetration = overlapY;
            }
            return MakeContact(a, b, normal, penetration);
        }

        std::optional<Contact> NarrowphaseCircleCircle(CircleCollider& a, CircleCollider& b) noexcept
        {
            const Vec2 centerA = GetColliderWorldPosition(a);
            const Vec2 centerB = GetColliderWorldPosition(b);
            const Vec2 delta = centerB - centerA;
            const float radiusSum = a.GetRadius() + b.GetRadius();
            const float distanceSq = LengthSq(delta);
            if (distanceSq >= radiusSum * radiusSum) {
                return std::nullopt;
            }

            const float distance = std::sqrt(distanceSq);
            Vec2 normal = (distance > 1e-6f) ? (delta / distance) : Vec2{ 1.0f, 0.0f };
            return MakeContact(a, b, normal, radiusSum - distance);
        }

        // Returns a contact whose normal points from `circle` toward `box`. The
        // resolver moves A by `-normal * penetration`, so for any case the math
        // here picks the normal whose negation is the direction the circle should
        // move to exit the overlap.
        std::optional<Contact> NarrowphaseCircleBox(CircleCollider& circle, BoxCollider& box) noexcept
        {
            const Vec2 circleCenter = GetColliderWorldPosition(circle);
            const Vec2 boxCenter = GetColliderWorldPosition(box);
            const Vec2 boxHalf = box.GetHalfExtents();
            const Vec2 boxMin = boxCenter - boxHalf;
            const Vec2 boxMax = boxCenter + boxHalf;

            const Vec2 closest{
                std::clamp(circleCenter.x, boxMin.x, boxMax.x),
                std::clamp(circleCenter.y, boxMin.y, boxMax.y)
            };

            const Vec2 delta = circleCenter - closest;
            const float distanceSq = LengthSq(delta);
            const float radius = circle.GetRadius();

            Vec2 normal{ 0.0f, 0.0f };
            float penetration = 0.0f;

            if (distanceSq > 1e-12f) {
                // Circle center is outside the box.
                const float distance = std::sqrt(distanceSq);
                if (distance >= radius) {
                    return std::nullopt;
                }
                normal = -(delta / distance);
                penetration = radius - distance;
            }
            else {
                // Circle center is inside the box; pop out the closest edge.
                const float distLeft   = circleCenter.x - boxMin.x;
                const float distRight  = boxMax.x - circleCenter.x;
                const float distBottom = circleCenter.y - boxMin.y;
                const float distTop    = boxMax.y - circleCenter.y;

                normal = {  1.0f,  0.0f }; penetration = distLeft;
                if (distRight  < penetration) { normal = { -1.0f,  0.0f }; penetration = distRight;  }
                if (distBottom < penetration) { normal = {  0.0f,  1.0f }; penetration = distBottom; }
                if (distTop    < penetration) { normal = {  0.0f, -1.0f }; penetration = distTop;    }
                penetration += radius;
            }

            return MakeContact(circle, box, normal, penetration);
        }

        std::optional<Contact> NarrowphaseBoxCircle(BoxCollider& box, CircleCollider& circle) noexcept
        {
            auto contact = NarrowphaseCircleBox(circle, box);
            if (!contact) {
                return std::nullopt;
            }
            // Returned contact has A=circle, B=box; we want A=box, B=circle.
            std::swap(contact->bodyA, contact->bodyB);
            std::swap(contact->colliderA, contact->colliderB);
            contact->normal = -contact->normal;
            return contact;
        }

        // ---- SAT building blocks --------------------------------------------------

        struct ProjectionRange { float min; float max; };

        ProjectionRange ProjectVertices(const Vec2* verts, std::size_t count, const Vec2& axis) noexcept
        {
            float lo = Dot(verts[0], axis);
            float hi = lo;
            for (std::size_t i = 1; i < count; ++i) {
                const float p = Dot(verts[i], axis);
                if (p < lo) lo = p;
                if (p > hi) hi = p;
            }
            return { lo, hi };
        }

        // Tests one separating-axis candidate. Returns false on a separating axis
        // (no overlap on that axis -> shapes do not collide). Otherwise updates
        // bestOverlap/bestNormal if this axis offers a tighter penetration.
        // Tangent (overlap == 0) is treated as separated, matching the circle/AABB
        // narrowphase convention where touching shapes do not generate a contact.
        bool TryAxisPolyPoly(const Vec2* vertsA, std::size_t countA,
                             const Vec2* vertsB, std::size_t countB,
                             const Vec2& axisRaw,
                             float& bestOverlap, Vec2& bestNormal) noexcept
        {
            const float lenSq = LengthSq(axisRaw);
            if (lenSq <= 1e-12f) return true;
            const Vec2 axis = axisRaw / std::sqrt(lenSq);

            const auto pa = ProjectVertices(vertsA, countA, axis);
            const auto pb = ProjectVertices(vertsB, countB, axis);
            const float overlap = std::min(pa.max, pb.max) - std::max(pa.min, pb.min);
            if (overlap <= 0.0f) return false;

            if (overlap < bestOverlap) {
                bestOverlap = overlap;
                bestNormal = axis;
            }
            return true;
        }

        Vec2 ComputeCentroid(const Vec2* verts, std::size_t count) noexcept
        {
            Vec2 sum{ 0.0f, 0.0f };
            for (std::size_t i = 0; i < count; ++i) sum += verts[i];
            return sum / static_cast<float>(count);
        }

        // Generic convex polygon SAT. Returns nullopt if separating axis exists.
        // The returned normal points from `vertsA` toward `vertsB`.
        std::optional<std::pair<Vec2, float>> SATConvex(const Vec2* vertsA, std::size_t countA,
                                                        const Vec2* vertsB, std::size_t countB) noexcept
        {
            if (countA < 3 || countB < 3) return std::nullopt;

            float bestOverlap = std::numeric_limits<float>::max();
            Vec2 bestNormal{ 1.0f, 0.0f };

            auto checkEdges = [&](const Vec2* verts, std::size_t count) -> bool {
                for (std::size_t i = 0; i < count; ++i) {
                    const Vec2 v0 = verts[i];
                    const Vec2 v1 = verts[(i + 1) % count];
                    const Vec2 edge = v1 - v0;
                    // Any perpendicular edge normal is a valid SAT candidate;
                    // final contact normal orientation is corrected below.
                    const Vec2 axis{ -edge.y, edge.x };
                    if (!TryAxisPolyPoly(vertsA, countA, vertsB, countB, axis, bestOverlap, bestNormal)) {
                        return false;
                    }
                }
                return true;
            };

            if (!checkEdges(vertsA, countA)) return std::nullopt;
            if (!checkEdges(vertsB, countB)) return std::nullopt;

            // Make sure the normal points from A toward B.
            const Vec2 cA = ComputeCentroid(vertsA, countA);
            const Vec2 cB = ComputeCentroid(vertsB, countB);
            if (Dot(cB - cA, bestNormal) < 0.0f) bestNormal = -bestNormal;

            return std::make_pair(bestNormal, bestOverlap);
        }

        // Builds the four world-space corners of an axis-aligned box.
        std::array<Vec2, 4> BoxToVertices(const BoxCollider& box) noexcept
        {
            const Vec2 c = GetColliderWorldPosition(box);
            const Vec2 h = box.GetHalfExtents();
            return { Vec2{ c.x - h.x, c.y - h.y },
                     Vec2{ c.x + h.x, c.y - h.y },
                     Vec2{ c.x + h.x, c.y + h.y },
                     Vec2{ c.x - h.x, c.y + h.y } };
        }

        std::vector<Vec2> PolygonToWorldVertices(const PolygonCollider& poly) noexcept
        {
            const std::size_t count = poly.GetVertexCount();
            const Vec2* verts = poly.GetVertices();
            std::vector<Vec2> world;
            if (verts == nullptr || count < 3) return world;
            world.reserve(count);
            const Vec2 offset = GetColliderWorldPosition(poly);
            for (std::size_t i = 0; i < count; ++i) world.push_back(verts[i] + offset);
            return world;
        }

        std::optional<Contact> NarrowphasePolyPoly(PolygonCollider& a, PolygonCollider& b) noexcept
        {
            const auto worldA = PolygonToWorldVertices(a);
            const auto worldB = PolygonToWorldVertices(b);
            if (worldA.empty() || worldB.empty()) return std::nullopt;

            auto sat = SATConvex(worldA.data(), worldA.size(), worldB.data(), worldB.size());
            if (!sat) return std::nullopt;
            return MakeContact(a, b, sat->first, sat->second);
        }

        std::optional<Contact> NarrowphasePolyBox(PolygonCollider& poly, BoxCollider& box) noexcept
        {
            const auto worldPoly = PolygonToWorldVertices(poly);
            if (worldPoly.empty()) return std::nullopt;
            const auto boxVerts = BoxToVertices(box);

            auto sat = SATConvex(worldPoly.data(), worldPoly.size(), boxVerts.data(), boxVerts.size());
            if (!sat) return std::nullopt;
            return MakeContact(poly, box, sat->first, sat->second);
        }

        std::optional<Contact> NarrowphaseBoxPoly(BoxCollider& box, PolygonCollider& poly) noexcept
        {
            auto contact = NarrowphasePolyBox(poly, box);
            if (!contact) return std::nullopt;
            std::swap(contact->bodyA, contact->bodyB);
            std::swap(contact->colliderA, contact->colliderB);
            contact->normal = -contact->normal;
            return contact;
        }

        // Polygon vs circle: SAT with edge-normal axes plus the axis from the
        // closest polygon vertex to the circle center (for the corner-Voronoi case).
        std::optional<Contact> NarrowphasePolyCircle(PolygonCollider& poly, CircleCollider& circle) noexcept
        {
            const auto world = PolygonToWorldVertices(poly);
            if (world.empty()) return std::nullopt;

            const Vec2 circleCenter = GetColliderWorldPosition(circle);
            const float radius = circle.GetRadius();
            const std::size_t count = world.size();

            float bestOverlap = std::numeric_limits<float>::max();
            Vec2 bestNormal{ 1.0f, 0.0f };

            auto tryAxis = [&](const Vec2& axisRaw) -> bool {
                const float lenSq = LengthSq(axisRaw);
                if (lenSq <= 1e-12f) return true;
                const Vec2 axis = axisRaw / std::sqrt(lenSq);

                const auto pp = ProjectVertices(world.data(), count, axis);
                const float center = Dot(circleCenter, axis);
                const float pcMin = center - radius;
                const float pcMax = center + radius;
                const float overlap = std::min(pp.max, pcMax) - std::max(pp.min, pcMin);
                if (overlap <= 0.0f) return false;
                if (overlap < bestOverlap) {
                    bestOverlap = overlap;
                    bestNormal = axis;
                }
                return true;
            };

            // Edge normals.
            for (std::size_t i = 0; i < count; ++i) {
                const Vec2 v0 = world[i];
                const Vec2 v1 = world[(i + 1) % count];
                const Vec2 edge = v1 - v0;
                if (!tryAxis(Vec2{ -edge.y, edge.x })) return std::nullopt;
            }

            // Closest-vertex axis: handles the case where the circle center sits in
            // a vertex's Voronoi region instead of an edge's region.
            std::size_t closestIdx = 0;
            float closestDistSq = DistanceSq(world[0], circleCenter);
            for (std::size_t i = 1; i < count; ++i) {
                const float d = DistanceSq(world[i], circleCenter);
                if (d < closestDistSq) { closestDistSq = d; closestIdx = i; }
            }
            if (!tryAxis(circleCenter - world[closestIdx])) return std::nullopt;

            // Make the normal point from polygon to circle.
            const Vec2 polyCenter = ComputeCentroid(world.data(), count);
            if (Dot(circleCenter - polyCenter, bestNormal) < 0.0f) bestNormal = -bestNormal;

            return MakeContact(poly, circle, bestNormal, bestOverlap);
        }

        std::optional<Contact> NarrowphaseCirclePoly(CircleCollider& circle, PolygonCollider& poly) noexcept
        {
            auto contact = NarrowphasePolyCircle(poly, circle);
            if (!contact) return std::nullopt;
            std::swap(contact->bodyA, contact->bodyB);
            std::swap(contact->colliderA, contact->colliderB);
            contact->normal = -contact->normal;
            return contact;
        }

        std::optional<Contact> Narrowphase(Collider& a, Collider& b) noexcept
        {
            const ColliderType ta = a.GetType();
            const ColliderType tb = b.GetType();

            if (ta == ColliderType::Circle && tb == ColliderType::Circle) {
                return NarrowphaseCircleCircle(static_cast<CircleCollider&>(a), static_cast<CircleCollider&>(b));
            }
            if (ta == ColliderType::Circle && tb == ColliderType::Box) {
                return NarrowphaseCircleBox(static_cast<CircleCollider&>(a), static_cast<BoxCollider&>(b));
            }
            if (ta == ColliderType::Box && tb == ColliderType::Circle) {
                return NarrowphaseBoxCircle(static_cast<BoxCollider&>(a), static_cast<CircleCollider&>(b));
            }
            if (ta == ColliderType::Polygon && tb == ColliderType::Polygon) {
                return NarrowphasePolyPoly(static_cast<PolygonCollider&>(a), static_cast<PolygonCollider&>(b));
            }
            if (ta == ColliderType::Polygon && tb == ColliderType::Box) {
                return NarrowphasePolyBox(static_cast<PolygonCollider&>(a), static_cast<BoxCollider&>(b));
            }
            if (ta == ColliderType::Box && tb == ColliderType::Polygon) {
                return NarrowphaseBoxPoly(static_cast<BoxCollider&>(a), static_cast<PolygonCollider&>(b));
            }
            if (ta == ColliderType::Polygon && tb == ColliderType::Circle) {
                return NarrowphasePolyCircle(static_cast<PolygonCollider&>(a), static_cast<CircleCollider&>(b));
            }
            if (ta == ColliderType::Circle && tb == ColliderType::Polygon) {
                return NarrowphaseCirclePoly(static_cast<CircleCollider&>(a), static_cast<PolygonCollider&>(b));
            }
            // Box-vs-box: AABB is exact since neither shape rotates.
            return NarrowphaseAABB(a, b);
        }

        // ---- Query support: validation, overlap probes, raycast -------------------

        bool IsFinite(float v) noexcept
        {
            return std::isfinite(v);
        }

        bool IsFinite(const Vec2& v) noexcept
        {
            return std::isfinite(v.x) && std::isfinite(v.y);
        }

        // Iterates the active world and returns the first / all colliders the probe
        // overlaps. The probe is a temporary not registered in the world, so it can
        // never match itself.
        const Collider* OverlapShapeFirst(Collider& probe)
        {
            PhysicsWorld* context = Physics2D::GetContext();
            if (context == nullptr) {
                return nullptr;
            }
            for (Collider* other : context->GetColliders()) {
                if (other == nullptr) {
                    continue;
                }
                if (Physics2D::OverlapsWith(probe, *other)) {
                    return other;
                }
            }
            return nullptr;
        }

        std::vector<const Collider*> OverlapShapeAll(Collider& probe)
        {
            std::vector<const Collider*> result;
            PhysicsWorld* context = Physics2D::GetContext();
            if (context == nullptr) {
                return result;
            }
            for (Collider* other : context->GetColliders()) {
                if (other == nullptr) {
                    continue;
                }
                if (Physics2D::OverlapsWith(probe, *other)) {
                    result.push_back(other);
                }
            }
            return result;
        }

        // Four local-space corners of a box of the given half-extents, rotated CCW.
        std::vector<Vec2> MakeRotatedBoxVerts(const Vec2& half, float degrees) noexcept
        {
            const float radians = degrees * 0.01745329252f; // pi / 180
            const float c = std::cos(radians);
            const float s = std::sin(radians);
            auto rotate = [&](float x, float y) noexcept -> Vec2 {
                return Vec2{ x * c - y * s, x * s + y * c };
            };
            return {
                rotate(-half.x, -half.y),
                rotate( half.x, -half.y),
                rotate( half.x,  half.y),
                rotate(-half.x,  half.y)
            };
        }

        struct RayHit
        {
            bool  hit = false;
            float t = 0.0f;
            Vec2  normal{ 0.0f, 0.0f };
        };

        // Ray (origin + dir*t, dir normalized) vs circle. Origin inside -> t = 0.
        RayHit RaycastCircle(const Vec2& origin, const Vec2& dir, float maxDistance,
                             const Vec2& center, float radius) noexcept
        {
            const Vec2 m = origin - center;
            const float b = Dot(m, dir);
            const float c = Dot(m, m) - radius * radius;

            if (c <= 0.0f) {
                return { true, 0.0f, -dir }; // origin inside / on the circle
            }

            const float discriminant = b * b - c;
            if (discriminant < 0.0f) {
                return {};
            }

            const float t = -b - std::sqrt(discriminant);
            if (t < 0.0f || t > maxDistance) {
                return {};
            }

            const Vec2 point = origin + dir * t;
            return { true, t, Normalize(point - center) };
        }

        // Ray vs axis-aligned box (slab method). Origin inside -> t = 0.
        RayHit RaycastBox(const Vec2& origin, const Vec2& dir, float maxDistance,
                          const Vec2& boxMin, const Vec2& boxMax) noexcept
        {
            float tNear = -std::numeric_limits<float>::infinity();
            float tFar  =  std::numeric_limits<float>::infinity();
            int   hitAxis = 0;
            float hitSign = 0.0f;

            const float o[2]  = { origin.x, origin.y };
            const float d[2]  = { dir.x, dir.y };
            const float lo[2] = { boxMin.x, boxMin.y };
            const float hi[2] = { boxMax.x, boxMax.y };

            for (int k = 0; k < 2; ++k) {
                if (std::fabs(d[k]) < 1e-8f) {
                    if (o[k] < lo[k] || o[k] > hi[k]) {
                        return {}; // parallel to this slab and outside it
                    }
                    continue;
                }
                const float inv = 1.0f / d[k];
                float t1 = (lo[k] - o[k]) * inv; // lo plane
                float t2 = (hi[k] - o[k]) * inv; // hi plane
                float sign = -1.0f;              // entering through the lo plane
                if (t1 > t2) {
                    std::swap(t1, t2);
                    sign = 1.0f;                 // entering through the hi plane
                }
                if (t1 > tNear) {
                    tNear = t1;
                    hitAxis = k;
                    hitSign = sign;
                }
                if (t2 < tFar) {
                    tFar = t2;
                }
                if (tNear > tFar) {
                    return {};
                }
            }

            if (tFar < 0.0f) {
                return {};
            }
            if (tNear < 0.0f) {
                return { true, 0.0f, -dir }; // origin inside the box
            }
            if (tNear > maxDistance) {
                return {};
            }

            const Vec2 normal = (hitAxis == 0) ? Vec2{ hitSign, 0.0f } : Vec2{ 0.0f, hitSign };
            return { true, tNear, normal };
        }

        // Ray vs convex polygon (Cyrus-Beck half-plane clip). Origin inside -> t = 0.
        RayHit RaycastPolygon(const Vec2& origin, const Vec2& dir, float maxDistance,
                              const std::vector<Vec2>& verts) noexcept
        {
            const std::size_t count = verts.size();
            if (count < 3) {
                return {};
            }

            const Vec2 centroid = ComputeCentroid(verts.data(), count);
            float tEnter = -std::numeric_limits<float>::infinity();
            float tLeave =  std::numeric_limits<float>::infinity();
            Vec2  enterNormal{ 0.0f, 0.0f };

            for (std::size_t i = 0; i < count; ++i) {
                const Vec2 v0 = verts[i];
                const Vec2 v1 = verts[(i + 1) % count];
                const Vec2 edge = v1 - v0;
                Vec2 normal{ edge.y, -edge.x };
                const Vec2 mid = (v0 + v1) * 0.5f;
                if (Dot(mid - centroid, normal) < 0.0f) {
                    normal = -normal; // make it point outward
                }
                normal = Normalize(normal);

                const float denominator = Dot(dir, normal);
                const float numerator = Dot(v0 - origin, normal);

                if (std::fabs(denominator) < 1e-8f) {
                    if (numerator < 0.0f) {
                        return {}; // parallel to this edge and outside it
                    }
                    continue;
                }

                const float t = numerator / denominator;
                if (denominator < 0.0f) {
                    if (t > tEnter) {
                        tEnter = t;
                        enterNormal = normal;
                    }
                }
                else {
                    if (t < tLeave) {
                        tLeave = t;
                    }
                }
                if (tEnter > tLeave) {
                    return {};
                }
            }

            if (tEnter > tLeave || tLeave < 0.0f) {
                return {};
            }
            if (tEnter < 0.0f) {
                return { true, 0.0f, -dir }; // origin inside the polygon
            }
            if (tEnter > maxDistance) {
                return {};
            }
            return { true, tEnter, enterNormal };
        }
    }

    void Physics2D::SetContext(PhysicsWorld& world) noexcept
    {
        s_Context = &world;
    }

    void Physics2D::ClearContext() noexcept
    {
        s_Context = nullptr;
    }

    PhysicsWorld* Physics2D::GetContext() noexcept
    {
        return s_Context;
    }

    std::optional<Contact> Physics2D::OverlapsWith(Collider& collider)
    {
        if (s_Context == nullptr) {
            return std::nullopt;
        }

        for (Collider* other : s_Context->GetColliders()) {
            if (other == nullptr || other == &collider) {
                continue;
            }

            if (auto contact = OverlapsWith(collider, *other)) {
                return contact;
            }
        }

        return std::nullopt;
    }

    std::optional<Contact> Physics2D::OverlapsWith(Collider& colliderA, Collider& colliderB)
    {
        return Narrowphase(colliderA, colliderB);
    }

    const Collider* Physics2D::ContainsPoint(const Vec2& point)
    {
        if (s_Context == nullptr) {
            return nullptr;
        }

        for (Collider* collider : s_Context->GetColliders()) {
            if (collider == nullptr) {
                continue;
            }

            if (ContainsPoint(*collider, point)) {
                return collider;
            }
        }

        return nullptr;
    }

    bool Physics2D::ContainsPoint(Collider& collider, const Vec2& point)
    {
        return ColliderContainsPoint(collider, point);
    }

    RaycastHit Physics2D::Raycast(const Vec2& origin, const Vec2& direction, float maxDistance)
    {
        RaycastHit result;

        if (LengthSq(direction) <= 1e-12f || !IsFinite(origin) || !IsFinite(direction)) {
            return result;
        }
        if (!(maxDistance > 0.0f) || std::isnan(maxDistance)) {
            return result;
        }
        if (s_Context == nullptr) {
            return result;
        }

        const Vec2 dir = Normalize(direction);

        float bestT = std::numeric_limits<float>::infinity();
        const Collider* bestCollider = nullptr;
        Vec2 bestNormal{ 0.0f, 0.0f };

        for (Collider* collider : s_Context->GetColliders()) {
            if (collider == nullptr) {
                continue;
            }

            RayHit rayHit;
            switch (collider->GetType()) {
            case ColliderType::Circle: {
                CircleCollider& circle = static_cast<CircleCollider&>(*collider);
                rayHit = RaycastCircle(origin, dir, maxDistance,
                                       GetColliderWorldPosition(circle), circle.GetRadius());
                break;
            }
            case ColliderType::Box: {
                BoxCollider& box = static_cast<BoxCollider&>(*collider);
                const Vec2 center = GetColliderWorldPosition(box);
                const Vec2 half = box.GetHalfExtents();
                rayHit = RaycastBox(origin, dir, maxDistance, center - half, center + half);
                break;
            }
            case ColliderType::Polygon: {
                PolygonCollider& polygon = static_cast<PolygonCollider&>(*collider);
                rayHit = RaycastPolygon(origin, dir, maxDistance, PolygonToWorldVertices(polygon));
                break;
            }
            }

            if (rayHit.hit && rayHit.t < bestT) {
                bestT = rayHit.t;
                bestCollider = collider;
                bestNormal = rayHit.normal;
            }
        }

        if (bestCollider == nullptr) {
            return result;
        }

        result.hit = true;
        result.collider = bestCollider;
        result.point = origin + dir * bestT;
        result.normal = bestNormal;
        result.distance = bestT;
        return result;
    }

    bool Physics2D::RaycastCheck(const Vec2& origin, const Vec2& direction, float maxDistance)
    {
        return Raycast(origin, direction, maxDistance).hit;
    }

    const Collider* Physics2D::OverlapCircle(const Vec2& origin, float radius)
    {
        if (!(radius > 0.0f) || !IsFinite(radius) || !IsFinite(origin)) {
            return nullptr;
        }
        Body probe;
        probe.SetPosition(origin);
        CircleCollider shape(radius);
        shape.SetBody(&probe);
        return OverlapShapeFirst(shape);
    }

    bool Physics2D::OverlapCircleCheck(const Vec2& origin, float radius)
    {
        return OverlapCircle(origin, radius) != nullptr;
    }

    std::vector<const Collider*> Physics2D::OverlapCircleAll(const Vec2& origin, float radius)
    {
        if (!(radius > 0.0f) || !IsFinite(radius) || !IsFinite(origin)) {
            return {};
        }
        Body probe;
        probe.SetPosition(origin);
        CircleCollider shape(radius);
        shape.SetBody(&probe);
        return OverlapShapeAll(shape);
    }

    const Collider* Physics2D::OverlapBox(const Vec2& origin, const Vec2& size, float rotationDegrees)
    {
        if (!(size.x > 0.0f) || !(size.y > 0.0f) || !IsFinite(size)
            || !IsFinite(rotationDegrees) || !IsFinite(origin)) {
            return nullptr;
        }

        Body probe;
        probe.SetPosition(origin);
        const Vec2 half = size * 0.5f;

        if (std::fabs(rotationDegrees) <= 1e-4f) {
            BoxCollider shape(half);
            shape.SetBody(&probe);
            return OverlapShapeFirst(shape);
        }

        PolygonCollider shape;
        shape.SetVertices(MakeRotatedBoxVerts(half, rotationDegrees));
        shape.SetBody(&probe);
        return OverlapShapeFirst(shape);
    }

    bool Physics2D::OverlapBoxCheck(const Vec2& origin, const Vec2& size, float rotationDegrees)
    {
        return OverlapBox(origin, size, rotationDegrees) != nullptr;
    }

    std::vector<const Collider*> Physics2D::OverlapBoxAll(const Vec2& origin, const Vec2& size, float rotationDegrees)
    {
        if (!(size.x > 0.0f) || !(size.y > 0.0f) || !IsFinite(size)
            || !IsFinite(rotationDegrees) || !IsFinite(origin)) {
            return {};
        }

        Body probe;
        probe.SetPosition(origin);
        const Vec2 half = size * 0.5f;

        if (std::fabs(rotationDegrees) <= 1e-4f) {
            BoxCollider shape(half);
            shape.SetBody(&probe);
            return OverlapShapeAll(shape);
        }

        PolygonCollider shape;
        shape.SetVertices(MakeRotatedBoxVerts(half, rotationDegrees));
        shape.SetBody(&probe);
        return OverlapShapeAll(shape);
    }

    const Collider* Physics2D::OverlapPolygon(const Vec2& origin, const Vec2* points, std::size_t count)
    {
        if (points == nullptr || count < 3 || !IsFinite(origin)) {
            return nullptr;
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (!IsFinite(points[i])) {
                return nullptr;
            }
        }
        Body probe;
        probe.SetPosition(origin);
        PolygonCollider shape;
        shape.SetVertices(points, count);
        shape.SetBody(&probe);
        return OverlapShapeFirst(shape);
    }

    bool Physics2D::OverlapPolygonCheck(const Vec2& origin, const Vec2* points, std::size_t count)
    {
        return OverlapPolygon(origin, points, count) != nullptr;
    }

    std::vector<const Collider*> Physics2D::OverlapPolygonAll(const Vec2& origin, const Vec2* points, std::size_t count)
    {
        if (points == nullptr || count < 3 || !IsFinite(origin)) {
            return {};
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (!IsFinite(points[i])) {
                return {};
            }
        }
        Body probe;
        probe.SetPosition(origin);
        PolygonCollider shape;
        shape.SetVertices(points, count);
        shape.SetBody(&probe);
        return OverlapShapeAll(shape);
    }

    std::vector<const Collider*> Physics2D::ContainsPointAll(const Vec2& point)
    {
        std::vector<const Collider*> result;
        if (s_Context == nullptr) {
            return result;
        }
        for (Collider* collider : s_Context->GetColliders()) {
            if (collider == nullptr) {
                continue;
            }
            if (ContainsPoint(*collider, point)) {
                result.push_back(collider);
            }
        }
        return result;
    }
}
