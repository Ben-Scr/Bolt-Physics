#include "PhysicsWorld.hpp"

#include "BoxCollider.hpp"
#include "CircleCollider.hpp"
#include "PolygonCollider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace BoltPhys {
    namespace {
        template <typename T>
        bool Contains(const std::vector<T*>& items, const T& item)
        {
            return std::find(items.begin(), items.end(), &item) != items.end();
        }

        float Clamp(float value, float minValue, float maxValue) noexcept
        {
            return std::max(minValue, std::min(value, maxValue));
        }

        Vec2 Clamp(const Vec2& value, const Vec2& minValue, const Vec2& maxValue) noexcept
        {
            return {
                Clamp(value.x, minValue.x, maxValue.x),
                Clamp(value.y, minValue.y, maxValue.y)
            };
        }
    }

    float Length(const Vec2& v) noexcept
    {
        return std::sqrt(LengthSq(v));
    }

    Vec2 Normalize(const Vec2& v) noexcept
    {
        const float length = Length(v);
        if (length <= std::numeric_limits<float>::epsilon()) {
            return {};
        }

        return v / length;
    }

    Collider::Collider(ColliderType type) noexcept
        : m_type(type)
    {}

    ColliderType Collider::GetType() const noexcept
    {
        return m_type;
    }

    Body* Collider::GetBody() noexcept
    {
        return m_body;
    }

    const Body* Collider::GetBody() const noexcept
    {
        return m_body;
    }

    void Collider::SetBody(Body* body) noexcept
    {
        m_body = body;
    }

    Body::Body(BodyType type) noexcept
        : m_type(type)
    {}

    BodyType Body::GetType() const noexcept
    {
        return m_type;
    }

    const Vec2& Body::GetPosition() const noexcept
    {
        return m_position;
    }

    void Body::SetPosition(const Vec2& p) noexcept
    {
        m_position = p;
    }

    const Vec2& Body::GetVelocity() const noexcept
    {
        return m_velocity;
    }

    void Body::SetVelocity(const Vec2& v) noexcept
    {
        m_velocity = v;
    }

    float Body::GetMass() const noexcept
    {
        return m_mass;
    }

    void Body::SetMass(float mass) noexcept
    {
        m_mass = mass > 0.0f ? mass : 1.0f;
    }

    bool Body::IsBoundaryCheckEnabled() const noexcept
    {
        return m_boundaryCheckEnabled;
    }

    void Body::SetBoundaryCheckEnabled(bool enabled) noexcept
    {
        m_boundaryCheckEnabled = enabled;
    }

    bool Body::IsGravityEnabled() const noexcept
    {
        return m_gravityEnabled;
    }

    void Body::SetGravityEnabled(bool enabled) noexcept
    {
        m_gravityEnabled = enabled;
    }

    Collider* Body::GetCollider() noexcept
    {
        return m_collider;
    }

    const Collider* Body::GetCollider() const noexcept
    {
        return m_collider;
    }

    void Body::AttachCollider(Collider* collider) noexcept
    {
        m_collider = collider;
    }

    BoxCollider::BoxCollider(const Vec2& halfExtents)
        : Collider(ColliderType::Box),
        m_halfExtents(halfExtents)
    {}

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

    CircleCollider::CircleCollider(float radius)
        : Collider(ColliderType::Circle),
        m_radius(radius > 0.0f ? radius : 0.5f)
    {}

    float CircleCollider::GetRadius() const noexcept
    {
        return m_radius;
    }

    void CircleCollider::SetRadius(float radius)
    {
        m_radius = radius > 0.0f ? radius : 0.5f;
    }

    AABB CircleCollider::ComputeAABB() const
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        const Vec2 extents{ m_radius, m_radius };
        return { center - extents, center + extents };
    }

    PolygonCollider::PolygonCollider()
        : Collider(ColliderType::Polygon)
    {}

    void PolygonCollider::SetVertices(const Vec2* vertices, std::size_t count)
    {
        m_vertices.clear();
        if (vertices == nullptr || count == 0) {
            return;
        }

        m_vertices.assign(vertices, vertices + count);
    }

    std::size_t PolygonCollider::GetVertexCount() const noexcept
    {
        return m_vertices.size();
    }

    const Vec2* PolygonCollider::GetVertices() const noexcept
    {
        return m_vertices.empty() ? nullptr : m_vertices.data();
    }

    AABB PolygonCollider::ComputeAABB() const
    {
        const Vec2 bodyPosition = GetBody() ? GetBody()->GetPosition() : Vec2{};
        if (m_vertices.empty()) {
            return { bodyPosition, bodyPosition };
        }

        Vec2 minValue{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        Vec2 maxValue{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

        for (const Vec2& vertex : m_vertices) {
            const Vec2 worldVertex = bodyPosition + vertex;
            minValue.x = std::min(minValue.x, worldVertex.x);
            minValue.y = std::min(minValue.y, worldVertex.y);
            maxValue.x = std::max(maxValue.x, worldVertex.x);
            maxValue.y = std::max(maxValue.y, worldVertex.y);
        }

        return { minValue, maxValue };
    }

    PhysicsWorld::PhysicsWorld() = default;

    PhysicsWorld::PhysicsWorld(const WorldSettings& settings)
        : m_settings(settings)
    {}

    void PhysicsWorld::SetSettings(const WorldSettings& settings) noexcept
    {
        m_settings = settings;
    }

    const WorldSettings& PhysicsWorld::GetSettings() const noexcept
    {
        return m_settings;
    }

    bool PhysicsWorld::RegisterBody(Body& body)
    {
        if (Contains(m_bodies, body)) {
            return false;
        }

        m_bodies.push_back(&body);
        return true;
    }

    bool PhysicsWorld::UnregisterBody(Body& body)
    {
        const auto it = std::find(m_bodies.begin(), m_bodies.end(), &body);
        if (it == m_bodies.end()) {
            return false;
        }

        DetachCollider(body);
        m_bodies.erase(it);
        return true;
    }

    bool PhysicsWorld::RegisterCollider(Collider& collider)
    {
        if (Contains(m_colliders, collider)) {
            return false;
        }

        m_colliders.push_back(&collider);
        return true;
    }

    bool PhysicsWorld::UnregisterCollider(Collider& collider)
    {
        const auto it = std::find(m_colliders.begin(), m_colliders.end(), &collider);
        if (it == m_colliders.end()) {
            return false;
        }

        if (Body* body = collider.GetBody()) {
            body->AttachCollider(nullptr);
            collider.SetBody(nullptr);
        }

        m_colliders.erase(it);
        return true;
    }

    bool PhysicsWorld::AttachCollider(Body& body, Collider& collider)
    {
        if (!Contains(m_bodies, body) || !Contains(m_colliders, collider)) {
            return false;
        }

        if (collider.GetBody() != nullptr && collider.GetBody() != &body) {
            return false;
        }

        if (body.GetCollider() != nullptr && body.GetCollider() != &collider) {
            body.GetCollider()->SetBody(nullptr);
        }

        body.AttachCollider(&collider);
        collider.SetBody(&body);
        return true;
    }

    void PhysicsWorld::DetachCollider(Body& body)
    {
        if (Collider* collider = body.GetCollider()) {
            collider->SetBody(nullptr);
            body.AttachCollider(nullptr);
        }
    }

    void PhysicsWorld::Step(float dt)
    {
        if (dt <= 0.0f) {
            return;
        }

        IntegrateBodies(dt);
        DetectCollisions();
    }

    std::size_t PhysicsWorld::GetBodyCount() const noexcept
    {
        return m_bodies.size();
    }

    std::size_t PhysicsWorld::GetColliderCount() const noexcept
    {
        return m_colliders.size();
    }

    const std::vector<Contact>& PhysicsWorld::GetContacts() const noexcept
    {
        return m_contacts;
    }

    void PhysicsWorld::IntegrateBodies(float dt)
    {
        for (Body* body : m_bodies) {
            if (body == nullptr || body->GetType() == BodyType::Static) {
                continue;
            }

            if (body->GetType() == BodyType::Dynamic && body->IsGravityEnabled()) {
                body->SetVelocity(body->GetVelocity() + (m_settings.gravity * dt));
            }

            body->SetPosition(body->GetPosition() + (body->GetVelocity() * dt));

            if (m_settings.enableWorldBounds && body->IsBoundaryCheckEnabled()) {
                ApplyWorldBounds(*body);
            }
        }
    }

    void PhysicsWorld::ApplyWorldBounds(Body& body) const noexcept
    {
        Vec2 position = Clamp(body.GetPosition(), m_settings.worldMin, m_settings.worldMax);
        Vec2 velocity = body.GetVelocity();

        if (position.x == m_settings.worldMin.x || position.x == m_settings.worldMax.x) {
            velocity.x = 0.0f;
        }
        if (position.y == m_settings.worldMin.y || position.y == m_settings.worldMax.y) {
            velocity.y = 0.0f;
        }

        body.SetPosition(position);
        body.SetVelocity(velocity);
    }

    void PhysicsWorld::DetectCollisions()
    {
        m_contacts.clear();

        for (std::size_t i = 0; i < m_bodies.size(); ++i) {
            Body* bodyA = m_bodies[i];
            if (bodyA == nullptr || bodyA->GetCollider() == nullptr) {
                continue;
            }

            for (std::size_t j = i + 1; j < m_bodies.size(); ++j) {
                Body* bodyB = m_bodies[j];
                if (bodyB == nullptr || bodyB->GetCollider() == nullptr) {
                    continue;
                }

                const AABB aabbA = bodyA->GetCollider()->ComputeAABB();
                const AABB aabbB = bodyB->GetCollider()->ComputeAABB();
                if (!aabbA.Intersects(aabbB)) {
                    continue;
                }

                m_contacts.push_back(BuildContact(*bodyA, *bodyB));
            }
        }
    }

    Contact PhysicsWorld::BuildContact(Body& bodyA, Body& bodyB) const
    {
        const AABB aabbA = bodyA.GetCollider()->ComputeAABB();
        const AABB aabbB = bodyB.GetCollider()->ComputeAABB();

        const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
        const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);

        Contact contact{};
        contact.bodyA = &bodyA;
        contact.bodyB = &bodyB;
        contact.penetration = std::min(overlapX, overlapY);

        const Vec2 delta = bodyB.GetPosition() - bodyA.GetPosition();
        if (overlapX < overlapY) {
            contact.normal = { delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f };
        }
        else {
            contact.normal = { 0.0f, delta.y >= 0.0f ? 1.0f : -1.0f };
        }

        return contact;
    }
}