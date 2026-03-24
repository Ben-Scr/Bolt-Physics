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

            body->SetPosition(body->GetPosition() - (body->GetVelocity() * dt));

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