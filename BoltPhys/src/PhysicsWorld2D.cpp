#include "PhysicsWorld2D.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

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

        struct CellCoord
        {
            int x = 0;
            int y = 0;

            bool operator==(const CellCoord& other) const noexcept
            {
                return x == other.x && y == other.y;
            }
        };

        struct CellCoordHasher
        {
            std::size_t operator()(const CellCoord& coord) const noexcept
            {
                const std::size_t x = static_cast<std::size_t>(static_cast<std::uint32_t>(coord.x));
                const std::size_t y = static_cast<std::size_t>(static_cast<std::uint32_t>(coord.y));
                return x ^ (y + 0x9e3779b9 + (x << 6) + (x >> 2));
            }
        };

        int ComputeCellIndex(float value, float cellSize) noexcept
        {
            return static_cast<int>(std::floor(value / cellSize));
        }

        std::uint64_t MakePairKey(std::size_t indexA, std::size_t indexB) noexcept
        {
            const std::uint64_t lower = static_cast<std::uint64_t>(std::min(indexA, indexB));
            const std::uint64_t upper = static_cast<std::uint64_t>(std::max(indexA, indexB));
            return (lower << 32) | upper;
        }

        float ComputeInverseMass(const Body2D& body) noexcept
        {
            const float mass = body.GetMass();
            if (mass <= std::numeric_limits<float>::epsilon()) {
                return 0.0f;
            }

            return 1.0f / mass;
        }

        void DetachBodyAndCollider(Body2D& body) noexcept
        {
            Collider2D* collider = body.GetCollider();
            if (collider == nullptr) {
                return;
            }

            collider->SetBody(nullptr);
            body.AttachCollider(nullptr);
        }

        struct BodyCollisionData
        {
            Body2D* body = nullptr;
            AABB aabb{};
        };
    }

    PhysicsWorld2D::PhysicsWorld2D() = default;

    PhysicsWorld2D::PhysicsWorld2D(const WorldSettings& settings)
        : m_settings(SanitizeSettings(settings))
    {}

    WorldSettings PhysicsWorld2D::SanitizeSettings(const WorldSettings& settings) noexcept
    {
        WorldSettings sanitized = settings;

        constexpr float kMinCellSize = 1e-3f;
        sanitized.broadphaseCellSize = std::max(sanitized.broadphaseCellSize, kMinCellSize);

        if (sanitized.worldMin.x > sanitized.worldMax.x) {
            std::swap(sanitized.worldMin.x, sanitized.worldMax.x);
        }
        if (sanitized.worldMin.y > sanitized.worldMax.y) {
            std::swap(sanitized.worldMin.y, sanitized.worldMax.y);
        }

        return sanitized;
    }

    void PhysicsWorld2D::SetSettings(const WorldSettings& settings) noexcept
    {
        m_settings = SanitizeSettings(settings);
    }

    const WorldSettings& PhysicsWorld2D::GetSettings() const noexcept
    {
        return m_settings;
    }

    const std::vector<Collider2D*>& PhysicsWorld2D::GetColliders() const noexcept
    {
        return m_colliders;
    }

    bool PhysicsWorld2D::RegisterBody(Body2D& body)
    {
        if (Contains(m_bodies, body)) {
            return false;
        }

        m_bodies.push_back(&body);
        return true;
    }

    bool PhysicsWorld2D::UnregisterBody(Body2D& body)
    {
        const auto it = std::find(m_bodies.begin(), m_bodies.end(), &body);
        if (it == m_bodies.end()) {
            return false;
        }

        DetachBodyAndCollider(body);
        m_bodies.erase(it);

        m_contacts.erase(
            std::remove_if(m_contacts.begin(), m_contacts.end(), [&body](const Contact& contact) {
                return contact.bodyA == &body || contact.bodyB == &body;
                }),
            m_contacts.end());

        return true;
    }

    bool PhysicsWorld2D::RegisterCollider(Collider2D& collider)
    {
        if (Contains(m_colliders, collider)) {
            return false;
        }

        m_colliders.push_back(&collider);
        return true;
    }

    bool PhysicsWorld2D::UnregisterCollider(Collider2D& collider)
    {
        const auto it = std::find(m_colliders.begin(), m_colliders.end(), &collider);
        if (it == m_colliders.end()) {
            return false;
        }

        if (Body2D* body = collider.GetBody()) {
            body->AttachCollider(nullptr);
            collider.SetBody(nullptr);
        }

        m_colliders.erase(it);

        m_contacts.erase(
            std::remove_if(m_contacts.begin(), m_contacts.end(), [&collider](const Contact& contact) {
                return (contact.bodyA != nullptr && contact.bodyA->GetCollider() == &collider) ||
                    (contact.bodyB != nullptr && contact.bodyB->GetCollider() == &collider);
                }),
            m_contacts.end());

        return true;
    }

    bool PhysicsWorld2D::AttachCollider(Body2D& body, Collider2D& collider)
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

    void PhysicsWorld2D::DetachCollider(Body2D& body)
    {
        DetachBodyAndCollider(body);
    }

    void PhysicsWorld2D::Step(float dt)
    {
        if (dt <= 0.0f) {
            return;
        }

        IntegrateBodies(dt);
        DetectCollisions();
        ResolveContacts();
    }

    std::size_t PhysicsWorld2D::GetBodyCount() const noexcept
    {
        return m_bodies.size();
    }

    std::size_t PhysicsWorld2D::GetColliderCount() const noexcept
    {
        return m_colliders.size();
    }

    const std::vector<Contact>& PhysicsWorld2D::GetContacts() const noexcept
    {
        return m_contacts;
    }

    void PhysicsWorld2D::IntegrateBodies(float dt)
    {
        for (Body2D* body : m_bodies) {
            if (body == nullptr || body->GetBodyType() == BodyType2D::Static) {
                continue;
            }

            if (body->GetBodyType() == BodyType2D::Dynamic && body->IsGravityEnabled()) {
                body->SetVelocity(body->GetVelocity() + (m_settings.gravity * dt));
            }

            body->SetPosition(body->GetPosition() + (body->GetVelocity() * dt));

            if (m_settings.enableWorldBounds && body->IsBoundaryCheckEnabled()) {
                ApplyWorldBounds(*body);
            }
        }
    }

    void PhysicsWorld2D::ApplyWorldBounds(Body2D& body) const noexcept
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

    void PhysicsWorld2D::DetectCollisions()
    {
        m_contacts.clear();

        const float cellSize = m_settings.broadphaseCellSize;

        std::vector<BodyCollisionData> collisionBodies;
        collisionBodies.reserve(m_bodies.size());

        for (Body2D* body : m_bodies) {
            if (body == nullptr) {
                continue;
            }

            Collider2D* collider = body->GetCollider();
            if (collider == nullptr) {
                continue;
            }

            collisionBodies.push_back({ body, collider->ComputeAABB() });
        }

        std::unordered_map<CellCoord, std::vector<std::size_t>, CellCoordHasher> grid;
        grid.reserve(collisionBodies.size() * 2);

        for (std::size_t i = 0; i < collisionBodies.size(); ++i) {
            const AABB& aabb = collisionBodies[i].aabb;
            const int minX = ComputeCellIndex(aabb.min.x, cellSize);
            const int maxX = ComputeCellIndex(aabb.max.x, cellSize);
            const int minY = ComputeCellIndex(aabb.min.y, cellSize);
            const int maxY = ComputeCellIndex(aabb.max.y, cellSize);

            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    grid[{ x, y }].push_back(i);
                }
            }
        }

        std::unordered_set<std::uint64_t> checkedPairs;
        checkedPairs.reserve(collisionBodies.size() * 2);

        for (const auto& entry : grid) {
            const std::vector<std::size_t>& cellBodyIndices = entry.second;
            if (cellBodyIndices.size() < 2) {
                continue;
            }

            for (std::size_t i = 0; i < cellBodyIndices.size(); ++i) {
                const std::size_t collisionIndexA = cellBodyIndices[i];
                Body2D* bodyA = collisionBodies[collisionIndexA].body;
                if (bodyA == nullptr) {
                    continue;
                }

                for (std::size_t j = i + 1; j < cellBodyIndices.size(); ++j) {
                    const std::size_t collisionIndexB = cellBodyIndices[j];
                    Body2D* bodyB = collisionBodies[collisionIndexB].body;
                    if (bodyB == nullptr || bodyA == bodyB) {
                        continue;
                    }

                    const std::uint64_t pairKey = MakePairKey(collisionIndexA, collisionIndexB);
                    if (!checkedPairs.insert(pairKey).second) {
                        continue;
                    }

                    if (!collisionBodies[collisionIndexA].aabb.Intersects(collisionBodies[collisionIndexB].aabb)) {
                        continue;
                    }

                    m_contacts.push_back(BuildContact(*bodyA, *bodyB));
                }
            }
        }
    }

    void PhysicsWorld2D::ResolveContacts()
    {
        for (const Contact& contact : m_contacts) {
            Body2D* bodyA = contact.bodyA;
            Body2D* bodyB = contact.bodyB;
            if (bodyA == nullptr || bodyB == nullptr || contact.penetration <= 0.0f) {
                continue;
            }

            const bool moveA = bodyA->GetBodyType() == BodyType2D::Dynamic;
            const bool moveB = bodyB->GetBodyType() == BodyType2D::Dynamic;

            if (!moveA && !moveB) {
                continue;
            }

            const float invMassA = moveA ? ComputeInverseMass(*bodyA) : 0.0f;
            const float invMassB = moveB ? ComputeInverseMass(*bodyB) : 0.0f;
            const float invMassSum = invMassA + invMassB;
            if (invMassSum <= 0.0f) {
                continue;
            }

            const Vec2 correction = contact.normal * contact.penetration;
            if (moveA) {
                const float ratioA = invMassA / invMassSum;
                bodyA->SetPosition(bodyA->GetPosition() - (correction * ratioA));
            }
            if (moveB) {
                const float ratioB = invMassB / invMassSum;
                bodyB->SetPosition(bodyB->GetPosition() + (correction * ratioB));
            }

            if (moveA) {
                Vec2 velocityA = bodyA->GetVelocity();
                const float normalVelocityA = Dot(velocityA, contact.normal);
                if (normalVelocityA > 0.0f) {
                    velocityA -= contact.normal * normalVelocityA;
                    bodyA->SetVelocity(velocityA);
                }
            }
            if (moveB) {
                Vec2 velocityB = bodyB->GetVelocity();
                const float normalVelocityB = Dot(velocityB, contact.normal);
                if (normalVelocityB < 0.0f) {
                    velocityB -= contact.normal * normalVelocityB;
                    bodyB->SetVelocity(velocityB);
                }
            }
        }
    }

    Contact PhysicsWorld2D::BuildContact(Body2D& bodyA, Body2D& bodyB) const
    {
        Collider2D* colliderA = bodyA.GetCollider();
        Collider2D* colliderB = bodyB.GetCollider();

        Contact contact{};
        contact.bodyA = &bodyA;
        contact.bodyB = &bodyB;

        if (colliderA == nullptr || colliderB == nullptr) {
            return contact;
        }

        const AABB aabbA = colliderA->ComputeAABB();
        const AABB aabbB = colliderB->ComputeAABB();

        const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
        const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);

        if (overlapX <= 0.0f || overlapY <= 0.0f) {
            return contact;
        }

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