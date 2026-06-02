#include "PhysicsWorld.hpp"
#include "Physics2D.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace IndexPhys {
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

        float ComputeInverseMass(const Body& body) noexcept
        {
            const float mass = body.GetMass();
            if (mass <= std::numeric_limits<float>::epsilon()) {
                return 0.0f;
            }
            return 1.0f / mass;
        }

        void DetachBodyAndCollider(Body& body) noexcept
        {
            Collider* collider = body.GetCollider();
            if (collider == nullptr) {
                return;
            }
            collider->SetBody(nullptr);
            body.SetCollider(nullptr);
        }

        struct BodyCollisionData
        {
            Body* body = nullptr;
            Collider* collider = nullptr;
            AABB aabb{};
        };
    }

    PhysicsWorld::PhysicsWorld() = default;

    PhysicsWorld::PhysicsWorld(const WorldSettings& settings)
        : m_Settings(SanitizeSettings(settings))
    {}

    WorldSettings PhysicsWorld::SanitizeSettings(const WorldSettings& settings) noexcept
    {
        WorldSettings sanitized = settings;

        constexpr float kMinCellSize = 1e-3f;
        sanitized.broadphaseCellSize = std::max(sanitized.broadphaseCellSize, kMinCellSize);
        sanitized.solverIterations = std::max(1, sanitized.solverIterations);

        if (sanitized.worldMin.x > sanitized.worldMax.x) {
            std::swap(sanitized.worldMin.x, sanitized.worldMax.x);
        }
        if (sanitized.worldMin.y > sanitized.worldMax.y) {
            std::swap(sanitized.worldMin.y, sanitized.worldMax.y);
        }

        return sanitized;
    }

    void PhysicsWorld::SetSettings(const WorldSettings& settings) noexcept
    {
        m_Settings = SanitizeSettings(settings);
    }

    const WorldSettings& PhysicsWorld::GetSettings() const noexcept
    {
        return m_Settings;
    }

    const std::vector<Body*>& PhysicsWorld::GetBodies() const noexcept
    {
        return m_Bodies;
    }

    const std::vector<Collider*>& PhysicsWorld::GetColliders() const noexcept
    {
        return m_Colliders;
    }

    bool PhysicsWorld::RegisterBody(Body& body)
    {
        if (Contains(m_Bodies, body)) {
            return false;
        }
        m_Bodies.push_back(&body);
        return true;
    }

    bool PhysicsWorld::UnregisterBody(Body& body)
    {
        const auto it = std::find(m_Bodies.begin(), m_Bodies.end(), &body);
        if (it == m_Bodies.end()) {
            return false;
        }

        DetachBodyAndCollider(body);
        m_Bodies.erase(it);

        m_Contacts.erase(
            std::remove_if(m_Contacts.begin(), m_Contacts.end(), [&body](const Contact& contact) {
                return contact.bodyA == &body || contact.bodyB == &body;
                }),
            m_Contacts.end());

        return true;
    }

    bool PhysicsWorld::RegisterCollider(Collider& collider)
    {
        if (Contains(m_Colliders, collider)) {
            return false;
        }
        m_Colliders.push_back(&collider);
        return true;
    }

    bool PhysicsWorld::UnregisterCollider(Collider& collider)
    {
        const auto it = std::find(m_Colliders.begin(), m_Colliders.end(), &collider);
        if (it == m_Colliders.end()) {
            return false;
        }

        if (Body* body = collider.GetBody()) {
            body->SetCollider(nullptr);
            collider.SetBody(nullptr);
        }

        m_Colliders.erase(it);

        m_Contacts.erase(
            std::remove_if(m_Contacts.begin(), m_Contacts.end(), [&collider](const Contact& contact) {
                return contact.colliderA == &collider || contact.colliderB == &collider;
                }),
            m_Contacts.end());

        return true;
    }

    bool PhysicsWorld::AttachCollider(Body& body, Collider& collider)
    {
        if (!Contains(m_Bodies, body) || !Contains(m_Colliders, collider)) {
            return false;
        }

        // Sever any prior partners so we never leave half-attached state.
        if (Body* existing = collider.GetBody(); existing != nullptr && existing != &body) {
            existing->SetCollider(nullptr);
        }
        if (Collider* existing = body.GetCollider(); existing != nullptr && existing != &collider) {
            existing->SetBody(nullptr);
        }

        body.SetCollider(&collider);
        collider.SetBody(&body);
        return true;
    }

    void PhysicsWorld::DetachCollider(Body& body)
    {
        DetachBodyAndCollider(body);
    }

    void PhysicsWorld::Step(float dt)
    {
        if (dt <= 0.0f) {
            return;
        }

        IntegrateBodies(dt);
        DetectCollisions();
        ResolveContacts();
    }

    std::size_t PhysicsWorld::GetBodyCount() const noexcept
    {
        return m_Bodies.size();
    }

    std::size_t PhysicsWorld::GetColliderCount() const noexcept
    {
        return m_Colliders.size();
    }

    const std::vector<Contact>& PhysicsWorld::GetContacts() const noexcept
    {
        return m_Contacts;
    }

    void PhysicsWorld::IntegrateBodies(float dt)
    {
        for (Body* body : m_Bodies) {
            if (body == nullptr || body->GetBodyType() == BodyType::Static) {
                continue;
            }

            if (body->GetBodyType() == BodyType::Dynamic && body->IsGravityEnabled()) {
                body->SetVelocity(body->GetVelocity() + (m_Settings.gravity * dt));
            }

            body->SetPosition(body->GetPosition() + (body->GetVelocity() * dt));

            if (m_Settings.enableWorldBounds && body->IsBoundaryCheckEnabled()) {
                ApplyWorldBounds(*body);
            }
        }
    }

    void PhysicsWorld::ApplyWorldBounds(Body& body) const noexcept
    {
        Vec2 position = Clamp(body.GetPosition(), m_Settings.worldMin, m_Settings.worldMax);
        Vec2 velocity = body.GetVelocity();

        if (position.x == m_Settings.worldMin.x || position.x == m_Settings.worldMax.x) {
            velocity.x = 0.0f;
        }
        if (position.y == m_Settings.worldMin.y || position.y == m_Settings.worldMax.y) {
            velocity.y = 0.0f;
        }

        body.SetPosition(position);
        body.SetVelocity(velocity);
    }

    void PhysicsWorld::DetectCollisions()
    {
        m_Contacts.clear();

        const float cellSize = m_Settings.broadphaseCellSize;

        std::vector<BodyCollisionData> collisionBodies;
        collisionBodies.reserve(m_Bodies.size());

        for (Body* body : m_Bodies) {
            if (body == nullptr) {
                continue;
            }

            Collider* collider = body->GetCollider();
            if (collider == nullptr) {
                continue;
            }

            collisionBodies.push_back({ body, collider, collider->ComputeAABB() });
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
                const std::size_t indexA = cellBodyIndices[i];

                for (std::size_t j = i + 1; j < cellBodyIndices.size(); ++j) {
                    const std::size_t indexB = cellBodyIndices[j];

                    const std::uint64_t pairKey = MakePairKey(indexA, indexB);
                    if (!checkedPairs.insert(pairKey).second) {
                        continue;
                    }

                    if (!collisionBodies[indexA].aabb.Intersects(collisionBodies[indexB].aabb)) {
                        continue;
                    }

                    Collider* colliderA = collisionBodies[indexA].collider;
                    Collider* colliderB = collisionBodies[indexB].collider;
                    if (colliderA == nullptr || colliderB == nullptr) {
                        continue;
                    }

                    if (auto contact = Physics2D::OverlapsWith(*colliderA, *colliderB)) {
                        m_Contacts.push_back(*contact);
                    }
                }
            }
        }
    }

    void PhysicsWorld::ResolveContacts()
    {
        // Positional correction: applied once. Each contact's penetration is the
        // amount measured at the start of the frame; running this in a loop with
        // stale data would over-correct.
        for (const Contact& contact : m_Contacts) {
            Body* bodyA = contact.bodyA;
            Body* bodyB = contact.bodyB;
            if (bodyA == nullptr || bodyB == nullptr || contact.penetration <= 0.0f) {
                continue;
            }

            const bool moveA = bodyA->GetBodyType() == BodyType::Dynamic;
            const bool moveB = bodyB->GetBodyType() == BodyType::Dynamic;
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
                bodyA->SetPosition(bodyA->GetPosition() - (correction * (invMassA / invMassSum)));
            }
            if (moveB) {
                bodyB->SetPosition(bodyB->GetPosition() + (correction * (invMassB / invMassSum)));
            }
        }

        // Velocity solver: looped to converge in stacks. Each iteration applies
        // a normal impulse (with restitution) plus a tangential impulse clamped
        // by Coulomb friction.
        const int iterations = std::max(1, m_Settings.solverIterations);
        for (int it = 0; it < iterations; ++it) {
            for (const Contact& contact : m_Contacts) {
                Body* bodyA = contact.bodyA;
                Body* bodyB = contact.bodyB;
                if (bodyA == nullptr || bodyB == nullptr) {
                    continue;
                }

                const bool moveA = bodyA->GetBodyType() == BodyType::Dynamic;
                const bool moveB = bodyB->GetBodyType() == BodyType::Dynamic;
                if (!moveA && !moveB) {
                    continue;
                }

                const float invMassA = moveA ? ComputeInverseMass(*bodyA) : 0.0f;
                const float invMassB = moveB ? ComputeInverseMass(*bodyB) : 0.0f;
                const float invMassSum = invMassA + invMassB;
                if (invMassSum <= 0.0f) {
                    continue;
                }

                const Vec2 relativeVelocity = bodyB->GetVelocity() - bodyA->GetVelocity();
                const float velocityAlongNormal = Dot(relativeVelocity, contact.normal);
                if (velocityAlongNormal >= 0.0f) {
                    continue;
                }

                // Normal impulse with restitution.
                const float restitution = std::min(bodyA->GetRestitution(), bodyB->GetRestitution());
                const float jn = -(1.0f + restitution) * velocityAlongNormal / invMassSum;
                const Vec2 normalImpulse = contact.normal * jn;
                if (moveA) bodyA->SetVelocity(bodyA->GetVelocity() - (normalImpulse * invMassA));
                if (moveB) bodyB->SetVelocity(bodyB->GetVelocity() + (normalImpulse * invMassB));

                // Coulomb friction along the contact tangent.
                const Vec2 postRelative = bodyB->GetVelocity() - bodyA->GetVelocity();
                Vec2 tangent = postRelative - contact.normal * Dot(postRelative, contact.normal);
                const float tangentLenSq = LengthSq(tangent);
                if (tangentLenSq <= 1e-10f) {
                    continue;
                }
                tangent = tangent / std::sqrt(tangentLenSq);

                const float jtRaw = -Dot(postRelative, tangent) / invMassSum;
                const float mu = std::sqrt(bodyA->GetFriction() * bodyB->GetFriction());
                const float jtMax = mu * jn;
                const float jt = std::clamp(jtRaw, -jtMax, jtMax);
                const Vec2 frictionImpulse = tangent * jt;
                if (moveA) bodyA->SetVelocity(bodyA->GetVelocity() - (frictionImpulse * invMassA));
                if (moveB) bodyB->SetVelocity(bodyB->GetVelocity() + (frictionImpulse * invMassB));
            }
        }
    }
}
