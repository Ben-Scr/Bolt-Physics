#pragma once
#include "Export.hpp"
#include "WorldSettings.hpp"
#include "Body.hpp"

#include <cstddef>
#include <vector>

namespace BoltPhys {
    class BOLT_PHYS_API PhysicsWorld
    {
    public:
        PhysicsWorld();
        explicit PhysicsWorld(const WorldSettings& settings);

        void SetSettings(const WorldSettings& settings) noexcept;
        const WorldSettings& GetSettings() const noexcept;

        bool RegisterBody(Body& body);
        bool UnregisterBody(Body& body);

        bool RegisterCollider(Collider& collider);
        bool UnregisterCollider(Collider& collider);

        bool AttachCollider(Body& body, Collider& collider);
        void DetachCollider(Body& body);

        void Step(float dt);

        std::size_t GetBodyCount() const noexcept;
        std::size_t GetColliderCount() const noexcept;

    private:
        WorldSettings m_settings;
        std::vector<Body*> m_bodies;
        std::vector<Collider*> m_colliders;
    };
}