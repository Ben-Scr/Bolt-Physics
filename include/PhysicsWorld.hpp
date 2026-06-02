#pragma once
#include "Export.hpp"
#include "WorldSettings.hpp"
#include "Body.hpp"
#include "Collider.hpp"
#include "Contact.hpp"

#include <vector>

namespace IndexPhys {
    class INDEX_PHYS_API PhysicsWorld
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

        // Pairs a registered body with a registered collider, keeping both
        // back-pointers in sync. If the body or collider is currently attached
        // to a different partner, the previous pairing is severed first.
        bool AttachCollider(Body& body, Collider& collider);
        void DetachCollider(Body& body);

        void Step(float dt);

        std::size_t GetBodyCount() const noexcept;
        std::size_t GetColliderCount() const noexcept;
        const std::vector<Body*>& GetBodies() const noexcept;
        const std::vector<Collider*>& GetColliders() const noexcept;
        const std::vector<Contact>& GetContacts() const noexcept;

    private:
        static WorldSettings SanitizeSettings(const WorldSettings& settings) noexcept;
        void IntegrateBodies(float dt);
        void ApplyWorldBounds(Body& body) const noexcept;
        void DetectCollisions();
        void ResolveContacts();

        WorldSettings m_Settings;
        std::vector<Body*> m_Bodies;
        std::vector<Collider*> m_Colliders;
        std::vector<Contact> m_Contacts;
    };
}
