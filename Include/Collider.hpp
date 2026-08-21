#pragma once
#include "Export.hpp"
#include "ColliderType.hpp"
#include "AABB.hpp"

namespace IndexPhys {
    class Body;

    class INDEX_PHYS_API Collider
    {
    public:
        virtual ~Collider() = default;

        virtual void Destroy() noexcept;

        ColliderType GetType() const noexcept;

        Body* GetBody() noexcept;
        const Body* GetBody() const noexcept;

        void SetBody(Body* body) noexcept;

        // Opaque, pointer-sized user value so a query hit can be mapped back to the
        // owning game object. Store a direct entity pointer (fastest - no lookup) or
        // an integer id via reinterpret_cast. nullptr = unset.
        void  SetUserData(void* userData) noexcept;
        void* GetUserData() const noexcept;

        virtual AABB ComputeAABB() const noexcept = 0;

    protected:
        explicit Collider(ColliderType type) noexcept;

    private:
        ColliderType m_Type;
        Body* m_Body = nullptr;
        void* m_UserData = nullptr;
    };
}
