#pragma once
#include "Export.hpp"
#include "ColliderType.hpp"
#include "AABB.hpp"

namespace BoltPhys {
    class Body;

    class BOLT_PHYS_API Collider
    {
    public:
        virtual ~Collider() = default;

        virtual void Destroy() noexcept;

        ColliderType GetType() const noexcept;

        Body* GetBody() noexcept;
        const Body* GetBody() const noexcept;

        void SetBody(Body* body) noexcept;

        virtual AABB ComputeAABB() const noexcept = 0;

    protected:
        explicit Collider(ColliderType type) noexcept;

    private:
        ColliderType m_type;
        Body* m_body = nullptr;
    };
}
