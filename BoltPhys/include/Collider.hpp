#pragma once
#include "Export.hpp"
#include "Body.hpp"
#include  "ColliderType.hpp"
#include "AABB.hpp"

namespace BoltPhys {
    class BOLT_PHYS_API Collider
    {
    public:
        virtual ~Collider() = default;

        ColliderType GetType() const noexcept;

        Body* GetBody() noexcept;
        const Body* GetBody() const noexcept;
        void SetBody(Body* body) noexcept;

        virtual AABB ComputeAABB() const = 0;

    protected:
        explicit Collider(ColliderType type);

    private:
        ColliderType m_type;
        Body* m_body = nullptr;
    };
}