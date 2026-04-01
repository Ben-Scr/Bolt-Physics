#include "Collider2D.hpp"
#include "Body2D.hpp"

namespace BoltPhys {
    Collider::Collider(ColliderType type) noexcept
        : m_type(type)
    {}

    void Collider::Destroy() noexcept
    {
        if (m_body != nullptr) {
            m_body->AttachCollider(nullptr);
            m_body = nullptr;
        }
    }

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
}