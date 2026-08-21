#include "Collider.hpp"
#include "Body.hpp"

namespace IndexPhys {
    Collider::Collider(ColliderType type) noexcept
        : m_Type(type)
    {}

    void Collider::Destroy() noexcept
    {
        if (m_Body != nullptr) {
            m_Body->SetCollider(nullptr);
            m_Body = nullptr;
        }
    }

    ColliderType Collider::GetType() const noexcept
    {
        return m_Type;
    }

    Body* Collider::GetBody() noexcept
    {
        return m_Body;
    }

    const Body* Collider::GetBody() const noexcept
    {
        return m_Body;
    }

    void Collider::SetBody(Body* body) noexcept
    {
        m_Body = body;
    }

    void Collider::SetUserData(void* userData) noexcept
    {
        m_UserData = userData;
    }

    void* Collider::GetUserData() const noexcept
    {
        return m_UserData;
    }
}