#include "Body.hpp"
#include "Collider.hpp"

#include <algorithm>

namespace IndexPhys {
    Body::Body() noexcept
    {
        SetBodyType(BodyType::Dynamic);
    }

    Body::Body(BodyType type) noexcept
    {
        SetBodyType(type);
    }

    void Body::Destroy() noexcept
    {
        if (m_Collider != nullptr) {
            m_Collider->SetBody(nullptr);
            m_Collider = nullptr;
        }
    }

    BodyType Body::GetBodyType() const noexcept
    {
        return m_BodyType;
    }

    void Body::SetBodyType(BodyType type) noexcept
    {
        m_BodyType = type;

        switch (m_BodyType) {
        case BodyType::Static:
            SetGravityEnabled(false);
            SetBoundaryCheckEnabled(false);
            SetVelocity({ 0.0f, 0.0f });
            break;
        case BodyType::Kinematic:
            SetGravityEnabled(false);
            SetBoundaryCheckEnabled(true);
            break;
        case BodyType::Dynamic:
            SetGravityEnabled(true);
            SetBoundaryCheckEnabled(true);
            break;
        }
    }

    const Vec2& Body::GetPosition() const noexcept
    {
        return m_Position;
    }

    void Body::SetPosition(const Vec2& p) noexcept
    {
        m_Position = p;
    }

    const Vec2& Body::GetVelocity() const noexcept
    {
        return m_Velocity;
    }

    void Body::SetVelocity(const Vec2& v) noexcept
    {
        m_Velocity = v;
    }

    float Body::GetMass() const noexcept
    {
        return m_Mass;
    }

    void Body::SetMass(float mass) noexcept
    {
        m_Mass = mass > 0.0f ? mass : 1.0f;
    }

    float Body::GetRestitution() const noexcept
    {
        return m_Restitution;
    }

    void Body::SetRestitution(float restitution) noexcept
    {
        m_Restitution = std::clamp(restitution, 0.0f, 1.0f);
    }

    float Body::GetFriction() const noexcept
    {
        return m_Friction;
    }

    void Body::SetFriction(float friction) noexcept
    {
        m_Friction = std::clamp(friction, 0.0f, 1.0f);
    }

    bool Body::IsBoundaryCheckEnabled() const noexcept
    {
        return m_BoundaryCheckEnabled;
    }

    void Body::SetBoundaryCheckEnabled(bool enabled) noexcept
    {
        m_BoundaryCheckEnabled = enabled;
    }

    bool Body::IsGravityEnabled() const noexcept
    {
        return m_GravityEnabled;
    }

    void Body::SetGravityEnabled(bool enabled) noexcept
    {
        m_GravityEnabled = enabled;
    }

    Collider* Body::GetCollider() noexcept
    {
        return m_Collider;
    }

    const Collider* Body::GetCollider() const noexcept
    {
        return m_Collider;
    }

    void Body::SetCollider(Collider* collider) noexcept
    {
        m_Collider = collider;
    }
}
