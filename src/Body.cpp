#include "Body2D.hpp"
#include "Collider2D.hpp"

namespace BoltPhys {
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
        if (m_collider != nullptr) {
            m_collider->SetBody(nullptr);
            m_collider = nullptr;
        }
    }

    BodyType Body::GetBodyType() const noexcept
    {
        return m_bodyType;
    }

    void Body::SetBodyType(BodyType type) noexcept
    {
        m_bodyType = type;

        switch (m_bodyType) {
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
        default:
            break;
        }
    }

    const Vec2& Body::GetPosition() const noexcept
    {
        return m_position;
    }

    void Body::SetPosition(const Vec2& p) noexcept
    {
        m_position = p;
    }

    const Vec2& Body::GetVelocity() const noexcept
    {
        return m_velocity;
    }

    void Body::SetVelocity(const Vec2& v) noexcept
    {
        m_velocity = v;
    }

    float Body::GetMass() const noexcept
    {
        return m_mass;
    }

    void Body::SetMass(float mass) noexcept
    {
        m_mass = mass > 0.0f ? mass : 1.0f;
    }

    bool Body::IsBoundaryCheckEnabled() const noexcept
    {
        return m_boundaryCheckEnabled;
    }

    void Body::SetBoundaryCheckEnabled(bool enabled) noexcept
    {
        m_boundaryCheckEnabled = enabled;
    }

    bool Body::IsGravityEnabled() const noexcept
    {
        return m_gravityEnabled;
    }

    void Body::SetGravityEnabled(bool enabled) noexcept
    {
        m_gravityEnabled = enabled;
    }

    Collider* Body::GetCollider() noexcept
    {
        return m_collider;
    }

    const Collider* Body::GetCollider() const noexcept
    {
        return m_collider;
    }

    void Body::AttachCollider(Collider* collider) noexcept
    {
        m_collider = collider;
    }
}