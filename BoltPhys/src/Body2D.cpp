#include "Body2D.hpp"
#include "Collider2D.hpp"

namespace BoltPhys {
    Body2D::Body2D() noexcept
    {
        SetBodyType(BodyType::Dynamic);
    }

    Body2D::Body2D(BodyType type) noexcept
    {
        SetBodyType(type); 
    }

    Body2D::~Body2D()
    {
        if (m_collider != nullptr) {
            m_collider->SetBody(nullptr);
            m_collider = nullptr;
        }
    }

    BodyType Body2D::GetBodyType() const noexcept
    {
        return m_bodyType;
    }

    void Body2D::SetBodyType(BodyType type) noexcept
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

    const Vec2& Body2D::GetPosition() const noexcept
    {
        return m_position;
    }

    void Body2D::SetPosition(const Vec2& p) noexcept
    {
        m_position = p;
    }

    const Vec2& Body2D::GetVelocity() const noexcept
    {
        return m_velocity;
    }

    void Body2D::SetVelocity(const Vec2& v) noexcept
    {
        m_velocity = v;
    }

    float Body2D::GetMass() const noexcept
    {
        return m_mass;
    }

    void Body2D::SetMass(float mass) noexcept
    {
        m_mass = mass > 0.0f ? mass : 1.0f;
    }

    bool Body2D::IsBoundaryCheckEnabled() const noexcept
    {
        return m_boundaryCheckEnabled;
    }

    void Body2D::SetBoundaryCheckEnabled(bool enabled) noexcept
    {
        m_boundaryCheckEnabled = enabled;
    }

    bool Body2D::IsGravityEnabled() const noexcept
    {
        return m_gravityEnabled;
    }

    void Body2D::SetGravityEnabled(bool enabled) noexcept
    {
        m_gravityEnabled = enabled;
    }

    Collider2D* Body2D::GetCollider() noexcept
    {
        return m_collider;
    }

    const Collider2D* Body2D::GetCollider() const noexcept
    {
        return m_collider;
    }

    void Body2D::AttachCollider(Collider2D* collider) noexcept
    {
        m_collider = collider;
    }
}