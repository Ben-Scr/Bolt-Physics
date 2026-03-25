# Bolt-Physics

Bolt-Physics2D is a minimalist 2D physics module for C++ with a clear separation between `Body2D`, colliders, and a registration-based `PhysicsWorld`.

## Build target
Bolt-Physics2D is configured as a **static library** in all Visual Studio configurations (`Debug/Release` and `Win32/x64`).

- Project output type: `StaticLibrary`
- Preprocessor define: `BT_STATIC`
- API macro behavior: `BOLT_PHYS_API` becomes empty for static builds

## Architecture
- **Body**: A single `Body2D` class with `BodyType` (`Static`, `Dynamic`, `Kinematic`).
- **Colliders**: `BoxCollider2D`, `CircleCollider`, and `PolygonCollider2D` derive from `Collider2D`.
- **World**: `PhysicsWorld` stores global settings, registration logic, simulation step, and contacts.
- **Utility Queries**: `Physics2D` provides overlap and point queries with and without a world context.

## Behavior
- Only bodies and colliders registered in `PhysicsWorld` are simulated.
- Gravity affects only bodies of type `BodyType::Dynamic` (if gravity is enabled for that body).
- World bounds can be enabled globally, while boundary checks can be toggled per body.
- `SetBodyType` applies sensible defaults:
  - `Static`: gravity off, boundary checks off, velocity reset to zero
  - `Kinematic`: gravity off, boundary checks on
  - `Dynamic`: gravity on, boundary checks on

## Example
```cpp
#include "PhysicsWorld.hpp"
#include "Body2D.hpp"
#include "BoxCollider2D.hpp"

int main()
{
    BoltPhys::PhysicsWorld world;

    BoltPhys::Body2D player;
    player.SetBodyType(BoltPhys::BodyType::Dynamic);
    player.SetPosition({ 0.0f, 2.0f });

    BoltPhys::BoxCollider2D playerCollider({ 0.5f, 1.0f });

    world.RegisterBody(player);
    world.RegisterCollider(playerCollider);
    world.AttachCollider(player, playerCollider);

    world.Step(1.0f / 60.0f);
    return 0;
}
```
