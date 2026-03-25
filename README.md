# BoltPhys

BoltPhys is a minimalist 2D physics module for C++ with a clear separation between bodies, colliders, and a registration-based `PhysicsWorld`.

## Build target
BoltPhys is configured as a **static library** in all Visual Studio configurations (`Debug/Release` and `Win32/x64`).

- Project output type: `StaticLibrary`
- Preprocessor define: `BT_STATIC`
- API macro behavior: `BOLT_PHYS_API` becomes empty for static builds

If you embed BoltPhys directly into BoltEngine, this setup avoids DLL deployment and keeps integration simple.

## Architecture
- **Bodies**: `StaticBody`, `DynamicBody`, and `KinematicBody` derive from `Body`.
- **Colliders**: `BoxCollider`, `CircleCollider`, and `PolygonCollider` derive from `Collider`.
- **World**: `PhysicsWorld` stores global settings, registration logic, the simulation step, and the contact list.
- **Utility Queries**: `Physics2D` provides overlap and point queries with and without a world context.
- **Contacts**: Collisions are reported as simple `Contact` entries for overlapping AABBs.

## Behavior
- Only bodies and colliders registered in `PhysicsWorld` are simulated.
- Gravity affects only `DynamicBody` instances with gravity enabled.
- World bounds can be enabled globally, while boundary checks can be toggled on or off per body.
- `StaticBody` disables gravity and boundary checks by default.
- There is intentionally **no rotation** and **no bounce/restitution physics**.
- Overlaps are resolved with a simple penetration correction for dynamic bodies.

## Example
```cpp
#include "PhysicsWorld.hpp"
#include "DynamicBody.hpp"
#include "BoxCollider.hpp"
#include "Physics2D.hpp"
