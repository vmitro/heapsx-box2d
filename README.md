# HeapsX Box2D - 2D Physics Engine for Heaps.io

Box2D bindings for HashLink/Heaps.io following the HeapsX native module architecture.

## Overview

Box2D is a feature-rich 2D rigid body physics engine written in portable C17. This module provides complete Haxe bindings with seamless Heaps.io integration.

## Features

- **World Management**: Create physics worlds with customizable gravity and simulation parameters
- **Rigid Bodies**: Static, kinematic, and dynamic bodies with full transform control
- **Shapes**: Circles, boxes, polygons, capsules, segments, and chains
- **Joints**: Distance, revolute, prismatic, motor, wheel, and weld joints
- **Forces & Impulses**: Apply forces, torques, and impulses to bodies
- **Collision Detection**: Ray casting, shape casting, and AABB queries
- **Events**: Contact callbacks, sensor triggers, and joint events
- **Performance**: SIMD-accelerated solver with multithreading support

## Architecture

This module follows the HeapsX 4-layer architecture:

1. **Native C Layer**: Box2D v3 C17 library
2. **C++ Wrapper Layer**: Modular wrappers (`module_world`, `module_body`, etc.)
3. **HashLink Bridge**: HL_PRIM bindings in `box2d_bindings.cpp`
4. **Low-Level Haxe**: `@:hlNative` declarations in `ll/Box2D.hx`
5. **High-Level Haxe**: Type-safe abstracts with `h3d.Vector` integration

## Quick Start

```haxe
import heapsx.box2d.*;
import h3d.Vector;

class Main extends hxd.App {
    var world: World;
    var ball: Body;

    override function init() {
        super.init();

        // Create world with gravity
        world = new World(new Vector(0, -10));

        // Create ground (static body)
        var ground = world.createBody(BodyType.Static, new Vector(0, -5));
        ground.createBoxShape(50, 1); // 100x2 box

        // Create falling ball
        ball = world.createBody(BodyType.Dynamic, new Vector(0, 10));
        ball.createCircleShape(1.0);
        ball.applyLinearImpulse(new Vector(5, 0), ball.getPosition());
    }

    override function update(dt: Float) {
        super.update(dt);

        // Step physics simulation
        world.step(dt);

        // Get updated position
        var pos = ball.getPosition();
        trace('Ball at: ${pos.x}, ${pos.y}');
    }
}
```

## API Reference

### World

```haxe
var world = new World(gravity: Vector);
world.step(timeStep: Float, velocityIterations: Int = 8, positionIterations: Int = 3);
world.setGravity(gravity: Vector);
world.getGravity(): Vector;
world.createBody(type: BodyType, position: Vector, angle: Float = 0): Body;
world.raycast(from: Vector, to: Vector): RaycastResult;
world.dispose();
```

### Body

```haxe
body.setTransform(position: Vector, angle: Float);
body.getPosition(): Vector;
body.getAngle(): Float;
body.setLinearVelocity(velocity: Vector);
body.getLinearVelocity(): Vector;
body.setAngularVelocity(omega: Float);
body.applyForce(force: Vector, point: Vector, wake: Bool = true);
body.applyLinearImpulse(impulse: Vector, point: Vector, wake: Bool = true);
body.applyAngularImpulse(impulse: Float, wake: Bool = true);
body.getMass(): Float;
body.createCircleShape(radius: Float, center: Vector = null): Shape;
body.createBoxShape(halfWidth: Float, halfHeight: Float): Shape;
body.createPolygonShape(vertices: Array<Vector>): Shape;
body.dispose();
```

### Shape

```haxe
Shape.createCircle(body: Body, radius: Float, center: Vector): Shape;
Shape.createBox(body: Body, halfWidth: Float, halfHeight: Float): Shape;
Shape.createPolygon(body: Body, vertices: Array<Vector>): Shape;
shape.setDensity(density: Float);
shape.setFriction(friction: Float);
shape.setRestitution(restitution: Float);
shape.dispose();
```

### Joint

```haxe
Joint.createDistance(bodyA: Body, bodyB: Body, anchorA: Vector, anchorB: Vector): Joint;
Joint.createRevolute(bodyA: Body, bodyB: Body, anchor: Vector): Joint;
Joint.createPrismatic(bodyA: Body, bodyB: Body, anchor: Vector, axis: Vector): Joint;
joint.setMotorSpeed(speed: Float);
joint.setMaxMotorForce(force: Float);
joint.dispose();
```

## Body Types

```haxe
BodyType.Static      // Zero mass, zero velocity, manually moved
BodyType.Kinematic   // Zero mass, non-zero velocity, driven by user
BodyType.Dynamic     // Positive mass, non-zero velocity, driven by forces
```

## Building

The module uses CMake and is auto-discovered by the HeapsX build system:

```bash
# Build for Android
./gradlew assembleDebug

# Build for Desktop
cd app/src/main/haxe
haxe desktop.hxml
```

## Examples

### Stacking Boxes

```haxe
var world = new World(new Vector(0, -10));

// Ground
var ground = world.createBody(BodyType.Static, new Vector(0, 0));
ground.createBoxShape(10, 0.5);

// Stack boxes
for (i in 0...10) {
    var box = world.createBody(BodyType.Dynamic, new Vector(0, 1 + i * 2));
    box.createBoxShape(0.5, 0.5);
}
```

### Distance Joint

```haxe
var bodyA = world.createBody(BodyType.Dynamic, new Vector(-5, 5));
bodyA.createCircleShape(0.5);

var bodyB = world.createBody(BodyType.Dynamic, new Vector(5, 5));
bodyB.createCircleShape(0.5);

// Connect with distance joint
var joint = Joint.createDistance(bodyA, bodyB,
    new Vector(-5, 5), new Vector(5, 5));
```

### Raycast

```haxe
var hit = world.raycast(new Vector(0, 10), new Vector(0, -10));
if (hit.hit) {
    trace('Hit body at ${hit.point}');
}
```

## Performance Tips

1. **Reuse bodies**: Use `body.setActive(false)` instead of destroying/recreating
2. **Adjust iteration counts**: Lower values = faster but less accurate
3. **Use sleeping**: Bodies automatically sleep when at rest
4. **Collision filtering**: Use collision groups to reduce unnecessary checks
5. **Batch operations**: Create many bodies before first step

## License

MIT License - See LICENSE file for details

Box2D itself is licensed under the MIT License.

## Credits

- **Box2D**: Created by Erin Catto (https://github.com/erincatto/box2d)
- **HeapsX Bindings**: Part of the HeapsX native modules ecosystem
