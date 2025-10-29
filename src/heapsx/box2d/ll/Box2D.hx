package heapsx.box2d.ll;

import heapsx.box2d.ll.Types;

/**
 * Low-level Box2D bindings for HashLink.
 * These are direct @:hlNative declarations matching the C++ bindings.
 *
 * For most use cases, use the high-level API (heapsx.box2d.World, Body, Shape, Joint)
 * instead of calling these functions directly.
 */
@:hlNative("box2d")
extern class Box2D {
    // === Initialization ===

    /**
     * Initialize the Box2D module. Must be called before using any Box2D functions.
     */
    public static function init(): Void;

    // === World Management ===

    /**
     * Create a new physics world with the specified gravity.
     * @param gx Gravity X component (typically 0)
     * @param gy Gravity Y component (typically -10 for downward gravity)
     * @return World ID handle
     */
    public static function world_create(gx: Float, gy: Float): WorldId;

    /**
     * Destroy a physics world and all bodies/joints within it.
     * @param worldId World to destroy
     */
    public static function world_destroy(worldId: WorldId): Void;

    /**
     * Step the physics simulation forward in time.
     * @param worldId World to simulate
     * @param timeStep Time to advance (in seconds, typically 1/60)
     * @param subStepCount Number of sub-steps for accuracy (4-8 recommended)
     */
    public static function world_step(worldId: WorldId, timeStep: Float, subStepCount: Int): Void;

    /**
     * Set world gravity.
     * @param worldId World to modify
     * @param x Gravity X component
     * @param y Gravity Y component
     */
    public static function world_set_gravity(worldId: WorldId, x: Float, y: Float): Void;

    /**
     * Get world gravity.
     * @param worldId World to query
     * @param outBytes Buffer to store gravity (8 bytes: x, y as floats)
     */
    public static function world_get_gravity(worldId: WorldId, outBytes: hl.Bytes): Void;

    /**
     * Enable/disable automatic sleeping of idle bodies.
     * @param worldId World to modify
     * @param flag True to enable sleeping
     */
    public static function world_enable_sleeping(worldId: WorldId, flag: Bool): Void;

    /**
     * Enable/disable continuous collision detection.
     * @param worldId World to modify
     * @param flag True to enable CCD
     */
    public static function world_enable_continuous(worldId: WorldId, flag: Bool): Void;

    /**
     * Enable/disable warm starting (improves solver convergence).
     * @param worldId World to modify
     * @param flag True to enable warm starting
     */
    public static function world_enable_warm_starting(worldId: WorldId, flag: Bool): Void;

    /**
     * Get the number of bodies in the world.
     */
    public static function world_get_body_count(worldId: WorldId): Int;

    /**
     * Get the number of contacts in the world.
     */
    public static function world_get_contact_count(worldId: WorldId): Int;

    // === Body Management ===

    /**
     * Create a rigid body in the world.
     * @param worldId World to create body in
     * @param type Body type (Static=0, Kinematic=1, Dynamic=2)
     * @param x Initial X position
     * @param y Initial Y position
     * @param angle Initial rotation angle (radians)
     * @return Body ID handle
     */
    public static function body_create(worldId: WorldId, type: Int, x: Float, y: Float, angle: Float): BodyId;

    /**
     * Destroy a body and all attached shapes.
     * @param bodyId Body to destroy
     */
    public static function body_destroy(bodyId: BodyId): Void;

    /**
     * Set body transform (position and rotation).
     * @param bodyId Body to modify
     * @param x New X position
     * @param y New Y position
     * @param angle New rotation angle (radians)
     */
    public static function body_set_transform(bodyId: BodyId, x: Float, y: Float, angle: Float): Void;

    /**
     * Get body position.
     * @param bodyId Body to query
     * @param outBytes Buffer to store position (8 bytes: x, y as floats)
     */
    public static function body_get_position(bodyId: BodyId, outBytes: hl.Bytes): Void;

    /**
     * Get body rotation angle in radians.
     */
    public static function body_get_angle(bodyId: BodyId): Float;

    /**
     * Get body transform (position and rotation).
     * @param bodyId Body to query
     * @param outBytes Buffer to store transform (12 bytes: x, y, angle as floats)
     */
    public static function body_get_transform(bodyId: BodyId, outBytes: hl.Bytes): Void;

    /**
     * Set linear velocity of the body.
     * @param bodyId Body to modify
     * @param vx X velocity component
     * @param vy Y velocity component
     */
    public static function body_set_linear_velocity(bodyId: BodyId, vx: Float, vy: Float): Void;

    /**
     * Get linear velocity of the body.
     * @param bodyId Body to query
     * @param outBytes Buffer to store velocity (8 bytes: vx, vy as floats)
     */
    public static function body_get_linear_velocity(bodyId: BodyId, outBytes: hl.Bytes): Void;

    /**
     * Set angular velocity of the body (radians per second).
     */
    public static function body_set_angular_velocity(bodyId: BodyId, omega: Float): Void;

    /**
     * Get angular velocity of the body (radians per second).
     */
    public static function body_get_angular_velocity(bodyId: BodyId): Float;

    /**
     * Apply a force at a world point.
     * @param bodyId Body to apply force to
     * @param fx Force X component
     * @param fy Force Y component
     * @param px Application point X (world coordinates)
     * @param py Application point Y (world coordinates)
     * @param wake Wake the body if sleeping
     */
    public static function body_apply_force(bodyId: BodyId, fx: Float, fy: Float, px: Float, py: Float, wake: Bool): Void;

    /**
     * Apply a force at the body's center of mass.
     */
    public static function body_apply_force_to_center(bodyId: BodyId, fx: Float, fy: Float, wake: Bool): Void;

    /**
     * Apply a torque (rotational force).
     */
    public static function body_apply_torque(bodyId: BodyId, torque: Float, wake: Bool): Void;

    /**
     * Apply a linear impulse at a world point (instant velocity change).
     */
    public static function body_apply_linear_impulse(bodyId: BodyId, ix: Float, iy: Float, px: Float, py: Float, wake: Bool): Void;

    /**
     * Apply a linear impulse at the body's center of mass.
     */
    public static function body_apply_linear_impulse_to_center(bodyId: BodyId, ix: Float, iy: Float, wake: Bool): Void;

    /**
     * Apply an angular impulse (instant angular velocity change).
     */
    public static function body_apply_angular_impulse(bodyId: BodyId, impulse: Float, wake: Bool): Void;

    /**
     * Get the body's mass in kilograms.
     */
    public static function body_get_mass(bodyId: BodyId): Float;

    /**
     * Get the body's rotational inertia.
     */
    public static function body_get_inertia(bodyId: BodyId): Float;

    /**
     * Get the local center of mass.
     * @param bodyId Body to query
     * @param outBytes Buffer to store center (8 bytes: x, y as floats)
     */
    public static function body_get_local_center_of_mass(bodyId: BodyId, outBytes: hl.Bytes): Void;

    /**
     * Set whether the body is awake or sleeping.
     */
    public static function body_set_awake(bodyId: BodyId, flag: Bool): Void;

    /**
     * Check if the body is awake.
     */
    public static function body_is_awake(bodyId: BodyId): Bool;

    /**
     * Enable/disable the body (disabled bodies don't participate in simulation).
     */
    public static function body_set_enabled(bodyId: BodyId, flag: Bool): Void;

    /**
     * Check if the body is enabled.
     */
    public static function body_is_enabled(bodyId: BodyId): Bool;

    /**
     * Set linear damping (velocity reduction over time, 0-1 typical).
     */
    public static function body_set_linear_damping(bodyId: BodyId, damping: Float): Void;

    /**
     * Get linear damping.
     */
    public static function body_get_linear_damping(bodyId: BodyId): Float;

    /**
     * Set angular damping (angular velocity reduction over time, 0-1 typical).
     */
    public static function body_set_angular_damping(bodyId: BodyId, damping: Float): Void;

    /**
     * Get angular damping.
     */
    public static function body_get_angular_damping(bodyId: BodyId): Float;

    /**
     * Set gravity scale (multiplier for world gravity, 0 to disable).
     */
    public static function body_set_gravity_scale(bodyId: BodyId, scale: Float): Void;

    /**
     * Get gravity scale.
     */
    public static function body_get_gravity_scale(bodyId: BodyId): Float;

    // === Shape Management ===

    /**
     * Create a circle shape attached to a body.
     * @param bodyId Body to attach shape to
     * @param radius Circle radius
     * @param centerX Local center X offset
     * @param centerY Local center Y offset
     * @return Shape ID handle
     */
    public static function shape_create_circle(bodyId: BodyId, radius: Float, centerX: Float, centerY: Float): ShapeId;

    /**
     * Create a capsule shape (pill shape) attached to a body.
     */
    public static function shape_create_capsule(bodyId: BodyId, c1x: Float, c1y: Float, c2x: Float, c2y: Float, radius: Float): ShapeId;

    /**
     * Create a box shape attached to a body.
     * @param bodyId Body to attach shape to
     * @param hx Half-width (box will be 2*hx wide)
     * @param hy Half-height (box will be 2*hy tall)
     */
    public static function shape_create_box(bodyId: BodyId, hx: Float, hy: Float): ShapeId;

    /**
     * Create a convex polygon shape from vertices.
     * @param bodyId Body to attach shape to
     * @param vertices Flat array of vertex coordinates [x1, y1, x2, y2, ...]
     * @param vertexCount Number of vertices (must be 3-8)
     */
    public static function shape_create_polygon(bodyId: BodyId, vertices: hl.Bytes, vertexCount: Int): ShapeId;

    /**
     * Create a line segment shape.
     */
    public static function shape_create_segment(bodyId: BodyId, p1x: Float, p1y: Float, p2x: Float, p2y: Float): ShapeId;

    /**
     * Destroy a shape.
     */
    public static function shape_destroy(shapeId: ShapeId): Void;

    /**
     * Set shape density (mass = density * area).
     */
    public static function shape_set_density(shapeId: ShapeId, density: Float): Void;

    /**
     * Get shape density.
     */
    public static function shape_get_density(shapeId: ShapeId): Float;

    /**
     * Set shape friction coefficient (0 = frictionless, 1 = high friction).
     */
    public static function shape_set_friction(shapeId: ShapeId, friction: Float): Void;

    /**
     * Get shape friction.
     */
    public static function shape_get_friction(shapeId: ShapeId): Float;

    /**
     * Set shape restitution/bounciness (0 = no bounce, 1 = perfect bounce).
     */
    public static function shape_set_restitution(shapeId: ShapeId, restitution: Float): Void;

    /**
     * Get shape restitution.
     */
    public static function shape_get_restitution(shapeId: ShapeId): Float;

    /**
     * Set whether the shape is a sensor (detects collisions but doesn't respond).
     */
    public static function shape_set_sensor(shapeId: ShapeId, flag: Bool): Void;

    /**
     * Check if the shape is a sensor.
     */
    public static function shape_is_sensor(shapeId: ShapeId): Bool;

    /**
     * Set collision filter category bits.
     */
    public static function shape_set_filter_category(shapeId: ShapeId, categoryBits: Int): Void;

    /**
     * Set collision filter mask bits.
     */
    public static function shape_set_filter_mask(shapeId: ShapeId, maskBits: Int): Void;

    /**
     * Set collision filter group index.
     */
    public static function shape_set_filter_group(shapeId: ShapeId, groupIndex: Int): Void;

    // === Joint Management ===

    /**
     * Create a distance joint (maintains fixed distance between two bodies).
     */
    public static function joint_create_distance(bodyA: BodyId, bodyB: BodyId, anchorAx: Float, anchorAy: Float, anchorBx: Float, anchorBy: Float): JointId;

    /**
     * Create a revolute joint (hinge joint, allows rotation).
     */
    public static function joint_create_revolute(bodyA: BodyId, bodyB: BodyId, anchorX: Float, anchorY: Float): JointId;

    /**
     * Create a prismatic joint (slider joint, allows linear motion along an axis).
     */
    public static function joint_create_prismatic(bodyA: BodyId, bodyB: BodyId, anchorX: Float, anchorY: Float, axisX: Float, axisY: Float): JointId;

    /**
     * Create a weld joint (rigid attachment, no relative motion).
     */
    public static function joint_create_weld(bodyA: BodyId, bodyB: BodyId, anchorX: Float, anchorY: Float): JointId;

    /**
     * Create a motor joint (drives relative position/rotation).
     */
    public static function joint_create_motor(bodyA: BodyId, bodyB: BodyId): JointId;

    /**
     * Create a wheel joint (suspension joint for vehicles).
     */
    public static function joint_create_wheel(bodyA: BodyId, bodyB: BodyId, anchorX: Float, anchorY: Float, axisX: Float, axisY: Float): JointId;

    /**
     * Destroy a joint.
     */
    public static function joint_destroy(jointId: JointId): Void;

    /**
     * Set motor speed (for revolute, prismatic, wheel joints).
     */
    public static function joint_set_motor_speed(jointId: JointId, speed: Float): Void;

    /**
     * Set maximum motor torque (for revolute joints).
     */
    public static function joint_set_max_motor_torque(jointId: JointId, torque: Float): Void;

    /**
     * Enable/disable motor.
     */
    public static function joint_enable_motor(jointId: JointId, flag: Bool): Void;

    /**
     * Enable/disable joint limits.
     */
    public static function joint_enable_limit(jointId: JointId, flag: Bool): Void;

    /**
     * Set joint limits (for revolute, prismatic joints).
     */
    public static function joint_set_limits(jointId: JointId, lower: Float, upper: Float): Void;
}
