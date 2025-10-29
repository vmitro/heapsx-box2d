package heapsx.box2d;

import h3d.Vector;
import heapsx.box2d.ll.Box2D;
import heapsx.box2d.ll.Types;
import heapsx.box2d.ll.HLBytes;

/**
 * High-level Box2D rigid body.
 * Bodies are the primary objects in the physics simulation.
 *
 * Example:
 * ```haxe
 * var ball = world.createBody(BodyType.Dynamic, new Vector(0, 10));
 * ball.createCircleShape(1.0); // Add a circle shape
 * ball.applyLinearImpulse(new Vector(5, 0), ball.getPosition());
 * ```
 */
@:forward(bodyId)
abstract Body(BodyData) from BodyData to BodyData {
    /**
     * Create a body in the specified world.
     * Typically called via world.createBody() instead of directly.
     *
     * @param world World to create body in
     * @param type Body type (Static, Kinematic, or Dynamic)
     * @param position Initial position
     * @param angle Initial rotation angle in radians
     */
    public static function create(world: World, type: BodyType, position: Vector, angle: Float): Body {
        var bodyId = Box2D.body_create(world.worldId, type, position.x, position.y, angle);
        return new Body({bodyId: bodyId});
    }

    inline function new(data: BodyData) {
        this = data;
    }

    // === Transform ===

    /**
     * Set body transform (position and rotation).
     * @param position New position
     * @param angle New rotation angle in radians
     */
    public inline function setTransform(position: Vector, angle: Float): Void {
        Box2D.body_set_transform(this.bodyId, position.x, position.y, angle);
    }

    /**
     * Get body position.
     */
    public function getPosition(): Vector {
        var bytes = HLBytes.alloc(16);  // 2 doubles = 16 bytes
        Box2D.body_get_position(this.bodyId, bytes.raw);
        return new Vector(bytes.getF64(0), bytes.getF64(8));  // Read as F64 at offsets 0 and 8
    }

    /**
     * Get body rotation angle in radians.
     */
    public inline function getAngle(): Float {
        return Box2D.body_get_angle(this.bodyId);
    }

    // === Velocity ===

    /**
     * Set linear velocity.
     * @param velocity Velocity vector
     */
    public inline function setLinearVelocity(velocity: Vector): Void {
        Box2D.body_set_linear_velocity(this.bodyId, velocity.x, velocity.y);
    }

    /**
     * Get linear velocity.
     */
    public function getLinearVelocity(): Vector {
        var bytes = HLBytes.alloc(16);  // 2 doubles = 16 bytes
        Box2D.body_get_linear_velocity(this.bodyId, bytes.raw);
        return new Vector(bytes.getF64(0), bytes.getF64(8));
    }

    /**
     * Set angular velocity (radians per second).
     */
    public inline function setAngularVelocity(omega: Float): Void {
        Box2D.body_set_angular_velocity(this.bodyId, omega);
    }

    /**
     * Get angular velocity (radians per second).
     */
    public inline function getAngularVelocity(): Float {
        return Box2D.body_get_angular_velocity(this.bodyId);
    }

    // === Forces and Impulses ===

    /**
     * Apply a force at a world point.
     * Forces accumulate and are applied continuously until cleared.
     *
     * @param force Force vector
     * @param point Application point in world coordinates
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyForce(force: Vector, point: Vector, wake: Bool = true): Void {
        Box2D.body_apply_force(this.bodyId, force.x, force.y, point.x, point.y, wake);
    }

    /**
     * Apply a force at the body's center of mass.
     * @param force Force vector
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyForceToCenter(force: Vector, wake: Bool = true): Void {
        Box2D.body_apply_force_to_center(this.bodyId, force.x, force.y, wake);
    }

    /**
     * Apply a torque (rotational force).
     * @param torque Torque magnitude
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyTorque(torque: Float, wake: Bool = true): Void {
        Box2D.body_apply_torque(this.bodyId, torque, wake);
    }

    /**
     * Apply a linear impulse at a world point.
     * Impulses cause instant velocity changes.
     *
     * @param impulse Impulse vector
     * @param point Application point in world coordinates
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyLinearImpulse(impulse: Vector, point: Vector, wake: Bool = true): Void {
        Box2D.body_apply_linear_impulse(this.bodyId, impulse.x, impulse.y, point.x, point.y, wake);
    }

    /**
     * Apply a linear impulse at the body's center of mass.
     * @param impulse Impulse vector
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyLinearImpulseToCenter(impulse: Vector, wake: Bool = true): Void {
        Box2D.body_apply_linear_impulse_to_center(this.bodyId, impulse.x, impulse.y, wake);
    }

    /**
     * Apply an angular impulse (instant angular velocity change).
     * @param impulse Angular impulse magnitude
     * @param wake Wake the body if sleeping (default true)
     */
    public inline function applyAngularImpulse(impulse: Float, wake: Bool = true): Void {
        Box2D.body_apply_angular_impulse(this.bodyId, impulse, wake);
    }

    // === Mass Properties ===

    /**
     * Get the body's total mass in kilograms.
     */
    public inline function getMass(): Float {
        return Box2D.body_get_mass(this.bodyId);
    }

    /**
     * Get the body's rotational inertia.
     */
    public inline function getInertia(): Float {
        return Box2D.body_get_inertia(this.bodyId);
    }

    /**
     * Get the local center of mass.
     */
    public function getLocalCenterOfMass(): Vector {
        var bytes = HLBytes.alloc(16);  // 2 doubles = 16 bytes
        Box2D.body_get_local_center_of_mass(this.bodyId, bytes.raw);
        return new Vector(bytes.getF64(0), bytes.getF64(8));
    }

    // === State ===

    /**
     * Set whether the body is awake or sleeping.
     * Sleeping bodies skip simulation for better performance.
     *
     * @param flag True to wake the body
     */
    public inline function setAwake(flag: Bool): Void {
        Box2D.body_set_awake(this.bodyId, flag);
    }

    /**
     * Check if the body is awake.
     */
    public inline function isAwake(): Bool {
        return Box2D.body_is_awake(this.bodyId);
    }

    /**
     * Enable/disable the body.
     * Disabled bodies don't participate in simulation.
     *
     * @param flag True to enable
     */
    public inline function setEnabled(flag: Bool): Void {
        Box2D.body_set_enabled(this.bodyId, flag);
    }

    /**
     * Check if the body is enabled.
     */
    public inline function isEnabled(): Bool {
        return Box2D.body_is_enabled(this.bodyId);
    }

    // === Properties ===

    /**
     * Set linear damping (velocity reduction over time, 0-1 typical).
     * Higher values cause faster slowdown.
     */
    public inline function setLinearDamping(damping: Float): Void {
        Box2D.body_set_linear_damping(this.bodyId, damping);
    }

    /**
     * Get linear damping.
     */
    public inline function getLinearDamping(): Float {
        return Box2D.body_get_linear_damping(this.bodyId);
    }

    /**
     * Set angular damping (angular velocity reduction over time, 0-1 typical).
     */
    public inline function setAngularDamping(damping: Float): Void {
        Box2D.body_set_angular_damping(this.bodyId, damping);
    }

    /**
     * Get angular damping.
     */
    public inline function getAngularDamping(): Float {
        return Box2D.body_get_angular_damping(this.bodyId);
    }

    /**
     * Set gravity scale (multiplier for world gravity).
     * Set to 0 to disable gravity for this body.
     */
    public inline function setGravityScale(scale: Float): Void {
        Box2D.body_set_gravity_scale(this.bodyId, scale);
    }

    /**
     * Get gravity scale.
     */
    public inline function getGravityScale(): Float {
        return Box2D.body_get_gravity_scale(this.bodyId);
    }

    // === Shape Creation Helpers ===

    /**
     * Create a circle shape attached to this body.
     * @param radius Circle radius
     * @param center Local center offset (default origin)
     * @return The created shape
     */
    public function createCircleShape(radius: Float, center: Vector = null): Shape {
        var c = center != null ? center : new Vector(0, 0);
        return Shape.createCircle(this, radius, c);
    }

    /**
     * Create a box shape attached to this body.
     * @param halfWidth Half-width (box will be 2*halfWidth wide)
     * @param halfHeight Half-height (box will be 2*halfHeight tall)
     * @return The created shape
     */
    public function createBoxShape(halfWidth: Float, halfHeight: Float): Shape {
        return Shape.createBox(this, halfWidth, halfHeight);
    }

    /**
     * Create a polygon shape from vertices.
     * @param vertices Array of vertices (3-8 vertices required)
     * @return The created shape
     */
    public function createPolygonShape(vertices: Array<Vector>): Shape {
        return Shape.createPolygon(this, vertices);
    }

    /**
     * Destroy the body and all attached shapes.
     * Call this when you're done with the body to free resources.
     */
    public inline function dispose(): Void {
        Box2D.body_destroy(this.bodyId);
    }
}

private typedef BodyData = {
    bodyId: BodyId
}
