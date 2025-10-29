package heapsx.box2d;

import h3d.Vector;
import heapsx.box2d.ll.Box2D;
import heapsx.box2d.ll.Types;

/**
 * High-level Box2D joint.
 * Joints constrain the relative motion between two bodies.
 *
 * Example:
 * ```haxe
 * var joint = Joint.createRevolute(bodyA, bodyB, new Vector(0, 0));
 * joint.enableMotor(true);
 * joint.setMotorSpeed(5.0);
 * ```
 */
@:forward(jointId)
abstract Joint(JointData) from JointData to JointData {
    /**
     * Create a distance joint.
     * Maintains a fixed distance between two points on two bodies.
     *
     * @param bodyA First body
     * @param bodyB Second body
     * @param anchorA Anchor point on bodyA (local coordinates)
     * @param anchorB Anchor point on bodyB (local coordinates)
     */
    public static function createDistance(bodyA: Body, bodyB: Body, anchorA: Vector, anchorB: Vector): Joint {
        var jointId = Box2D.joint_create_distance(bodyA.bodyId, bodyB.bodyId, anchorA.x, anchorA.y, anchorB.x, anchorB.y);
        return new Joint({jointId: jointId});
    }

    /**
     * Create a revolute joint (hinge).
     * Allows rotation around a shared anchor point.
     *
     * @param bodyA First body
     * @param bodyB Second body
     * @param anchor Shared anchor point (local coordinates for both bodies)
     */
    public static function createRevolute(bodyA: Body, bodyB: Body, anchor: Vector): Joint {
        var jointId = Box2D.joint_create_revolute(bodyA.bodyId, bodyB.bodyId, anchor.x, anchor.y);
        return new Joint({jointId: jointId});
    }

    /**
     * Create a prismatic joint (slider).
     * Allows linear motion along an axis.
     *
     * @param bodyA First body
     * @param bodyB Second body
     * @param anchor Anchor point (local coordinates)
     * @param axis Slide axis (local coordinates, will be normalized)
     */
    public static function createPrismatic(bodyA: Body, bodyB: Body, anchor: Vector, axis: Vector): Joint {
        var jointId = Box2D.joint_create_prismatic(bodyA.bodyId, bodyB.bodyId, anchor.x, anchor.y, axis.x, axis.y);
        return new Joint({jointId: jointId});
    }

    /**
     * Create a weld joint.
     * Rigidly attaches two bodies together (no relative motion).
     *
     * @param bodyA First body
     * @param bodyB Second body
     * @param anchor Anchor point (local coordinates)
     */
    public static function createWeld(bodyA: Body, bodyB: Body, anchor: Vector): Joint {
        var jointId = Box2D.joint_create_weld(bodyA.bodyId, bodyB.bodyId, anchor.x, anchor.y);
        return new Joint({jointId: jointId});
    }

    /**
     * Create a motor joint.
     * Drives the relative position and rotation between two bodies.
     *
     * @param bodyA First body
     * @param bodyB Second body
     */
    public static function createMotor(bodyA: Body, bodyB: Body): Joint {
        var jointId = Box2D.joint_create_motor(bodyA.bodyId, bodyB.bodyId);
        return new Joint({jointId: jointId});
    }

    /**
     * Create a wheel joint (suspension).
     * Allows rotation and translation along an axis, ideal for vehicle wheels.
     *
     * @param bodyA First body (typically the vehicle chassis)
     * @param bodyB Second body (typically the wheel)
     * @param anchor Anchor point (local coordinates)
     * @param axis Suspension axis (local coordinates, will be normalized)
     */
    public static function createWheel(bodyA: Body, bodyB: Body, anchor: Vector, axis: Vector): Joint {
        var jointId = Box2D.joint_create_wheel(bodyA.bodyId, bodyB.bodyId, anchor.x, anchor.y, axis.x, axis.y);
        return new Joint({jointId: jointId});
    }

    inline function new(data: JointData) {
        this = data;
    }

    // === Motor Control ===

    /**
     * Set motor speed (for revolute, prismatic, wheel joints).
     * @param speed Target speed in radians/second (revolute) or meters/second (prismatic, wheel)
     */
    public inline function setMotorSpeed(speed: Float): Void {
        Box2D.joint_set_motor_speed(this.jointId, speed);
    }

    /**
     * Set maximum motor torque (for revolute joints).
     * @param torque Maximum torque in Newton-meters
     */
    public inline function setMaxMotorTorque(torque: Float): Void {
        Box2D.joint_set_max_motor_torque(this.jointId, torque);
    }

    /**
     * Enable/disable the motor.
     * @param flag True to enable motor
     */
    public inline function enableMotor(flag: Bool): Void {
        Box2D.joint_enable_motor(this.jointId, flag);
    }

    // === Limits ===

    /**
     * Enable/disable joint limits (for revolute, prismatic joints).
     * @param flag True to enable limits
     */
    public inline function enableLimit(flag: Bool): Void {
        Box2D.joint_enable_limit(this.jointId, flag);
    }

    /**
     * Set joint limits (for revolute, prismatic joints).
     * @param lower Lower limit (radians for revolute, meters for prismatic)
     * @param upper Upper limit (radians for revolute, meters for prismatic)
     */
    public inline function setLimits(lower: Float, upper: Float): Void {
        Box2D.joint_set_limits(this.jointId, lower, upper);
    }

    /**
     * Destroy the joint.
     * Call this when you're done with the joint to free resources.
     */
    public inline function dispose(): Void {
        Box2D.joint_destroy(this.jointId);
    }
}

private typedef JointData = {
    jointId: JointId
}
