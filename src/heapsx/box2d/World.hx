package heapsx.box2d;

import h3d.Vector;
import heapsx.box2d.ll.Box2D;
import heapsx.box2d.ll.Types;
import heapsx.box2d.ll.HLBytes;

/**
 * High-level Box2D physics world.
 * Manages the physics simulation and all bodies/joints within it.
 *
 * Example:
 * ```haxe
 * var world = new World(new Vector(0, -10)); // Gravity pointing down
 * world.step(1/60); // Advance simulation by one frame
 * world.dispose(); // Clean up when done
 * ```
 */
@:forward(worldId)
abstract World(WorldData) from WorldData to WorldData {
    /**
     * Create a new physics world.
     * @param gravity Gravity vector (typically new Vector(0, -10) for downward)
     */
    public inline function new(gravity: Vector) {
        Box2D.init(); // Ensure module is initialized
        this = {
            worldId: Box2D.world_create(gravity.x, gravity.y)
        };
    }

    /**
     * Step the physics simulation forward in time.
     * Call this once per frame with your fixed timestep.
     *
     * @param timeStep Time to advance in seconds (typically 1/60 for 60 FPS)
     * @param subStepCount Number of sub-steps for accuracy (4-8 recommended, default 4)
     */
    public inline function step(timeStep: Float, subStepCount: Int = 4): Void {
        Box2D.world_step(this.worldId, timeStep, subStepCount);
    }

    /**
     * Set world gravity.
     * @param gravity New gravity vector
     */
    public inline function setGravity(gravity: Vector): Void {
        Box2D.world_set_gravity(this.worldId, gravity.x, gravity.y);
    }

    /**
     * Get world gravity.
     * @return Gravity vector
     */
    public function getGravity(): Vector {
        var bytes = HLBytes.alloc(16);  // 2 doubles = 16 bytes
        Box2D.world_get_gravity(this.worldId, bytes.raw);
        return new Vector(bytes.getF64(0), bytes.getF64(8));
    }

    /**
     * Enable/disable automatic sleeping of idle bodies.
     * Sleeping improves performance by skipping simulation of resting bodies.
     *
     * @param flag True to enable sleeping (default in Box2D)
     */
    public inline function enableSleeping(flag: Bool): Void {
        Box2D.world_enable_sleeping(this.worldId, flag);
    }

    /**
     * Enable/disable continuous collision detection.
     * CCD prevents fast-moving bodies from tunneling through thin obstacles.
     *
     * @param flag True to enable CCD
     */
    public inline function enableContinuous(flag: Bool): Void {
        Box2D.world_enable_continuous(this.worldId, flag);
    }

    /**
     * Enable/disable warm starting.
     * Warm starting improves solver convergence by reusing contact data from previous frames.
     *
     * @param flag True to enable warm starting (default in Box2D)
     */
    public inline function enableWarmStarting(flag: Bool): Void {
        Box2D.world_enable_warm_starting(this.worldId, flag);
    }

    /**
     * Get the number of bodies in the world.
     */
    public inline function getBodyCount(): Int {
        return Box2D.world_get_body_count(this.worldId);
    }

    /**
     * Get the number of active contacts in the world.
     */
    public inline function getContactCount(): Int {
        return Box2D.world_get_contact_count(this.worldId);
    }

    /**
     * Create a new body in this world.
     *
     * @param type Body type (Static, Kinematic, or Dynamic)
     * @param position Initial position
     * @param angle Initial rotation angle in radians (default 0)
     * @return The created body
     */
    public function createBody(type: BodyType, position: Vector, angle: Float = 0): Body {
        return Body.create(this, type, position, angle);
    }

    /**
     * Destroy the world and all bodies/joints within it.
     * Call this when you're done with the simulation to free resources.
     */
    public inline function dispose(): Void {
        Box2D.world_destroy(this.worldId);
    }
}

private typedef WorldData = {
    worldId: WorldId
}
