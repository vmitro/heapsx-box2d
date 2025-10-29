package heapsx.box2d.ll;

/**
 * Body types in Box2D physics simulation
 */
enum abstract BodyType(Int) to Int {
    /**
     * Static bodies have zero velocity and are not affected by forces.
     * Ideal for walls, floors, and immovable terrain.
     */
    var Static = 0;

    /**
     * Kinematic bodies move at a set velocity but are not affected by forces.
     * Ideal for moving platforms and doors.
     */
    var Kinematic = 1;

    /**
     * Dynamic bodies have mass and are fully simulated by physics.
     * Ideal for characters, projectiles, and interactive objects.
     */
    var Dynamic = 2;
}

/**
 * World ID - Opaque handle to a Box2D world (stored as Int/uint32)
 */
typedef WorldId = Int;

/**
 * Body ID - Opaque handle to a Box2D body (stored as Int64/uint64)
 */
typedef BodyId = hl.I64;

/**
 * Shape ID - Opaque handle to a Box2D shape (stored as Int64/uint64)
 */
typedef ShapeId = hl.I64;

/**
 * Joint ID - Opaque handle to a Box2D joint (stored as Int64/uint64)
 */
typedef JointId = hl.I64;
