package heapsx.box2d;

import h3d.Vector;
import heapsx.box2d.ll.Box2D;
import heapsx.box2d.ll.Types;
import heapsx.box2d.ll.HLBytes;

/**
 * High-level Box2D collision shape.
 * Shapes define collision geometry and material properties.
 *
 * Example:
 * ```haxe
 * var shape = body.createCircleShape(1.0);
 * shape.setDensity(2.0); // Heavy object
 * shape.setFriction(0.3);
 * shape.setRestitution(0.8); // Bouncy
 * ```
 */
@:forward(shapeId)
abstract Shape(ShapeData) from ShapeData to ShapeData {
    /**
     * Create a circle shape attached to a body.
     * @param body Body to attach to
     * @param radius Circle radius
     * @param center Local center offset
     */
    public static function createCircle(body: Body, radius: Float, center: Vector): Shape {
        var shapeId = Box2D.shape_create_circle(body.bodyId, radius, center.x, center.y);
        return new Shape({shapeId: shapeId});
    }

    /**
     * Create a capsule (pill) shape attached to a body.
     * @param body Body to attach to
     * @param center1 First end center
     * @param center2 Second end center
     * @param radius Capsule radius
     */
    public static function createCapsule(body: Body, center1: Vector, center2: Vector, radius: Float): Shape {
        var shapeId = Box2D.shape_create_capsule(body.bodyId, center1.x, center1.y, center2.x, center2.y, radius);
        return new Shape({shapeId: shapeId});
    }

    /**
     * Create a box shape attached to a body.
     * @param body Body to attach to
     * @param halfWidth Half-width (box will be 2*halfWidth wide)
     * @param halfHeight Half-height (box will be 2*halfHeight tall)
     */
    public static function createBox(body: Body, halfWidth: Float, halfHeight: Float): Shape {
        var shapeId = Box2D.shape_create_box(body.bodyId, halfWidth, halfHeight);
        return new Shape({shapeId: shapeId});
    }

    /**
     * Create a convex polygon shape from vertices.
     * @param body Body to attach to
     * @param vertices Array of vertices (must have 3-8 vertices)
     */
    public static function createPolygon(body: Body, vertices: Array<Vector>): Shape {
        if (vertices.length < 3 || vertices.length > 8) {
            throw 'Polygon must have 3-8 vertices, got ${vertices.length}';
        }

        // Marshal vertices to flat float array
        var bytes = HLBytes.alloc(vertices.length * 8); // 2 floats per vertex
        for (i in 0...vertices.length) {
            bytes.setF32(i * 8, vertices[i].x);
            bytes.setF32(i * 8 + 4, vertices[i].y);
        }

        var shapeId = Box2D.shape_create_polygon(body.bodyId, bytes.raw, vertices.length);
        return new Shape({shapeId: shapeId});
    }

    /**
     * Create a line segment shape.
     * @param body Body to attach to
     * @param point1 First endpoint
     * @param point2 Second endpoint
     */
    public static function createSegment(body: Body, point1: Vector, point2: Vector): Shape {
        var shapeId = Box2D.shape_create_segment(body.bodyId, point1.x, point1.y, point2.x, point2.y);
        return new Shape({shapeId: shapeId});
    }

    inline function new(data: ShapeData) {
        this = data;
    }

    // === Material Properties ===

    /**
     * Set shape density (mass = density * area).
     * Default density is 1.0. Higher values make objects heavier.
     * Must be >= 0.
     */
    public inline function setDensity(density: Float): Void {
        Box2D.shape_set_density(this.shapeId, density);
    }

    /**
     * Get shape density.
     */
    public inline function getDensity(): Float {
        return Box2D.shape_get_density(this.shapeId);
    }

    /**
     * Set friction coefficient.
     * 0 = frictionless (ice), 1 = high friction (rubber).
     * Typical values: 0.2-0.5
     */
    public inline function setFriction(friction: Float): Void {
        Box2D.shape_set_friction(this.shapeId, friction);
    }

    /**
     * Get friction coefficient.
     */
    public inline function getFriction(): Float {
        return Box2D.shape_get_friction(this.shapeId);
    }

    /**
     * Set restitution (bounciness).
     * 0 = no bounce (clay), 1 = perfect bounce (ideal rubber ball).
     * Values > 1 will add energy to the system.
     */
    public inline function setRestitution(restitution: Float): Void {
        Box2D.shape_set_restitution(this.shapeId, restitution);
    }

    /**
     * Get restitution.
     */
    public inline function getRestitution(): Float {
        return Box2D.shape_get_restitution(this.shapeId);
    }

    // === Sensor ===

    /**
     * Set whether the shape is a sensor.
     * Sensors detect collisions but don't create physical responses.
     * Useful for triggers and detection zones.
     *
     * @param flag True to make this a sensor
     */
    public inline function setSensor(flag: Bool): Void {
        Box2D.shape_set_sensor(this.shapeId, flag);
    }

    /**
     * Check if the shape is a sensor.
     */
    public inline function isSensor(): Bool {
        return Box2D.shape_is_sensor(this.shapeId);
    }

    // === Collision Filtering ===

    /**
     * Set collision filter category bits.
     * This shape belongs to these categories.
     * Use powers of 2: 0x0001, 0x0002, 0x0004, etc.
     */
    public inline function setFilterCategory(categoryBits: Int): Void {
        Box2D.shape_set_filter_category(this.shapeId, categoryBits);
    }

    /**
     * Set collision filter mask bits.
     * This shape collides with these categories.
     * Use powers of 2: 0x0001, 0x0002, 0x0004, etc.
     */
    public inline function setFilterMask(maskBits: Int): Void {
        Box2D.shape_set_filter_mask(this.shapeId, maskBits);
    }

    /**
     * Set collision filter group index.
     * Shapes with the same positive group always collide.
     * Shapes with the same negative group never collide.
     * Zero means no group filtering.
     */
    public inline function setFilterGroup(groupIndex: Int): Void {
        Box2D.shape_set_filter_group(this.shapeId, groupIndex);
    }

    /**
     * Destroy the shape.
     * Call this when you're done with the shape to free resources.
     */
    public inline function dispose(): Void {
        Box2D.shape_destroy(this.shapeId);
    }
}

private typedef ShapeData = {
    shapeId: ShapeId
}
