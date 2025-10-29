// Box2D Shape Creation Wrapper Implementation
// Part of HeapsX Box2D bindings

#include "module_shape.h"
#include "box2d/box2d.h"
#include <vector>

#ifdef ANDROID
    #include <android/log.h>
    #define LOG_TAG "heapsx-box2d"
    #define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
    #define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
    #include <cstdio>
    #define LOGD(...) fprintf(stdout, "[heapsx-box2d] " __VA_ARGS__), fprintf(stdout, "\n")
    #define LOGE(...) fprintf(stderr, "[heapsx-box2d ERROR] " __VA_ARGS__), fprintf(stderr, "\n")
#endif

namespace module_shape {

void init() {
    LOGD("Box2D Shape module initialized");
}

uint64_t create_circle(uint64_t body_id, double radius, double center_x, double center_y) {
    if (body_id == 0) {
        LOGE("Cannot create shape on null body");
        return 0;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    b2Circle circle;
    circle.center = (b2Vec2){(float)center_x, (float)center_y};
    circle.radius = radius;

    b2ShapeId shapeId = b2CreateCircleShape(bodyId, &shapeDef, &circle);

    if (B2_IS_NULL(shapeId)) {
        LOGE("Failed to create circle shape");
        return 0;
    }

    uint64_t stored_id = b2StoreShapeId(shapeId);
    LOGD("Created circle shape (id: %llu) radius=%.2f", (unsigned long long)stored_id, radius);
    return stored_id;
}

uint64_t create_capsule(uint64_t body_id, double center1_x, double center1_y, double center2_x, double center2_y, double radius) {
    if (body_id == 0) {
        LOGE("Cannot create shape on null body");
        return 0;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    b2Capsule capsule;
    capsule.center1 = (b2Vec2){(float)center1_x, (float)center1_y};
    capsule.center2 = (b2Vec2){(float)center2_x, (float)center2_y};
    capsule.radius = radius;

    b2ShapeId shapeId = b2CreateCapsuleShape(bodyId, &shapeDef, &capsule);

    if (B2_IS_NULL(shapeId)) {
        LOGE("Failed to create capsule shape");
        return 0;
    }

    uint64_t stored_id = b2StoreShapeId(shapeId);
    LOGD("Created capsule shape (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_box(uint64_t body_id, double hx, double hy) {
    if (body_id == 0) {
        LOGE("Cannot create shape on null body");
        return 0;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    LOGD("create_box called with hx=%f, hy=%f", hx, hy);
    b2Polygon box = b2MakeBox((float)hx, (float)hy);

    b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);

    if (B2_IS_NULL(shapeId)) {
        LOGE("Failed to create box shape");
        return 0;
    }

    uint64_t stored_id = b2StoreShapeId(shapeId);
    LOGD("Created box shape (id: %llu) size=(%.2f, %.2f)", (unsigned long long)stored_id, hx * 2, hy * 2);
    return stored_id;
}

uint64_t create_polygon(uint64_t body_id, const float* vertices, int vertex_count) {
    if (body_id == 0) {
        LOGE("Cannot create shape on null body");
        return 0;
    }

    if (vertex_count < 3 || vertex_count > 8) {
        LOGE("Polygon vertex count must be between 3 and 8, got %d", vertex_count);
        return 0;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    // Convert flat array to b2Vec2 array
    std::vector<b2Vec2> points(vertex_count);
    for (int i = 0; i < vertex_count; ++i) {
        points[i].x = vertices[i * 2];
        points[i].y = vertices[i * 2 + 1];
    }

    b2Hull hull = b2ComputeHull(points.data(), vertex_count);
    b2Polygon polygon = b2MakePolygon(&hull, 0.0f);

    b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &polygon);

    if (B2_IS_NULL(shapeId)) {
        LOGE("Failed to create polygon shape");
        return 0;
    }

    uint64_t stored_id = b2StoreShapeId(shapeId);
    LOGD("Created polygon shape (id: %llu) vertices=%d", (unsigned long long)stored_id, vertex_count);
    return stored_id;
}

uint64_t create_segment(uint64_t body_id, float p1_x, float p1_y, float p2_x, float p2_y) {
    if (body_id == 0) {
        LOGE("Cannot create shape on null body");
        return 0;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2ShapeDef shapeDef = b2DefaultShapeDef();

    b2Segment segment;
    segment.point1 = (b2Vec2){(float)p1_x, (float)p1_y};
    segment.point2 = (b2Vec2){(float)p2_x, (float)p2_y};

    b2ShapeId shapeId = b2CreateSegmentShape(bodyId, &shapeDef, &segment);

    if (B2_IS_NULL(shapeId)) {
        LOGE("Failed to create segment shape");
        return 0;
    }

    uint64_t stored_id = b2StoreShapeId(shapeId);
    LOGD("Created segment shape (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

void destroy_shape(uint64_t shape_id) {
    if (shape_id == 0) {
        LOGE("Cannot destroy null shape");
        return;
    }

    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2DestroyShape(shapeId, true);
    LOGD("Destroyed shape (id: %llu)", (unsigned long long)shape_id);
}

void set_density(uint64_t shape_id, double density) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Shape_SetDensity(shapeId, density, true); // updateBodyMass = true
}

double get_density(uint64_t shape_id) {
    if (shape_id == 0) return 0.0f;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    return b2Shape_GetDensity(shapeId);
}

void set_friction(uint64_t shape_id, double friction) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Shape_SetFriction(shapeId, friction);
}

double get_friction(uint64_t shape_id) {
    if (shape_id == 0) return 0.0f;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    return b2Shape_GetFriction(shapeId);
}

void set_restitution(uint64_t shape_id, double restitution) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Shape_SetRestitution(shapeId, restitution);
}

double get_restitution(uint64_t shape_id) {
    if (shape_id == 0) return 0.0f;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    return b2Shape_GetRestitution(shapeId);
}

void set_sensor(uint64_t shape_id, bool flag) {
    if (shape_id == 0) return;
    // NOTE: In Box2D v3.1, sensor status is set at shape creation and cannot be changed
    // This function is a no-op for API compatibility
    LOGE("set_sensor() is not supported in Box2D v3.1 - sensor status is set at shape creation");
}

bool is_sensor(uint64_t shape_id) {
    if (shape_id == 0) return false;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    return b2Shape_IsSensor(shapeId);
}

void set_filter_category(uint64_t shape_id, uint32_t category_bits) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Filter filter = b2Shape_GetFilter(shapeId);
    filter.categoryBits = category_bits;
    b2Shape_SetFilter(shapeId, filter);
}

void set_filter_mask(uint64_t shape_id, uint32_t mask_bits) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Filter filter = b2Shape_GetFilter(shapeId);
    filter.maskBits = mask_bits;
    b2Shape_SetFilter(shapeId, filter);
}

void set_filter_group(uint64_t shape_id, int32_t group_index) {
    if (shape_id == 0) return;
    b2ShapeId shapeId = b2LoadShapeId(shape_id);
    b2Filter filter = b2Shape_GetFilter(shapeId);
    filter.groupIndex = group_index;
    b2Shape_SetFilter(shapeId, filter);
}

} // namespace module_shape
