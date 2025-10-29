// Box2D Body Management Wrapper Implementation
// Part of HeapsX Box2D bindings

#include "module_body.h"
#include "box2d/box2d.h"

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

namespace module_body {

void init() {
    LOGD("Box2D Body module initialized");
}

uint64_t create_body(uint32_t world_id, int type, double x, double y, double angle) {
    if (world_id == 0) {
        LOGE("Cannot create body in null world");
        return 0;
    }

    b2WorldId worldId = b2LoadWorldId(world_id);
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = (b2BodyType)type;
    bodyDef.position = (b2Vec2){(float)x, (float)y};
    bodyDef.rotation = b2MakeRot(angle);

    b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

    if (B2_IS_NULL(bodyId)) {
        LOGE("Failed to create body");
        return 0;
    }

    uint64_t stored_id = b2StoreBodyId(bodyId);
    LOGD("Created body (id: %llu) type=%d at (%.2f, %.2f)",
         (unsigned long long)stored_id, type, x, y);
    return stored_id;
}

void destroy_body(uint64_t body_id) {
    if (body_id == 0) {
        LOGE("Cannot destroy null body");
        return;
    }

    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2DestroyBody(bodyId);
    LOGD("Destroyed body (id: %llu)", (unsigned long long)body_id);
}

void set_transform(uint64_t body_id, double x, double y, double angle) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetTransform(bodyId, (b2Vec2){(float)x, (float)y}, b2MakeRot(angle));
}

void get_position(uint64_t body_id, float* out_x, float* out_y) {
    LOGD("    MODULE get_position: body_id=%llu", (unsigned long long)body_id);
    if (body_id == 0) {
        LOGD("    MODULE get_position: body_id is 0, returning (0,0)");
        *out_x = 0.0f;
        *out_y = 0.0f;
        return;
    }
    b2BodyId bodyId = b2LoadBodyId(body_id);
    LOGD("    MODULE get_position: loaded bodyId (index1=%d, world0=%d)",
         bodyId.index1, bodyId.world0);
    b2Vec2 pos = b2Body_GetPosition(bodyId);
    LOGD("    MODULE get_position: Box2D returned pos.x=%f, pos.y=%f", pos.x, pos.y);
    *out_x = pos.x;
    *out_y = pos.y;
    LOGD("    MODULE get_position: setting *out_x=%f, *out_y=%f", *out_x, *out_y);
}

double get_angle(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Rot rotation = b2Body_GetRotation(bodyId);
    return b2Rot_GetAngle(rotation);
}

void get_transform(uint64_t body_id, float* out_x, float* out_y, float* out_angle) {
    if (body_id == 0) {
        *out_x = 0.0f;
        *out_y = 0.0f;
        *out_angle = 0.0f;
        return;
    }
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Transform transform = b2Body_GetTransform(bodyId);
    *out_x = transform.p.x;
    *out_y = transform.p.y;
    *out_angle = b2Rot_GetAngle(transform.q);
}

void set_linear_velocity(uint64_t body_id, double vx, double vy) {
    LOGD("    MODULE set_linear_velocity: body_id=%llu, vx=%f, vy=%f", (unsigned long long)body_id, vx, vy);
    if (body_id == 0) {
        LOGD("    MODULE set_linear_velocity: body_id is 0, returning");
        return;
    }
    b2BodyId bodyId = b2LoadBodyId(body_id);
    LOGD("    MODULE set_linear_velocity: about to call b2Body_SetLinearVelocity with vx=%f, vy=%f", (float)vx, (float)vy);
    b2Body_SetLinearVelocity(bodyId, (b2Vec2){(float)vx, (float)vy});
    LOGD("    MODULE set_linear_velocity: b2Body_SetLinearVelocity completed");
}

void get_linear_velocity(uint64_t body_id, float* out_vx, float* out_vy) {
    if (body_id == 0) {
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        return;
    }
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Vec2 vel = b2Body_GetLinearVelocity(bodyId);
    *out_vx = vel.x;
    *out_vy = vel.y;
}

void set_angular_velocity(uint64_t body_id, double omega) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetAngularVelocity(bodyId, omega);
}

double get_angular_velocity(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetAngularVelocity(bodyId);
}

void apply_force(uint64_t body_id, double fx, double fy, double px, double py, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyForce(bodyId, (b2Vec2){(float)fx, (float)fy}, (b2Vec2){(float)px, (float)py}, wake);
}

void apply_force_to_center(uint64_t body_id, double fx, double fy, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyForceToCenter(bodyId, (b2Vec2){(float)fx, (float)fy}, wake);
}

void apply_torque(uint64_t body_id, double torque, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyTorque(bodyId, torque, wake);
}

void apply_linear_impulse(uint64_t body_id, double ix, double iy, double px, double py, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyLinearImpulse(bodyId, (b2Vec2){(float)ix, (float)iy}, (b2Vec2){(float)px, (float)py}, wake);
}

void apply_linear_impulse_to_center(uint64_t body_id, double ix, double iy, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyLinearImpulseToCenter(bodyId, (b2Vec2){(float)ix, (float)iy}, wake);
}

void apply_angular_impulse(uint64_t body_id, double impulse, bool wake) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_ApplyAngularImpulse(bodyId, impulse, wake);
}

double get_mass(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetMass(bodyId);
}

double get_inertia(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetRotationalInertia(bodyId);
}

void get_local_center_of_mass(uint64_t body_id, float* out_x, float* out_y) {
    if (body_id == 0) {
        *out_x = 0.0f;
        *out_y = 0.0f;
        return;
    }
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Vec2 center = b2Body_GetLocalCenterOfMass(bodyId);
    *out_x = center.x;
    *out_y = center.y;
}

void set_awake(uint64_t body_id, bool flag) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetAwake(bodyId, flag);
}

bool is_awake(uint64_t body_id) {
    if (body_id == 0) return false;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_IsAwake(bodyId);
}

void set_enabled(uint64_t body_id, bool flag) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    if (flag) {
        b2Body_Enable(bodyId);
    } else {
        b2Body_Disable(bodyId);
    }
}

bool is_enabled(uint64_t body_id) {
    if (body_id == 0) return false;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_IsEnabled(bodyId);
}

void set_linear_damping(uint64_t body_id, double damping) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetLinearDamping(bodyId, damping);
}

double get_linear_damping(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetLinearDamping(bodyId);
}

void set_angular_damping(uint64_t body_id, double damping) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetAngularDamping(bodyId, damping);
}

double get_angular_damping(uint64_t body_id) {
    if (body_id == 0) return 0.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetAngularDamping(bodyId);
}

void set_gravity_scale(uint64_t body_id, double scale) {
    if (body_id == 0) return;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    b2Body_SetGravityScale(bodyId, scale);
}

double get_gravity_scale(uint64_t body_id) {
    if (body_id == 0) return 1.0f;
    b2BodyId bodyId = b2LoadBodyId(body_id);
    return b2Body_GetGravityScale(bodyId);
}

} // namespace module_body
