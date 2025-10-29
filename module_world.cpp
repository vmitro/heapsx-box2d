// Box2D World Management Wrapper Implementation
// Part of HeapsX Box2D bindings

#include "module_world.h"
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

namespace module_world {

void init() {
    LOGD("Box2D World module initialized");
}

uint32_t create_world(double gravity_x, double gravity_y) {
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = (b2Vec2){(float)gravity_x, (float)gravity_y};

    b2WorldId worldId = b2CreateWorld(&worldDef);

    if (B2_IS_NULL(worldId)) {
        LOGE("Failed to create Box2D world");
        return 0;
    }

    uint32_t stored_id = b2StoreWorldId(worldId);
    LOGD("Created Box2D world (id: %u) with gravity (%.2f, %.2f)", stored_id, gravity_x, gravity_y);
    return stored_id;
}

void destroy_world(uint32_t world_id) {
    if (world_id == 0) {
        LOGE("Cannot destroy null world");
        return;
    }

    b2WorldId worldId = b2LoadWorldId(world_id);
    b2DestroyWorld(worldId);
    LOGD("Destroyed Box2D world (id: %u)", world_id);
}

void step(uint32_t world_id, double time_step, int sub_step_count) {
    if (world_id == 0) {
        LOGE("Cannot step null world");
        return;
    }

    b2WorldId worldId = b2LoadWorldId(world_id);
    b2World_Step(worldId, time_step, sub_step_count);
}

void set_gravity(uint32_t world_id, double x, double y) {
    if (world_id == 0) {
        LOGE("Cannot set gravity on null world");
        return;
    }

    b2WorldId worldId = b2LoadWorldId(world_id);
    b2World_SetGravity(worldId, (b2Vec2){(float)x, (float)y});
}

void get_gravity(uint32_t world_id, float* out_x, float* out_y) {
    if (world_id == 0) {
        LOGE("Cannot get gravity from null world");
        *out_x = 0.0f;
        *out_y = 0.0f;
        return;
    }

    b2WorldId worldId = b2LoadWorldId(world_id);
    b2Vec2 gravity = b2World_GetGravity(worldId);
    *out_x = gravity.x;
    *out_y = gravity.y;
}

void enable_sleeping(uint32_t world_id, bool flag) {
    if (world_id == 0) return;
    b2WorldId worldId = b2LoadWorldId(world_id);
    b2World_EnableSleeping(worldId, flag);
}

void enable_continuous(uint32_t world_id, bool flag) {
    if (world_id == 0) return;
    b2WorldId worldId = b2LoadWorldId(world_id);
    b2World_EnableContinuous(worldId, flag);
}

void enable_warm_starting(uint32_t world_id, bool flag) {
    if (world_id == 0) return;
    b2WorldId worldId = b2LoadWorldId(world_id);
    b2World_EnableWarmStarting(worldId, flag);
}

int get_body_count(uint32_t world_id) {
    if (world_id == 0) return 0;
    b2WorldId worldId = b2LoadWorldId(world_id);
    b2Counters counters = b2World_GetCounters(worldId);
    return counters.bodyCount;
}

int get_contact_count(uint32_t world_id) {
    if (world_id == 0) return 0;
    b2WorldId worldId = b2LoadWorldId(world_id);
    b2Counters counters = b2World_GetCounters(worldId);
    return counters.contactCount;
}

} // namespace module_world
