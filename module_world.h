// Box2D World Management Wrapper
// Part of HeapsX Box2D bindings

#pragma once

#include <stdint.h>

namespace module_world {
    void init();

    // World creation/destruction (uses uint32_t for cross-language compatibility)
    uint32_t create_world(double gravity_x, double gravity_y);
    void destroy_world(uint32_t world_id);

    // Simulation stepping
    void step(uint32_t world_id, double time_step, int sub_step_count);

    // Gravity
    void set_gravity(uint32_t world_id, double x, double y);
    void get_gravity(uint32_t world_id, float* out_x, float* out_y);

    // World properties
    void enable_sleeping(uint32_t world_id, bool flag);
    void enable_continuous(uint32_t world_id, bool flag);
    void enable_warm_starting(uint32_t world_id, bool flag);

    // Queries
    int get_body_count(uint32_t world_id);
    int get_contact_count(uint32_t world_id);
}
