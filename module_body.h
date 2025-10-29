// Box2D Body Management Wrapper
// Part of HeapsX Box2D bindings

#pragma once

#include <stdint.h>

namespace module_body {
    void init();

    // Body creation/destruction (uses uint64_t for body IDs)
    uint64_t create_body(uint32_t world_id, int type, double x, double y, double angle);
    void destroy_body(uint64_t body_id);

    // Transform
    void set_transform(uint64_t body_id, double x, double y, double angle);
    void get_position(uint64_t body_id, float* out_x, float* out_y);
    double get_angle(uint64_t body_id);
    void get_transform(uint64_t body_id, float* out_x, float* out_y, float* out_angle);

    // Velocity
    void set_linear_velocity(uint64_t body_id, double vx, double vy);
    void get_linear_velocity(uint64_t body_id, float* out_vx, float* out_vy);
    void set_angular_velocity(uint64_t body_id, double omega);
    double get_angular_velocity(uint64_t body_id);

    // Forces and impulses
    void apply_force(uint64_t body_id, double fx, double fy, double px, double py, bool wake);
    void apply_force_to_center(uint64_t body_id, double fx, double fy, bool wake);
    void apply_torque(uint64_t body_id, double torque, bool wake);
    void apply_linear_impulse(uint64_t body_id, double ix, double iy, double px, double py, bool wake);
    void apply_linear_impulse_to_center(uint64_t body_id, double ix, double iy, bool wake);
    void apply_angular_impulse(uint64_t body_id, double impulse, bool wake);

    // Mass properties
    double get_mass(uint64_t body_id);
    double get_inertia(uint64_t body_id);
    void get_local_center_of_mass(uint64_t body_id, float* out_x, float* out_y);

    // State
    void set_awake(uint64_t body_id, bool flag);
    bool is_awake(uint64_t body_id);
    void set_enabled(uint64_t body_id, bool flag);
    bool is_enabled(uint64_t body_id);

    // Properties
    void set_linear_damping(uint64_t body_id, double damping);
    double get_linear_damping(uint64_t body_id);
    void set_angular_damping(uint64_t body_id, double damping);
    double get_angular_damping(uint64_t body_id);
    void set_gravity_scale(uint64_t body_id, double scale);
    double get_gravity_scale(uint64_t body_id);
}
