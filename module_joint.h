// Box2D Joint Management Wrapper
// Part of HeapsX Box2D bindings

#pragma once

#include <stdint.h>

namespace module_joint {
    void init();

    // Joint creation (returns uint64_t joint IDs)
    uint64_t create_distance_joint(uint64_t body_a_id, uint64_t body_b_id,
                                     double anchor_a_x, double anchor_a_y,
                                     double anchor_b_x, double anchor_b_y);

    uint64_t create_revolute_joint(uint64_t body_a_id, uint64_t body_b_id,
                                     double anchor_x, double anchor_y);

    uint64_t create_prismatic_joint(uint64_t body_a_id, uint64_t body_b_id,
                                      double anchor_x, double anchor_y,
                                      double axis_x, double axis_y);

    uint64_t create_weld_joint(uint64_t body_a_id, uint64_t body_b_id,
                                double anchor_x, double anchor_y);

    uint64_t create_motor_joint(uint64_t body_a_id, uint64_t body_b_id);

    uint64_t create_wheel_joint(uint64_t body_a_id, uint64_t body_b_id,
                                 double anchor_x, double anchor_y,
                                 double axis_x, double axis_y);

    // Joint destruction
    void destroy_joint(uint64_t joint_id);

    // Motor control (for revolute, prismatic, wheel joints)
    void set_motor_speed(uint64_t joint_id, double speed);
    void set_max_motor_torque(uint64_t joint_id, double torque);
    void enable_motor(uint64_t joint_id, bool flag);

    // Limits (for revolute, prismatic joints)
    void enable_limit(uint64_t joint_id, bool flag);
    void set_limits(uint64_t joint_id, double lower, double upper);
}
