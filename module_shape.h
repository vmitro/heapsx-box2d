// Box2D Shape Creation Wrapper
// Part of HeapsX Box2D bindings

#pragma once

#include <stdint.h>

namespace module_shape {
    void init();

    // Shape creation (returns uint64_t shape IDs)
    uint64_t create_circle(uint64_t body_id, double radius, double center_x, double center_y);
    uint64_t create_capsule(uint64_t body_id, double center1_x, double center1_y, double center2_x, double center2_y, double radius);
    uint64_t create_box(uint64_t body_id, double hx, double hy);
    uint64_t create_polygon(uint64_t body_id, const float* vertices, int vertex_count);
    uint64_t create_segment(uint64_t body_id, float p1_x, float p1_y, float p2_x, float p2_y);

    // Shape destruction
    void destroy_shape(uint64_t shape_id);

    // Shape properties
    void set_density(uint64_t shape_id, double density);
    double get_density(uint64_t shape_id);
    void set_friction(uint64_t shape_id, double friction);
    double get_friction(uint64_t shape_id);
    void set_restitution(uint64_t shape_id, double restitution);
    double get_restitution(uint64_t shape_id);

    // Sensor
    void set_sensor(uint64_t shape_id, bool flag);
    bool is_sensor(uint64_t shape_id);

    // Filter
    void set_filter_category(uint64_t shape_id, uint32_t category_bits);
    void set_filter_mask(uint64_t shape_id, uint32_t mask_bits);
    void set_filter_group(uint64_t shape_id, int32_t group_index);
}
