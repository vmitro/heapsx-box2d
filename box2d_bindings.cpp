// Box2D HashLink Bindings
// Part of HeapsX Box2D bindings

// Section 1: HL_NAME macro
#define HL_NAME(n) box2d_##n

#include <hl.h>

// Section 2: Include module implementations
#include "module_world.cpp"
#include "module_body.cpp"
#include "module_shape.cpp"
#include "module_joint.cpp"

// Section 3: HL_PRIM wrapper functions

// === Initialization ===
HL_PRIM void HL_NAME(init)() {
    module_world::init();
    module_body::init();
    module_shape::init();
    module_joint::init();
}

// === World Management ===
HL_PRIM int HL_NAME(world_create)(double gx, double gy) {
    return (int)module_world::create_world(gx, gy);
}

HL_PRIM void HL_NAME(world_destroy)(int world_id) {
    module_world::destroy_world((uint32_t)world_id);
}

HL_PRIM void HL_NAME(world_step)(int world_id, double time_step, int sub_step_count) {
    module_world::step((uint32_t)world_id, time_step, sub_step_count);
}

HL_PRIM void HL_NAME(world_set_gravity)(int world_id, double x, double y) {
    module_world::set_gravity((uint32_t)world_id, x, y);
}

HL_PRIM void HL_NAME(world_get_gravity)(int world_id, vbyte* out_bytes) {
    float x, y;
    module_world::get_gravity((uint32_t)world_id, &x, &y);
    ((double*)out_bytes)[0] = (double)x;
    ((double*)out_bytes)[1] = (double)y;
}

HL_PRIM void HL_NAME(world_enable_sleeping)(int world_id, bool flag) {
    module_world::enable_sleeping((uint32_t)world_id, flag);
}

HL_PRIM void HL_NAME(world_enable_continuous)(int world_id, bool flag) {
    module_world::enable_continuous((uint32_t)world_id, flag);
}

HL_PRIM void HL_NAME(world_enable_warm_starting)(int world_id, bool flag) {
    module_world::enable_warm_starting((uint32_t)world_id, flag);
}

HL_PRIM int HL_NAME(world_get_body_count)(int world_id) {
    return module_world::get_body_count((uint32_t)world_id);
}

HL_PRIM int HL_NAME(world_get_contact_count)(int world_id) {
    return module_world::get_contact_count((uint32_t)world_id);
}

// === Body Management ===
HL_PRIM int64 HL_NAME(body_create)(int world_id, int type, double x, double y, double angle) {
    return (int64)module_body::create_body((uint32_t)world_id, type, x, y, angle);
}

HL_PRIM void HL_NAME(body_destroy)(int64 body_id) {
    module_body::destroy_body((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_set_transform)(int64 body_id, double x, double y, double angle) {
    module_body::set_transform((uint64_t)body_id, x, y, angle);
}

HL_PRIM void HL_NAME(body_get_position)(int64 body_id, vbyte* out_bytes) {
    LOGD(">>> body_get_position CALLED: body_id=%llu", (unsigned long long)body_id);
    float x, y;
    module_body::get_position((uint64_t)body_id, &x, &y);
    LOGD(">>> module returned x=%f, y=%f", x, y);
    ((double*)out_bytes)[0] = (double)x;
    ((double*)out_bytes)[1] = (double)y;
    LOGD(">>> wrote to buffer[0]=%f, buffer[1]=%f", ((double*)out_bytes)[0], ((double*)out_bytes)[1]);
}

HL_PRIM double HL_NAME(body_get_angle)(int64 body_id) {
    return module_body::get_angle((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_get_transform)(int64 body_id, vbyte* out_bytes) {
    float x, y, angle;
    module_body::get_transform((uint64_t)body_id, &x, &y, &angle);
    ((double*)out_bytes)[0] = (double)x;
    ((double*)out_bytes)[1] = (double)y;
    ((double*)out_bytes)[2] = (double)angle;
}

HL_PRIM void HL_NAME(body_set_linear_velocity)(int64 body_id, double vx, double vy) {
    LOGD(">>> body_set_linear_velocity CALLED: body_id=%llu, vx=%f, vy=%f", (unsigned long long)body_id, vx, vy);
    module_body::set_linear_velocity((uint64_t)body_id, vx, vy);
    LOGD(">>> body_set_linear_velocity COMPLETED");
}

HL_PRIM void HL_NAME(body_get_linear_velocity)(int64 body_id, vbyte* out_bytes) {
    float vx, vy;
    module_body::get_linear_velocity((uint64_t)body_id, &vx, &vy);
    ((double*)out_bytes)[0] = (double)vx;
    ((double*)out_bytes)[1] = (double)vy;
}

HL_PRIM void HL_NAME(body_set_angular_velocity)(int64 body_id, double omega) {
    module_body::set_angular_velocity((uint64_t)body_id, omega);
}

HL_PRIM double HL_NAME(body_get_angular_velocity)(int64 body_id) {
    return module_body::get_angular_velocity((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_apply_force)(int64 body_id, double fx, double fy, double px, double py, bool wake) {
    module_body::apply_force((uint64_t)body_id, fx, fy, px, py, wake);
}

HL_PRIM void HL_NAME(body_apply_force_to_center)(int64 body_id, double fx, double fy, bool wake) {
    module_body::apply_force_to_center((uint64_t)body_id, fx, fy, wake);
}

HL_PRIM void HL_NAME(body_apply_torque)(int64 body_id, double torque, bool wake) {
    module_body::apply_torque((uint64_t)body_id, torque, wake);
}

HL_PRIM void HL_NAME(body_apply_linear_impulse)(int64 body_id, double ix, double iy, double px, double py, bool wake) {
    module_body::apply_linear_impulse((uint64_t)body_id, ix, iy, px, py, wake);
}

HL_PRIM void HL_NAME(body_apply_linear_impulse_to_center)(int64 body_id, double ix, double iy, bool wake) {
    module_body::apply_linear_impulse_to_center((uint64_t)body_id, ix, iy, wake);
}

HL_PRIM void HL_NAME(body_apply_angular_impulse)(int64 body_id, double impulse, bool wake) {
    module_body::apply_angular_impulse((uint64_t)body_id, impulse, wake);
}

HL_PRIM double HL_NAME(body_get_mass)(int64 body_id) {
    return module_body::get_mass((uint64_t)body_id);
}

HL_PRIM double HL_NAME(body_get_inertia)(int64 body_id) {
    return module_body::get_inertia((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_get_local_center_of_mass)(int64 body_id, vbyte* out_bytes) {
    float x, y;
    module_body::get_local_center_of_mass((uint64_t)body_id, &x, &y);
    ((double*)out_bytes)[0] = (double)x;
    ((double*)out_bytes)[1] = (double)y;
}

HL_PRIM void HL_NAME(body_set_awake)(int64 body_id, bool flag) {
    module_body::set_awake((uint64_t)body_id, flag);
}

HL_PRIM bool HL_NAME(body_is_awake)(int64 body_id) {
    return module_body::is_awake((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_set_enabled)(int64 body_id, bool flag) {
    module_body::set_enabled((uint64_t)body_id, flag);
}

HL_PRIM bool HL_NAME(body_is_enabled)(int64 body_id) {
    return module_body::is_enabled((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_set_linear_damping)(int64 body_id, double damping) {
    module_body::set_linear_damping((uint64_t)body_id, damping);
}

HL_PRIM double HL_NAME(body_get_linear_damping)(int64 body_id) {
    return module_body::get_linear_damping((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_set_angular_damping)(int64 body_id, double damping) {
    module_body::set_angular_damping((uint64_t)body_id, damping);
}

HL_PRIM double HL_NAME(body_get_angular_damping)(int64 body_id) {
    return module_body::get_angular_damping((uint64_t)body_id);
}

HL_PRIM void HL_NAME(body_set_gravity_scale)(int64 body_id, double scale) {
    module_body::set_gravity_scale((uint64_t)body_id, scale);
}

HL_PRIM double HL_NAME(body_get_gravity_scale)(int64 body_id) {
    return module_body::get_gravity_scale((uint64_t)body_id);
}

// === Shape Management ===
HL_PRIM int64 HL_NAME(shape_create_circle)(int64 body_id, double radius, double center_x, double center_y) {
    return (int64)module_shape::create_circle((uint64_t)body_id, radius, center_x, center_y);
}

HL_PRIM int64 HL_NAME(shape_create_capsule)(int64 body_id, double c1x, double c1y, double c2x, double c2y, double radius) {
    return (int64)module_shape::create_capsule((uint64_t)body_id, c1x, c1y, c2x, c2y, radius);
}

HL_PRIM int64 HL_NAME(shape_create_box)(int64 body_id, double hx, double hy) {
    return (int64)module_shape::create_box((uint64_t)body_id, hx, hy);
}

HL_PRIM int64 HL_NAME(shape_create_polygon)(int64 body_id, vbyte* vertices, int vertex_count) {
    // Convert double array to float array
    const double* doubles = (const double*)vertices;
    float* floats = new float[vertex_count * 2];
    for (int i = 0; i < vertex_count * 2; i++) {
        floats[i] = (float)doubles[i];
    }
    int64 result = (int64)module_shape::create_polygon((uint64_t)body_id, floats, vertex_count);
    delete[] floats;
    return result;
}

HL_PRIM int64 HL_NAME(shape_create_segment)(int64 body_id, double p1x, double p1y, double p2x, double p2y) {
    return (int64)module_shape::create_segment((uint64_t)body_id, p1x, p1y, p2x, p2y);
}

HL_PRIM void HL_NAME(shape_destroy)(int64 shape_id) {
    module_shape::destroy_shape((uint64_t)shape_id);
}

HL_PRIM void HL_NAME(shape_set_density)(int64 shape_id, double density) {
    module_shape::set_density((uint64_t)shape_id, density);
}

HL_PRIM double HL_NAME(shape_get_density)(int64 shape_id) {
    return module_shape::get_density((uint64_t)shape_id);
}

HL_PRIM void HL_NAME(shape_set_friction)(int64 shape_id, double friction) {
    module_shape::set_friction((uint64_t)shape_id, friction);
}

HL_PRIM double HL_NAME(shape_get_friction)(int64 shape_id) {
    return module_shape::get_friction((uint64_t)shape_id);
}

HL_PRIM void HL_NAME(shape_set_restitution)(int64 shape_id, double restitution) {
    module_shape::set_restitution((uint64_t)shape_id, restitution);
}

HL_PRIM double HL_NAME(shape_get_restitution)(int64 shape_id) {
    return module_shape::get_restitution((uint64_t)shape_id);
}

HL_PRIM void HL_NAME(shape_set_sensor)(int64 shape_id, bool flag) {
    module_shape::set_sensor((uint64_t)shape_id, flag);
}

HL_PRIM bool HL_NAME(shape_is_sensor)(int64 shape_id) {
    return module_shape::is_sensor((uint64_t)shape_id);
}

HL_PRIM void HL_NAME(shape_set_filter_category)(int64 shape_id, int category_bits) {
    module_shape::set_filter_category((uint64_t)shape_id, (uint32_t)category_bits);
}

HL_PRIM void HL_NAME(shape_set_filter_mask)(int64 shape_id, int mask_bits) {
    module_shape::set_filter_mask((uint64_t)shape_id, (uint32_t)mask_bits);
}

HL_PRIM void HL_NAME(shape_set_filter_group)(int64 shape_id, int group_index) {
    module_shape::set_filter_group((uint64_t)shape_id, (int32_t)group_index);
}

// === Joint Management ===
HL_PRIM int64 HL_NAME(joint_create_distance)(int64 body_a, int64 body_b, double ax, double ay, double bx, double by) {
    return (int64)module_joint::create_distance_joint((uint64_t)body_a, (uint64_t)body_b, ax, ay, bx, by);
}

HL_PRIM int64 HL_NAME(joint_create_revolute)(int64 body_a, int64 body_b, double x, double y) {
    return (int64)module_joint::create_revolute_joint((uint64_t)body_a, (uint64_t)body_b, x, y);
}

HL_PRIM int64 HL_NAME(joint_create_prismatic)(int64 body_a, int64 body_b, double ax, double ay, double axis_x, double axis_y) {
    return (int64)module_joint::create_prismatic_joint((uint64_t)body_a, (uint64_t)body_b, ax, ay, axis_x, axis_y);
}

HL_PRIM int64 HL_NAME(joint_create_weld)(int64 body_a, int64 body_b, double x, double y) {
    return (int64)module_joint::create_weld_joint((uint64_t)body_a, (uint64_t)body_b, x, y);
}

HL_PRIM int64 HL_NAME(joint_create_motor)(int64 body_a, int64 body_b) {
    return (int64)module_joint::create_motor_joint((uint64_t)body_a, (uint64_t)body_b);
}

HL_PRIM int64 HL_NAME(joint_create_wheel)(int64 body_a, int64 body_b, double ax, double ay, double axis_x, double axis_y) {
    return (int64)module_joint::create_wheel_joint((uint64_t)body_a, (uint64_t)body_b, ax, ay, axis_x, axis_y);
}

HL_PRIM void HL_NAME(joint_destroy)(int64 joint_id) {
    module_joint::destroy_joint((uint64_t)joint_id);
}

HL_PRIM void HL_NAME(joint_set_motor_speed)(int64 joint_id, double speed) {
    module_joint::set_motor_speed((uint64_t)joint_id, speed);
}

HL_PRIM void HL_NAME(joint_set_max_motor_torque)(int64 joint_id, double torque) {
    module_joint::set_max_motor_torque((uint64_t)joint_id, torque);
}

HL_PRIM void HL_NAME(joint_enable_motor)(int64 joint_id, bool flag) {
    module_joint::enable_motor((uint64_t)joint_id, flag);
}

HL_PRIM void HL_NAME(joint_enable_limit)(int64 joint_id, bool flag) {
    module_joint::enable_limit((uint64_t)joint_id, flag);
}

HL_PRIM void HL_NAME(joint_set_limits)(int64 joint_id, double lower, double upper) {
    module_joint::set_limits((uint64_t)joint_id, lower, upper);
}

// Section 4: DEFINE_PRIM registrations

// Initialization
DEFINE_PRIM(_VOID, init, _NO_ARG);

// World
DEFINE_PRIM(_I32, world_create, _F64 _F64);
DEFINE_PRIM(_VOID, world_destroy, _I32);
DEFINE_PRIM(_VOID, world_step, _I32 _F64 _I32);
DEFINE_PRIM(_VOID, world_set_gravity, _I32 _F64 _F64);
DEFINE_PRIM(_VOID, world_get_gravity, _I32 _BYTES);
DEFINE_PRIM(_VOID, world_enable_sleeping, _I32 _BOOL);
DEFINE_PRIM(_VOID, world_enable_continuous, _I32 _BOOL);
DEFINE_PRIM(_VOID, world_enable_warm_starting, _I32 _BOOL);
DEFINE_PRIM(_I32, world_get_body_count, _I32);
DEFINE_PRIM(_I32, world_get_contact_count, _I32);

// Body
DEFINE_PRIM(_I64, body_create, _I32 _I32 _F64 _F64 _F64);
DEFINE_PRIM(_VOID, body_destroy, _I64);
DEFINE_PRIM(_VOID, body_set_transform, _I64 _F64 _F64 _F64);
DEFINE_PRIM(_VOID, body_get_position, _I64 _BYTES);
DEFINE_PRIM(_F64, body_get_angle, _I64);
DEFINE_PRIM(_VOID, body_get_transform, _I64 _BYTES);
DEFINE_PRIM(_VOID, body_set_linear_velocity, _I64 _F64 _F64);
DEFINE_PRIM(_VOID, body_get_linear_velocity, _I64 _BYTES);
DEFINE_PRIM(_VOID, body_set_angular_velocity, _I64 _F64);
DEFINE_PRIM(_F64, body_get_angular_velocity, _I64);
DEFINE_PRIM(_VOID, body_apply_force, _I64 _F64 _F64 _F64 _F64 _BOOL);
DEFINE_PRIM(_VOID, body_apply_force_to_center, _I64 _F64 _F64 _BOOL);
DEFINE_PRIM(_VOID, body_apply_torque, _I64 _F64 _BOOL);
DEFINE_PRIM(_VOID, body_apply_linear_impulse, _I64 _F64 _F64 _F64 _F64 _BOOL);
DEFINE_PRIM(_VOID, body_apply_linear_impulse_to_center, _I64 _F64 _F64 _BOOL);
DEFINE_PRIM(_VOID, body_apply_angular_impulse, _I64 _F64 _BOOL);
DEFINE_PRIM(_F64, body_get_mass, _I64);
DEFINE_PRIM(_F64, body_get_inertia, _I64);
DEFINE_PRIM(_VOID, body_get_local_center_of_mass, _I64 _BYTES);
DEFINE_PRIM(_VOID, body_set_awake, _I64 _BOOL);
DEFINE_PRIM(_BOOL, body_is_awake, _I64);
DEFINE_PRIM(_VOID, body_set_enabled, _I64 _BOOL);
DEFINE_PRIM(_BOOL, body_is_enabled, _I64);
DEFINE_PRIM(_VOID, body_set_linear_damping, _I64 _F64);
DEFINE_PRIM(_F64, body_get_linear_damping, _I64);
DEFINE_PRIM(_VOID, body_set_angular_damping, _I64 _F64);
DEFINE_PRIM(_F64, body_get_angular_damping, _I64);
DEFINE_PRIM(_VOID, body_set_gravity_scale, _I64 _F64);
DEFINE_PRIM(_F64, body_get_gravity_scale, _I64);

// Shape
DEFINE_PRIM(_I64, shape_create_circle, _I64 _F64 _F64 _F64);
DEFINE_PRIM(_I64, shape_create_capsule, _I64 _F64 _F64 _F64 _F64 _F64);
DEFINE_PRIM(_I64, shape_create_box, _I64 _F64 _F64);
DEFINE_PRIM(_I64, shape_create_polygon, _I64 _BYTES _I32);
DEFINE_PRIM(_I64, shape_create_segment, _I64 _F64 _F64 _F64 _F64);
DEFINE_PRIM(_VOID, shape_destroy, _I64);
DEFINE_PRIM(_VOID, shape_set_density, _I64 _F64);
DEFINE_PRIM(_F64, shape_get_density, _I64);
DEFINE_PRIM(_VOID, shape_set_friction, _I64 _F64);
DEFINE_PRIM(_F64, shape_get_friction, _I64);
DEFINE_PRIM(_VOID, shape_set_restitution, _I64 _F64);
DEFINE_PRIM(_F64, shape_get_restitution, _I64);
DEFINE_PRIM(_VOID, shape_set_sensor, _I64 _BOOL);
DEFINE_PRIM(_BOOL, shape_is_sensor, _I64);
DEFINE_PRIM(_VOID, shape_set_filter_category, _I64 _I32);
DEFINE_PRIM(_VOID, shape_set_filter_mask, _I64 _I32);
DEFINE_PRIM(_VOID, shape_set_filter_group, _I64 _I32);

// Joint
DEFINE_PRIM(_I64, joint_create_distance, _I64 _I64 _F64 _F64 _F64 _F64);
DEFINE_PRIM(_I64, joint_create_revolute, _I64 _I64 _F64 _F64);
DEFINE_PRIM(_I64, joint_create_prismatic, _I64 _I64 _F64 _F64 _F64 _F64);
DEFINE_PRIM(_I64, joint_create_weld, _I64 _I64 _F64 _F64);
DEFINE_PRIM(_I64, joint_create_motor, _I64 _I64);
DEFINE_PRIM(_I64, joint_create_wheel, _I64 _I64 _F64 _F64 _F64 _F64);
DEFINE_PRIM(_VOID, joint_destroy, _I64);
DEFINE_PRIM(_VOID, joint_set_motor_speed, _I64 _F64);
DEFINE_PRIM(_VOID, joint_set_max_motor_torque, _I64 _F64);
DEFINE_PRIM(_VOID, joint_enable_motor, _I64 _BOOL);
DEFINE_PRIM(_VOID, joint_enable_limit, _I64 _BOOL);
DEFINE_PRIM(_VOID, joint_set_limits, _I64 _F64 _F64);
