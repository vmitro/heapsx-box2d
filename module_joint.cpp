// Box2D Joint Management Wrapper Implementation
// Part of HeapsX Box2D bindings

#include "module_joint.h"
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

namespace module_joint {

void init() {
    LOGD("Box2D Joint module initialized");
}

uint64_t create_distance_joint(uint64_t body_a_id, uint64_t body_b_id,
                                 double anchor_a_x, double anchor_a_y,
                                 double anchor_b_x, double anchor_b_y) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;
    jointDef.base.localFrameA.p = (b2Vec2){(float)anchor_a_x, (float)anchor_a_y};
    jointDef.base.localFrameB.p = (b2Vec2){(float)anchor_b_x, (float)anchor_b_y};
    jointDef.length = b2Distance((b2Vec2){(float)anchor_a_x, (float)anchor_a_y}, (b2Vec2){(float)anchor_b_x, (float)anchor_b_y});

    b2JointId jointId = b2CreateDistanceJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create distance joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created distance joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_revolute_joint(uint64_t body_a_id, uint64_t body_b_id,
                                 double anchor_x, double anchor_y) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;
    jointDef.base.localFrameA.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    jointDef.base.localFrameB.p = (b2Vec2){(float)anchor_x, (float)anchor_y};

    b2JointId jointId = b2CreateRevoluteJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create revolute joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created revolute joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_prismatic_joint(uint64_t body_a_id, uint64_t body_b_id,
                                  double anchor_x, double anchor_y,
                                  double axis_x, double axis_y) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;
    jointDef.base.localFrameA.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    jointDef.base.localFrameB.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    // Set the axis as the rotation of localFrameA
    b2Vec2 normalizedAxis = b2Normalize((b2Vec2){(float)axis_x, (float)axis_y});
    jointDef.base.localFrameA.q = b2MakeRotFromUnitVector(normalizedAxis);

    b2JointId jointId = b2CreatePrismaticJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create prismatic joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created prismatic joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_weld_joint(uint64_t body_a_id, uint64_t body_b_id,
                            double anchor_x, double anchor_y) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2WeldJointDef jointDef = b2DefaultWeldJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;
    jointDef.base.localFrameA.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    jointDef.base.localFrameB.p = (b2Vec2){(float)anchor_x, (float)anchor_y};

    b2JointId jointId = b2CreateWeldJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create weld joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created weld joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_motor_joint(uint64_t body_a_id, uint64_t body_b_id) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2MotorJointDef jointDef = b2DefaultMotorJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;

    b2JointId jointId = b2CreateMotorJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create motor joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created motor joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

uint64_t create_wheel_joint(uint64_t body_a_id, uint64_t body_b_id,
                             double anchor_x, double anchor_y,
                             double axis_x, double axis_y) {
    if (body_a_id == 0 || body_b_id == 0) {
        LOGE("Cannot create joint with null bodies");
        return 0;
    }

    b2BodyId bodyA = b2LoadBodyId(body_a_id);
    b2BodyId bodyB = b2LoadBodyId(body_b_id);

    b2WheelJointDef jointDef = b2DefaultWheelJointDef();
    jointDef.base.bodyIdA = bodyA;
    jointDef.base.bodyIdB = bodyB;
    jointDef.base.localFrameA.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    jointDef.base.localFrameB.p = (b2Vec2){(float)anchor_x, (float)anchor_y};
    // Set the axis as the rotation of localFrameA
    b2Vec2 normalizedAxis = b2Normalize((b2Vec2){(float)axis_x, (float)axis_y});
    jointDef.base.localFrameA.q = b2MakeRotFromUnitVector(normalizedAxis);

    b2JointId jointId = b2CreateWheelJoint(b2Body_GetWorld(bodyA), &jointDef);

    if (B2_IS_NULL(jointId)) {
        LOGE("Failed to create wheel joint");
        return 0;
    }

    uint64_t stored_id = b2StoreJointId(jointId);
    LOGD("Created wheel joint (id: %llu)", (unsigned long long)stored_id);
    return stored_id;
}

void destroy_joint(uint64_t joint_id) {
    if (joint_id == 0) {
        LOGE("Cannot destroy null joint");
        return;
    }

    b2JointId jointId = b2LoadJointId(joint_id);
    b2DestroyJoint(jointId, true);
    LOGD("Destroyed joint (id: %llu)", (unsigned long long)joint_id);
}

void set_motor_speed(uint64_t joint_id, double speed) {
    if (joint_id == 0) return;
    b2JointId jointId = b2LoadJointId(joint_id);

    b2JointType type = b2Joint_GetType(jointId);
    if (type == b2_revoluteJoint) {
        b2RevoluteJoint_SetMotorSpeed(jointId, speed);
    } else if (type == b2_prismaticJoint) {
        b2PrismaticJoint_SetMotorSpeed(jointId, speed);
    } else if (type == b2_wheelJoint) {
        b2WheelJoint_SetMotorSpeed(jointId, speed);
    }
}

void set_max_motor_torque(uint64_t joint_id, double torque) {
    if (joint_id == 0) return;
    b2JointId jointId = b2LoadJointId(joint_id);

    b2JointType type = b2Joint_GetType(jointId);
    if (type == b2_revoluteJoint) {
        b2RevoluteJoint_SetMaxMotorTorque(jointId, torque);
    }
}

void enable_motor(uint64_t joint_id, bool flag) {
    if (joint_id == 0) return;
    b2JointId jointId = b2LoadJointId(joint_id);

    b2JointType type = b2Joint_GetType(jointId);
    if (type == b2_revoluteJoint) {
        b2RevoluteJoint_EnableMotor(jointId, flag);
    } else if (type == b2_prismaticJoint) {
        b2PrismaticJoint_EnableMotor(jointId, flag);
    } else if (type == b2_wheelJoint) {
        b2WheelJoint_EnableMotor(jointId, flag);
    }
}

void enable_limit(uint64_t joint_id, bool flag) {
    if (joint_id == 0) return;
    b2JointId jointId = b2LoadJointId(joint_id);

    b2JointType type = b2Joint_GetType(jointId);
    if (type == b2_revoluteJoint) {
        b2RevoluteJoint_EnableLimit(jointId, flag);
    } else if (type == b2_prismaticJoint) {
        b2PrismaticJoint_EnableLimit(jointId, flag);
    }
}

void set_limits(uint64_t joint_id, double lower, double upper) {
    if (joint_id == 0) return;
    b2JointId jointId = b2LoadJointId(joint_id);

    b2JointType type = b2Joint_GetType(jointId);
    if (type == b2_revoluteJoint) {
        b2RevoluteJoint_SetLimits(jointId, lower, upper);
    } else if (type == b2_prismaticJoint) {
        b2PrismaticJoint_SetLimits(jointId, lower, upper);
    }
}

} // namespace module_joint
