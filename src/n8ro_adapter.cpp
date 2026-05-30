#include "arkheon/character/ICharacterController.h"
#include "n8ro_motion/motion_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>

namespace {

constexpr float kDefaultWalkSpeedMps = 0.85f;
constexpr float kRunSpeedMps = 1.25f;
constexpr float kContactProbeM = 0.08f;
constexpr int kHidW = 26;
constexpr int kHidS = 22;

struct CharacterControllerHandle {
    n8ro_motion::GaitController gait;
    std::array<float, ARK_JOINT_COUNT> segment_lengths{};
    float selected_speed_mps = kDefaultWalkSpeedMps;
    int32_t last_goal_sequence_id = -1;
    bool reported_current_goal = false;
};

float clampFloat(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

arkheon_vec3 makeVec3(float x, float y, float z)
{
    return {x, y, z};
}

n8ro_motion::Vec3 toMotionVec3(arkheon_vec3 value)
{
    return {static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z)};
}

arkheon_vec3 add(arkheon_vec3 a, arkheon_vec3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

arkheon_vec3 subtract(arkheon_vec3 a, arkheon_vec3 b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

arkheon_vec3 scale(arkheon_vec3 value, float scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

arkheon_vec3 midpoint(arkheon_vec3 a, arkheon_vec3 b)
{
    return scale(add(a, b), 0.5f);
}

float lengthXZ(arkheon_vec3 value)
{
    return std::sqrt(value.x * value.x + value.z * value.z);
}

arkheon_vec3 normalizeXZ(arkheon_vec3 value)
{
    const float len = lengthXZ(value);
    if (len <= 1.0e-5f) {
        return makeVec3(1.0f, 0.0f, 0.0f);
    }

    return makeVec3(value.x / len, 0.0f, value.z / len);
}

arkheon_quat identityQuat()
{
    return {0.0f, 0.0f, 0.0f, 1.0f};
}

arkheon_quat quatFromPitch(float radians)
{
    const float half = 0.5f * radians;
    return {std::sin(half), 0.0f, 0.0f, std::cos(half)};
}

arkheon_quat quatFromYaw(float radians)
{
    const float half = 0.5f * radians;
    return {0.0f, std::sin(half), 0.0f, std::cos(half)};
}

float jointAngle(const n8ro_motion::JointAngles& angles, n8ro_motion::JointIndex joint)
{
    return static_cast<float>(angles.radians[static_cast<std::size_t>(joint)]);
}

void writeOverride(arkheon_bone_override& override_out, float pitch_radians)
{
    override_out.local_rotation = quatFromPitch(pitch_radians);
    override_out.apply = 1;
}

bool queryFootContact(const arkheon_env_api* env, arkheon_vec3 foot_world)
{
    if (env == nullptr || env->raycast == nullptr) {
        return true;
    }

    arkheon_vec3 hit{};
    arkheon_vec3 normal{};
    int32_t object_id = -1;
    const int32_t hit_result = env->raycast(
        env->host_ctx,
        add(foot_world, makeVec3(0.0f, 0.03f, 0.0f)),
        makeVec3(0.0f, -1.0f, 0.0f),
        kContactProbeM,
        &hit,
        &normal,
        &object_id);

    return hit_result != 0;
}

void applyHotkeys(CharacterControllerHandle& controller, const arkheon_input_state* input)
{
    if (input == nullptr) {
        return;
    }

    if (input->hotkey_motion_a != 0) {
        controller.selected_speed_mps = kDefaultWalkSpeedMps;
    }

    if (input->hotkey_motion_b != 0) {
        controller.selected_speed_mps = 0.0f;
    }

    if (input->hotkey_motion_c != 0) {
        controller.selected_speed_mps = kRunSpeedMps;
    }

    if (input->keys[kHidW] != 0) {
        controller.selected_speed_mps = kDefaultWalkSpeedMps;
    } else if (input->keys[kHidS] != 0) {
        controller.selected_speed_mps = 0.0f;
    }
}

n8ro_motion::MotionInput buildMotionInput(
    const CharacterControllerHandle& controller,
    const arkheon_frame& frame,
    const arkheon_bone_state* in_bones,
    const arkheon_env_api* env)
{
    n8ro_motion::MotionInput motion{};
    motion.time_seconds = frame.simulation_time_s;
    motion.delta_seconds = frame.delta_time_s;
    motion.target_speed = clampFloat(controller.selected_speed_mps / kDefaultWalkSpeedMps, 0.0f, 1.6f);

    if (in_bones != nullptr) {
        const arkheon_vec3 left_thigh = in_bones[ARK_JOINT_THIGH_L].world_position;
        const arkheon_vec3 right_thigh = in_bones[ARK_JOINT_THIGH_R].world_position;
        const arkheon_vec3 left_upper_arm = in_bones[ARK_JOINT_UPPERARM_L].world_position;
        const arkheon_vec3 right_upper_arm = in_bones[ARK_JOINT_UPPERARM_R].world_position;
        const arkheon_vec3 left_foot = in_bones[ARK_JOINT_FOOT_L].world_position;
        const arkheon_vec3 right_foot = in_bones[ARK_JOINT_FOOT_R].world_position;

        motion.has_body_points = true;
        motion.pelvis_position = toMotionVec3(midpoint(left_thigh, right_thigh));
        motion.torso_position = toMotionVec3(midpoint(left_upper_arm, right_upper_arm));
        motion.left_foot_position = toMotionVec3(left_foot);
        motion.right_foot_position = toMotionVec3(right_foot);
        motion.left_foot_contact = queryFootContact(env, left_foot);
        motion.right_foot_contact = queryFootContact(env, right_foot);
    }

    return motion;
}

void writeRootMotion(CharacterControllerHandle& controller,
                     const arkheon_frame& frame,
                     const arkheon_bone_state* in_bones,
                     arkheon_vec3* out_root_translation_delta,
                     arkheon_quat* out_root_rotation_delta,
                     const arkheon_input_state* input,
                     const arkheon_mission_goal* current_goal,
                     const arkheon_env_api* env)
{
    if (out_root_translation_delta != nullptr) {
        *out_root_translation_delta = makeVec3(0.0f, 0.0f, 0.0f);
    }

    if (out_root_rotation_delta != nullptr) {
        *out_root_rotation_delta = identityQuat();
    }

    if (frame.is_paused != 0 || out_root_translation_delta == nullptr) {
        return;
    }

    const float dt = clampFloat(static_cast<float>(frame.delta_time_s), 0.001f, 0.05f);
    const arkheon_vec3 root_position = in_bones != nullptr ? in_bones[0].world_position : makeVec3(0.0f, 0.0f, 0.0f);

    arkheon_vec3 direction = makeVec3(0.0f, 0.0f, 0.0f);
    float speed_mps = controller.selected_speed_mps;

    if (current_goal != nullptr && current_goal->type == ARK_GOAL_GOTO) {
        if (current_goal->sequence_id != controller.last_goal_sequence_id) {
            controller.last_goal_sequence_id = current_goal->sequence_id;
            controller.reported_current_goal = false;
        }

        const arkheon_vec3 to_target = subtract(current_goal->target_position, root_position);
        const float distance_m = lengthXZ(to_target);
        const float tolerance_m = current_goal->tolerance_m > 0.0f ? current_goal->tolerance_m : 0.3f;

        if (distance_m <= tolerance_m) {
            speed_mps = 0.0f;
            if (!controller.reported_current_goal && env != nullptr && env->report_goal_complete != nullptr) {
                env->report_goal_complete(env->host_ctx, current_goal->sequence_id, ARK_GOAL_RESULT_OK);
                controller.reported_current_goal = true;
            }
        } else {
            direction = normalizeXZ(to_target);
        }
    } else if (input != nullptr && input->keys[kHidW] != 0) {
        direction = makeVec3(std::cos(input->look_yaw_rad), 0.0f, std::sin(input->look_yaw_rad));
    }

    if (lengthXZ(direction) <= 1.0e-5f || speed_mps <= 0.0f) {
        return;
    }

    *out_root_translation_delta = scale(direction, speed_mps * dt);

    if (out_root_rotation_delta != nullptr) {
        const float yaw = std::atan2(direction.z, direction.x);
        *out_root_rotation_delta = quatFromYaw(yaw);
    }
}

void writeJointOverrides(const n8ro_motion::JointAngles& angles, arkheon_bone_override out_overrides[ARK_JOINT_COUNT])
{
    if (out_overrides == nullptr) {
        return;
    }

    writeOverride(out_overrides[ARK_JOINT_UPPERARM_L], jointAngle(angles, n8ro_motion::JointIndex::LeftShoulderPitch));
    writeOverride(out_overrides[ARK_JOINT_UPPERARM_R], jointAngle(angles, n8ro_motion::JointIndex::RightShoulderPitch));
    writeOverride(out_overrides[ARK_JOINT_LOWERARM_L], jointAngle(angles, n8ro_motion::JointIndex::LeftElbowPitch));
    writeOverride(out_overrides[ARK_JOINT_LOWERARM_R], jointAngle(angles, n8ro_motion::JointIndex::RightElbowPitch));
    writeOverride(out_overrides[ARK_JOINT_THIGH_L], jointAngle(angles, n8ro_motion::JointIndex::LeftHipPitch));
    writeOverride(out_overrides[ARK_JOINT_THIGH_R], jointAngle(angles, n8ro_motion::JointIndex::RightHipPitch));
    writeOverride(out_overrides[ARK_JOINT_CALF_L], jointAngle(angles, n8ro_motion::JointIndex::LeftKneePitch));
    writeOverride(out_overrides[ARK_JOINT_CALF_R], jointAngle(angles, n8ro_motion::JointIndex::RightKneePitch));
    writeOverride(out_overrides[ARK_JOINT_FOOT_L], jointAngle(angles, n8ro_motion::JointIndex::LeftAnklePitch));
    writeOverride(out_overrides[ARK_JOINT_FOOT_R], jointAngle(angles, n8ro_motion::JointIndex::RightAnklePitch));
}

}  // namespace

uint32_t arkheon_character_sdk_version(void)
{
    return ARKHEON_CHARACTER_SDK_VERSION;
}

const char* arkheon_character_plugin_name(void)
{
    return "CoM Aware 10 Joint Kinematic Walk";
}

void arkheon_character_get_motion_clips(void* handle, int32_t out_clip_ids[3])
{
    (void)handle;
    if (out_clip_ids == nullptr) {
        return;
    }

    out_clip_ids[0] = 0;
    out_clip_ids[1] = 1;
    out_clip_ids[2] = 2;
}

void* arkheon_character_create(const float segment_lengths_m[10])
{
    CharacterControllerHandle* controller = new (std::nothrow) CharacterControllerHandle();
    if (controller == nullptr) {
        return nullptr;
    }

    if (segment_lengths_m != nullptr) {
        std::copy(segment_lengths_m, segment_lengths_m + ARK_JOINT_COUNT, controller->segment_lengths.begin());
    }

    return controller;
}

void arkheon_character_destroy(void* handle)
{
    CharacterControllerHandle* controller = static_cast<CharacterControllerHandle*>(handle);
    delete controller;
}

int32_t arkheon_character_tick(void* handle,
                               const arkheon_frame* frame,
                               const arkheon_bone_state in_bones[66],
                               arkheon_bone_override out_overrides[10],
                               arkheon_vec3* out_root_translation_delta,
                               arkheon_quat* out_root_rotation_delta,
                               const arkheon_input_state* input,
                               const arkheon_mission_goal* current_goal,
                               const arkheon_env_api* env)
{
    try {
        CharacterControllerHandle* controller = static_cast<CharacterControllerHandle*>(handle);
        if (controller == nullptr || frame == nullptr || out_overrides == nullptr) {
            return 1;
        }

        applyHotkeys(*controller, input);

        if (frame->is_paused != 0) {
            for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
                out_overrides[i].local_rotation = identityQuat();
                out_overrides[i].apply = 0;
            }
            return 0;
        }

        const n8ro_motion::MotionInput motion_input = buildMotionInput(*controller, *frame, in_bones, env);
        const n8ro_motion::JointAngles angles = controller->gait.update(motion_input);

        writeJointOverrides(angles, out_overrides);
        writeRootMotion(*controller, *frame, in_bones, out_root_translation_delta, out_root_rotation_delta, input, current_goal, env);
        return 0;
    } catch (...) {
        return 1;
    }
}
