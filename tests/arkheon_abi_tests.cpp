#include "arkheon/character/ICharacterController.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool finiteQuat(arkheon_quat quat)
{
    return std::isfinite(quat.x) && std::isfinite(quat.y) && std::isfinite(quat.z) && std::isfinite(quat.w);
}

int fail(const char* message)
{
    std::cerr << "FAIL: " << message << '\n';
    return EXIT_FAILURE;
}

}  // namespace

int main()
{
    if (arkheon_character_sdk_version() != ARKHEON_CHARACTER_SDK_VERSION) {
        return fail("unexpected SDK version");
    }

    float segment_lengths[10] = {
        0.32f, 0.32f, 0.27f, 0.27f, 0.43f, 0.43f, 0.41f, 0.41f, 0.18f, 0.18f,
    };

    void* handle = arkheon_character_create(segment_lengths);
    if (handle == nullptr) {
        return fail("create returned null");
    }

    arkheon_bone_state bones[66]{};
    bones[ARK_JOINT_UPPERARM_L].world_position = {-0.2f, 1.35f, 0.0f};
    bones[ARK_JOINT_UPPERARM_R].world_position = {0.2f, 1.35f, 0.0f};
    bones[ARK_JOINT_THIGH_L].world_position = {-0.12f, 0.9f, 0.0f};
    bones[ARK_JOINT_THIGH_R].world_position = {0.12f, 0.9f, 0.0f};
    bones[ARK_JOINT_FOOT_L].world_position = {-0.12f, 0.02f, 0.08f};
    bones[ARK_JOINT_FOOT_R].world_position = {0.12f, 0.02f, -0.08f};

    arkheon_frame frame{};
    frame.simulation_time_s = 0.02;
    frame.delta_time_s = 0.02;
    frame.frame_number = 1;

    arkheon_bone_override overrides[10]{};
    arkheon_vec3 root_translation{};
    arkheon_quat root_rotation{};
    arkheon_input_state input{};

    const int32_t tick_result = arkheon_character_tick(
        handle,
        &frame,
        bones,
        overrides,
        &root_translation,
        &root_rotation,
        &input,
        nullptr,
        nullptr);

    if (tick_result != 0) {
        arkheon_character_destroy(handle);
        return fail("tick failed");
    }

    for (int i = 0; i < ARK_JOINT_COUNT; ++i) {
        if (overrides[i].apply != 1 || !finiteQuat(overrides[i].local_rotation)) {
            arkheon_character_destroy(handle);
            return fail("invalid joint override");
        }
    }

    if (!finiteQuat(root_rotation)) {
        arkheon_character_destroy(handle);
        return fail("invalid root rotation");
    }

    arkheon_character_destroy(handle);
    std::cout << "PASS: Arkheon ABI smoke test completed\n";
    return EXIT_SUCCESS;
}
