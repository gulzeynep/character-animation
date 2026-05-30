#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace n8ro_motion {

constexpr std::size_t kJointCount = 10;

enum class JointIndex : std::size_t {
    LeftHipPitch = 0,
    RightHipPitch,
    LeftKneePitch,
    RightKneePitch,
    LeftAnklePitch,
    RightAnklePitch,
    LeftShoulderPitch,
    RightShoulderPitch,
    LeftElbowPitch,
    RightElbowPitch
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct SegmentSample {
    Vec3 position{};
    double mass = 0.0;
};

struct MotionInput {
    double time_seconds = 0.0;
    double delta_seconds = 1.0 / 60.0;
    double target_speed = 1.0;

    bool left_foot_contact = true;
    bool right_foot_contact = true;

    bool has_body_points = false;
    Vec3 pelvis_position{};
    Vec3 torso_position{0.0, 1.0, 0.0};
    Vec3 left_foot_position{-0.12, 0.0, 0.08};
    Vec3 right_foot_position{0.12, 0.0, -0.08};
};

struct JointAngles {
    std::array<double, kJointCount> radians{};
};

struct JointLimits {
    std::array<double, kJointCount> minimum{};
    std::array<double, kJointCount> maximum{};
};

class GaitController {
public:
    GaitController();

    void reset();
    JointAngles update(const MotionInput& input);

private:
    double phase_radians_ = 0.0;
    double last_time_seconds_ = 0.0;
    bool has_last_time_ = false;
};

std::string_view jointName(std::size_t index);
JointLimits defaultJointLimits();
Vec3 computeCenterOfMass(const MotionInput& input);
Vec3 computeCenterOfMass(const SegmentSample* segments, std::size_t count);
bool areAnglesFiniteAndWithinLimits(const JointAngles& angles, const JointLimits& limits);

}  // namespace n8ro_motion
