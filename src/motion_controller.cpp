#include "n8ro_motion/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace n8ro_motion {
namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr std::array<std::string_view, kJointCount> kJointNames{
    "left_hip_pitch",
    "right_hip_pitch",
    "left_knee_pitch",
    "right_knee_pitch",
    "left_ankle_pitch",
    "right_ankle_pitch",
    "left_shoulder_pitch",
    "right_shoulder_pitch",
    "left_elbow_pitch",
    "right_elbow_pitch",
};

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

double sanitize(double value, double fallback = 0.0)
{
    return std::isfinite(value) ? value : fallback;
}

double positivePart(double value)
{
    return std::max(0.0, value);
}

double smoothPositive(double value)
{
    const double p = positivePart(value);
    return p * p;
}

void setAngle(JointAngles& angles, JointIndex index, double value, const JointLimits& limits)
{
    const auto i = static_cast<std::size_t>(index);
    angles.radians[i] = clamp(sanitize(value), limits.minimum[i], limits.maximum[i]);
}

Vec3 add(Vec3 a, Vec3 b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 scale(Vec3 value, double scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

Vec3 midpoint(Vec3 a, Vec3 b)
{
    return scale(add(a, b), 0.5);
}

}  // namespace

GaitController::GaitController() = default;

void GaitController::reset()
{
    phase_radians_ = 0.0;
    last_time_seconds_ = 0.0;
    has_last_time_ = false;
}

JointAngles GaitController::update(const MotionInput& raw_input)
{
    MotionInput input = raw_input;
    input.time_seconds = sanitize(input.time_seconds);
    input.delta_seconds = sanitize(input.delta_seconds, 1.0 / 60.0);
    input.target_speed = clamp(sanitize(input.target_speed, 1.0), 0.0, 2.0);

    double dt = input.delta_seconds;
    if (has_last_time_) {
        const double time_dt = input.time_seconds - last_time_seconds_;
        if (std::isfinite(time_dt) && time_dt > 0.0 && time_dt < 0.25) {
            dt = time_dt;
        }
    }

    dt = clamp(dt, 1.0 / 240.0, 1.0 / 20.0);
    last_time_seconds_ = input.time_seconds;
    has_last_time_ = true;

    const double cadence_hz = 0.85 + 0.25 * input.target_speed;
    phase_radians_ += 2.0 * kPi * cadence_hz * dt;
    if (phase_radians_ > 2.0 * kPi) {
        phase_radians_ = std::fmod(phase_radians_, 2.0 * kPi);
    }

    const JointLimits limits = defaultJointLimits();
    JointAngles output{};

    const double phase = phase_radians_;
    const double left_swing = std::sin(phase);
    const double right_swing = -left_swing;
    const double speed_scale = 0.45 + 0.55 * input.target_speed;

    const Vec3 com = computeCenterOfMass(input);
    const bool both_contacts = input.left_foot_contact && input.right_foot_contact;
    const bool no_contacts = !input.left_foot_contact && !input.right_foot_contact;

    Vec3 support_center = midpoint(input.left_foot_position, input.right_foot_position);
    if (!both_contacts && !no_contacts) {
        support_center = input.left_foot_contact ? input.left_foot_position : input.right_foot_position;
    }

    const double com_forward_error = clamp(com.x - support_center.x, -0.25, 0.25);
    const double balance_pitch = clamp(-0.80 * com_forward_error, -0.12, 0.12);

    const double hip_amplitude = 0.42 * speed_scale;
    const double shoulder_amplitude = 0.34 * speed_scale;
    const double knee_base = 0.08;
    const double knee_amplitude = 0.58 * speed_scale;
    const double ankle_push = 0.18 * speed_scale;

    const double left_knee = knee_base + knee_amplitude * smoothPositive(std::sin(phase - 0.35 * kPi));
    const double right_knee = knee_base + knee_amplitude * smoothPositive(std::sin(phase + 0.65 * kPi));

    setAngle(output, JointIndex::LeftHipPitch, hip_amplitude * left_swing + 0.20 * balance_pitch, limits);
    setAngle(output, JointIndex::RightHipPitch, hip_amplitude * right_swing + 0.20 * balance_pitch, limits);
    setAngle(output, JointIndex::LeftKneePitch, left_knee, limits);
    setAngle(output, JointIndex::RightKneePitch, right_knee, limits);
    setAngle(output, JointIndex::LeftAnklePitch, -ankle_push * left_swing + balance_pitch, limits);
    setAngle(output, JointIndex::RightAnklePitch, -ankle_push * right_swing + balance_pitch, limits);
    setAngle(output, JointIndex::LeftShoulderPitch, -shoulder_amplitude * left_swing, limits);
    setAngle(output, JointIndex::RightShoulderPitch, -shoulder_amplitude * right_swing, limits);
    setAngle(output, JointIndex::LeftElbowPitch, 0.34 + 0.14 * smoothPositive(-left_swing), limits);
    setAngle(output, JointIndex::RightElbowPitch, 0.34 + 0.14 * smoothPositive(-right_swing), limits);

    return output;
}

std::string_view jointName(std::size_t index)
{
    if (index >= kJointNames.size()) {
        return {};
    }
    return kJointNames[index];
}

JointLimits defaultJointLimits()
{
    JointLimits limits{};

    limits.minimum = {
        -0.75,  // left hip pitch
        -0.75,  // right hip pitch
        0.00,   // left knee pitch
        0.00,   // right knee pitch
        -0.55,  // left ankle pitch
        -0.55,  // right ankle pitch
        -0.65,  // left shoulder pitch
        -0.65,  // right shoulder pitch
        0.05,   // left elbow pitch
        0.05,   // right elbow pitch
    };

    limits.maximum = {
        0.75,  // left hip pitch
        0.75,  // right hip pitch
        1.10,  // left knee pitch
        1.10,  // right knee pitch
        0.55,  // left ankle pitch
        0.55,  // right ankle pitch
        0.65,  // left shoulder pitch
        0.65,  // right shoulder pitch
        1.05,  // left elbow pitch
        1.05,  // right elbow pitch
    };

    return limits;
}

Vec3 computeCenterOfMass(const MotionInput& input)
{
    if (!input.has_body_points) {
        return input.pelvis_position;
    }

    const Vec3 upper_leg_center = midpoint(input.pelvis_position, midpoint(input.left_foot_position, input.right_foot_position));
    const SegmentSample samples[] = {
        {input.torso_position, 0.48},
        {input.pelvis_position, 0.22},
        {upper_leg_center, 0.16},
        {input.left_foot_position, 0.07},
        {input.right_foot_position, 0.07},
    };

    return computeCenterOfMass(samples, sizeof(samples) / sizeof(samples[0]));
}

Vec3 computeCenterOfMass(const SegmentSample* segments, std::size_t count)
{
    if (segments == nullptr || count == 0) {
        return {};
    }

    Vec3 weighted_sum{};
    double total_mass = 0.0;

    for (std::size_t i = 0; i < count; ++i) {
        const double mass = sanitize(segments[i].mass);
        if (mass <= 0.0) {
            continue;
        }

        weighted_sum = add(weighted_sum, scale(segments[i].position, mass));
        total_mass += mass;
    }

    if (total_mass <= std::numeric_limits<double>::epsilon()) {
        return {};
    }

    return scale(weighted_sum, 1.0 / total_mass);
}

bool areAnglesFiniteAndWithinLimits(const JointAngles& angles, const JointLimits& limits)
{
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const double value = angles.radians[i];
        if (!std::isfinite(value)) {
            return false;
        }

        if (value < limits.minimum[i] || value > limits.maximum[i]) {
            return false;
        }
    }

    return true;
}

}  // namespace n8ro_motion
