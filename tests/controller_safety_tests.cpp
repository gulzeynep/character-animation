#include "n8ro_motion/motion_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool nearlyEqual(double a, double b, double epsilon)
{
    return std::abs(a - b) <= epsilon;
}

int fail(const char* message)
{
    std::cerr << "FAIL: " << message << '\n';
    return EXIT_FAILURE;
}

}  // namespace

int main()
{
    const n8ro_motion::JointLimits limits = n8ro_motion::defaultJointLimits();

    {
        const n8ro_motion::SegmentSample samples[] = {
            {{0.0, 1.0, 0.0}, 1.0},
            {{2.0, 1.0, 0.0}, 1.0},
        };
        const n8ro_motion::Vec3 com = n8ro_motion::computeCenterOfMass(samples, 2);
        if (!nearlyEqual(com.x, 1.0, 1.0e-9) || !nearlyEqual(com.y, 1.0, 1.0e-9)) {
            return fail("center of mass weighted average is incorrect");
        }
    }

    n8ro_motion::GaitController controller;
    n8ro_motion::JointAngles previous{};
    bool has_previous = false;

    constexpr double dt = 1.0 / 60.0;
    constexpr int frame_count = static_cast<int>(10.0 / dt);

    for (int frame = 0; frame <= frame_count; ++frame) {
        const double t = frame * dt;
        const double phase = 2.0 * kPi * 1.1 * t;
        const double stride = std::sin(phase);

        n8ro_motion::MotionInput input{};
        input.time_seconds = t;
        input.delta_seconds = dt;
        input.target_speed = 1.0;
        input.left_foot_contact = stride <= 0.2;
        input.right_foot_contact = stride >= -0.2;
        input.has_body_points = true;
        input.pelvis_position = {0.04 * std::sin(phase), 0.95, 0.0};
        input.torso_position = {input.pelvis_position.x, 1.45, 0.0};
        input.left_foot_position = {-0.16 * stride, 0.0, 0.08};
        input.right_foot_position = {0.16 * stride, 0.0, -0.08};

        const n8ro_motion::JointAngles angles = controller.update(input);
        if (!n8ro_motion::areAnglesFiniteAndWithinLimits(angles, limits)) {
            return fail("controller produced a non-finite or out-of-limit angle");
        }

        if (has_previous) {
            for (std::size_t i = 0; i < n8ro_motion::kJointCount; ++i) {
                const double jump = std::abs(angles.radians[i] - previous.radians[i]);
                if (jump > 0.35) {
                    return fail("controller produced a large frame-to-frame jump");
                }
            }
        }

        previous = angles;
        has_previous = true;
    }

    std::cout << "PASS: controller safety checks completed\n";
    return EXIT_SUCCESS;
}
