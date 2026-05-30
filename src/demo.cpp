#include "n8ro_motion/motion_controller.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

void printHeader()
{
    std::cout << "time";
    for (std::size_t i = 0; i < n8ro_motion::kJointCount; ++i) {
        std::cout << ',' << n8ro_motion::jointName(i);
    }
    std::cout << '\n';
}

}  // namespace

int main()
{
    n8ro_motion::GaitController controller;

    constexpr double dt = 1.0 / 60.0;
    constexpr double duration_seconds = 10.0;

    printHeader();
    std::cout << std::fixed << std::setprecision(5);

    for (int frame = 0; frame <= static_cast<int>(duration_seconds / dt); ++frame) {
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
        input.torso_position = {input.pelvis_position.x + 0.02 * std::sin(phase + 0.4), 1.45, 0.0};
        input.left_foot_position = {-0.16 * std::sin(phase), 0.0, 0.08};
        input.right_foot_position = {0.16 * std::sin(phase), 0.0, -0.08};

        const n8ro_motion::JointAngles angles = controller.update(input);

        if (frame % 15 == 0) {
            std::cout << t;
            for (double angle : angles.radians) {
                std::cout << ',' << angle;
            }
            std::cout << '\n';
        }
    }

    return 0;
}
