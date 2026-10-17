#include "OpenEAIArm.hpp"

int main() {
    // OpenEAIArm robot(OpenEAIArm::ControlMode::MIT_MIX, "/dev/ttyACM1");

    // OpenEAIArm robot("configs/default_right.yml", OpenEAIArm::ControlMode::MIT_MIX);
    OpenEAIArm robot("configs/default_right.yml", OpenEAIArm::ControlMode::MIT_MIX);
    // Move to the reset pose.
    robot.reset();
    bool ik_success;
    // OpenEAIArm::JointArray position = robot.get_ee_pose();
    // std::cout << "EE Pose: " << position << std::endl;
    // std::cout << "IK: " << robot.inverse_kinetics(position, ik_success) << std::endl;
    // std::cout << "Joint: " << robot.get_joint_positions() << std::endl;
    // Set a single-step target.

    OpenEAIArm::JointArray target = {0.0f, 0.0f, 0.1f, -0.1f, 0.0f, 0.0f, 0.0f};
    robot.set_joint_targets(target, 1.0f, robot.manager().uniform(0.0f), {}, true); // One-second interpolation.

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;
    // Move using interpolation.
    std::cout << "Press Enter to continue...";
    std::cin.get();

    std::cout << "Start Interpolating moving" << std::endl;
    target = {0.0f, 0.0f, 1.1f, -1.1f, 0.0f, 0.0f, 0.0f};
    robot.set_joint_targets(target, 2.0f, robot.manager().uniform(0.0f), {}, true); // Two-second interpolation.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "End Interpolating moving" << std::endl;
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;

    std::cout << "Press Enter to continue...";
    std::cin.get();

    // robot.compute_static_tau(target);

    // === Continuous-control test ===
    bool test_cos = false;
    if (test_cos) {
        constexpr float test_time = 12.0f; // Test duration in seconds.
        constexpr float freq_hz = 0.25f;   // Command-cycle frequency.
        constexpr float amplitude = 0.3f;  // Maximum motion range.
        constexpr float send_freq = 50.0f; // Command frequency in hertz.
        constexpr float dt = 1.0f/send_freq;

        auto t0 = std::chrono::steady_clock::now();
        for (int k=0; k < int(test_time*send_freq); ++k) {
            auto tn = std::chrono::steady_clock::now();
            float t = std::chrono::duration<float>(tn-t0).count();

            // Generate a continuous sinusoidal target trajectory.
            OpenEAIArm::JointArray cycle;
            for (size_t j=0; j < OpenEAIArm::NUM_JOINTS; ++j) {
                // Oscillate around the target within the configured amplitude.
                if (j == OpenEAIArm::NUM_JOINTS - 1) cycle[j] = target[j] + 0.04 * std::sin(2 * M_PI * freq_hz * t);
                else cycle[j] = target[j] + amplitude * std::sin(2 * M_PI * freq_hz * t);
            }
            robot.set_joint_targets(cycle, 0.0f); // Send one real-time target per cycle.

            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
            // Feedback logging can be added here if needed.
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // robot.go_home();
    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;

    // Disable motors.
    robot.disable_all();

    return 0;
}
