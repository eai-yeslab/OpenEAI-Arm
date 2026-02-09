#include "OpenEAIArm.hpp"

int main() {
    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_MIX);

    robot.reset();
    bool ik_success;
    OpenEAIArm::JointArray position = robot.get_ee_pose();
    std::cout << "EE Pose: " << position << std::endl;
    std::cout << "IK: " << robot.inverse_kinetics(position, ik_success) << std::endl;
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;
    OpenEAIArm::JointArray target = {0.0f, 1.25f, 1.5f, -1.0f, 0.0f, 0.0f, 0.00f};

    robot.set_joint_targets(target, 5.0f, robot.manager().uniform(0.0f), {}, true); 
    std::this_thread::sleep_for(std::chrono::seconds(5));

    bool test_cos = false;
    if (test_cos) {
        constexpr float test_time = 12.0f; 
        constexpr float freq_hz = 0.25f; 
        constexpr float amplitude = 0.3f;
        constexpr float send_freq = 50.0f;
        constexpr float dt = 1.0f/send_freq;

        auto t0 = std::chrono::steady_clock::now();
        for (int k=0; k < int(test_time*send_freq); ++k) {
            auto tn = std::chrono::steady_clock::now();
            float t = std::chrono::duration<float>(tn-t0).count();

            OpenEAIArm::JointArray cycle;
            for (size_t j=0; j < OpenEAIArm::NUM_JOINTS; ++j) {
                if (j == OpenEAIArm::NUM_JOINTS - 1) cycle[j] = target[j] + 0.04 * std::sin(2 * M_PI * freq_hz * t);
                else cycle[j] = target[j] + amplitude * std::sin(2 * M_PI * freq_hz * t);
            }
            robot.set_joint_targets(cycle, 0.0f); 

            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.disable_all();

    return 0;
}