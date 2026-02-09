#include "OpenEAIArm.hpp"

int main() {
    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_MIX);

    robot.go_home();
    bool ik_success;

    OpenEAIArm::JointArray target = {-1.0f, 1.0f, 1.5f, -0.5f, 0.0f, 0.44f, 0.04f};

    robot.set_joint_targets(target, 1.0f, robot.manager().uniform(0.0f), {}, true); // 1s插值

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // === test contiguous move ===
    bool test_cos = true;
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
    OpenEAIArm::JointArray reset_pos = robot.get_ee_pose();
    std::cout << "EE Pose: " << reset_pos << std::endl;
    std::cout << "IK: " << robot.inverse_kinetics(reset_pos, ik_success) << std::endl;
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;

    OpenEAIArm::JointArray pose1(reset_pos);
    pose1[0] += 0.15;
    pose1[1] = -0.2;
    pose1[2] += 0.2;
    pose1[6] = 0.08f;
    OpenEAIArm::JointArray pose2(pose1);
    pose2[1] *= -1;
    pose2[6] = 0.0f;
    OpenEAIArm::JointArray joint1 = robot.inverse_kinetics(pose1, ik_success);
    OpenEAIArm::JointArray joint2 = robot.inverse_kinetics(pose2, ik_success);

    std::cout << "Joint 1: " << joint1 << std::endl;
    std::cout << "Joint 2: " << joint2 << std::endl;

    robot.set_joint_targets(joint1, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    robot.set_joint_targets(joint2, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    robot.set_joint_targets(robot.inverse_kinetics(reset_pos, ik_success), 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));


    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.disable_all();

    return 0;
}