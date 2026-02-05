#include "OpenEAIArm.hpp"

int main() {
    // OpenEAIArm robot(OpenEAIArm::ControlMode::MIT_MIX, "/dev/ttyACM1");

    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_MIX);
    // 回零
    robot.go_home();
    bool ik_success;
    OpenEAIArm::JointArray reset_pos = robot.get_ee_pose();
    std::cout << "EE Pose: " << reset_pos << std::endl;
    std::cout << "IK: " << robot.inverse_kinetics(reset_pos, ik_success) << std::endl;
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;

    OpenEAIArm::IKOptions options;
    options.policy = OpenEAIArm::IKJumpPolicy::ACCEPT;

    OpenEAIArm::JointArray pose1(reset_pos);
    pose1[0] += 0.1;
    // pose1[1] = -0.2;
    // pose1[2] += 0.1;
    OpenEAIArm::JointArray pose2(pose1);
    pose2[0] += 0.05;
    OpenEAIArm::JointArray joint1 = robot.inverse_kinetics(pose1, ik_success, options);
    OpenEAIArm::JointArray joint2 = robot.inverse_kinetics(pose2, ik_success, options);

    std::cout << "Joint 1: " << joint1 << std::endl;
    std::cout << "Joint 2: " << joint2 << std::endl;

    robot.set_joint_targets(joint1, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    robot.set_joint_targets(joint2, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Press Enter to continue...";
    char tmp;
    std::cin >> tmp;


    bool test_cos = false;
    if (test_cos) {
        robot.set_joint_targets(joint1, 2.0f, {}, {}, true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        OpenEAIArm::JointArray target = {-1.0f, 1.0f, 1.5f, -0.5f, 0.67f, 0.44f, 0.00f};

        robot.set_joint_targets(target, 2.0f, robot.manager().uniform(0.0f), {}, true);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        constexpr float test_time = 12.0f; // 持续测试时长秒
        constexpr float freq_hz = 0.25f;   // 控制信号周期频率
        constexpr float amplitude = 0.3f; // 最大运动边界
        constexpr float send_freq = 50.0f;// 发送频率Hz
        constexpr float dt = 1.0f/send_freq;

        auto t0 = std::chrono::steady_clock::now();
        for (int k=0; k < int(test_time*send_freq); ++k) {
            auto tn = std::chrono::steady_clock::now();
            float t = std::chrono::duration<float>(tn-t0).count();

            // 生成连续目标位：正弦轨迹（你可用cos、双边step等）
            OpenEAIArm::JointArray cycle;
            for (size_t j=0; j < OpenEAIArm::NUM_JOINTS - 1; ++j) {
                // 以target为中点，在 ±amplitude 内运动
                if (j == OpenEAIArm::NUM_JOINTS - 1) cycle[j] = target[j] + 0.04 * std::sin(2 * M_PI * freq_hz * t);
                else cycle[j] = target[j] + amplitude * std::sin(2 * M_PI * freq_hz * t); // 相位区分关节
            }
            robot.set_joint_targets(cycle, 0.0f); // 单步实时命令（不插值，每帧目标跟随），你也可以用joint_step

            std::this_thread::sleep_for(std::chrono::duration<float>(dt));
            // 如需采集反馈可在这里加 robot.get_joint_positions() 日志
        }
    }



    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.set_joint_targets(joint1, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    robot.set_joint_targets(joint2, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Press Enter to continue...";
    std::cin >> tmp;
    robot.set_joint_targets(joint1, 2.0f, {}, {}, true);
    std::this_thread::sleep_for(std::chrono::seconds(3));


    // robot.go_home();
    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 停用电机
    robot.disable_all();

    return 0;
}