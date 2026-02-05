#include "OpenEAIArm.hpp"

int main() {
    // OpenEAIArm robot(OpenEAIArm::ControlMode::MIT_MIX, "/dev/ttyACM1");

    // OpenEAIArm robot("configs/default_right.yml", OpenEAIArm::ControlMode::MIT_MIX);
    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_MIX);
    // 回零
    robot.reset();
    bool ik_success;
    OpenEAIArm::JointArray position = robot.get_ee_pose();
    std::cout << "EE Pose: " << position << std::endl;
    std::cout << "IK: " << robot.inverse_kinetics(position, ik_success) << std::endl;
    std::cout << "Joint: " << robot.get_joint_positions() << std::endl;
    // std::cout << "Reset done." << std::endl;
    // 单步设定
    // robot.compute_static_tau(robot.get_joint_positions());
    // OpenEAIArm::JointArray target = {-0.00212508,     2.11126 ,    1.77926   , 0.332004, -0.00211878 ,3.56378e-06, -0.96};
    // OpenEAIArm::JointArray target = {-1.0f, 1.0f, 1.5f, -0.5f, 0.67f, 0.44f, 0.04f};
    OpenEAIArm::JointArray target = {0.0f, 1.25f, 1.5f, -1.0f, 0.0f, 0.0f, 0.00f};
    // OpenEAIArm::JointArray target = {0.0f, 0.5f, 1.0f, -0.5f, 0.0f, 0.0f, 0.0f};
    // OpenEAIArm::JointArray target = {0.0f, 0.2f, 0.84f, -0.74f, 0.0f, 0.0f, 0.0f};
    // OpenEAIArm::JointArray target1 = {0.0846, 0.3242, 1.3508, -1.8726, 0.0106, -0.0107, 0.0002};
    // OpenEAIArm::JointArray target2 = {0.0079, 1.2566, 1.1532, -0.5378, 0.0106, -0.0107, 0.0002};
    // OpenEAIArm::JointArray target1 = {-0.0264, 1.1675, 1.8032, 0.0407, -0.0341, -0.0638, 0.0002};
    // OpenEAIArm::JointArray target2 = {-0.0264, 0.2035, 1.4232, -0.6700, -0.0341, -0.0638, 0.0002};
    // robot.set_joint_targets(target1, 2.0f, robot.manager().uniform(0.0f), {}, true);
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    // std::cout << robot.get_joint_positions() << std::endl;
    // robot.set_joint_targets(target2, 2.0f, robot.manager().uniform(0.0f), {}, true);
    // std::this_thread::sleep_for(std::chrono::seconds(20));
    // std::cout << robot.get_joint_positions() << std::endl;



    // robot.joint_step(target); // MIT模式单周期

    // 插值移动
    robot.set_joint_targets(target, 5.0f, robot.manager().uniform(0.0f), {}, true); // 1s插值
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // std::cout << "EE Pose: " << position << std::endl;
    // std::cout << "IK: " << robot.inverse_kinetics(position) << std::endl;
    // std::cout << "Joint: " << robot.get_joint_positions() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // std::cout << "Press Enter to continue...";
    // char tmp;
    // std::cin >> tmp;
    // robot.go_home();
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // robot.set_joint_targets(target, 5.0f, robot.manager().uniform(0.0f), {}, true);
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // std::cout << "Press Enter to continue...";
    // std::cin >> tmp;

    // robot.compute_static_tau(target);

    // === 连续控制测试区段 ===
    bool test_cos = false;
    if (test_cos) {
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
            for (size_t j=0; j < OpenEAIArm::NUM_JOINTS; ++j) {
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

    // robot.go_home();
    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 停用电机
    robot.disable_all();

    return 0;
}