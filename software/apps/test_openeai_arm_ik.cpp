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

    robot.set_joint_targets(joint1);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    robot.set_joint_targets(joint2);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    robot.set_joint_targets(robot.inverse_kinetics(reset_pos, ik_success));
    std::this_thread::sleep_for(std::chrono::seconds(2));


    // robot.go_home();
    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 停用电机
    robot.disable_all();

    return 0;
}