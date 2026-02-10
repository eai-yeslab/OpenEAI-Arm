#include "OpenEAIArm.hpp"

int main() {
    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_MIX);

    robot.reset();

    OpenEAIArm::JointArray target = {0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.00f};

    robot.set_joint_targets(target, 3.0f, robot.manager().uniform(0.0f), {}, true); 
    std::this_thread::sleep_for(std::chrono::seconds(3));

    robot.set_control_mode(OpenEAIArm::ControlMode::MIT_DRAG);
    std::cout << "Enter anything to switch back to MIT_MIX mode..." << std::endl;
    char dummy;
    std::cin >> dummy;
    std::cout << "Switching back to MIT_MIX mode..." << std::endl;
    robot.set_control_mode(OpenEAIArm::ControlMode::MIT_MIX);

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.go_home();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.disable_all();

    return 0;
}