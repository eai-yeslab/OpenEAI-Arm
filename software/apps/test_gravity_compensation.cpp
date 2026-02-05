#include "OpenEAIArm.hpp"

int main() {
    OpenEAIArm robot(OpenEAIArm::ControlMode::MIT_POS, "/dev/ttyACM1");

    // // OpenEAIArm::JointArray target = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // OpenEAIArm::JointArray target = {0.0f, 1.2f, 1.2f, 0.0f, 0.2f, 0.0f, 0.0f};
    // robot.compute_static_tau(robot.q_control_to_phys(target), true);
    // // target = {0.0f, 1.5f, 1.5f, -1.5f, 0.0f, 0.0f, 0.0f};
    // target[3] = -1.5f;
    // robot.compute_static_tau(robot.q_control_to_phys(target), true);

    OpenEAIArm::JointArray real_pos = {1.56996,-0.28935,1.92092,-1.65427,0.584229,2.51068,-1.21099};
    // real_pos = {1.56996,-0.27638,1.92092,-0.0810642,0.592622,2.51068,-1.21099};r
    real_pos = {1.58751,-0.405699,1.42653,0.272565,0.749409,2.59041,-1.22625};
    std::cout << robot.compute_static_tau(real_pos, real_pos);


    // 停用电机
    robot.disable_all();

    return 0;
}