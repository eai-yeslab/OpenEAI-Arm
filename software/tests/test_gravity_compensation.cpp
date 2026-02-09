#include "OpenEAIArm.hpp"

int main() {
    OpenEAIArm robot(OpenEAIArm::ControlMode::MIT_POS, "/dev/ttyACM1");

    OpenEAIArm::JointArray real_pos = {1.56996,-0.28935,1.92092,-1.65427,0.584229,2.51068,-1.21099};
    real_pos = {1.58751,-0.405699,1.42653,0.272565,0.749409,2.59041,-1.22625};
    std::cout << robot.compute_static_tau(real_pos, real_pos);

    robot.disable_all();

    return 0;
}