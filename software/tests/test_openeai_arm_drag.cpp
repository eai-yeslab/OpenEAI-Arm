#include "OpenEAIArm.hpp"
#include <atomic>
#include <csignal>

// Used to signal program interruption
std::atomic<bool> stopFlag(false);

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stopFlag = true;
}


int main() {
    signal(SIGINT, signalHandler);

    OpenEAIArm robot("configs/default.yml", OpenEAIArm::ControlMode::MIT_DRAG);
    robot.go_home();
    OpenEAIArm::JointArray target = {};

    robot.set_joint_targets(target, 1.0f, robot.manager().uniform(0.0f), {}, true); 

    constexpr int drag_time = 6000;

    std::cout << "Sleeping for " << drag_time << " seconds. Press Ctrl+C to interrupt and disable the motors.\n";
    
    for (int i = 0; i < drag_time; ++i) {
        if (stopFlag) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            robot.disable_all();
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


    if (!stopFlag) {

        std::this_thread::sleep_for(std::chrono::seconds(1));
        robot.go_home();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        robot.disable_all();
    }

    return 0;
}