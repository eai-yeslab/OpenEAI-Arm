#include "dm.hpp"
#include "dm_manager.hpp"
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <array>
#include <vector>

constexpr size_t NUM_JOINTS = 7;

// Set motor IDs and types
std::array<DM_Motor::Motor*, NUM_JOINTS> initMotors() {
    std::array<DM_Motor::Motor*, NUM_JOINTS> motors;
    std::array<DM_Motor::DM_Motor_Type, NUM_JOINTS> types = {DM_Motor::DM4340, DM_Motor::DM4340, DM_Motor::DM4340, DM_Motor::DM4310, DM_Motor::DM4310, DM_Motor::DM4310, DM_Motor::DM4310};
    std::array<uint8_t, NUM_JOINTS>  slave_ids = {0x02, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07};
    std::array<uint8_t, NUM_JOINTS>  master_ids = {0x12, 0x11, 0x13, 0x14, 0x15, 0x16, 0x17};

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        DM_Motor::Motor* m = new DM_Motor::Motor(types[i], slave_ids[i], master_ids[i]);
        motors[i] = m;
    }
    return motors;
}

int main() {
    auto motors = initMotors();

    std::shared_ptr<SerialPort> serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    DM_Motor::Motor_Control motor_ctl(serial);
    for (auto m : motors){
        motor_ctl.addMotor(m);
        motor_ctl.enable(*m);
    }

    MotorManager mm(motors, &motor_ctl);

    auto q_feedback = mm.getPosition();
    auto dq = mm.getVelocity();
    auto tau = mm.getTau();

    for (size_t j=0;j<NUM_JOINTS;++j) {
        std::cout << std::fixed << std::setprecision(4) << q_feedback[j]
                << "," << dq[j]
                << "," << tau[j] << std::endl;
    }
    
    mm.stop();
    for(auto m : motors) motor_ctl.disable(*m);

    return 0;
}