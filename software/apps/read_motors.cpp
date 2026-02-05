#include "dm.hpp"
#include "dm_manager.hpp"
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <array>
#include <vector>

// 关节数
constexpr size_t NUM_JOINTS = 7;

// 电机id、类型实际要与你硬件编号对应
std::array<DM_Motor::Motor*, NUM_JOINTS> initMotors() {
    std::array<DM_Motor::Motor*, NUM_JOINTS> motors;
    // for (size_t i = 0; i < NUM_JOINTS; ++i) {
    //     // 注意CAN ID必须和硬件实际对应
    //     DM_Motor::DM_Motor_Type type;
    //     if (i < 3) type = DM_Motor::DM4340;
    //     else type = DM_Motor::DM4310;
    //     DM_Motor::Motor* m = new DM_Motor::Motor(type, 0x01+i, 0x11+i); // id从0x01到0x07
    //     motors[i] = m;
    // }
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
    // 初始化电机和控制器
    auto motors = initMotors();

    std::shared_ptr<SerialPort> serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    DM_Motor::Motor_Control motor_ctl(serial);
    for (auto m : motors){
        motor_ctl.addMotor(m);
        motor_ctl.enable(*m);
    }

    MotorManager mm(motors, &motor_ctl);

    // 启动控制线程
    // mm.start();
    // mm.refresh();
    // mm.reset();

    // sleep(1);

    // 日志文件
    auto q_feedback = mm.getPosition();
    auto dq = mm.getVelocity();
    auto tau = mm.getTau();

    for (size_t j=0;j<NUM_JOINTS;++j) {
        std::cout << std::fixed << std::setprecision(4) << q_feedback[j]
                << "," << dq[j]
                << "," << tau[j] << std::endl;
    }
    

    // 测试结束，停机
    mm.stop();

    

    // 若需安全退出，执行disable操作
    for(auto m : motors) motor_ctl.disable(*m);

    return 0;
}