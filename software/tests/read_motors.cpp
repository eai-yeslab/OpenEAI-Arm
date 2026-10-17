#include "dm.hpp"
#include "config_loader.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>

namespace {
constexpr std::size_t NUM_JOINTS = 7;
constexpr float RESET_WARNING_THRESHOLD_RAD = 0.28f;
constexpr int QUERY_ATTEMPTS = 5;
constexpr int POLLS_PER_ATTEMPT = 5;

class PassiveMotorControl final : public DM_Motor::Motor_Control {
public:
    using DM_Motor::Motor_Control::Motor_Control;

    void receive() override {
        receive_data = {};
        DM_Motor::Motor_Control::receive();
    }
};

DM_Motor::DM_Motor_Type parse_motor_type(const std::string& type) {
    if (type == "DM4310") return DM_Motor::DM4310;
    if (type == "DM4310_48V") return DM_Motor::DM4310_48V;
    if (type == "DM4340") return DM_Motor::DM4340;
    if (type == "DM4340_48V") return DM_Motor::DM4340_48V;
    if (type == "DM6006") return DM_Motor::DM6006;
    if (type == "DM8006") return DM_Motor::DM8006;
    if (type == "DM8009") return DM_Motor::DM8009;
    if (type == "DM10010L") return DM_Motor::DM10010L;
    if (type == "DM10010") return DM_Motor::DM10010;
    if (type == "DMH3510") return DM_Motor::DMH3510;
    if (type == "DMH6215") return DM_Motor::DMH6215;
    if (type == "DMG6220") return DM_Motor::DMG6220;
    throw std::runtime_error("Unsupported motor type: " + type);
}

speed_t parse_baud_rate(unsigned int baud_rate) {
    if (baud_rate == 921600) return B921600;
    if (baud_rate == 460800) return B460800;
    if (baud_rate == 230400) return B230400;
    if (baud_rate == 115200) return B115200;
    throw std::runtime_error(
        "Unsupported baud rate in config: " + std::to_string(baud_rate));
}

void validate_motor_ids(const ArmConfig& config) {
    std::unordered_set<uint32_t> registered_ids;
    for (const MotorConfig& motor : config.motors) {
        if (motor.slave_id == 0 || motor.master_id == 0) {
            throw std::runtime_error(
                "Motor IDs must be non-zero for " + motor.name);
        }
        if (!registered_ids.insert(motor.slave_id).second) {
            throw std::runtime_error(
                "Duplicate motor ID in config: " +
                std::to_string(motor.slave_id));
        }
        if (!registered_ids.insert(motor.master_id).second) {
            throw std::runtime_error(
                "Duplicate motor ID in config: " +
                std::to_string(motor.master_id));
        }
    }
}

bool has_valid_feedback(const DM_Motor::Motor& motor) {
    return std::isfinite(motor.Get_Position()) &&
           std::isfinite(motor.Get_Velocity()) &&
           std::isfinite(motor.Get_tau());
}

bool read_fresh_status(PassiveMotorControl& motor_control,
                       DM_Motor::Motor& motor) {
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    motor.receive_data(invalid, invalid, invalid);
    for (int attempt = 0; attempt < QUERY_ATTEMPTS; ++attempt) {
        motor_control.refresh_motor_status(motor);
        if (has_valid_feedback(motor)) return true;

        for (int poll = 0; poll < POLLS_PER_ATTEMPT; ++poll) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            motor_control.receive();
            if (has_valid_feedback(motor)) return true;
        }
    }
    return false;
}

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [config_path]\n"
              << "       " << program << " --config config_path\n"
              << "Default: configs/default_right.yml\n";
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && (std::string(argv[1]) == "-h" ||
                      std::string(argv[1]) == "--help")) {
        print_usage(argv[0]);
        return 0;
    }

    std::string config_path = "configs/default_right.yml";
    if (argc == 2) {
        config_path = argv[1];
    } else if (argc == 3 && std::string(argv[1]) == "--config") {
        config_path = argv[2];
    } else if (argc != 1) {
        print_usage(argv[0]);
        return 2;
    }

    try {
        const ArmConfig config = load_arm_config(config_path);
        if (config.motors.size() != NUM_JOINTS) {
            throw std::runtime_error(
                "Expected exactly 7 motors in config, found " +
                std::to_string(config.motors.size()));
        }
        validate_motor_ids(config);

        auto serial = std::make_shared<SerialPort>(
            config.can.id, parse_baud_rate(config.can.baud_rate));
        PassiveMotorControl motor_control(serial);

        std::array<std::unique_ptr<DM_Motor::Motor>, NUM_JOINTS> motor_storage;
        std::array<DM_Motor::Motor*, NUM_JOINTS> motors{};
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            const MotorConfig& motor_config = config.motors[i];
            motor_storage[i] = std::make_unique<DM_Motor::Motor>(
                parse_motor_type(motor_config.type), motor_config.slave_id,
                motor_config.master_id);
            motors[i] = motor_storage[i].get();
            motor_control.addMotor(motors[i]);
        }

        std::cout << "Passive motor status read\n"
                  << "Config: " << config_path << "\n"
                  << "No motors will be enabled, disabled, or commanded.\n\n";
        std::cout << "index,name,slave_id,position_rad,velocity_rad_s,torque_nm,"
                     "reset_pose_rad,delta_rad\n";

        bool all_received = true;
        for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
            DM_Motor::Motor& motor = *motors[i];
            const MotorConfig& motor_config = config.motors[i];
            if (!read_fresh_status(motor_control, motor)) {
                all_received = false;
                std::cerr << "READ TIMEOUT: joint " << i << " ("
                          << motor_config.name << ", slave_id=0x" << std::hex
                          << motor_config.slave_id << std::dec
                          << ") did not return a valid status frame.\n";
                continue;
            }

            const float position = motor.Get_Position();
            const float delta = position - motor_config.reset_pose;
            std::cout << i << ',' << motor_config.name << ",0x" << std::hex
                      << motor_config.slave_id << std::dec << ',' << std::fixed
                      << std::setprecision(6) << position << ','
                      << motor.Get_Velocity() << ',' << motor.Get_tau() << ','
                      << motor_config.reset_pose << ',' << delta << '\n';

            if (std::fabs(delta) > RESET_WARNING_THRESHOLD_RAD) {
                std::cout << "WARNING: joint " << i << " ("
                          << motor_config.name << ") is " << std::fabs(delta)
                          << " rad away from reset_pose; threshold is "
                          << RESET_WARNING_THRESHOLD_RAD
                          << " rad. This is informational only.\n";
            }
        }

        return all_received ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Failed to read motor status: " << error.what() << '\n';
        return 1;
    }
}
