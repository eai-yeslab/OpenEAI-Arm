#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <iostream>
struct DynamicCoefficients {
    float kp, ki, kd, friction, friction_alpha;
};

struct MotorConfig {
    std::string name;
    std::string type;
    uint32_t master_id;
    uint32_t slave_id;
    float min_position;
    float max_position;
    float reset_pose;
    DynamicCoefficients dynamic_coeff;
};

struct URDFConfig {
    std::string path;
    std::string base_link;
    std::string ee_link;
};

struct CANConfig {
    std::string id;
    unsigned int baud_rate;
};

struct ControllerConfig {
    std::string interpolation;
    float filter_alpha;
    bool enable_safety_check;
};

struct ArmConfig {
    CANConfig can;
    URDFConfig urdf;
    ControllerConfig controller;
    std::vector<MotorConfig> motors;
};

ArmConfig load_arm_config(const std::string& file);