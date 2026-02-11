#include "config_loader.hpp"

ArmConfig load_arm_config(const std::string& file) {
    YAML::Node config = YAML::LoadFile(file);
    ArmConfig arm;

    // CAN
    arm.can.id = config["can_config"]["id"].as<std::string>();
    arm.can.baud_rate = config["can_config"]["baud_rate"].as<unsigned int>();

    // URDF
    arm.urdf.path = config["urdf"]["path"].as<std::string>();
    arm.urdf.base_link = config["urdf"]["base_link"].as<std::string>();
    arm.urdf.ee_link = config["urdf"]["ee_link"].as<std::string>();

    // Controller
    arm.controller.interpolation = config["controller"]["interpolation"].as<std::string>();
    arm.controller.filter_alpha = config["controller"]["filter_alpha"].as<float>();
    arm.controller.enable_safety_check = config["controller"]["enable_safety_check"].as<bool>();
    arm.controller.drag_stationary_vel_threshold = config["controller"]["drag_stationary_vel_threshold"].as<float>();

    // Motors
    for(const auto& node : config["motors"]) {
        MotorConfig m;
        m.name = node["name"].as<std::string>();
        m.type = node["type"].as<std::string>();
        m.master_id = node["master_id"].as<uint32_t>();
        m.slave_id = node["slave_id"].as<uint32_t>();
        m.min_position = node["min_position"].as<float>();
        m.max_position = node["max_position"].as<float>();
        m.reset_pose = node["reset_pose"].as<float>();
        // Dynamic coefficients
        auto dyn = node["dynamic_coefficients"];
        m.dynamic_coeff.kp = dyn["kp"].as<float>();
        m.dynamic_coeff.ki = dyn["ki"].as<float>();
        m.dynamic_coeff.kd = dyn["kd"].as<float>();
        m.dynamic_coeff.friction = dyn["friction"].as<float>();
        m.dynamic_coeff.friction_alpha = dyn["friction_alpha"].as<float>();
        
        auto drag = dyn["drag"];
        m.dynamic_coeff.drag.stationary_kp = drag["stationary_kp"].as<float>();
        m.dynamic_coeff.drag.stationary_kd = drag["stationary_kd"].as<float>();
        m.dynamic_coeff.drag.moving_kd = drag["moving_kd"].as<float>();
        m.dynamic_coeff.drag.moving_tanh = drag["moving_tanh"].as<float>();
        m.dynamic_coeff.drag.moving_tanh_alpha = drag["moving_tanh_alpha"].as<float>();
        arm.motors.push_back(m);
    }
    return arm;
}