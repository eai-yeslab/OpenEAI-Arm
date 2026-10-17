#include "config_loader.hpp"
#include <stdexcept>

ArmConfig load_arm_config(const std::string& file) {
    YAML::Node config = YAML::LoadFile(file);
    ArmConfig arm;

    arm.can.id = config["can_config"]["id"].as<std::string>();
    arm.can.baud_rate = config["can_config"]["baud_rate"].as<unsigned int>();

    arm.urdf.path = config["urdf"]["path"].as<std::string>();
    arm.urdf.base_link = config["urdf"]["base_link"].as<std::string>();
    arm.urdf.ee_link = config["urdf"]["ee_link"].as<std::string>();

    if (config["gripper_version"]) {
        arm.gripper_version = config["gripper_version"].as<std::string>();
    }
    if (arm.gripper_version != "raw" && arm.gripper_version != "v1" &&
        arm.gripper_version != "v2") {
        throw std::runtime_error("gripper_version must be 'raw', 'v1', or 'v2'");
    }

    const auto controller = config["controller"];
    const auto interpolation = controller["interpolation"];
    if (interpolation.IsScalar()) {
        arm.controller.interpolation.method = interpolation.as<std::string>();
    } else if (interpolation["method"]) {
        arm.controller.interpolation.method = interpolation["method"].as<std::string>();
    }
    if (arm.controller.interpolation.method == "sigmoid") {
        throw std::runtime_error(
            "interpolation method 'sigmoid' is not available in this build");
    }
    if (controller["filter_alpha"])
        arm.controller.filter_alpha = controller["filter_alpha"].as<float>();
    if (controller["enable_safety_check"])
        arm.controller.enable_safety_check = controller["enable_safety_check"].as<bool>();
    if (controller["frequency"])
        arm.controller.frequency = controller["frequency"].as<float>();
    if (controller["drag_stationary_vel_threshold"])
        arm.controller.drag_stationary_vel_threshold =
            controller["drag_stationary_vel_threshold"].as<float>();

    const auto motion_filter = controller["motion_filter"];
    if (motion_filter) {
        if (motion_filter["enabled"])
            arm.controller.motion_filter.enabled = motion_filter["enabled"].as<bool>();
        if (motion_filter["position_tolerance"])
            arm.controller.motion_filter.position_tolerance =
                motion_filter["position_tolerance"].as<float>();
        if (motion_filter["velocity_tolerance"])
            arm.controller.motion_filter.velocity_tolerance =
                motion_filter["velocity_tolerance"].as<float>();
        if (motion_filter["send_velocity"])
            arm.controller.motion_filter.send_velocity = motion_filter["send_velocity"].as<bool>();
        if (motion_filter["delay_frames"]) {
            const int parsed_delay_frames = motion_filter["delay_frames"].as<int>();
            if (parsed_delay_frames < 0)
                throw std::runtime_error("motion_filter delay_frames must be non-negative");
            arm.controller.motion_filter.delay_frames =
                static_cast<std::size_t>(parsed_delay_frames);
        }
        const auto input_shaper = motion_filter["input_shaper"];
        if (input_shaper) {
            if (input_shaper["enabled"])
                arm.controller.motion_filter.input_shaper.enabled = input_shaper["enabled"].as<bool>();
            if (input_shaper["type"])
                arm.controller.motion_filter.input_shaper.type = input_shaper["type"].as<std::string>();
            if (input_shaper["frequency_hz"])
                arm.controller.motion_filter.input_shaper.frequency_hz = input_shaper["frequency_hz"].as<float>();
            if (input_shaper["damping_ratio"])
                arm.controller.motion_filter.input_shaper.damping_ratio = input_shaper["damping_ratio"].as<float>();
            if (input_shaper["bandwidth_hz"])
                arm.controller.motion_filter.input_shaper.bandwidth_hz = input_shaper["bandwidth_hz"].as<float>();
        }
    } else {
        // Older public YAML files had no post-interpolation motion filter.
        arm.controller.motion_filter.enabled = false;
    }

    for (const auto& node : config["motors"]) {
        MotorConfig motor;
        motor.name = node["name"].as<std::string>();
        motor.type = node["type"].as<std::string>();
        motor.master_id = node["master_id"].as<uint32_t>();
        motor.slave_id = node["slave_id"].as<uint32_t>();
        motor.min_position = node["min_position"].as<float>();
        motor.max_position = node["max_position"].as<float>();
        motor.reset_pose = node["reset_pose"].as<float>();

        const auto limits = node["motion_limits"];
        if (limits) {
            if (limits["vmax"]) motor.motion_limits.vmax = limits["vmax"].as<float>();
            if (limits["amax"]) motor.motion_limits.amax = limits["amax"].as<float>();
        }
        if (motor.motion_limits.vmax <= 0.0f || motor.motion_limits.amax <= 0.0f)
            throw std::runtime_error("motion_limits vmax/amax must be positive for " + motor.name);

        const auto dynamic = node["dynamic_coefficients"];
        motor.dynamic_coeff.kp = dynamic["kp"].as<float>();
        motor.dynamic_coeff.ki = dynamic["ki"].as<float>();
        motor.dynamic_coeff.kd = dynamic["kd"].as<float>();
        motor.dynamic_coeff.friction = dynamic["friction"].as<float>();
        motor.dynamic_coeff.friction_alpha = dynamic["friction_alpha"].as<float>();
        if (dynamic["coriolis_scale"])
            motor.dynamic_coeff.coriolis_scale = dynamic["coriolis_scale"].as<float>();
        if (dynamic["inertia_scale"])
            motor.dynamic_coeff.inertia_scale = dynamic["inertia_scale"].as<float>();
        if (motor.dynamic_coeff.coriolis_scale < 0.0f || motor.dynamic_coeff.inertia_scale < 0.0f)
            throw std::runtime_error("dynamic coefficient scales must be non-negative for " + motor.name);

        const auto drag = dynamic["drag"];
        if (drag) {
            if (drag["stationary_kp"]) motor.dynamic_coeff.drag.stationary_kp = drag["stationary_kp"].as<float>();
            if (drag["stationary_kd"]) motor.dynamic_coeff.drag.stationary_kd = drag["stationary_kd"].as<float>();
            if (drag["moving_kd"]) motor.dynamic_coeff.drag.moving_kd = drag["moving_kd"].as<float>();
            if (drag["moving_tanh"]) motor.dynamic_coeff.drag.moving_tanh = drag["moving_tanh"].as<float>();
            if (drag["moving_tanh_alpha"]) motor.dynamic_coeff.drag.moving_tanh_alpha = drag["moving_tanh_alpha"].as<float>();
        }
        arm.motors.push_back(motor);
    }
    return arm;
}
