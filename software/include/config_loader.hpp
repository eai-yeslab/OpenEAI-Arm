#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <iostream>
#include <cstddef>
#include <cstdint>

struct DragDynamicsCoefficients {
    float stationary_kp = 0.0f;
    float stationary_kd = 0.0f;
    float moving_kd = 0.0f;
    float moving_tanh = 0.0f;
    float moving_tanh_alpha = 1.0f;
};

struct DynamicCoefficients {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, friction = 0.0f, friction_alpha = 1.0f;
    float coriolis_scale = 1.0f;
    float inertia_scale = 1.0f;
    DragDynamicsCoefficients drag;
};

struct MotionLimits {
    float vmax = 2.5f;
    float amax = 50.0f;
};

struct MotorConfig {
    std::string name;
    std::string type;
    uint32_t master_id = 0;
    uint32_t slave_id = 0;
    float min_position = 0.0f;
    float max_position = 0.0f;
    float reset_pose = 0.0f;
    MotionLimits motion_limits;
    DynamicCoefficients dynamic_coeff;
};

struct URDFConfig {
    std::string path;
    std::string base_link;
    std::string ee_link;
};

struct CANConfig {
    std::string id;
    unsigned int baud_rate = 921600;
};

struct InterpolationConfig {
    std::string method = "minjerk";
    // Legacy scalar limits remain available to standalone applications.
    float vmax = 2.5f;
    float amax = 50.0f;
};

struct InputShaperConfig {
    bool enabled = false;
    std::string type = "zv";
    float frequency_hz = 2.6f;
    float damping_ratio = 0.0f;
    float bandwidth_hz = 1.0f;
};

struct MotionFilterConfig {
    bool enabled = true;
    float position_tolerance = 0.001f;
    float velocity_tolerance = 0.01f;
    bool send_velocity = false;
    std::size_t delay_frames = 0;
    InputShaperConfig input_shaper;
};

struct ControllerConfig {
    InterpolationConfig interpolation;
    MotionFilterConfig motion_filter;
    float filter_alpha = 0.9f;
    bool enable_safety_check = false;
    float frequency = 200.0f;
    // Kept for public drag-mode API compatibility.
    float drag_stationary_vel_threshold = 0.25f;
};

struct ArmConfig {
    CANConfig can;
    URDFConfig urdf;
    std::string gripper_version = "v1";
    ControllerConfig controller;
    std::vector<MotorConfig> motors;
};

ArmConfig load_arm_config(const std::string& file);
