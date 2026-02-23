#ifndef OPENEAIARM_HPP
#define OPENEAIARM_HPP

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>
#include <thread>
#include <iostream>
#include <cassert>
#include <iomanip>

#include "config_loader.hpp"
#include "dm.hpp"
#include "dm_manager.hpp"
#include "motor_controller.hpp"
#include "kd_solver.hpp"
#include "sim_motor_manager.hpp"

class OpenEAIArm {
public:
    enum class ControlMode {
        MIT_MIX,
        MIT_DRAG,
        SIM
    };

    enum class IKJumpPolicy {
        REJECT,      // Fail on jump over threshold
        CLIP,        // Move with fixed_step when jump over threshold
        ACCEPT       // Always accept (equivalent to original logic)
    };

    struct IKOptions {
        float angle_jump_threshold = 0.2f; // rad/s
        float fixed_step = 0.2f;           // rad/s, only valid if policy==CLIP
        IKJumpPolicy policy = IKJumpPolicy::REJECT;
        IKOptions() {}
    };

    static constexpr size_t NUM_JOINTS = 7;
    using JointArray = std::array<float, NUM_JOINTS>;
    using MotorPtrArray = std::array<DM_Motor::Motor*, NUM_JOINTS>;

    OpenEAIArm(const std::string& config_path, ControlMode control_mode = ControlMode::MIT_MIX);
    ~OpenEAIArm();

    // Joint step control (MIT mode), optional velocity and torque
    void joint_step(const JointArray& joint_angles, 
                    const JointArray& joint_velocities = JointArray{}, 
                    const JointArray& joint_torques = JointArray{});

    // Return to preset zero position (interpolation mode), parameters are interpolation steps and time
    void go_home(float move_time = 2);

    // Reset the robot arm. If there is a large angle jump, it will be rejected.
    void reset(float move_time = 2, float angle_jump_threshold = 0.28f);

    JointArray get_joint_positions()   const;
    JointArray get_joint_velocities()  const;
    JointArray get_joint_torques()     const;
    JointArray get_ee_pose()           const;
    std::array<std::string, NUM_JOINTS> get_joint_names() const;

    // FK, the output is {x, y, z, rx, ry, rz, gripper_width} which rx, ry, rz is the euler angle in ZYX rotation
    JointArray forward_kinetics(JointArray joint) const;
    // IK, the input should be {x, y, z, rx, ry, rz, gripper_width} which rx, ry, rz is the euler angle in ZYX rotation
    JointArray inverse_kinetics(JointArray ee_pose, bool& success, IKOptions options = IKOptions()) const;


    // Directly set joint targets (interpolatable), internally calls MotorManager
    void set_joint_targets(const JointArray& target, float move_time = 0.0,
                           const JointArray& vel = JointArray{},
                           const JointArray& tau = JointArray{},
                           bool interpolate = false);


    /*
    * @brief Switch control mode at runtime. Currently, switching to/from SIM mode is not fully tested and may cause unexpected behavior, so it is disabled.
    */
    void set_control_mode(ControlMode mode);


    float gripper_width_to_joint(const float gripper_width) const;
    float joint_to_gripper_width(const float joint) const;
    JointArray q_control_to_phys(const JointArray& q) const;
    JointArray q_phys_to_control(const JointArray& q_phys) const;
    JointArray compute_drag_dynamics(JointArray q, JointArray target_q = {}, bool output=false);
    void send_drag_command();

    // Joint limits
    JointArray lower_limits() const { return joint_limits_low_; }
    JointArray upper_limits() const { return joint_limits_high_; }

    // Motor object array and manager interface
    const MotorPtrArray& motors() const { return motors_; }
    BaseMotorManager& manager() { return *manager_; }

    // Disable all motors
    void disable_all();

    // Enable all motors
    void enable_all();

private:
    MotorPtrArray motors_;
    std::shared_ptr<SerialPort> serial_;
    std::unique_ptr<DM_Motor::Motor_Control> motor_ctl_;
    std::unique_ptr<BaseMotorManager> manager_;

    JointArray kp_, kd_;
    JointArray ki_, friction_, friction_alpha_;
    float drag_stationary_vel_threshold_;
    JointArray drag_stationary_kp_, drag_stationary_kd_, drag_moving_kd_, drag_moving_tanh_, drag_moving_tanh_alpha_;
    JointArray joint_limits_low_;
    JointArray joint_limits_high_;
    JointArray reset_pose_;
    std::array<std::string, NUM_JOINTS> joint_names;
    
    ControlMode mode_ = ControlMode::MIT_MIX;
    ArmConfig config;
    std::shared_ptr<KDSolver> kd_solver;
    std::unique_ptr<AdvancedPIDController> pid_controller;
    std::array<float, NUM_JOINTS> motor_mass={0.362, 0.362, 0.362, 0.3, 0.3, 0.3, 0.3};
    std::array<float, NUM_JOINTS> link_mass={0.089464, 0.8789, 0.71598, 0.23515, 0.16797, 0.02, 0.3 };
    std::array<float, NUM_JOINTS+1> motor_cg_z={-0.0267-0.046, 0, 0, 0, -0.023-0.048, 0, 0, -0.055};

    void initialize_arm(bool reinitialize_manager = true);

};

std::ostream& operator<<(std::ostream& os, const OpenEAIArm::JointArray& arr);

#endif // OPENEAIARM_HPP