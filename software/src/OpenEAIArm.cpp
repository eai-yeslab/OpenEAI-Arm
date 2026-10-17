#include "OpenEAIArm.hpp"
#include <cctype>
#include <cmath>

namespace {
std::string motor_log_path(const std::string& arm_name) {
    if (arm_name.empty()) {
        return "motor_state_log.csv";
    }
    std::string suffix = arm_name;
    for (char& character : suffix) {
        const unsigned char value = static_cast<unsigned char>(character);
        if (!std::isalnum(value) && character != '_' && character != '-') {
            character = '_';
        }
    }
    return "motor_state_log_" + suffix + ".csv";
}
}

/*
    * @description: Constructor that loads configuration from a file
    * @param config_path: Path to the configuration file
    * @param control_mode: Control mode for the arm
*/
OpenEAIArm::OpenEAIArm(const std::string& config_path, ControlMode control_mode,
                       const std::string& arm_name):
    mode_(control_mode)
{
    std::cout << "Loading config file from " << config_path << std::endl;
    ArmConfig config = load_arm_config(config_path);
    gripper_version_ = config.gripper_version;

    if (mode_ == ControlMode::SIM) {
        serial_ = nullptr;
        motor_ctl_ = std::make_unique<DM_Motor::SimMotorControl>(nullptr);
    } else {
        serial_ = std::make_shared<SerialPort>(config.can.id, config.can.baud_rate);
        motor_ctl_ = std::make_unique<DM_Motor::Motor_Control>(serial_);
    }

    JointArray joint_amax;
    JointArray coriolis_scale, inertia_scale;

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        auto motor = config.motors[i];
        DM_Motor::DM_Motor_Type type = DM_Motor::DM4310;
        if (motor.type == "DM4340") type = DM_Motor::DM4340;
        else if (motor.type == "DM4310") type = DM_Motor::DM4310;
        DM_Motor::Motor* m = new DM_Motor::Motor(type, motor.slave_id, motor.master_id);
        motors_[i] = m;
        motor_ctl_->addMotor(m);

        joint_names[i] = motor.name;
        joint_limits_low_[i] = motor.min_position;
        joint_limits_high_[i] = motor.max_position;
        reset_pose_[i] = motor.reset_pose;
        joint_vmax_[i] = motor.motion_limits.vmax;
        joint_amax[i] = motor.motion_limits.amax;
        kp_[i] = motor.dynamic_coeff.kp;
        kd_[i] = motor.dynamic_coeff.kd;
        ki_[i] = motor.dynamic_coeff.ki;
        friction_[i] = motor.dynamic_coeff.friction;
        friction_alpha_[i] = motor.dynamic_coeff.friction_alpha;
        coriolis_scale[i] = motor.dynamic_coeff.coriolis_scale;
        inertia_scale[i] = motor.dynamic_coeff.inertia_scale;
        drag_stationary_kp_[i] = motor.dynamic_coeff.drag.stationary_kp;
        drag_stationary_kd_[i] = motor.dynamic_coeff.drag.stationary_kd;
        drag_moving_kd_[i] = motor.dynamic_coeff.drag.moving_kd;
        drag_moving_tanh_[i] = motor.dynamic_coeff.drag.moving_tanh;
        drag_moving_tanh_alpha_[i] = motor.dynamic_coeff.drag.moving_tanh_alpha;
    }

    enable_all();
    InterpolationConfig interpolation_config = config.controller.interpolation;
    float mm_alpha = config.controller.filter_alpha;
    drag_stationary_vel_threshold_ = config.controller.drag_stationary_vel_threshold;
    manager_ = std::make_unique<MotorManager>(motors_, motor_ctl_.get(), interpolation_config,
                                              config.controller.motion_filter,
                                              joint_vmax_, joint_amax, mm_alpha,
                                              config.controller.frequency,
                                              motor_log_path(arm_name));

    kd_solver = std::make_shared<KDSolver>(config.urdf.path, config.urdf.base_link, config.urdf.ee_link);

    float dt = manager_->getControlCycle();
    pid_controller = std::make_unique<AdvancedPIDController>(dt, kd_solver);
    pid_controller->setResetPose(reset_pose_);
    pid_controller->setKpKiParams(friction_, ki_);
    pid_controller->setNonlinearParams(friction_alpha_, 0.001f);
    pid_controller->setDynamicsParams(
        coriolis_scale, inertia_scale, joint_amax);

    if (mode_ == ControlMode::MIT_MIX) {
        manager_->compute_full_dynamic_torque =
            [this](const JointArray& q, const JointArray& target_q,
                   const JointArray& target_dq, const JointArray& target_ddq) {
                return this->pid_controller->computeControlTorque(
                    q, target_q, target_dq, target_ddq);
            };
    }
    else if (mode_ == ControlMode::MIT_DRAG) {
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->compute_drag_dynamics(q, target_q); };
    }
    manager_->setEnableSafetyCheck(config.controller.enable_safety_check);

    std::cout << "OpenEAIArm initialized and ready." << std::endl;
}

OpenEAIArm::~OpenEAIArm() {
    if (manager_) manager_->stop();
    // if (motor_ctl_) disable_all();
    for (auto* m : motors_) delete m;
}

void OpenEAIArm::enable_all() {
    for (auto* m : motors_) {
        motor_ctl_->enable(*m);
    }
}

void OpenEAIArm::disable_all() {
    for (auto* m : motors_) motor_ctl_->disable(*m);
}

void OpenEAIArm::joint_step(const JointArray& joint_angles, 
                         const JointArray& joint_velocities, 
                         const JointArray& joint_torques)
{
    JointArray q = joint_angles;
    JointArray dq = joint_velocities;
    JointArray tau = joint_torques;
    JointArray clipped_q;
    for (size_t i=0; i<NUM_JOINTS; ++i)
        clipped_q[i] = std::min(std::max(q[i], joint_limits_low_[i]), joint_limits_high_[i]);
    manager_->move(clipped_q, dq, tau);
}

void OpenEAIArm::go_home(float move_time)
{
    manager_->move(reset_pose_, std::nullopt, std::nullopt, kp_, kd_, move_time);
    std::this_thread::sleep_for(std::chrono::duration<float>(move_time));
}

void OpenEAIArm::reset(float move_time, float angle_jump_threshold) {
    JointArray current_joints = get_joint_positions();
    int first_jump_joint_index = -1;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        if (std::fabs(current_joints[i]) > angle_jump_threshold) {
            first_jump_joint_index = static_cast<int>(i);
            break;
        }
    }
    if (first_jump_joint_index != -1) {
        char cont = 'n';
        std::cout << "Error! Large Angle Jump Detected. Check if the robot is in a safe state and the configuration file is correct." << std::endl;
        std::cout << "Joint " << first_jump_joint_index + 1 << " has a jump of " << current_joints[first_jump_joint_index]
                  << " rad which exceeds the threshold of " << angle_jump_threshold << " rad." << std::endl;
        std::cout << "Continue? (y/n): ";
        std::cin >> cont;
        if (cont != 'y' && cont != 'Y') {
            disable_all();
            exit(-1);
        }
    }
    go_home(move_time);
}

OpenEAIArm::JointArray OpenEAIArm::get_joint_positions() const {
    return q_phys_to_control(manager_->getPosition());
}
OpenEAIArm::JointArray OpenEAIArm::get_joint_velocities() const {
    return manager_->getVelocity();
}
OpenEAIArm::JointArray OpenEAIArm::get_joint_torques() const {
    return manager_->getTau();
}
OpenEAIArm::JointArray OpenEAIArm::get_ee_pose() const {
    JointArray joints = q_phys_to_control(manager_->getPosition());
    return forward_kinetics(joints);
}

std::array<std::string, OpenEAIArm::NUM_JOINTS> OpenEAIArm::get_joint_names() const {
    return joint_names;
}

OpenEAIArm::JointArray OpenEAIArm::forward_kinetics(JointArray joint) const {
    std::vector<double> joints_in(joint.begin(), joint.end() - 1);
    KDL::Frame ee_pose_out;
    bool success = kd_solver->forwardKinematics(joints_in, ee_pose_out);
    JointArray ee_pose{};
    if (success) {
        ee_pose[0] = ee_pose_out.p.x();
        ee_pose[1] = ee_pose_out.p.y();
        ee_pose[2] = ee_pose_out.p.z();
        double rx, ry, rz;
        ee_pose_out.M.GetEulerZYX(rz, ry, rx);
        ee_pose[3] = static_cast<float>(rx);
        ee_pose[4] = static_cast<float>(ry);
        ee_pose[5] = static_cast<float>(rz);       
    }
    else {
        std::cout << "Something wrong!" << std::endl;
    }
    ee_pose[NUM_JOINTS - 1] = joint[NUM_JOINTS - 1];
    return ee_pose;
}

OpenEAIArm::JointArray OpenEAIArm::inverse_kinetics(JointArray ee_pose, bool& success, IKOptions options) const {
    return inverse_kinetics(ee_pose, get_joint_positions(), success, options);
}

OpenEAIArm::JointArray OpenEAIArm::inverse_kinetics(
    JointArray ee_pose, const JointArray& seed_joints, bool& success,
    IKOptions options) const {
    static int index = 0;
    std::vector<double> joints_out;
    std::vector<double> initial_joints(seed_joints.begin(), seed_joints.end() - 1);
    KDL::Frame ee_pose_in;
    ee_pose_in.p = KDL::Vector(ee_pose[0],ee_pose[1],ee_pose[2]);
    ee_pose_in.M = KDL::Rotation::EulerZYX(ee_pose[5], ee_pose[4], ee_pose[3]);
    
    success = kd_solver->inverseKinematics(ee_pose_in, joints_out, initial_joints);
    JointArray ik_result{};
    if (success) {
        std::transform(joints_out.begin(), joints_out.begin() + (NUM_JOINTS - 1), ik_result.begin(),
               [](double x) { return static_cast<float>(x); });

        float max_jump = 0.0f;
        JointArray delta_ik_result = {};
        for (size_t i = 0; i < NUM_JOINTS - 1; ++i) {
            delta_ik_result[i] = ik_result[i] - seed_joints[i];
            if (fabs(delta_ik_result[i]) > max_jump) {
                max_jump = fabs(delta_ik_result[i]);
            }
        }

        if (max_jump > options.angle_jump_threshold) {
            switch(options.policy) {
            case IKJumpPolicy::REJECT:
                success = false;
                break;
            case IKJumpPolicy::CLIP: {
                std::cout << "IK clipping from " << seed_joints << " to " << ik_result << std::endl;
                JointArray result = seed_joints;
                for (size_t i = 0; i < NUM_JOINTS - 1; ++i) {
                    if (fabs(delta_ik_result[i]) > options.angle_jump_threshold) {
                        result[i] = seed_joints[i] + options.fixed_step * (delta_ik_result[i] > 0 ? 1 : -1);
                    } else {
                        result[i] = ik_result[i];
                    }
                }
                ik_result = result;
                break;
            }
            case IKJumpPolicy::ACCEPT:
                // Do nothing, accept the result as is
                break;
            }
        }

    }
    if (!success) {
        if (options.report_failure) {
            std::cout << "[" << index << "]" << "IK failed!" << std::endl;
        }
        ik_result = seed_joints;
        index++;
    }
    ik_result[NUM_JOINTS - 1] = ee_pose[NUM_JOINTS - 1];
    return ik_result;
}

float OpenEAIArm::gripper_width_to_joint(const float gripper_width) const {
    if (gripper_version_ == "raw") {
        return gripper_width;
    }
    if (gripper_version_ == "v2") {
        constexpr float R = 28.0f, L = 35.0f, H = 21.0f, S0 = 15.3f;
        constexpr float ALPHA0 = 2.350360787f, MAX_WIDTH = 0.082f;
        const float width = std::clamp(gripper_width, 0.0f, MAX_WIDTH);
        const float s = S0 + width * 500.0f;
        const float d = std::hypot(s, H);
        const float c = std::clamp(
            (R * R + d * d - L * L) / (2.0f * R * d), -1.0f, 1.0f);
        return -(ALPHA0 - std::atan2(H, s) - std::acos(c));
    }

    constexpr float theta = 1.702582745f;
    float single_width = gripper_width * 500.0f + 23.0f; // Convert from m to mm
    float width_cos = (single_width * single_width - 698.25f) / (56.0f * single_width);
    if (width_cos > 1.0f) width_cos = 1.0f;
    else if (width_cos < -1.0f) width_cos = -1.0f;
    float joint = theta - acos(width_cos);
    return joint;
}
float OpenEAIArm::joint_to_gripper_width(const float joint) const {
    if (gripper_version_ == "raw") {
        return joint;
    }
    if (gripper_version_ == "v2") {
        constexpr float R = 28.0f, L = 35.0f, H = 21.0f, S0 = 15.3f;
        constexpr float ALPHA0 = 2.350360787f;
        constexpr float MAX_JOINT = 1.650975960f, MAX_WIDTH = 0.082f;
        const float alpha = ALPHA0 - std::clamp(-joint, 0.0f, MAX_JOINT);
        const float ay = R * std::sin(alpha);
        const float s = R * std::cos(alpha) +
                        std::sqrt(std::max(0.0f, L * L - (H - ay) * (H - ay)));
        return std::clamp((s - S0) / 500.0f, 0.0f, MAX_WIDTH);
    }

    constexpr float theta = 1.702582745f;
    constexpr float MAX_WIDTH = 0.08f;
    float width = 2.0f * (28.0f * cos(theta - joint) + sqrt(784.0f * cos(theta - joint) * cos(theta - joint) + 698.25) - 23) / 1000.0f;
    if (width > 0.08f) width = MAX_WIDTH;
    else if (width < 0.0f) width = 0.0f;
    return width;
}

// control angles -> physical motor angles
OpenEAIArm::JointArray OpenEAIArm::q_control_to_phys(const JointArray& q) const {
    JointArray q_phys;
    size_t i = 0;
    for (; i < NUM_JOINTS - 1; ++i) {
        q_phys[i] = q[i] + reset_pose_[i];
    }
    q_phys[i] = reset_pose_[i] + gripper_width_to_joint(q[i]);
    return q_phys;
}

// physical motor angles -> control angles
OpenEAIArm::JointArray OpenEAIArm::q_phys_to_control(const JointArray& q_phys) const {
    JointArray q;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        q[i] = q_phys[i] - reset_pose_[i];
    }
    q[NUM_JOINTS - 1] = joint_to_gripper_width(q[NUM_JOINTS - 1]);
    return q;
}

/*
    * @description: Set joint targets with various control modes
    * @param target: Target joint positions
    * @param move_time: Time to move to the target, default is 0.0s
    * @param vel: Target joint velocities (optional)
    * @param tau: Target joint torques (optional)
    * @param interpolate: Whether to interpolate the movement, default is false
*/
void OpenEAIArm::set_joint_targets(const JointArray& target, float move_time,
                               const JointArray& vel, const JointArray& tau,
                               bool interpolate)
{
    JointArray clipped_target;
    for (size_t i=0; i<NUM_JOINTS; ++i) {
        clipped_target[i] = std::min(std::max(target[i], joint_limits_low_[i]), joint_limits_high_[i]);
    }
    JointArray physical_target = q_control_to_phys(clipped_target);
    if (!interpolate)
        move_time = 0.0f;
    switch(mode_) {
        case ControlMode::MIT_POS:
            manager_->move(physical_target, manager_->uniform(1.0f), std::nullopt, kp_, kd_, move_time);
            break;
        case ControlMode::MIT_MIX:
        case ControlMode::SIM:
            manager_->move(physical_target, manager_->uniform(0.0f), std::nullopt, kp_, kd_, move_time);
            break;
        case ControlMode::MIT_DRAG:
            send_drag_command();
            break;
    }
}

void OpenEAIArm::set_joint_targets(const JointArray& target, float move_time, bool interpolate,
                               const JointArray& vel, const JointArray& tau) {
    set_joint_targets(target, move_time, vel, tau, interpolate);
                               }

void OpenEAIArm::set_control_mode(ControlMode mode) {
    if (mode_ == mode) return;

    if (mode == ControlMode::SIM || mode_ == ControlMode::SIM) {
        std::cerr << "Warning: Switching to/from SIM mode is not supported at runtime." << std::endl;
        return;
    }

    mode_ = mode;

    if (mode_ == ControlMode::MIT_MIX) {
        pid_controller->setKpKiParams(friction_, ki_);
        pid_controller->setNonlinearParams(friction_alpha_, 0.001f);
        manager_->compute_dynamic_torque = nullptr;
        manager_->compute_full_dynamic_torque =
            [this](const JointArray& q, const JointArray& target_q,
                   const JointArray& target_dq, const JointArray& target_ddq) {
                return this->pid_controller->computeControlTorque(
                    q, target_q, target_dq, target_ddq);
            };
        // Hold the current pose while re-entering program control.
        manager_->move(manager_->getPosition(), manager_->uniform(0.0f),
                       std::nullopt, kp_, kd_, 0.0f);
    }
    else if (mode_ == ControlMode::MIT_DRAG) {
        manager_->compute_full_dynamic_torque = nullptr;
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q) {
            return this->compute_drag_dynamics(q, target_q);
        };
        // Hold the current pose with zero velocity while entering drag teaching.
        set_joint_targets(get_joint_positions(), 0.0f, manager_->uniform(0.0f),
                          JointArray{}, false);
    }
}

void OpenEAIArm::send_drag_command() {
    auto current_vel = get_joint_velocities();
    bool is_stationary = true;
    for (const auto& v : current_vel) {
        if (std::fabs(v) > drag_stationary_vel_threshold_) {
            is_stationary = false;
            break;
        }
    }

    if (is_stationary)
        manager_->move(manager_->getPosition(), manager_->uniform(0.0f), std::nullopt, drag_stationary_kp_, drag_stationary_kd_, 0.0f);
    else
        manager_->move(manager_->getPosition(), manager_->uniform(0.0f), std::nullopt, manager_->uniform(0.0f), manager_->uniform(0.0f), 0.0f);
}

OpenEAIArm::JointArray OpenEAIArm::compute_drag_dynamics(OpenEAIArm::JointArray q, OpenEAIArm::JointArray target_q, bool output) {
    if (output) std::cout << "============== Tau args ==============" << std::endl;
    q = q_phys_to_control(q);
    target_q = q_phys_to_control(target_q);
    auto current_vel = get_joint_velocities();

    std::vector<double> q_in(q.begin(), q.end() - 1), t_out;

    bool success = kd_solver->gravityCompensation(q_in, t_out);
    JointArray tau{};
    if (success) {
        if (output) {
            std::cout << "Gravity compensation: ";
            for (auto v : t_out) std::cout << v << " "; std::cout << std::endl;
        }
        std::transform(t_out.begin(), t_out.begin() + (NUM_JOINTS - 1), tau.begin(),
               [](double x) { return static_cast<float>(x); });
        // Add drag compensation for more stable and smoother control
        for (int i = 1; i < NUM_JOINTS; i++) {
            tau[i] += drag_moving_tanh_[i] * tanh(drag_moving_tanh_alpha_[i] * current_vel[i]);
            tau[i] -= drag_moving_kd_[i] * current_vel[i];
        }
    }
    else {
        std::cout << "Something wrong!" << std::endl;
    }

    send_drag_command();

    return tau;
}
