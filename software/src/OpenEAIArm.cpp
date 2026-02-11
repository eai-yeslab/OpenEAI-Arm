#include "OpenEAIArm.hpp"

std::ostream& operator<<(std::ostream& os, const OpenEAIArm::JointArray& arr) {
    os << "[";
    for (size_t i = 0; i < arr.size(); ++i) {
        os << arr[i];
        if (i != arr.size()-1) os << ", ";
    }
    os << "]";
    return os;
}

/*
    * @description: Constructor that loads configuration from a file
    * @param config_path: Path to the configuration file
    * @param control_mode: Control mode for the arm
*/
OpenEAIArm::OpenEAIArm(const std::string& config_path, ControlMode control_mode): 
    mode_(control_mode)
{
    std::cout << "Loading config file from " << config_path << std::endl;
    config = load_arm_config(config_path);

    serial_ = std::make_shared<SerialPort>(config.can.id, config.can.baud_rate);
    motor_ctl_ = std::make_unique<DM_Motor::Motor_Control>(serial_);


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
        kp_[i] = motor.dynamic_coeff.kp;
        kd_[i] = motor.dynamic_coeff.kd;
        drag_stationary_kp_[i] = motor.dynamic_coeff.drag.stationary_kp;
        drag_stationary_kd_[i] = motor.dynamic_coeff.drag.stationary_kd;
        drag_moving_kd_[i] = motor.dynamic_coeff.drag.moving_kd;
        drag_moving_tanh_[i] = motor.dynamic_coeff.drag.moving_tanh;
        drag_moving_tanh_alpha_[i] = motor.dynamic_coeff.drag.moving_tanh_alpha;
    }

    drag_stationary_vel_threshold_ = config.controller.drag_stationary_vel_threshold;


    initialize_arm(true);

    std::cout << "OpenEAIArm initialized and ready." << std::endl;
}


void OpenEAIArm::initialize_arm(bool reinitialize_manager) {
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        if (mode_ != ControlMode::MIT_DRAG) {
            ki_[i] = config.motors[i].dynamic_coeff.ki;
            friction_[i] = config.motors[i].dynamic_coeff.friction;
            friction_alpha_[i] = config.motors[i].dynamic_coeff.friction_alpha;
        }
        else {
            ki_[i] = 0;
            friction_[i] = 0;
            friction_alpha_[i] = 0;
        }
    }

    std::string interpolation_method = config.controller.interpolation;
    MotorManager::InterpolationMethod mmim;
    if (interpolation_method == "linear") mmim = MotorManager::InterpolationMethod::Linear;
    else if (interpolation_method == "cubics") mmim = MotorManager::InterpolationMethod::CubicS;
    else mmim = MotorManager::InterpolationMethod::MinJerk;
    float mm_alpha = config.controller.filter_alpha;

    if (reinitialize_manager)  {
        if (mode_ != ControlMode::SIM)
            enable_all();

        if (manager_) manager_->stop();
        switch (mode_) {
            case ControlMode::MIT_MIX:
            case ControlMode::MIT_DRAG:
                manager_ = std::make_unique<MotorManager>(motors_, motor_ctl_.get(), mmim, mm_alpha);
                break;
            case ControlMode::SIM:
                manager_ = std::make_unique<SimMotorManager>(config.urdf.path, reset_pose_);
                break;
        }
        
        kd_solver = std::make_shared<KDSolver>(config.urdf.path, config.urdf.base_link, config.urdf.ee_link);
        
        float dt = manager_->getControlCycle();
        pid_controller = std::make_unique<AdvancedPIDController>(dt, kd_solver);
        pid_controller->setResetPose(reset_pose_);
    }
    pid_controller->setKpKiParams(friction_, ki_);
    pid_controller->setNonlinearParams(friction_alpha_, 0.001f);

    if (mode_ == ControlMode::MIT_MIX) {
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->pid_controller->computeControlTorque(q, target_q); };
    }
    else if (mode_ == ControlMode::MIT_DRAG) {
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->compute_drag_dynamics(q, target_q); };
    }
    manager_->setEnableSafetyCheck(config.controller.enable_safety_check);
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
    manager_->move(clipped_q);
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
    static int index = 0;
    std::vector<double> joints_out;
    auto current_joint_raw = get_joint_positions();
    std::vector<double> current_joints(current_joint_raw.begin(), current_joint_raw.end());
    KDL::Frame ee_pose_in;
    ee_pose_in.p = KDL::Vector(ee_pose[0],ee_pose[1],ee_pose[2]);
    ee_pose_in.M = KDL::Rotation::EulerZYX(ee_pose[5], ee_pose[4], ee_pose[3]);
    
    success = kd_solver->inverseKinematics(ee_pose_in, joints_out, current_joints);
    JointArray ik_result{};
    if (success) {
        std::transform(joints_out.begin(), joints_out.begin() + (NUM_JOINTS - 1), ik_result.begin(),
               [](double x) { return static_cast<float>(x); });

        float max_jump = 0.0f;
        size_t idx_max = 0;
        JointArray delta_ik_result = {};
        for (size_t i = 0; i < NUM_JOINTS - 1; ++i) {
            delta_ik_result[i] = ik_result[i] - current_joint_raw[i];
            if (fabs(delta_ik_result[i]) > max_jump) {
                max_jump = fabs(delta_ik_result[i]);
                idx_max = i;
            }
        }

        if (max_jump > options.angle_jump_threshold) {
            switch(options.policy) {
            case IKJumpPolicy::REJECT:
                success = false;
                break;
            case IKJumpPolicy::CLIP: {
                std::cout << "IK clipping from " << current_joint_raw << " to " << ik_result << std::endl;
                JointArray result = current_joint_raw;
                for (size_t i = 0; i < NUM_JOINTS - 1; ++i) {
                    if (fabs(delta_ik_result[i]) > options.angle_jump_threshold) {
                        result[i] = current_joint_raw[i] + options.fixed_step * (delta_ik_result[i] > 0 ? 1 : -1);
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
        std::cout << "[" << index << "]" << "IK got wrong!" << std::endl;
        ik_result = current_joint_raw;
        index++;
    }
    ik_result[NUM_JOINTS - 1] = ee_pose[NUM_JOINTS - 1];
    return ik_result;
}

void OpenEAIArm::set_control_mode(ControlMode mode) {
    if (mode_ == mode) return;
    if (mode == ControlMode::SIM || mode_ == ControlMode::SIM) {
        std::cerr << "Warning: Switching to/from SIM mode may cause unexpected behavior, and is not supported yet." << std::endl;
        std::cerr << "The robot is stopped. Please restart the program." << std::endl;
        if (manager_) manager_->stop();
    }
    bool reinitialize_manager = false;
    if ((mode_ == ControlMode::SIM && mode != ControlMode::SIM) ||
        (mode_ != ControlMode::SIM && mode == ControlMode::SIM)) {
        reinitialize_manager = true;
    }
    mode_ = mode;
    initialize_arm(reinitialize_manager);
    if (mode_ != ControlMode::MIT_DRAG) {
        manager_->setPosition(manager_->getPosition());
    }
    else {
        set_joint_targets(get_joint_positions(), 0.0f, manager_->uniform(0.0f), {}, false);
    }
}

float OpenEAIArm::gripper_width_to_joint(const float gripper_width) const {
    constexpr float theta = 1.702582745f;
    float single_width = gripper_width * 500.0f + 23.0f; // Convert from m to mm
    float width_cos = (single_width * single_width - 698.25f) / (56.0f * single_width);
    if (width_cos > 1.0f) width_cos = 1.0f;
    else if (width_cos < -1.0f) width_cos = -1.0f;
    float joint = theta - acos(width_cos);
    return joint;
}
float OpenEAIArm::joint_to_gripper_width(const float joint) const {
    constexpr float theta = 1.702582745f;
    constexpr float MAX_WIDTH = 0.82f;
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
        case ControlMode::MIT_MIX:
        case ControlMode::SIM:
            manager_->move(physical_target, manager_->uniform(0.0f), std::nullopt, kp_, kd_, move_time);
            break;
        case ControlMode::MIT_DRAG:
            send_drag_command();
            break;
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
        if (mode_ == ControlMode::MIT_MIX) {
            int error_tolenrance = 0.01f;
            for (int i = 0; i < 3; i++) {
                if (target_q[i] - q[i] > error_tolenrance) tau[i] += 0.5f;
                else if (target_q[i] - q[i] < -error_tolenrance) tau[i] -= 0.5f;
            }
        }
        else if (mode_ == ControlMode::MIT_DRAG) { // Add drag compensation for more stable and smoother control
            auto current_vel = get_joint_velocities();
            auto current_tau = get_joint_torques();
            for (int i = 1; i < NUM_JOINTS; i++) {
                tau[i] += drag_moving_tanh_[i] * tanh(drag_moving_tanh_alpha_[i] * current_vel[i]);
                tau[i] -= drag_moving_kd_[i] * current_vel[i];
            }
        }
    }
    else {
        std::cout << "Something wrong!" << std::endl;
    }

    send_drag_command();

    return tau;
}