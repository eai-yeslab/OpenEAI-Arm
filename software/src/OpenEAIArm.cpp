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

OpenEAIArm::OpenEAIArm(ControlMode control_mode, const std::string& serial_dev, speed_t baud):  // Deprecated
    mode_(control_mode)
{
    serial_ = std::make_shared<SerialPort>(serial_dev, baud);
    motor_ctl_ = std::make_unique<DM_Motor::Motor_Control>(serial_);

    std::array<DM_Motor::DM_Motor_Type, NUM_JOINTS> types = {DM_Motor::DM4340, DM_Motor::DM4340, DM_Motor::DM4340, DM_Motor::DM4310, DM_Motor::DM4310, DM_Motor::DM4310, DM_Motor::DM4310};
    std::array<uint8_t, NUM_JOINTS>  slave_ids = {0x02, 0x01, 0x03, 0x04, 0x05, 0x06, 0x07};
    std::array<uint8_t, NUM_JOINTS>  master_ids = {0x12, 0x11, 0x13, 0x14, 0x15, 0x16, 0x17};

    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        DM_Motor::Motor* m = new DM_Motor::Motor(types[i], slave_ids[i], master_ids[i]);
        motors_[i] = m;
        motor_ctl_->addMotor(m);
    }
    joint_limits_low_  = {-3.14f, -3.14f, -3.14f, -3.14f, -3.14f, -3.14f, 0.0f};
    joint_limits_high_ = { 3.14f,  3.14f,  3.14f,  3.14f,  3.14f,  3.14f, 1.37f};
    reset_pose_        = {M_PI * 0.5f, -1.5, 0.25f, 0.01f, 0.6f, 2.6f, -1.25f};
    kp_ = {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f};
    kd_ = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
    joint_names = {
            "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"
        };

    if (mode_ != ControlMode::SIM)
        enable_all();


    // Control main thread management
    switch (mode_) {
        case ControlMode::MIT_POS:
        case ControlMode::MIT_MIX:
        case ControlMode::MIT_DRAG:
            manager_ = std::make_unique<MotorManager>(motors_, motor_ctl_.get());
            break;
        case ControlMode::SIM:
            manager_ = std::make_unique<SimMotorManager>("ros2/src/openeai_arm_urdf_ros2/urdf/STEP.urdf");
            break;
    }

    kd_solver = std::make_shared<KDSolver>("ros2/src/openeai_arm_urdf_ros2/urdf/STEP.urdf", "base_link", "link6");
    float dt = manager_->getControlCycle();
    pid_controller = std::make_unique<AdvancedPIDController>(dt);
    pid_controller->setResetPose(reset_pose_);

    if (mode_ == ControlMode::MIT_MIX)
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->pid_controller->computeControlTorque(q, target_q); };
    else if (mode_ == ControlMode::MIT_DRAG)
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->compute_drag_dynamics(q, target_q); };

    std::cout << "OpenEAIArm initialized and ready." << std::endl;
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
    ArmConfig config = load_arm_config(config_path);

    serial_ = std::make_shared<SerialPort>(config.can.id, config.can.baud_rate);
    motor_ctl_ = std::make_unique<DM_Motor::Motor_Control>(serial_);

    JointArray ki, friction, friction_alpha;

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
        if (mode_ != ControlMode::MIT_DRAG) {
            ki[i] = motor.dynamic_coeff.ki;
            friction[i] = motor.dynamic_coeff.friction;
            friction_alpha[i] = motor.dynamic_coeff.friction_alpha;
        }
        else {
            ki[i] = 0;
            friction[i] = 0;
            friction_alpha[i] = 0;
        }
    }

    if (mode_ != ControlMode::SIM)
        enable_all();

    std::string interpolation_method = config.controller.interpolation;
    MotorManager::InterpolationMethod mmim;
    if (interpolation_method == "linear") mmim = MotorManager::InterpolationMethod::Linear;
    else if (interpolation_method == "cubics") mmim = MotorManager::InterpolationMethod::CubicS;
    else mmim = MotorManager::InterpolationMethod::MinJerk;
    float mm_alpha = config.controller.filter_alpha;

    switch (mode_) {
        case ControlMode::MIT_POS:
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
    pid_controller->setKpKiParams(friction, ki);
    pid_controller->setNonlinearParams(friction_alpha, 0.001f);

    if (mode_ == ControlMode::MIT_MIX) {
        manager_->compute_dynamic_torque = [this](const JointArray& q, const JointArray& target_q){ return this->pid_controller->computeControlTorque(q, target_q); };
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
        case ControlMode::MIT_POS:
            manager_->move(physical_target, manager_->uniform(1.0f), std::nullopt, kp_, kd_, move_time);
            break;
        case ControlMode::MIT_MIX:
        case ControlMode::SIM:
            manager_->move(physical_target, manager_->uniform(0.0f), std::nullopt, kp_, kd_, move_time);
            break;
        case ControlMode::MIT_DRAG:
            auto current_vel = get_joint_velocities();
            const float velocity_threshold = 0.25f;
            bool is_stationary = true;
            for (const auto& v : current_vel) {
                if (std::fabs(v) > velocity_threshold) {
                    is_stationary = false;
                    break;
                }
            }
            const JointArray stationary_kp = {20.0f, 20.0f, 20.0f, 5.0f, 5.0f, 5.0f, 0.0f};
            const JointArray stationary_kd = {1.0f, 1.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.0f};
            if (is_stationary)
                manager_->move(manager_->getPosition(), manager_->uniform(0.0f), std::nullopt, stationary_kp, stationary_kd, move_time);
            else
                manager_->move(physical_target, manager_->uniform(0.0f), std::nullopt, manager_->uniform(0.0f), manager_->uniform(0.0f), move_time);
            break;
    }
}

OpenEAIArm::JointArray OpenEAIArm::compute_static_tau(OpenEAIArm::JointArray q, OpenEAIArm::JointArray target_q, bool output) // Deprecated
{
    if (output) std::cout << "============== Tau args ==============" << std::endl;
    q = q_phys_to_control(q);
    // 1. Define robot physical parameters
    constexpr double DEG = M_PI / 180.0;
    constexpr double MM = 0.001;
    constexpr double g = 9.80665;
    // 2. MDH parameters: theta, d, a, alpha
    double mdh[NUM_JOINTS][4] = {
        {0*DEG,           (71.3+45.96)*MM,  0*MM,       0*DEG},
        {180*DEG,         0*MM,  19*MM,       -90*DEG},
        {(180+14.927)*DEG,0*MM, 264*MM,      180*DEG},
        {-14.927*DEG,     0*MM, 233.89*MM,   0*DEG},
        {90*DEG,          (46.5-48.06)*MM, 75*MM, 90*DEG},
        {0*DEG,           35.95*MM, 0*MM,   90*DEG},
        {0*DEG,           65*MM, 0*MM,  0*DEG},
    };
    // You need to set motor_mass, link_mass, motor_cg_z
    JointArray tau{};
    
    // ----- 3. Calculate the rotation centers and centers of gravity for each joint -----
    struct Body {
        Eigen::Vector3d joint_pos;   // 旋转中心
        Eigen::Vector3d motor_cg;    // 电机重心
        Eigen::Vector3d link_cg;     // 连杆重心
    };
    std::array<Body, NUM_JOINTS> bodies;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::array<Eigen::Matrix4d, NUM_JOINTS+1> Tj; 
    Tj[0] = T;

    // Joint transformations
    for (size_t i=0; i<NUM_JOINTS; ++i) {
        double th = q[i] + mdh[i][0];
        double d = mdh[i][1];
        double a = mdh[i][2];
        double alpha = mdh[i][3];
        Eigen::Matrix4d Ai;

        Ai << 
            cos(th),            -sin(th),             0,           a,
            sin(th)*cos(alpha),  cos(th)*cos(alpha), -sin(alpha), -d*sin(alpha),
            sin(th)*sin(alpha),  cos(th)*sin(alpha),  cos(alpha),  d*cos(alpha),
            0,        0,                   0,                  1;

        T = T * Ai;
        Tj[i+1] = T;

        bodies[i].joint_pos = T.block<3,1>(0,3);
        Eigen::Vector4d cg_local(0,0,motor_cg_z[i],1);
        Eigen::Vector4d cg_world = T * cg_local;
        bodies[i].motor_cg = cg_world.head<3>();
    }
    for (size_t i=0; i+1<NUM_JOINTS; ++i) {
        bodies[i].link_cg = 0.5*(bodies[i].motor_cg + bodies[i+1].motor_cg);
    }
    bodies[NUM_JOINTS-1].link_cg = (T * Eigen::Vector4d(0,0,motor_cg_z[NUM_JOINTS],1)).head<3>();

    // ----- 4. Torque sum up -----
    Eigen::Vector3d gravity(0,0,-g);
    double delta_tau;
    for (size_t i=0; i<NUM_JOINTS; ++i) {
        double tau_i = 0.0;
        auto origin = bodies[i].joint_pos;
        // Torques contributed by all subsequent motors and links to joint i
        for (size_t j=i; j<NUM_JOINTS; ++j) {
            // Motor center of gravity
            Eigen::Vector3d r = bodies[j].motor_cg - origin;
            Eigen::Vector3d Fg = motor_mass[j] * gravity;
            Eigen::Vector3d tau_g = r.cross(Fg);
            // Project onto the joint axis
            Eigen::Vector3d z_axis = (Tj[i+1].block<3,1>(0,2)).normalized(); // Joint i's z-axis direction
            delta_tau = tau_g.dot(z_axis);
            tau_i += delta_tau;
            if (output) std::cout << "Tau " << j << " @ " << i << " = " << delta_tau << std::endl;
            // Link
            Eigen::Vector3d r_link = bodies[j].link_cg - origin;
            Eigen::Vector3d Fg_link = link_mass[j] * gravity;
            Eigen::Vector3d tau_g_link = r_link.cross(Fg_link);
            delta_tau = tau_g_link.dot(z_axis);
            if (output) std::cout << "Tau " << j << "-" << j+1 << " @ " << i << " = " << delta_tau << std::endl;
            tau_i += delta_tau;
        }
        tau[i] = -float(tau_i);
        if (output) std::cout << "Total tau " << i << " = " << tau[i] << std::endl;
        if (output) std::cout << "----------------" << std::endl;
    }
    if (output) {
        std::cout << "---Joint rotation centers---\n";
        for (size_t i=0; i<NUM_JOINTS; ++i)
            std::cout << "Joint " << i << ": (" << bodies[i].joint_pos.transpose() << ")\n";
        std::cout << "---Motor centers of gravity---\n";
        for (size_t i=0; i<NUM_JOINTS; ++i)
            std::cout << "Motor " << i << ": (" << bodies[i].motor_cg.transpose() << ")\n";
        std::cout << "---Link centers of gravity---\n";
        for (size_t i=0; i < NUM_JOINTS; ++i)
            std::cout << "Link " << i << ": (" << bodies[i].link_cg.transpose() << ")\n";

        std::cout << "Angles (q): ";
        for (float v : q) std::cout << std::fixed << std::setprecision(4) << v << ", ";
        std::cout << "\nStatic torques (tau): ";
        for (float v : tau) std::cout << std::fixed << std::setprecision(4) << v << ", ";
        std::cout << std::endl;
    }

    return tau;
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
            for (int i = 1; i < 3; i++) {
                tau[i] += 0.5f * tanh(2.0f * current_vel[i]);
                tau[i] += 0.3f * current_vel[i];
            }
        }
    }
    else {
        std::cout << "Something wrong!" << std::endl;
    }
    return tau;
}