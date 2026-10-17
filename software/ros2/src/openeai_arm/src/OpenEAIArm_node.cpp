#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <string>
#include "OpenEAIArm.hpp"

class OpenEAIArmNode : public rclcpp::Node {
public:
    OpenEAIArmNode()
    : Node("OpenEAIArm_node")
    {
        // Declare parameters, with defaults
        this->declare_parameter<std::string>("config", "configs/default.yml");
        this->declare_parameter<int>("ctrl_mode", 0);           // 0: Program control, 1: Drag, 2: Simulation
        this->declare_parameter<int>("frequency", 50);          // Hz
        this->declare_parameter<int>("ee_pose",   0);          // Hz
        this->declare_parameter<std::string>("arm_name", "openeai_arm");
        this->declare_parameter<double>("ik_linear_speed", 0.35);
        this->declare_parameter<double>("ik_angular_speed", 1.2);
        this->declare_parameter<double>("ik_joint_speed_scale", 1.0);
        this->declare_parameter<double>("ik_branch_step_multiplier", 3.0);
        this->declare_parameter<int>("ik_recovery_valid_frames", 3);
        this->declare_parameter<double>("ik_joint_limit_margin", 0.02);
        this->declare_parameter<double>("ik_fk_position_tolerance", 0.01);
        this->declare_parameter<double>("ik_fk_orientation_tolerance", 0.05);

        // Get initial parameter values
        config_      = this->get_parameter("config").as_string();
        ctrl_mode_   = this->get_parameter("ctrl_mode").as_int();
        frequency_   = this->get_parameter("frequency").as_int();
        ee_pose_     = this->get_parameter("ee_pose").as_int();
        arm_name_    = this->get_parameter("arm_name").as_string();
        ik_linear_speed_ = this->get_parameter("ik_linear_speed").as_double();
        ik_angular_speed_ = this->get_parameter("ik_angular_speed").as_double();
        ik_joint_speed_scale_ = this->get_parameter("ik_joint_speed_scale").as_double();
        ik_branch_step_multiplier_ = this->get_parameter("ik_branch_step_multiplier").as_double();
        ik_recovery_valid_frames_ = this->get_parameter("ik_recovery_valid_frames").as_int();
        ik_joint_limit_margin_ = this->get_parameter("ik_joint_limit_margin").as_double();
        ik_fk_position_tolerance_ = this->get_parameter("ik_fk_position_tolerance").as_double();
        ik_fk_orientation_tolerance_ = this->get_parameter("ik_fk_orientation_tolerance").as_double();
        if (ik_linear_speed_ <= 0.0 || ik_angular_speed_ <= 0.0 ||
            ik_joint_speed_scale_ <= 0.0 || ik_joint_speed_scale_ > 1.0 ||
            ik_branch_step_multiplier_ <= 1.0 || ik_recovery_valid_frames_ < 1 ||
            ik_joint_limit_margin_ < 0.0 || ik_fk_position_tolerance_ <= 0.0 ||
            ik_fk_orientation_tolerance_ <= 0.0) {
            throw std::runtime_error("Invalid stateful IK parameters");
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Stateful IK: linear=%.3f m/s angular=%.3f rad/s joint_scale=%.2f "
            "branch_multiplier=%.2f recovery_frames=%d",
            ik_linear_speed_, ik_angular_speed_, ik_joint_speed_scale_,
            ik_branch_step_multiplier_, ik_recovery_valid_frames_);
        control_period_ = std::chrono::milliseconds(1000 / std::max(1, frequency_));

        // Subscribe to parameter changes (optional, for dynamic adjustment)
        param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) -> rcl_interfaces::msg::SetParametersResult {
                for (const auto &param: params) {
                    if (param.get_name() == "frequency") {
                        frequency_ = param.as_int();
                        control_period_ = std::chrono::milliseconds(1000 / std::max(1, frequency_));
                        timer_->cancel();
                        timer_ = this->create_wall_timer(control_period_, [this]() { send_and_publish(); });
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "Updated";
                return result;
            }
        );

        if (ctrl_mode_ == 0) {
            arm_ = std::make_unique<OpenEAIArm>(
                config_, OpenEAIArm::ControlMode::MIT_MIX, arm_name_);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with control mode");
            arm_->reset();
        }
        else if (ctrl_mode_ == 1) {
            arm_ = std::make_unique<OpenEAIArm>(
                config_, OpenEAIArm::ControlMode::MIT_DRAG, arm_name_);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with drag mode");
        }
        else {
            arm_ = std::make_unique<OpenEAIArm>(
                config_, OpenEAIArm::ControlMode::SIM, arm_name_);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with sim mode");
            arm_->reset();
        }

        pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>(arm_name_ + "/joint_states", 10);
        pub_joint_6d_ = this->create_publisher<sensor_msgs::msg::JointState>(arm_name_ + "/joint_states_6d", 10);
        pub_eef_pose_ = this->create_publisher<sensor_msgs::msg::JointState>(arm_name_ + "/eef_pose", 10);
        if (ctrl_mode_ == 1) pub_joint_fake_target_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);

        // (if controlling by JointState message)
        sub_cmd_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_targets", 10, // target subscription topic here
            [this](sensor_msgs::msg::JointState::UniquePtr msg){
                if(msg->position.size() != 7) { /* error handling */ return; }
                for(size_t i=0;i<7;++i) target_[i] = msg->position[i];
            });

        reset_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "robot_reset", 10,
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                if (msg->data == 1 && !in_reset_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Entering reset mode.");
                    in_reset_mode_ = true;
                }
                else if (msg->data == 0 && in_reset_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Leaving reset mode.");
                    in_reset_mode_ = false;
                }
            });
        prepare_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "robot_prepare", 10,
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                if (msg->data == 1 && !in_prepare_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Entering prepare mode.");
                    in_prepare_mode_ = true;
                }
                else if (msg->data == 0 && in_prepare_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Leaving prepare mode.");
                    in_prepare_mode_ = false;
                }
            });

        set_mode_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "set_control_mode", 10,
            [this](sensor_msgs::msg::JointState::UniquePtr msg) {
                if (msg->position.size() >= 2) {
                    int new_ctrl_mode = static_cast<int>(msg->position[0]);
                    int new_ee_pose = static_cast<int>(msg->position[1]);

                    RCLCPP_INFO(this->get_logger(), "Setting control mode: ctrl_mode=%d, ee_pose=%d",
                                new_ctrl_mode, new_ee_pose);

                    if (new_ctrl_mode != ctrl_mode_) {
                        set_control_mode(new_ctrl_mode);
                    }
                    if (new_ee_pose != ee_pose_) {
                        set_ee_pose_mode(new_ee_pose);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Control mode message must have at least 2 values");
                }
            });

        timer_ = this->create_wall_timer(control_period_, [this]() { send_and_publish(); });

        update_init_ee_pose();

    }

    ~OpenEAIArmNode() override {
        if (arm_) {
            RCLCPP_INFO(this->get_logger(), "Disabling all motors before shutdown.");
            arm_->disable_all();
        }
    }

private:
    enum class StatefulIKState {
        TRACKING,
        HOLD_INVALID
    };

    static Eigen::Quaterniond pose_rotation(const OpenEAIArm::JointArray& pose) {
        Eigen::Quaterniond rotation =
            Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());
        return rotation.normalized();
    }

    static double rotation_distance(const OpenEAIArm::JointArray& from,
                                    const OpenEAIArm::JointArray& to) {
        Eigen::Quaterniond delta = pose_rotation(from).conjugate() * pose_rotation(to);
        delta.normalize();
        if (delta.w() < 0.0) {
            delta.coeffs() *= -1.0;
        }
        return Eigen::AngleAxisd(delta).angle();
    }

    OpenEAIArm::JointArray cartesian_step(
        const OpenEAIArm::JointArray& from,
        const OpenEAIArm::JointArray& target) const {
        OpenEAIArm::JointArray result = from;
        const double dt = 1.0 / static_cast<double>(std::max(1, frequency_));

        Eigen::Vector3d from_position(from[0], from[1], from[2]);
        Eigen::Vector3d target_position(target[0], target[1], target[2]);
        Eigen::Vector3d position_delta = target_position - from_position;
        const double max_position_step = ik_linear_speed_ * dt;
        if (position_delta.norm() > max_position_step) {
            position_delta *= max_position_step / position_delta.norm();
        }
        const Eigen::Vector3d next_position = from_position + position_delta;
        for (size_t i = 0; i < 3; ++i) {
            result[i] = static_cast<float>(next_position[i]);
        }

        Eigen::Quaterniond from_rotation = pose_rotation(from);
        Eigen::Quaterniond target_rotation = pose_rotation(target);
        if (from_rotation.dot(target_rotation) < 0.0) {
            target_rotation.coeffs() *= -1.0;
        }
        const double angle = Eigen::AngleAxisd(
            from_rotation.conjugate() * target_rotation).angle();
        const double max_angle_step = ik_angular_speed_ * dt;
        const double interpolation_ratio = angle > max_angle_step
            ? max_angle_step / angle
            : 1.0;
        const Eigen::Quaterniond next_rotation =
            from_rotation.slerp(interpolation_ratio, target_rotation).normalized();
        const Eigen::Vector3d zyx = next_rotation.toRotationMatrix().eulerAngles(2, 1, 0);
        result[3] = static_cast<float>(zyx[2]);
        result[4] = static_cast<float>(zyx[1]);
        result[5] = static_cast<float>(zyx[0]);
        result[OpenEAIArm::NUM_JOINTS - 1] = target[OpenEAIArm::NUM_JOINTS - 1];
        return result;
    }

    void initialize_stateful_ik() {
        planned_joint_target_ = arm_->get_joint_positions();
        planned_ee_pose_ = arm_->forward_kinetics(planned_joint_target_);
        target_ee_pose_ = planned_ee_pose_;
        stateful_ik_state_ = StatefulIKState::TRACKING;
        ik_valid_recovery_frames_ = 0;
        stateful_ik_initialized_ = true;
    }

    bool validate_stateful_ik_candidate(
        const OpenEAIArm::JointArray& candidate,
        const OpenEAIArm::JointArray& candidate_pose,
        std::string& reason) const {
        const auto lower = arm_->lower_limits();
        const auto upper = arm_->upper_limits();
        const auto velocity_limits = arm_->velocity_limits();
        const double dt = 1.0 / static_cast<double>(std::max(1, frequency_));

        for (size_t i = 0; i < OpenEAIArm::NUM_JOINTS; ++i) {
            if (!std::isfinite(candidate[i])) {
                reason = "non-finite joint candidate";
                return false;
            }
        }
        for (size_t i = 0; i < OpenEAIArm::NUM_JOINTS - 1; ++i) {
            if (candidate[i] < lower[i] + ik_joint_limit_margin_ ||
                candidate[i] > upper[i] - ik_joint_limit_margin_) {
                reason = "joint limit margin reached at joint " + std::to_string(i + 1);
                return false;
            }
            const double branch_limit =
                ik_branch_step_multiplier_ * velocity_limits[i] * dt;
            if (std::fabs(candidate[i] - planned_joint_target_[i]) > branch_limit) {
                reason = "IK branch jump detected at joint " + std::to_string(i + 1);
                return false;
            }
        }

        const auto solved_pose = arm_->forward_kinetics(candidate);
        const double dx = solved_pose[0] - candidate_pose[0];
        const double dy = solved_pose[1] - candidate_pose[1];
        const double dz = solved_pose[2] - candidate_pose[2];
        const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (position_error > ik_fk_position_tolerance_) {
            reason = "FK position residual is too large";
            return false;
        }
        if (rotation_distance(solved_pose, candidate_pose) >
            ik_fk_orientation_tolerance_) {
            reason = "FK orientation residual is too large";
            return false;
        }
        return true;
    }

    OpenEAIArm::JointArray stateful_inverse_kinetics(
        const OpenEAIArm::JointArray& requested_pose) {
        if (!stateful_ik_initialized_) {
            initialize_stateful_ik();
        }

        for (float value : requested_pose) {
            if (!std::isfinite(value)) {
                stateful_ik_state_ = StatefulIKState::HOLD_INVALID;
                ik_valid_recovery_frames_ = 0;
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Stateful IK hold: non-finite Cartesian target");
                return planned_joint_target_;
            }
        }

        planned_joint_target_[OpenEAIArm::NUM_JOINTS - 1] =
            requested_pose[OpenEAIArm::NUM_JOINTS - 1];
        planned_ee_pose_[OpenEAIArm::NUM_JOINTS - 1] =
            requested_pose[OpenEAIArm::NUM_JOINTS - 1];

        const auto candidate_pose = cartesian_step(planned_ee_pose_, requested_pose);
        OpenEAIArm::IKOptions options;
        options.policy = OpenEAIArm::IKJumpPolicy::ACCEPT;
        options.report_failure = false;
        bool success = false;
        const auto candidate = arm_->inverse_kinetics(
            candidate_pose, planned_joint_target_, success, options);

        std::string invalid_reason;
        const bool valid = success &&
            validate_stateful_ik_candidate(candidate, candidate_pose, invalid_reason);
        if (!valid) {
            stateful_ik_state_ = StatefulIKState::HOLD_INVALID;
            ik_valid_recovery_frames_ = 0;
            if (!success) {
                invalid_reason = "IK solver failed";
            }
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Stateful IK hold: %s", invalid_reason.c_str());
            return planned_joint_target_;
        }

        if (stateful_ik_state_ == StatefulIKState::HOLD_INVALID) {
            ++ik_valid_recovery_frames_;
            if (ik_valid_recovery_frames_ < ik_recovery_valid_frames_) {
                return planned_joint_target_;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Stateful IK recovered after %d consecutive valid frames",
                        ik_valid_recovery_frames_);
        }

        const auto velocity_limits = arm_->velocity_limits();
        const double dt = 1.0 / static_cast<double>(std::max(1, frequency_));
        double scale = 1.0;
        for (size_t i = 0; i < OpenEAIArm::NUM_JOINTS - 1; ++i) {
            const double delta = std::fabs(candidate[i] - planned_joint_target_[i]);
            if (delta > 0.0) {
                const double max_step =
                    ik_joint_speed_scale_ * velocity_limits[i] * dt;
                scale = std::min(scale, max_step / delta);
            }
        }

        OpenEAIArm::JointArray next_plan = planned_joint_target_;
        for (size_t i = 0; i < OpenEAIArm::NUM_JOINTS - 1; ++i) {
            next_plan[i] += static_cast<float>(
                scale * (candidate[i] - planned_joint_target_[i]));
        }
        next_plan[OpenEAIArm::NUM_JOINTS - 1] =
            requested_pose[OpenEAIArm::NUM_JOINTS - 1];

        planned_joint_target_ = next_plan;
        planned_ee_pose_ = arm_->forward_kinetics(planned_joint_target_);
        planned_ee_pose_[OpenEAIArm::NUM_JOINTS - 1] =
            requested_pose[OpenEAIArm::NUM_JOINTS - 1];
        target_ee_pose_ = planned_ee_pose_;
        stateful_ik_state_ = StatefulIKState::TRACKING;
        ik_valid_recovery_frames_ = 0;
        return planned_joint_target_;
    }

    void set_control_mode(int new_mode) {
        if (new_mode < 0 || new_mode > 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid control mode: %d. Must be 0, 1, or 2", new_mode);
            return;
        }
        if (new_mode == ctrl_mode_) return;

        RCLCPP_INFO(this->get_logger(), "Changing control mode from %d to %d", ctrl_mode_, new_mode);

        if (new_mode == 2) {
            RCLCPP_ERROR(this->get_logger(), "Switching to SIM mode is not supported at runtime.");
            return;
        }

        ctrl_mode_ = new_mode;

        if (ctrl_mode_ == 0) {
            arm_->set_control_mode(OpenEAIArm::ControlMode::MIT_MIX);
        } else if (ctrl_mode_ == 1) {
            arm_->set_control_mode(OpenEAIArm::ControlMode::MIT_DRAG);
        }

        if (ctrl_mode_ == 1 && !pub_joint_fake_target_) {
            pub_joint_fake_target_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_targets", 10);
        } else if (ctrl_mode_ != 1 && pub_joint_fake_target_) {
            pub_joint_fake_target_.reset();
        }
    }

    void set_ee_pose_mode(int new_ee_pose) {
        if (new_ee_pose < 0 || new_ee_pose > 2) {
            RCLCPP_ERROR(this->get_logger(), "Invalid ee_pose mode: %d. Must be 0, 1, or 2", new_ee_pose);
            return;
        }
        if (new_ee_pose == ee_pose_) return;

        RCLCPP_INFO(this->get_logger(), "Changing ee_pose mode from %d to %d", ee_pose_, new_ee_pose);
        ee_pose_ = new_ee_pose;

        if (ee_pose_ == 2) {
            init_ee_pose_ = arm_->get_ee_pose();
            for (size_t i = 0; i < 3; ++i) {
                target_[i] = 0.0f;
            }
            for (size_t i = 3; i < arm_->NUM_JOINTS - 1; ++i) {
                target_[i] = init_ee_pose_[i];
            }
            initialize_stateful_ik();
            RCLCPP_INFO(this->get_logger(), "Switched to absolute ee control mode");
        } else if (ee_pose_ == 0) {
            stateful_ik_initialized_ = false;
            target_ = arm_->get_joint_positions();
            RCLCPP_INFO(this->get_logger(), "Switched to joint control mode");
        } else if (ee_pose_ == 1) {
            stateful_ik_initialized_ = false;
            target_ee_pose_ = arm_->get_ee_pose();
            for (size_t i = 0; i < arm_->NUM_JOINTS; ++i) {
                target_[i] = 0.0f;
            }
            RCLCPP_INFO(this->get_logger(), "Switched to delta ee control mode");
        }
    }

    void update_init_ee_pose() {
        target_ee_pose_ = arm_->get_ee_pose();
        init_ee_pose_ = arm_->get_ee_pose();
        if (ee_pose_ == 2) {
            for (size_t i = 0; i < 3; ++i) {
                target_[i] = 0.0f;
            }
            for (size_t i = 3; i < arm_->NUM_JOINTS - 1; ++i) {
                target_[i] = init_ee_pose_[i];
            }
            initialize_stateful_ik();
        }
        else if (ee_pose_ == 0) {
            stateful_ik_initialized_ = false;
            target_ = arm_->get_joint_positions();
        }
        std::cout << arm_->get_joint_positions() << std::endl;
        std::cout << target_ee_pose_ << std::endl;
    }

    enum class ManualMotionMode {
        NONE,
        RESET,
        PREPARE
    };

    OpenEAIArm::JointArray advance_manual_motion(
        ManualMotionMode mode,
        const OpenEAIArm::JointArray& goal)
    {
        if (manual_motion_mode_ != mode) {
            manual_motion_target_ = arm_->get_joint_positions();
            manual_motion_mode_ = mode;
        }

        constexpr float JOINT_STEP = 0.005f;
        for (size_t i = 0; i < arm_->NUM_JOINTS - 1; ++i) {
            const float delta = goal[i] - manual_motion_target_[i];
            manual_motion_target_[i] += std::clamp(delta, -JOINT_STEP, JOINT_STEP);
        }

        // The gripper command is a width for v1/v2, not a joint angle. Keep the
        // width captured on entry instead of applying the angular joint step.
        target_[arm_->NUM_JOINTS - 1] =
            manual_motion_target_[arm_->NUM_JOINTS - 1];
        return manual_motion_target_;
    }

    void send_and_publish()
    {
        // Send
        if (in_reset_mode_) {
            const auto q_next = advance_manual_motion(
                ManualMotionMode::RESET, restore_target_);
            arm_->set_joint_targets(q_next, 1.0f / frequency_, true);
            update_init_ee_pose();
            target_ee_pose_ = arm_->forward_kinetics(q_next);
        }
        else if (in_prepare_mode_) {
            const auto q_next = advance_manual_motion(
                ManualMotionMode::PREPARE, prepare_target_);
            arm_->set_joint_targets(q_next, 1.0f / frequency_, true);
            update_init_ee_pose();
            target_ee_pose_ = arm_->forward_kinetics(q_next);
        }
        else {
            manual_motion_mode_ = ManualMotionMode::NONE;
            if (ee_pose_ == 0) {
                arm_->set_joint_targets(target_, 1.0f / frequency_, true);
            }
            else {
                auto temp_ee_pose = target_ee_pose_;
                bool success;
                if (ee_pose_ == 1) {
                    for (size_t i = 0; i < arm_->NUM_JOINTS - 1; i++) {
                        temp_ee_pose[i] += target_[i];
                    }
                }
                else if (ee_pose_ == 2) {
                    for (size_t i = 0; i < arm_->NUM_JOINTS - 1; i++) {
                        temp_ee_pose[i] = target_[i];
                        if (i < 3) {
                            temp_ee_pose[i] += init_ee_pose_[i];
                        }
                    }
                }
                temp_ee_pose[arm_->NUM_JOINTS - 1] = target_[arm_->NUM_JOINTS - 1];
                OpenEAIArm::JointArray target;
                if (ee_pose_ == 2) {
                    target = stateful_inverse_kinetics(temp_ee_pose);
                } else {
                    OpenEAIArm::IKOptions ik_options = OpenEAIArm::IKOptions();
                    ik_options.angle_jump_threshold = 6.0f / frequency_;
                    ik_options.fixed_step = 6.0f / frequency_;
                    ik_options.policy = OpenEAIArm::IKJumpPolicy::CLIP;
                    target = arm_->inverse_kinetics(temp_ee_pose, success, ik_options);
                    if (success) {
                        target_ee_pose_ = temp_ee_pose;
                    }
                    else {
                        target = arm_->inverse_kinetics(target_ee_pose_, success, ik_options);
                    }
                }
                arm_->set_joint_targets(target, 1.0f / frequency_, true);
            }
        }

        // Publish
        auto pos = arm_->get_joint_positions();
        auto vel = arm_->get_joint_velocities();
        auto tau = arm_->get_joint_torques();
        auto eef_pose = arm_->get_ee_pose();

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "";
        const auto names = arm_->get_joint_names();
        static const std::vector<std::string> joint_names(names.begin(), names.end());
        msg.name = joint_names;
        msg.position.insert(msg.position.end(), pos.begin(), pos.end());
        msg.velocity.insert(msg.velocity.end(), vel.begin(), vel.end());
        msg.effort.insert(msg.effort.end(), tau.begin(), tau.end());

        pub_joint_->publish(msg);


        auto msg_6d = sensor_msgs::msg::JointState();
        msg_6d.header.stamp = this->now();
        msg_6d.header.frame_id = "";
        static const std::vector<std::string> joint_names_6d = {
                "1", "2", "3", "4", "5", "6"
            };
        msg_6d.name = joint_names_6d;
        msg_6d.position.insert(msg_6d.position.end(), pos.begin(), pos.end()-1);
        msg_6d.velocity.insert(msg_6d.velocity.end(), vel.begin(), vel.end()-1);
        msg_6d.effort.insert(msg_6d.effort.end(), tau.begin(), tau.end()-1);

        pub_joint_6d_->publish(msg_6d);

        if (ctrl_mode_ == 1 && pub_joint_fake_target_) {
            pub_joint_fake_target_->publish(msg);
        }

        auto eef_msg = sensor_msgs::msg::JointState();
        eef_msg.header.stamp = this->now();
        eef_msg.header.frame_id = "eef_pose";
        static const std::vector<std::string> eef_names = {
                "x", "y", "z", "rx", "ry", "rz", "gripper_width"
            };
        eef_msg.name = eef_names;
        eef_msg.position.insert(eef_msg.position.end(), eef_pose.begin(), eef_pose.end());
        pub_eef_pose_->publish(eef_msg);
    }

    std::unique_ptr<OpenEAIArm> arm_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_6d_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_fake_target_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_eef_pose_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr set_mode_sub_;
    bool in_reset_mode_ = false, in_prepare_mode_ = false;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr prepare_sub_;


    OpenEAIArm::JointArray target_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray target_ee_pose_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray init_ee_pose_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray planned_joint_target_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray planned_ee_pose_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray manual_motion_target_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray restore_target_{0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray prepare_target_{0.0,1.0,1.5,-1.0,0.0,0.0,0.0};
    ManualMotionMode manual_motion_mode_ = ManualMotionMode::NONE;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::string config_;
    int ctrl_mode_;
    int frequency_;
    int ee_pose_;
    std::string arm_name_;
    std::chrono::milliseconds control_period_;
    double ik_linear_speed_;
    double ik_angular_speed_;
    double ik_joint_speed_scale_;
    double ik_branch_step_multiplier_;
    int ik_recovery_valid_frames_;
    double ik_joint_limit_margin_;
    double ik_fk_position_tolerance_;
    double ik_fk_orientation_tolerance_;
    StatefulIKState stateful_ik_state_ = StatefulIKState::TRACKING;
    int ik_valid_recovery_frames_ = 0;
    bool stateful_ik_initialized_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenEAIArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
