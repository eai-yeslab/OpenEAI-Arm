#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <std_msgs/msg/int32.hpp> 
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
        this->declare_parameter<int>("ee_pose",   0);           // 0: Joint Position, 1: relative ee pose, 2: absolute ee pose
        this->declare_parameter<std::string>("arm_name", "openeai_arm");

        // Get initial parameter values
        config_      = this->get_parameter("config").as_string();
        ctrl_mode_   = this->get_parameter("ctrl_mode").as_int();
        frequency_   = this->get_parameter("frequency").as_int();
        ee_pose_     = this->get_parameter("ee_pose").as_int();
        arm_name_    = this->get_parameter("arm_name").as_string();
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
            arm_ = std::make_unique<OpenEAIArm>(config_, OpenEAIArm::ControlMode::MIT_MIX);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with control mode");
            arm_->reset();
        }
        else if (ctrl_mode_ == 1) {
            arm_ = std::make_unique<OpenEAIArm>(config_, OpenEAIArm::ControlMode::MIT_DRAG);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with drag mode");
        }
        else {
            arm_ = std::make_unique<OpenEAIArm>(config_, OpenEAIArm::ControlMode::SIM);
            RCLCPP_INFO(this->get_logger(), "Starting SII Arm with sim mode");
            arm_->go_home();
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
        }
        else if (ee_pose_ == 0) {
            target_ = arm_->get_joint_positions();
        }
        std::cout << arm_->get_joint_positions() << std::endl;
        std::cout << target_ee_pose_ << std::endl;
    }

    void send_and_publish()
    {
        // Send
        if (in_reset_mode_) {
            auto q_cur = arm_->get_joint_positions();
            OpenEAIArm::JointArray q_next;
            constexpr float JOINT_VELOCITY_LIMIT = 1.0f;
            for (size_t i=0; i<arm_->NUM_JOINTS; ++i) {
                float dq = restore_target_[i] - q_cur[i];
                float move = std::clamp(dq, -JOINT_VELOCITY_LIMIT / frequency_, JOINT_VELOCITY_LIMIT / frequency_);
                q_next[i] = q_cur[i] + move;
            }
            arm_->set_joint_targets(q_next, 1.0f / frequency_);
            update_init_ee_pose();
            target_ee_pose_ = arm_->forward_kinetics(q_next);
        }
        else if (in_prepare_mode_) {
            auto q_cur = arm_->get_joint_positions();
            OpenEAIArm::JointArray q_next;
            constexpr float JOINT_VELOCITY_LIMIT = 1.0f;
            for (size_t i=0; i<arm_->NUM_JOINTS; ++i) {
                float dq = prepare_target_[i] - q_cur[i];
                float move = std::clamp(dq, -JOINT_VELOCITY_LIMIT / frequency_, JOINT_VELOCITY_LIMIT / frequency_);
                q_next[i] = q_cur[i] + move;
            }
            arm_->set_joint_targets(q_next, 1.0f / frequency_);
            update_init_ee_pose();
            target_ee_pose_ = arm_->forward_kinetics(q_next);
        }
        else {
            if (ee_pose_ == 0) {
                arm_->set_joint_targets(target_, 1.0f / frequency_);
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
                OpenEAIArm::IKOptions ik_options = OpenEAIArm::IKOptions();
                ik_options.angle_jump_threshold = 4.0f / frequency_;
                ik_options.fixed_step = 4.0f / frequency_;
                ik_options.policy = OpenEAIArm::IKJumpPolicy::CLIP;
                auto target = arm_->inverse_kinetics(temp_ee_pose, success, ik_options);
                if (success) {
                    target_ee_pose_ = temp_ee_pose;
                }
                else {
                    target = arm_->inverse_kinetics(target_ee_pose_, success, ik_options);
                }
                arm_->set_joint_targets(target, 1.0f / frequency_);
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
        static const std::vector<std::string> joint_names(arm_->get_joint_names().begin(), arm_->get_joint_names().end());
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

        if (ctrl_mode_ == 1) {
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
    bool in_reset_mode_ = false, in_prepare_mode_ = false;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr prepare_sub_;


    OpenEAIArm::JointArray target_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray target_ee_pose_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray init_ee_pose_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray restore_target_{0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    OpenEAIArm::JointArray prepare_target_{0.0,1.0,1.5,-1.0,0.0,0.0,0.0};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::string config_;
    int ctrl_mode_;
    int frequency_;
    int ee_pose_;
    std::string arm_name_;
    std::chrono::milliseconds control_period_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenEAIArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}