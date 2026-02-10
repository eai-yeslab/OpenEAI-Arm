#pragma once

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
// #include <pinocchio/algorithm/visualize.hpp>
#include <fstream>
#include <array>
#include <variant>
#include <optional>
#include <chrono>
#include <string>

#include "dm_manager.hpp"

class SimMotorManager: public BaseMotorManager {
public:

    SimMotorManager(const std::string &urdf_path,
                float control_freq = 200.0f);

    SimMotorManager(const std::string &urdf_path,
            const JointArray& reset_pose,
            float control_freq = 200.0f);

    ~SimMotorManager();

    void setGain(JointArray kp, JointArray kd) {
        kp_ = kp;
        kd_ = kd;
    }

    void move(
        const JointArray& q,
        std::optional<JointArray> dq = std::nullopt,
        std::optional<JointArray> tau = std::nullopt,
        std::optional<JointArray> kp = std::nullopt,
        std::optional<JointArray> kd = std::nullopt,
        std::optional<float> move_time = std::nullopt) override;

    std::array<float, NUM_JOINTS> getPosition();
    std::array<float, NUM_JOINTS> getVelocity();
    std::array<float, NUM_JOINTS> getTau();

    void start();
    void stop();
    void refresh();
    void setPosition(const JointArray& q) override { move(q); };

    // Reset: move all joints to zero and hold for move_time (default: 1.0s)
    void reset(float move_time = 1.0f);
    std::function<JointArray(const JointArray&, const JointArray&)> compute_dynamic_torque;
    float getControlFreq() {return control_freq;}
    float getControlCycle() {return control_cycle;}

private:
    JointArray s_curve(float step, const JointArray& curr_q, const JointArray& target_q);
    void move_mit(const JointArray& q, const JointArray& dq, const JointArray& tau,
                  JointArray kp, JointArray kd);
    void move_mit_linear_interpolated(const JointArray& start_q, const JointArray& q,
                               const JointArray& dq, const JointArray& tau,
                               JointArray kp, JointArray kd, float move_time);
    void move_mit_s_interpolated(const JointArray& start_q, const JointArray& q,
                               const JointArray& dq, const JointArray& tau,
                               JointArray kp, JointArray kd, float move_time);


    JointArray kp_ = uniform(10.0f);
    JointArray kd_ = uniform(0.1f);
    JointArray max_vel_ = {M_PI*1.0f, M_PI*1.0f, M_PI*1.0f, M_PI*1.25f, M_PI*1.25f, M_PI*1.25f, M_PI*0.5f}; 
    JointArray last_q_, last_dq_, last_tau_;
    std::chrono::steady_clock::time_point last_cmd_time_;

    void controlLoop();
    void mc_control(size_t idx, float new_q, float new_dq);

    pinocchio::Model model_;
    pinocchio::Data data_;
    JointArray q_;
    JointArray dq_;
    JointArray tau_;
    std::array<std::string, NUM_JOINTS> joint_names_;
    JointArray reset_pose_ = {M_PI * 0.5f, -M_PI * 0.5f, 0.25f, 0.01f, 0.6f, 2.6f, -1.25f};


    std::deque<CmdArray> traj_queue_;
    std::mutex traj_mutex_;

    CmdArray last_cmd_;

    std::atomic<bool> running_;
    std::thread thread_;

    std::array<size_t, NUM_JOINTS> stuck_cnt_;
    bool check_joint_safety(float v_thresh = 0.01f, float tau_thresh_ratio = 0.7f, float stuck_time = 0.3f);
    std::ofstream state_log_;
    std::chrono::steady_clock::time_point t_start_;
};