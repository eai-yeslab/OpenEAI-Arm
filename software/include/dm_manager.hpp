#pragma once
#include <vector>
#include <optional>
#include <fstream>
#include <array>
#include <deque>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <cassert>
#include <variant>
#include <functional>
#include "config_loader.hpp"
#include "dm.hpp"
#include "joint_array.hpp"
#include "interpolator.hpp"

struct MITCmd {
    float kp, kd, q, dq, tau;
};


class BaseMotorManager {
public:
    static constexpr size_t NUM_JOINTS = 7;
    static constexpr size_t GRIPPER_INDEX = NUM_JOINTS - 1;
    using CmdArray = std::array<MITCmd, NUM_JOINTS>;
    using JointArray = std::array<float, NUM_JOINTS>;

    BaseMotorManager(float control_freq): control_freq(control_freq), control_cycle(1.0f / control_freq) {}

    virtual void move(
        const JointArray& q,
        std::optional<JointArray> dq = std::nullopt,
        std::optional<JointArray> tau = std::nullopt,
        std::optional<JointArray> kp = std::nullopt,
        std::optional<JointArray> kd = std::nullopt,
        std::optional<float> move_time = std::nullopt) = 0;

    virtual std::array<float, NUM_JOINTS> getPosition() = 0;
    virtual std::array<float, NUM_JOINTS> getVelocity() = 0;
    virtual std::array<float, NUM_JOINTS> getTau() = 0;
    std::array<float, NUM_JOINTS> uniform(float value);

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void refresh() = 0;
    virtual void setPosition(const JointArray& q) = 0;

    // Reset: move all joints to zero and hold for move_time (default: 1.0s)
    virtual void reset(float move_time = 1.0f) = 0;
    float getControlFreq() {return control_freq;}
    float getControlCycle() {return control_cycle;}
    std::function<JointArray(const JointArray&, const JointArray&)> compute_dynamic_torque;
    std::function<JointArray(const JointArray&, const JointArray&,
                             const JointArray&, const JointArray&)>
        compute_full_dynamic_torque;
    void setEnableSafetyCheck(bool enable) { enable_safety_check_ = enable; }
    
protected:
    float control_freq, control_cycle;
    bool enable_safety_check_ = false;

};

class MotorManager: public BaseMotorManager {
public:
    enum class InterpolationMethod { Linear, CubicS, MinJerk };

    // Backward-compatible constructor retained for existing public examples.
    MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                 DM_Motor::Motor_Control* mc,
                 InterpolationMethod interpolation_method = InterpolationMethod::MinJerk,
                 float filter_alpha = 0.9f,
                 float control_freq = 200.0f);

    MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                 DM_Motor::Motor_Control* mc,
                 InterpolationConfig interpolation_config,
                 MotionFilterConfig motion_filter_config,
                 const JointArray& joint_vmax,
                 const JointArray& joint_amax,
                 float filter_alpha = 0.9,
                 float control_freq = 200,
                 const std::string& state_log_path = "motor_state_log.csv");

    // Backward-compatible global-limit overload.
    MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                 DM_Motor::Motor_Control* mc,
                 InterpolationConfig interpolation_config,
                 MotionFilterConfig motion_filter_config,
                 float filter_alpha = 0.9,
                 float control_freq = 200);

    // Backward-compatible constructor for apps that only provide interpolation
    // settings. The post-interpolation filter reuses the same vmax/amax and
    // defaults to zero velocity feedforward.
    MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                 DM_Motor::Motor_Control* mc,
                 InterpolationConfig interpolation_config,
                 float filter_alpha = 0.9,
                 float control_freq = 200);
    ~MotorManager();


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
    void setPosition(const JointArray& q) override;

    void reset(float move_time = 1.0f) override;

private:

    struct InputShaperTap {
        std::size_t delay_frames = 0;
        float weight = 0.0f;
    };

    struct IIRNotchState {
        bool initialized = false;
        float q_x1 = 0.0f;
        float q_x2 = 0.0f;
        float q_y1 = 0.0f;
        float q_y2 = 0.0f;
        float dq_x1 = 0.0f;
        float dq_x2 = 0.0f;
        float dq_y1 = 0.0f;
        float dq_y2 = 0.0f;
    };

    struct TrajectoryFrame {
        CmdArray command{};
        bool apply_delay_smoothing = false;
    };

    void move_mit(const JointArray& q, const JointArray& dq, const JointArray& tau,
                  JointArray kp, JointArray kd, bool apply_delay_smoothing);
    void move_mit_interpolated(const JointArray& tau, const JointArray& kp,
                               const JointArray& kd, float move_time,
                               bool apply_delay_smoothing);

    std::unique_ptr<Interpolator<JointArray, 3>> interpolator_;
    std::unique_ptr<Interpolator<JointArray, 3>> test_interpolator_;
    JointArray kp_ = uniform(10.0f);
    JointArray kd_ = uniform(0.1f);
    JointArray joint_vmax_{};
    JointArray joint_amax_{};
    std::string interpolation_method_;
    JointArray last_q_, last_dq_, last_tau_;
    std::chrono::steady_clock::time_point last_cmd_time_;

    void controlLoop();

    const std::array<DM_Motor::Motor*, NUM_JOINTS> motors_;
    DM_Motor::Motor_Control* mc_;

    std::deque<TrajectoryFrame> traj_queue_;
    bool traj_queue_updated_ = false;
    std::mutex traj_mutex_;

    CmdArray last_cmd_;
    CmdArray filtered_cmd_; 
    JointArray external_target_{};
    JointArray motion_filter_q_{};
    JointArray motion_filter_dq_{};
    MotionFilterConfig motion_filter_config_;
    std::deque<CmdArray> input_shaper_history_;
    std::vector<InputShaperTap> input_shaper_taps_;
    std::size_t input_shaper_max_delay_ = 0;
    std::size_t input_shaper_gripper_delay_ = 0;
    std::array<IIRNotchState, GRIPPER_INDEX> iir_notch_states_{};
    float iir_b0_ = 1.0f;
    float iir_b1_ = 0.0f;
    float iir_b2_ = 0.0f;
    float iir_a1_ = 0.0f;
    float iir_a2_ = 0.0f;
    std::deque<CmdArray> delay_smoothing_history_;
    std::vector<float> delay_smoothing_weights_;
    bool last_delay_smoothing_enabled_ = false;
    float filter_alpha;
    MITCmd filter_cmd(const MITCmd& prev, const MITCmd& curr, float alpha);
    MITCmd filter_motion_frame(size_t joint, const MITCmd& prev, const MITCmd& curr);
    CmdArray shape_input(const CmdArray& curr, bool apply_delay_smoothing);
    CmdArray smooth_delayed_input(const CmdArray& curr, bool apply_delay_smoothing);

    std::atomic<bool> running_;
    std::thread thread_;

    std::array<size_t, NUM_JOINTS> stuck_cnt_;
    bool check_joint_safety(JointArray target_q, JointArray target_v = {}, JointArray target_t = {}, JointArray kp = {}, JointArray kd = {});
    std::ofstream state_log_;
    std::string state_log_path_;
    std::chrono::steady_clock::time_point t_start_;
};
