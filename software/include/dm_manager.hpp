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
#include "dm.hpp"
#include "interpolator.hpp"

struct MITCmd {
    float kp, kd, q, dq, tau;
};

template <size_t N>
inline std::array<float, N> operator+(const std::array<float, N>& a, const std::array<float, N>& b) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) {
        out[i] = a[i] + b[i];
    }
    return out;
}

template <size_t N>
inline std::array<float, N> operator-(const std::array<float, N>& a, const std::array<float, N>& b) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) {
        out[i] = a[i] - b[i];
    }
    return out;
}

template <size_t N>
inline std::array<float, N> operator*(const std::array<float, N>& a, float b) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) {
        out[i] = a[i] * b;
    }
    return out;
}

class BaseMotorManager {
public:
    static constexpr size_t NUM_JOINTS = 7;
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

    // Reset: move all joints to zero and hold for move_time (default: 1.0s)
    virtual void reset(float move_time = 1.0f) = 0;
    float getControlFreq() {return control_freq;}
    float getControlCycle() {return control_cycle;}
    std::function<JointArray(const JointArray&, const JointArray&)> compute_dynamic_torque;
    void setEnableSafetyCheck(bool enable) { enable_safety_check_ = enable; }
    
protected:
    float control_freq, control_cycle;
    bool enable_safety_check_ = false;

};

class MotorManager: public BaseMotorManager {
public:
    enum class InterpolationMethod { Linear, CubicS, MinJerk };

    MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                 DM_Motor::Motor_Control* mc,
                 InterpolationMethod interplation_method = InterpolationMethod::MinJerk,
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

    void reset(float move_time = 1.0f) override;

private:

    void move_mit(const JointArray& q, const JointArray& dq, const JointArray& tau, JointArray kp, JointArray kd);
    void move_mit_interpolated(const JointArray& tau, const JointArray& kp, const JointArray& kd, float move_time);

    std::unique_ptr<Interpolator<JointArray, 3>> interpolator_;
    JointArray kp_ = uniform(10.0f);
    JointArray kd_ = uniform(0.1f);
    JointArray max_vel_ = {M_PI*1.0f, M_PI*1.0f, M_PI*1.0f, M_PI*1.25f, M_PI*1.25f, M_PI*1.25f, M_PI*0.5f}; 
    JointArray last_q_, last_dq_, last_tau_;
    std::chrono::steady_clock::time_point last_cmd_time_;

    void controlLoop();

    const std::array<DM_Motor::Motor*, NUM_JOINTS> motors_;
    DM_Motor::Motor_Control* mc_;

    std::deque<CmdArray> traj_queue_;
    std::mutex traj_mutex_;

    CmdArray last_cmd_;
    CmdArray filtered_cmd_; 
    float filter_alpha;
    MITCmd filter_cmd(const MITCmd& prev, const MITCmd& curr, float alpha);

    std::atomic<bool> running_;
    std::thread thread_;

    std::array<size_t, NUM_JOINTS> stuck_cnt_;
    bool check_joint_safety(JointArray target_q, JointArray target_v = {}, JointArray target_t = {}, JointArray kp = {}, JointArray kd = {});
    std::ofstream state_log_;
    std::chrono::steady_clock::time_point t_start_;
};