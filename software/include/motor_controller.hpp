#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "kd_solver.hpp"

class BaseController {
public:
    static constexpr size_t NUM_JOINTS = 7;
    using JointArray = std::array<float, NUM_JOINTS>;
    JointArray uniform(float value);
    void setResetPose(JointArray reset_pose) { reset_pose_ = reset_pose; }

    JointArray computeControlTorque(const JointArray& current_q, const JointArray& target_q, bool output = false) {return JointArray{};};


protected:
    JointArray q_phys_to_control(const JointArray& q_phys) const;
    JointArray reset_pose_{};

};

class AdvancedPIDController: public BaseController {

public:
    AdvancedPIDController(float Ts = 0.001f);
    AdvancedPIDController(float Ts, std::shared_ptr<KDSolver>kd_solver);
    void setKpKiParams(JointArray Kp, JointArray Ki) {Kp_ = Kp; Ki_ = Ki;}
    void setNonlinearParams(JointArray alpha, float deadzone) { alpha_ = alpha; error_deadzone_ = deadzone; }
    void setOutputLimits(float min, float max) { output_min_ = min; output_max_ = max; }
    void setDynamicsParams(const JointArray& coriolis_scale,
                           const JointArray& inertia_scale,
                           const JointArray& acceleration_limit) {
        coriolis_scale_ = coriolis_scale;
        inertia_scale_ = inertia_scale;
        acceleration_limit_ = acceleration_limit;
    }

    void reset() {
        integral_.fill(0.0f);
        prev_error_.fill(0.0f);
        prev_measurement_.fill(0.0f);
        first_run_ = true;
    }


    // Full rigid-body feedforward torque calculation.
    JointArray computeFeedforwardTorque(const JointArray& current_q,
                                        const JointArray& target_dq,
                                        const JointArray& target_ddq);

    // Compatibility entry point without requested acceleration feedforward.
    JointArray computeControlTorque(const JointArray& q, const JointArray& target,
                                    const JointArray& dq, bool output = false) {
        return computeControlTorque(q, target, dq, JointArray{}, output);
    }

    // Nonlinear proportional term with deadzone
    float nonlinearProportional(float Kp, float alpha, float error) const;

    // Integral update with anti-windup
    void updateIntegralWithAntiWindup(float Ki, float error, float& integral, float p_term, float g_term, int joint_idx);

    // Main control function
    JointArray computeControlTorque(
        const JointArray& current_q,
        const JointArray& target_q,
        bool output = false) {
        return computeControlTorque(
            current_q, target_q, JointArray{}, JointArray{}, output);
    }

    JointArray computeControlTorque(
        const JointArray& current_q,
        const JointArray& target_q,
        const JointArray& target_dq,
        const JointArray& target_ddq,
        bool output = false);

private:

    // PID parameters
    JointArray Kp_, Ki_;
    float Ts_; // Sampling time

    // Nonlinear parameters
    JointArray alpha_; // tanh nonlinearity coefficient
    JointArray coriolis_scale_{};
    JointArray inertia_scale_{};
    JointArray acceleration_limit_{};
    float error_deadzone_; // error deadzone

    // Anti-windup parameters
    float output_min_, output_max_;

    // Controller state
    JointArray integral_{};
    JointArray prev_error_{};
    JointArray prev_measurement_{};
    bool first_run_ = true;

    std::shared_ptr<KDSolver> kd_solver;
};
