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

    void reset() {
        integral_.fill(0.0f);
        prev_error_.fill(0.0f);
        prev_measurement_.fill(0.0f);
        first_run_ = true;
    }


    // Gravity compensation torque calculation
    JointArray computeGravityTorque(JointArray q);

    // Nonlinear proportional term with deadzone
    float nonlinearProportional(float Kp, float alpha, float error) const;

    // Integral update with anti-windup
    void updateIntegralWithAntiWindup(float Ki, float error, float& integral, float p_term, float g_term, int joint_idx);

    // Main control function
    JointArray computeControlTorque(const JointArray& current_q, const JointArray& target_q, bool output = false) ;

private:

    // PID parameters
    JointArray Kp_, Ki_;
    float Ts_; // Sampling time

    // Nonlinear parameters
    JointArray alpha_; // tanh nonlinearity coefficient
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