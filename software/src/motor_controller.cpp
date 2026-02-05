#include "motor_controller.hpp"


BaseController::JointArray BaseController::q_phys_to_control(const JointArray& q_phys) const {
    JointArray q;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        q[i] = q_phys[i] - reset_pose_[i];
    }
    return q;
}

BaseController::JointArray BaseController::uniform(float value) {
    JointArray ret;
    ret.fill(value);
    return ret;
}

AdvancedPIDController::AdvancedPIDController(float Ts): Ts_(Ts) {
    // 默认参数
    Kp_ = {1.0f, 1.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.5f};
    Ki_ = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};
    alpha_ = {50.0f, 50.0f, 50.0f, 5.0f, 5.0f, 5.0f, 5.0f};           // tanh nonlinear strength
    error_deadzone_ = 0.001f;
    output_min_ = -9.0f;
    output_max_ = 9.0f;

    // 初始化状态
    integral_.fill(0.0f);
    prev_error_.fill(0.0f);
    prev_measurement_.fill(0.0f);


    kd_solver = std::make_shared<KDSolver>("assets/openeai_arm_urdf_ros2/urdf/STEP.urdf", "base_link", "link6");
}

AdvancedPIDController::AdvancedPIDController(float Ts, std::shared_ptr<KDSolver>kd_solver): Ts_(Ts), kd_solver(kd_solver) {
    Kp_ = {1.0f, 1.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.5f};
    Ki_ = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};
    alpha_ = {50.0f, 50.0f, 50.0f, 5.0f, 5.0f, 5.0f, 5.0f};           // tanh nonlinear strength
    error_deadzone_ = 0.001f;
    output_min_ = -9.0f;
    output_max_ = 9.0f;

    integral_.fill(0.0f);
    prev_error_.fill(0.0f);
    prev_measurement_.fill(0.0f);
}


AdvancedPIDController::JointArray AdvancedPIDController::computeGravityTorque(JointArray q) {
    q = q_phys_to_control(q);

    std::vector<double> q_in(q.begin(), q.end() - 1), t_out;

    bool success = kd_solver->gravityCompensation(q_in, t_out);
    std::array<float, 7> tau{};
    if (success) {
        std::transform(t_out.begin(), t_out.begin() + (NUM_JOINTS - 1), tau.begin(),
               [](double x) { return static_cast<float>(x); });
    }
    else {
        std::cout << "Something wrong!" << std::endl;
    }
    return tau;
}

float AdvancedPIDController::nonlinearProportional(float Kp, float alpha, float error) const {
    // Deadzone handling
    if (std::fabs(error) < error_deadzone_) {
        return 0.0f;
    }

    // Tanh nonlinear P term - provides smooth saturation characteristics
    return Kp * std::tanh(alpha * error);
}

void AdvancedPIDController::updateIntegralWithAntiWindup(float Ki, float error, float& integral, float p_term, float g_term, int joint_idx) {
    // Calculate theoretical integral update
    float integral_increment = Ki * Ts_ * error;
    float new_integral = integral + integral_increment;

    // Calculate theoretical total output (PID part only, excluding gravity compensation)
    float u_pid_raw = p_term + new_integral + g_term;

    // Anti-windup check
    if (u_pid_raw >= output_min_ && u_pid_raw <= output_max_) {
        // Not saturated, update integral normally
        integral = new_integral;
    }
    // Optional: when saturated, keep integral unchanged (Clamping method)
    // else: integral = integral (no change)

    // Optional: clamp integral term for additional protection
    // integral = std::clamp(integral, -10.0f, 10.0f);
}

AdvancedPIDController::JointArray AdvancedPIDController::computeControlTorque(const JointArray& current_q, const JointArray& target_q, bool output) {
    if (output) std::cout << "============== Tau args ==============" << std::endl;

    JointArray control_torque{};
    JointArray current_error{};

    // Calculate gravity compensation
    JointArray gravity_torque = computeGravityTorque(current_q);

    for (int i = 0; i < 7; ++i) {
        // 1. Calculate current error
        current_error[i] = target_q[i] - current_q[i];

        // 2. Calculate nonlinear proportional term
        float p_term = nonlinearProportional(Kp_[i], alpha_[i], current_error[i]);

        // 3. Update integral term with anti-windup
        updateIntegralWithAntiWindup(Ki_[i], current_error[i], integral_[i], p_term, gravity_torque[i], i);

        // 4. Calculate PID output
        float pid_output = p_term + integral_[i] + gravity_torque[i];

        // 5. Output clamping
        pid_output = std::clamp(pid_output, output_min_, output_max_);


        // Update state
        prev_error_[i] = current_error[i];
        prev_measurement_[i] = current_q[i];
        control_torque[i] = pid_output;

        // Debug output
        if (output) {
            std::cout << "Joint " << i << ": ";
            std::cout << "Error=" << current_error[i] << ", ";
            std::cout << "P=" << p_term << ", ";
            std::cout << "I=" << integral_[i] << ", ";
            std::cout << "PID=" << pid_output << ", ";
            std::cout << "Gravity=" << gravity_torque[i] << ", ";
            std::cout << "Total=" << control_torque[i] << std::endl;
        }
    }

    first_run_ = false;
    return control_torque;
}