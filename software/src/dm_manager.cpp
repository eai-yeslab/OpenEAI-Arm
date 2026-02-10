#include "dm_manager.hpp"
#include <cmath>
#include <iostream>
#include <cstring>
#include <fstream>
#include <iomanip>

MotorManager::MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                           DM_Motor::Motor_Control* mc,
                           InterpolationMethod interpolation_method,
                           float filter_alpha,
                           float control_freq)
    : motors_(motors), mc_(mc), running_(false), BaseMotorManager(control_freq), filter_alpha(filter_alpha) {
    // 初始化last_cmd_:
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        mc_->refresh_motor_status(*motors_[i]);
        last_q_[i] = motors_[i]->Get_Position();
        mc_->switchControlMode(*motors_[i], DM_Motor::Control_Mode::MIT_MODE);
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (interpolation_method == InterpolationMethod::Linear)
        interpolator_ = std::make_unique<LinearInterpolator<JointArray, 3>>();
    else if (interpolation_method == InterpolationMethod::CubicS)
        interpolator_ = std::make_unique<CubicSInterpolator<JointArray, 3>>();
    else // MinJerk
        interpolator_ = std::make_unique<MinJerkInterpolator<JointArray, 3>>();
    interpolator_->fill(last_q_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    last_cmd_time_ = std::chrono::steady_clock::now();
}

MotorManager::~MotorManager() { stop(); }


void MotorManager::move(
    const JointArray& q,
    std::optional<JointArray> dq_arg,
    std::optional<JointArray> tau_arg,
    std::optional<JointArray> kp_arg,
    std::optional<JointArray> kd_arg,
    std::optional<float> move_time)
{
    JointArray kp = kp_arg ? *kp_arg : kp_;
    JointArray kd = kd_arg ? *kd_arg : kd_;

    // Current State
    JointArray q_curr = getPosition(), delta_q{};
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
        delta_q[j] = q[j] - q_curr[j];
    }

    // Calculate time interval
    auto now = std::chrono::steady_clock::now();
    float cmd_interval = std::chrono::duration<float>(now - last_cmd_time_).count();
    float dt = move_time.value_or(cmd_interval);
    dt = std::max(dt, control_cycle);

    // Velocity and torque parameter preparation
    JointArray dq_res = interpolator_->getVel(), tau_res{};
    if (dq_arg) dq_res = *dq_arg;
    else {
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            // float d_q = q[j] - last_q_[j];
            dq_res[j] = std::clamp(dq_res[j] / dt, -2.0f, 2.0f);
        }
    }
    if (tau_arg) tau_res = *tau_arg;

    // Automatic interpolation judgment / duration adjustment
    bool need_interpolate = false;
    float min_time = dt;
    for (size_t j=0; j < NUM_JOINTS; ++j) {
        float planned_vel = std::fabs(delta_q[j] / dt);
        if (planned_vel > max_vel_[j]) {
            need_interpolate = true;
            float required = std::fabs(delta_q[j]) / max_vel_[j];
            if (required > min_time) min_time = required;
        }
    }

    interpolator_->push_command(q);

    if ((need_interpolate && dt >= 0.1) || (move_time && *move_time > control_cycle) || (cmd_interval > control_cycle * 2))
        move_mit_interpolated(tau_res, kp, kd, min_time);
    else
        move_mit(q, dq_res, tau_res, kp, kd);

    last_dq_ = interpolator_->getVel();
    last_cmd_time_ = std::chrono::steady_clock::now();
}

void MotorManager::setPosition(const JointArray& q) {
    interpolator_->fill(q);
    move_mit(q, uniform(0.0f), uniform(0.0f), kp_, kd_);
}

void MotorManager::move_mit(const JointArray& q, const JointArray& dq, const JointArray& tau, JointArray kp, JointArray kd) {
    CmdArray cmd;
    for (size_t j=0; j<NUM_JOINTS; ++j)
        cmd[j] = MITCmd{kp[j], kd[j], q[j], dq[j], tau[j]};
    std::deque<CmdArray> single{cmd};
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(single);
}

void MotorManager::move_mit_interpolated(const JointArray& tau, const JointArray& kp, const JointArray& kd, float move_time)
{
    size_t N = std::max<size_t>(std::ceil(move_time / control_cycle), 1);
    auto new_traj = interpolator_->interpolate(N);
    std::deque<CmdArray> new_traj_deque;
    for (size_t step = 0; step < N; ++step) {
        CmdArray group;
        JointArray qi = new_traj[step];
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            group[j] = MITCmd{kp[j], kd[j], qi[j], 0, tau[j]};
        }
        new_traj_deque.push_back(group);
    }
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(new_traj_deque);
}

MotorManager::JointArray MotorManager::getPosition() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = motors_[i]->Get_Position();
    return ret;
}
MotorManager::JointArray MotorManager::getVelocity() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = motors_[i]->Get_Velocity();
    return ret;
}
MotorManager::JointArray MotorManager::getTau() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = motors_[i]->Get_tau();
    return ret;
}

BaseMotorManager::JointArray BaseMotorManager::uniform(float value) {
    std::array<float, NUM_JOINTS> ret;
    ret.fill(value);
    return ret;
}

void MotorManager::start() {
    t_start_ = std::chrono::steady_clock::now();
    state_log_.open("motor_state_log.csv");
    state_log_ << "timestamp_ms";
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_cmd_" << j << ",dq_cmd_" << j << ",tau_cmd_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_" << j << ",dq_" << j << ",tau_" << j;
    state_log_ << ",ok,kp,kd" << std::endl;
    running_ = true;
    thread_ = std::thread(&MotorManager::controlLoop, this);
}
void MotorManager::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void MotorManager::reset(float move_time) {
        JointArray reset_pose_ = {M_PI * 0.2f, -0.5f, -0.2f, 0.0f, M_PI * 0.2f, 0.25f, -0.96f};

        this->move(reset_pose_, std::nullopt, std::nullopt, std::nullopt, std::nullopt, move_time);
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(move_time * 1e6)));
    }


MITCmd MotorManager::filter_cmd(const MITCmd& prev, const MITCmd& curr, float alpha) {
    return MITCmd{
        curr.kp,
        curr.kd,
        alpha * prev.q   + (1-alpha) * curr.q,
        alpha * prev.dq  + (1-alpha) * curr.dq,
        alpha * prev.tau + (1-alpha) * curr.tau
    };
}


void MotorManager::controlLoop() {
    constexpr int inter_joint_delay_us = 200;
    int cycle_time_us = control_cycle * 1e6;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        mc_->refresh_motor_status(*motors_[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        last_q_[i] = motors_[i]->Get_Position();
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
    }
    interpolator_->fill(last_q_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        mc_->refresh_motor_status(*motors_[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        last_q_[i] = motors_[i]->Get_Position();
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
    }
    interpolator_->fill(last_q_);
    if (compute_dynamic_torque) {
        std::cout << "Gravity Compensation Enabled." << std::endl;
    }

    filtered_cmd_ = last_cmd_;
    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        CmdArray group;
        bool ok = false;

        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            if (!traj_queue_.empty()) {
                group = traj_queue_.front();
                traj_queue_.pop_front();
                ok = true;
            }
        }
        CmdArray input_cmd = ok ? group : last_cmd_;
        CmdArray send_cmd;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& prev = filtered_cmd_[j];
            const auto& curr = input_cmd[j];
            send_cmd[j] = filter_cmd(prev, curr, filter_alpha);

        }


        JointArray target_q = {}, target_v = {}, target_t = {}, target_kp = {}, target_kd = {};
        for (size_t j=0; j < NUM_JOINTS; ++j) {
            target_q[j] = send_cmd[j].q;
        }


        auto tau_computed = JointArray{};
        if (compute_dynamic_torque) {
            tau_computed = compute_dynamic_torque(getPosition(), target_q);
        }
        for (size_t j=0; j < NUM_JOINTS; ++j) {
            try {
                const auto& c = send_cmd[j];
                if (compute_dynamic_torque) {
                    mc_->control_mit(*motors_[j], c.kp, c.kd, c.q, c.dq, tau_computed[j]);
                    target_t[j] = tau_computed[j];
                }
                else {
                    mc_->control_mit(*motors_[j], c.kp, c.kd, c.q, c.dq, c.tau);
                    target_t[j] = c.tau;
                }
                target_v[j] = c.dq;
                target_kp[j] = c.kp;
                target_kd[j] = c.kd;
            } catch (...) {}
            usleep(inter_joint_delay_us);
        }
        

        if (!check_joint_safety(target_q, target_v, target_t, target_kp, target_kd)) {
            std::cerr << "Error: MotorManager detected unsafe state.\n";
        }

        filtered_cmd_ = send_cmd;

        if (ok)
            last_cmd_ = group;
        
        auto t_now = std::chrono::steady_clock::now();
        long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start_).count();
        auto q = getPosition();
        auto dq = getVelocity();
        auto tau = getTau();

        const CmdArray& cmd_group = input_cmd;

        state_log_ << ts;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& c = cmd_group[j];
            state_log_ << "," << c.q << "," << c.dq << "," << (compute_dynamic_torque ? tau_computed[j] : c.tau) ;
        }

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << q[j] << "," << dq[j] << "," << tau[j];

        state_log_ << "," << ok;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& c = cmd_group[j];
            state_log_ << "," << c.kp << "," << c.kd;
        }

        state_log_ << std::endl;
        
        auto t1 = std::chrono::steady_clock::now();
        auto used = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
        if (used < cycle_time_us) usleep(cycle_time_us-used);
    }
    if (state_log_.is_open())
        state_log_.close();
}

bool MotorManager::check_joint_safety(JointArray target_q, JointArray target_v, JointArray target_t, JointArray kp, JointArray kd)
{
    if (!enable_safety_check_) return true;
    float v_thresh = 0.05f;
    float stuck_time = 0.5f;
    static constexpr float POS_TOL = 0.1f; // stuck error pos tolerance
    size_t stuck_cnt_lim = static_cast<size_t>(stuck_time / control_cycle);

    auto pos = getPosition();
    auto vel = getVelocity();
    auto tau = getTau();
  
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
        if (std::fabs(vel[j]) < v_thresh && std::fabs(pos[j] - target_q[j]) > POS_TOL) {
            stuck_cnt_[j]++;
            if (stuck_cnt_[j] > stuck_cnt_lim) {
                std::cerr << "[Safety] Joint " << j+1 << " stuck for " << stuck_time << "s @"
                          << pos[j] << std::endl;
                stuck_cnt_[j] = 0;
                return false;
            }
        } else {
            stuck_cnt_[j] = 0;
        }
        float expected_tau = kp[j] * (target_q[j] - pos[j]) - kd[j] * vel[j] + target_t[j];
        float delta_tau = std::fabs(expected_tau - tau[j]);
        if (delta_tau > 0.5) {
            std::cerr << "[Safety] Joint " << j+1 << " delta torque over safe limit: "
                      << delta_tau << " (max_safe=" << 0.5 << ")." << "Expected: " << expected_tau << ", Measured: " << tau[j] << std::endl;
            return false;
        }
    }
    return true;
}  

void MotorManager::refresh(){
    for (size_t j=0; j < NUM_JOINTS; ++j) {
        mc_->refresh_motor_status(*motors_[j]);
    }
}