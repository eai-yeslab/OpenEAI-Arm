#include "sim_motor_manager.hpp"
#include <iostream>
#include <thread>
#include <iomanip>
#include <cmath>

SimMotorManager::SimMotorManager(const std::string &urdf_path,
                   float control_freq)
    : model_(), data_(), BaseMotorManager(control_freq)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    // Joint name list
    for(size_t j=0;j<NUM_JOINTS;++j) joint_names_[j] = model_.names[j+1];
    // Initial position velocity
    q_ = reset_pose_;
    // q_.fill(0.0f);
    dq_.fill(0.0f); tau_.fill(0.0f);
    kp_.fill(3.0f); kd_.fill(0.1f);
    last_cmd_.fill(MITCmd{kp_[0],kd_[0],0,0,0});
    // Data log
    state_log_.open("sim_motor_state_log.csv");
    state_log_ << "timestamp_ms";
    for(size_t j=0;j<NUM_JOINTS;++j)
        state_log_ << ",q_cmd_" << j << ",dq_cmd_" << j << ",tau_cmd_" << j;
    for(size_t j=0;j<NUM_JOINTS;++j)
        state_log_ << ",q_" << j << ",dq_" << j << ",tau_" << j;
    state_log_ << ",ok" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    last_cmd_time_ = std::chrono::steady_clock::now();
}

SimMotorManager::SimMotorManager(const std::string &urdf_path, 
                const JointArray& reset_pose,
                   float control_freq)
    : model_(), data_(), BaseMotorManager(control_freq)
{
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    // Joint name list
    for(size_t j=0;j<NUM_JOINTS;++j) joint_names_[j] = model_.names[j+1];
    // Initial position velocity
    q_ = reset_pose_;
    // q_.fill(0.0f);
    dq_.fill(0.0f); tau_.fill(0.0f);
    kp_.fill(3.0f); kd_.fill(0.1f);
    last_cmd_.fill(MITCmd{kp_[0],kd_[0],0,0,0});
    // Data log
    state_log_.open("sim_motor_state_log.csv");
    state_log_ << "timestamp_ms";
    for(size_t j=0;j<NUM_JOINTS;++j)
        state_log_ << ",q_cmd_" << j << ",dq_cmd_" << j << ",tau_cmd_" << j;
    for(size_t j=0;j<NUM_JOINTS;++j)
        state_log_ << ",q_" << j << ",dq_" << j << ",tau_" << j;
    state_log_ << ",ok" << std::endl;
    reset_pose_ = reset_pose;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    last_cmd_time_ = std::chrono::steady_clock::now();
}

SimMotorManager::~SimMotorManager()
{
    running_ = false;
    if(state_log_.is_open()) state_log_.close();
    stop();
}


void SimMotorManager::move(
    const JointArray& q,
    std::optional<JointArray> dq_arg,
    std::optional<JointArray> tau_arg,
    std::optional<JointArray> kp_arg,
    std::optional<JointArray> kd_arg,
    std::optional<float> move_time)
{
    // Get the actual kp/kd to use
    JointArray kp = kp_arg ? *kp_arg : kp_;
    JointArray kd = kd_arg ? *kd_arg : kd_;

    // Fill in default values for dq, tau
    JointArray dq_res{}, tau_res{}, q_curr{}, delta_q{};
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
        q_curr[j] = q_[j];
        delta_q[j] = (q[j]- q_curr[j]);
    }
    
    float dt = 0.0f;
    auto now = std::chrono::steady_clock::now();
    float cmd_interval = std::chrono::duration<float>(now - last_cmd_time_).count();
    dt = (move_time && *move_time > control_cycle) ?  *move_time : 
        ((cmd_interval > control_cycle) ? cmd_interval : control_cycle);
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
        delta_q[j] = (q[j]- last_q_[j]);
        dq_res[j]  = dq_arg ? (*dq_arg)[j] : std::clamp(delta_q[j] / dt, -2.0f, 2.0f);
    }

    if (tau_arg) tau_res = *tau_arg;

    bool need_interpolate = false;
    float min_time = dt;
    for (size_t j=0; j<NUM_JOINTS; ++j) {
        float planned_vel = std::fabs(delta_q[j] / dt);
        if (planned_vel > max_vel_[j]) {
            need_interpolate = true;
            float required = std::fabs(delta_q[j]) / max_vel_[j];
            if (required > min_time) min_time = required;
        }
    }

    if ((need_interpolate && dt >= 0.1) || (move_time && *move_time > control_cycle) || (cmd_interval > control_cycle * 2))
        move_mit_s_interpolated(last_q_, q, dq_res, tau_res, kp, kd, min_time);
    else
        move_mit(q, dq_res, tau_res, kp, kd);

    last_q_ = q;
    last_dq_ = dq_res;
    last_cmd_time_ = std::chrono::steady_clock::now();
}

void SimMotorManager::move_mit(const JointArray& q, const JointArray& dq, const JointArray& tau, JointArray kp, JointArray kd) {
    CmdArray cmd;
    for (size_t j=0; j<NUM_JOINTS; ++j)
        cmd[j] = MITCmd{kp[j], kd[j], q[j], dq[j], tau[j]};
    std::deque<CmdArray> single{cmd};
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(single);
}

SimMotorManager::JointArray SimMotorManager::s_curve(float step, const JointArray& curr_q, const JointArray& target_q)
{
    step = 3 * step * step - 2 * step * step * step;
    JointArray q;
    for(size_t i=0; i<target_q.size(); ++i)
        q[i] = curr_q[i] + (target_q[i] - curr_q[i]) * step;
    return q;
}


void SimMotorManager::move_mit_s_interpolated(const JointArray& q0, const JointArray& q, const JointArray& dq, const JointArray& tau, JointArray kp, JointArray kd, float move_time) {
    size_t N = std::max<size_t>(std::ceil(move_time/control_cycle), 1);
    std::deque<CmdArray> new_traj;
    for (size_t step=1; step<=N; ++step) {
        CmdArray group;
        JointArray qi = s_curve(float(step) / N, q0, q);
        for (size_t j=0;j<NUM_JOINTS;++j) {
            group[j] = MITCmd{kp[j], kd[j], qi[j], 0, tau[j]};
        }
        new_traj.push_back(group);
    }
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(new_traj);
}

void SimMotorManager::move_mit_linear_interpolated(const JointArray& q0, const JointArray& q, const JointArray& dq, const JointArray& tau, JointArray kp, JointArray kd, float move_time) {
    size_t N = std::max<size_t>(std::ceil(move_time/control_cycle), 1);
    std::deque<CmdArray> new_traj;
    for (size_t step=1; step<=N; ++step) {
        float s = float(step)/N;
        CmdArray group;
        for (size_t j=0;j<NUM_JOINTS;++j) {
            float qi = q0[j] + (q[j] - q0[j]) * s;
            float vi = 0 ;
            group[j] = MITCmd{kp[j], kd[j], qi, vi, tau[j]};
        }
        new_traj.push_back(group);
    }
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(new_traj);
}


SimMotorManager::JointArray SimMotorManager::getPosition() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = q_[i];
    return ret;
}
SimMotorManager::JointArray SimMotorManager::getVelocity() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = dq_[i];
    return ret;
}
SimMotorManager::JointArray SimMotorManager::getTau() {
    JointArray ret;
    for (size_t i = 0; i < NUM_JOINTS; ++i) ret[i] = tau_[i];
    return ret;
}

void SimMotorManager::start() {
    t_start_ = std::chrono::steady_clock::now();
    running_ = true;
    thread_ = std::thread(&SimMotorManager::controlLoop, this);
}
void SimMotorManager::stop() {
    running_ = false;
    if (thread_.joinable())
        thread_.join();
}

void SimMotorManager::mc_control(size_t idx, float new_q, float new_dq) {
    dq_[idx] = (new_q - q_[idx]) / control_cycle;
    q_[idx] = new_q;
    Eigen::VectorXd q_eigen(NUM_JOINTS - 1);
    for(size_t i=0; i < NUM_JOINTS - 1; ++i) q_eigen[i] = q_[i];
    pinocchio::forwardKinematics(model_, data_, q_eigen);
    Eigen::VectorXd dq_eigen(NUM_JOINTS - 1);
    for(size_t i=0; i < NUM_JOINTS - 1; ++i) dq_eigen[i] = dq_[i];
    auto tau_eigen = pinocchio::rnea(model_, data_, q_eigen, dq_eigen, Eigen::VectorXd(NUM_JOINTS - 1)); 
    for(size_t i=0; i < NUM_JOINTS - 1; ++i) tau_[i] = tau_eigen[i];
}

void SimMotorManager::reset(float move_time) {
        this->move(reset_pose_, std::nullopt, std::nullopt, std::nullopt, std::nullopt, move_time);
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(move_time * 1e6)));
    }

void SimMotorManager::controlLoop() {
    constexpr int inter_joint_delay_us = 200;
    int cycle_time_us = control_cycle * 1e6;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        last_q_[i] = q_[i];
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        last_q_[i] = q_[i];
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
    }
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
        
        auto tau_computed = JointArray{};
        if (!ok) {
            if (compute_dynamic_torque) {
                JointArray target_q = {};
                for (size_t j=0; j < NUM_JOINTS; ++j) {
                    target_q[j] = last_cmd_[j].q;
                }
                tau_computed = compute_dynamic_torque(getPosition(), target_q);
            }
            for (size_t j=0; j < NUM_JOINTS; ++j) {
                try {
                    const auto& c = last_cmd_[j];
                    mc_control(j, c.q, c.dq);
                } catch (...) {}
                usleep(inter_joint_delay_us);
            }
        } else {
            if (compute_dynamic_torque) {
                JointArray target_q = {};
                for (size_t j=0; j < NUM_JOINTS; ++j) {
                    target_q[j] = group[j].q;
                }
                tau_computed = compute_dynamic_torque(getPosition(), target_q);
            }
            for (size_t j=0; j < NUM_JOINTS; ++j) {
                try {
                    const auto& c = group[j];
                    mc_control(j, c.q, c.dq);
                } catch (std::exception& e) {
                    std::cerr << "CMD exception: " << e.what() << std::endl;
                }
                usleep(inter_joint_delay_us);
            }
            
            last_cmd_ = group;
        }
        auto t_now = std::chrono::steady_clock::now();
        long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start_).count();
        auto q = getPosition();
        auto dq = getVelocity();
        auto tau = getTau();

        const CmdArray& cmd_group = ok ? group : last_cmd_;

        state_log_ << ts;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& c = cmd_group[j];
            state_log_ << "," << c.q << "," << c.dq << "," << (compute_dynamic_torque ? tau_computed[j] : c.tau) ;
        }

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << q[j] << "," << dq[j] << "," << tau[j];

        state_log_ << "," << ok << std::endl;
        
        auto t1 = std::chrono::steady_clock::now();
        auto used = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
        if (used < cycle_time_us) usleep(cycle_time_us-used);
    }
    if (state_log_.is_open())
        state_log_.close();
}

bool SimMotorManager::check_joint_safety(float v_thresh, float tau_thresh_ratio, float stuck_time)
{
    static constexpr float POS_TOL = 0.1f;
    size_t stuck_cnt_lim = static_cast<size_t>(stuck_time / 0.01f);
  
    // read feedbacks
    auto pos = getPosition();
    auto vel = getVelocity();
    auto tau = getTau();
  
    for (size_t j = 0; j < NUM_JOINTS; ++j) {
        float max_tau = 9.0f;
        // torque protection
        if (std::fabs(tau[j]) > (max_tau * tau_thresh_ratio)) {
            std::cerr << "[Safety] Joint " << j << " torque over threshold: " << tau[j]
            << " (max=" << max_tau << ")" << std::endl;
            return false;
        }
    }
    return true;
}  

void SimMotorManager::refresh(){}
