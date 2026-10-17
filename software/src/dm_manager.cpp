#include "dm_manager.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <stdexcept>

namespace {
MotorManager::JointArray uniform_limits(float value) {
    MotorManager::JointArray limits{};
    limits.fill(value);
    return limits;
}

InterpolationConfig legacy_interpolation_config(MotorManager::InterpolationMethod method) {
    InterpolationConfig config;
    switch (method) {
    case MotorManager::InterpolationMethod::Linear:
        config.method = "linear";
        break;
    case MotorManager::InterpolationMethod::CubicS:
        config.method = "cubics";
        break;
    case MotorManager::InterpolationMethod::MinJerk:
        config.method = "minjerk";
        break;
    }
    return config;
}
}

MotorManager::MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                           DM_Motor::Motor_Control* mc,
                           InterpolationMethod interpolation_method,
                           float filter_alpha,
                           float control_freq)
    : MotorManager(motors, mc,
                   legacy_interpolation_config(interpolation_method),
                   filter_alpha, control_freq) {}

MotorManager::MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                           DM_Motor::Motor_Control* mc,
                           InterpolationConfig interpolation_config,
                           float filter_alpha,
                           float control_freq)
    : MotorManager(
          motors,
          mc,
          interpolation_config,
          MotionFilterConfig{
              true,
              0.001f,
              0.01f,
              false,
              0,
              InputShaperConfig{}},
          uniform_limits(interpolation_config.vmax),
          uniform_limits(interpolation_config.amax),
          filter_alpha,
          control_freq) {}

MotorManager::MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                           DM_Motor::Motor_Control* mc,
                           InterpolationConfig interpolation_config,
                           MotionFilterConfig motion_filter_config,
                           float filter_alpha,
                           float control_freq)
    : MotorManager(motors, mc, interpolation_config, motion_filter_config,
                   uniform_limits(interpolation_config.vmax),
                   uniform_limits(interpolation_config.amax), filter_alpha, control_freq) {}

MotorManager::MotorManager(const std::array<DM_Motor::Motor*, NUM_JOINTS>& motors,
                           DM_Motor::Motor_Control* mc,
                           InterpolationConfig interpolation_config,
                           MotionFilterConfig motion_filter_config,
                           const JointArray& joint_vmax,
                           const JointArray& joint_amax,
                           float filter_alpha,
                           float control_freq,
                           const std::string& state_log_path)
    : motors_(motors), mc_(mc), running_(false), BaseMotorManager(control_freq),
      joint_vmax_(joint_vmax), joint_amax_(joint_amax),
      interpolation_method_(interpolation_config.method),
      motion_filter_config_(motion_filter_config), filter_alpha(filter_alpha),
      state_log_path_(state_log_path) {
    const InputShaperConfig& shaper = motion_filter_config_.input_shaper;
    if (shaper.enabled) {
        if (shaper.type != "zv" && shaper.type != "zvd" &&
            shaper.type != "iir_notch") {
            throw std::runtime_error(
                "input_shaper type must be 'zv', 'zvd', or 'iir_notch'");
        }
        if (!(shaper.frequency_hz > 0.0f) ||
            !(shaper.frequency_hz < control_freq * 0.5f)) {
            throw std::runtime_error(
                "input_shaper frequency_hz must be positive and below Nyquist frequency");
        }
        const float pi = std::acos(-1.0f);
        if (shaper.type == "iir_notch") {
            if (!(shaper.bandwidth_hz > 0.0f) ||
                !(shaper.bandwidth_hz < control_freq * 0.5f)) {
                throw std::runtime_error(
                    "input_shaper bandwidth_hz must be positive and below Nyquist frequency");
            }

            const float omega =
                2.0f * pi * shaper.frequency_hz / control_freq;
            const float pole_radius =
                std::exp(-pi * shaper.bandwidth_hz / control_freq);
            const float cosine = std::cos(omega);
            const float dc_gain =
                (1.0f - 2.0f * pole_radius * cosine +
                 pole_radius * pole_radius) /
                (2.0f - 2.0f * cosine);
            iir_b0_ = dc_gain;
            iir_b1_ = -2.0f * dc_gain * cosine;
            iir_b2_ = dc_gain;
            iir_a1_ = -2.0f * pole_radius * cosine;
            iir_a2_ = pole_radius * pole_radius;

            std::cout << "Input shaper enabled: type=iir_notch"
                      << ", frequency_hz=" << shaper.frequency_hz
                      << ", bandwidth_hz=" << shaper.bandwidth_hz
                      << ", pole_radius=" << pole_radius << std::endl;
        } else {
            if (!(shaper.damping_ratio >= 0.0f) ||
                !(shaper.damping_ratio < 1.0f)) {
                throw std::runtime_error(
                    "input_shaper damping_ratio must be in [0, 1)");
            }

            const float damped_ratio =
                std::sqrt(1.0f - shaper.damping_ratio * shaper.damping_ratio);
            const auto half_period_frames = static_cast<std::size_t>(std::llround(
                control_freq /
                (2.0f * shaper.frequency_hz * damped_ratio)));
            if (half_period_frames == 0) {
                throw std::runtime_error(
                    "input_shaper frequency_hz produces a sub-frame tap delay");
            }

            const float residual_ratio = std::exp(
                -shaper.damping_ratio * pi / damped_ratio);
            if (shaper.type == "zv") {
                const float normalization = 1.0f + residual_ratio;
                input_shaper_taps_ = {
                    InputShaperTap{0, 1.0f / normalization},
                    InputShaperTap{half_period_frames,
                                   residual_ratio / normalization}};
            } else {
                const float normalization =
                    (1.0f + residual_ratio) * (1.0f + residual_ratio);
                input_shaper_taps_ = {
                    InputShaperTap{0, 1.0f / normalization},
                    InputShaperTap{half_period_frames,
                                   2.0f * residual_ratio / normalization},
                    InputShaperTap{2 * half_period_frames,
                                   residual_ratio * residual_ratio / normalization}};
            }

            float weighted_delay = 0.0f;
            for (const InputShaperTap& tap : input_shaper_taps_) {
                input_shaper_max_delay_ =
                    std::max(input_shaper_max_delay_, tap.delay_frames);
                weighted_delay +=
                    tap.weight * static_cast<float>(tap.delay_frames);
            }
            input_shaper_gripper_delay_ = 0;
                // static_cast<std::size_t>(std::llround(weighted_delay));
            std::cout << "Input shaper enabled: type=" << shaper.type
                      << ", frequency_hz=" << shaper.frequency_hz
                      << ", damping_ratio=" << shaper.damping_ratio
                      << ", tap_spacing_frames=" << half_period_frames
                      << ", equivalent_delay_frames="
                      << input_shaper_gripper_delay_ << std::endl;
        }
    }

    const size_t smoothing_window = 2 * motion_filter_config_.delay_frames + 1;
    delay_smoothing_weights_.resize(smoothing_window, 1.0f);
    if (smoothing_window > 1) {
        const float pi = std::acos(-1.0f);
        float weight_sum = 0.0f;
        for (size_t i = 0; i < smoothing_window; ++i) {
            delay_smoothing_weights_[i] =
                0.5f - 0.5f * std::cos(2.0f * pi * static_cast<float>(i) /
                                       static_cast<float>(smoothing_window - 1));
            weight_sum += delay_smoothing_weights_[i];
        }
        for (float& weight : delay_smoothing_weights_) {
            weight /= weight_sum;
        }
    }
    // 初始化last_cmd_:
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        mc_->refresh_motor_status(*motors_[i]);
        last_q_[i] = motors_[i]->Get_Position();
        mc_->switchControlMode(*motors_[i], DM_Motor::Control_Mode::MIT_MODE);
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
        motion_filter_q_[i] = last_q_[i];
        motion_filter_dq_[i] = 0.0f;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    const std::string& method = interpolation_config.method;
    if (method == "linear") {
        interpolator_ = std::make_unique<LinearInterpolator<JointArray, 3>>();
        test_interpolator_ = std::make_unique<LinearInterpolator<JointArray, 3>>();
    }
    else if (method == "cubics") {
        interpolator_ = std::make_unique<CubicSInterpolator<JointArray, 3>>();
        test_interpolator_ = std::make_unique<CubicSInterpolator<JointArray, 3>>();
    }
    else if (method == "minjerk") {
        interpolator_ = std::make_unique<MinJerkInterpolator<JointArray, 3>>();
        test_interpolator_ = std::make_unique<MinJerkInterpolator<JointArray, 3>>();
    }
    else {
        throw std::runtime_error("unsupported interpolation method: " + method);
    }
    interpolator_->fill(last_q_);
    test_interpolator_->fill(last_q_);
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
    external_target_ = q;
    // std::cout << "==============================================================================================" << std::endl;

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
    JointArray dq_res = interpolator_->getVel(dt), tau_res{};
    if (dq_arg) {
        // dq_res = *dq_arg;
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            dq_res[j] = (*dq_arg)[j] / dt;
        }
    } else {
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            // float d_q = q[j] - last_q_[j];
            dq_res[j] = std::clamp(dq_res[j], -2.0f, 2.0f);
        }
    }
    if (tau_arg) tau_res = *tau_arg;

    // Automatic interpolation judgment / duration adjustment
    bool need_interpolate = false;
    float min_time = dt;
    for (size_t j = 0; j < GRIPPER_INDEX; ++j) {
        float planned_vel = std::fabs(delta_q[j] / dt);
        if (planned_vel > joint_vmax_[j]) {
            need_interpolate = true;
            break;
        }
    }

    interpolator_->push_command(q);
    test_interpolator_->push_command(q);
    // DEBUG
    // std::cout << "Pushing command: ";
    // for (size_t j=0; j < NUM_JOINTS; ++j) {
    //     std::cout << std::fixed << std::setprecision(4) << q[j] << (j < NUM_JOINTS - 1 ? ", " : "");
    // }
    // std::cout << " with dt: " << dt << "s" << std::endl;
    // std::cout << "Min time: " << min_time << " dt: " << dt << " cmd_interval: " << cmd_interval << " move_time: " << (move_time ? std::to_string(*move_time) : "nullopt") << std::endl;
    // ROS streaming commands explicitly carry one frame period as move_time.
    // Treat short positive intervals as stream frames, while zero-duration
    // mode holds and longer one-shot motions bypass and reset delayed smoothing.
    const bool apply_delay_smoothing =
        !move_time.has_value() || (*move_time > 0.0f && *move_time <= 0.1f);
    if ((need_interpolate && dt >= 0.1f) ||
        (move_time && *move_time > control_cycle) ||
        (cmd_interval > control_cycle * 2))
        move_mit_interpolated(tau_res, kp, kd, min_time, apply_delay_smoothing);
    else
        move_mit(q, dq_res, tau_res, kp, kd, apply_delay_smoothing);

    last_dq_ = interpolator_->getVel(dt);
    last_cmd_time_ = std::chrono::steady_clock::now();
}

void MotorManager::move_mit(const JointArray& q, const JointArray& dq,
                            const JointArray& tau, JointArray kp, JointArray kd,
                            bool apply_delay_smoothing) {
    CmdArray cmd{};
    for (size_t j=0; j<NUM_JOINTS; ++j) {
        cmd[j] = MITCmd{kp[j], kd[j], q[j], dq[j], tau[j]};
        assert(std::isfinite(cmd[j].q));
        assert(std::fabs(cmd[j].q) < 100);
    }
    std::deque<TrajectoryFrame> single{
        TrajectoryFrame{cmd, apply_delay_smoothing}};
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(single);
    traj_queue_updated_ = true;
}

void MotorManager::move_mit_interpolated(const JointArray& tau, const JointArray& kp,
                                         const JointArray& kd, float move_time,
                                         bool apply_delay_smoothing)
{
    size_t N = std::max<size_t>(std::ceil(move_time / control_cycle), 1);
    auto new_traj = interpolator_->interpolate(N, getPosition(), getVelocity(), move_time);
    const float gripper_target = (*interpolator_)[0][GRIPPER_INDEX];
    // DEBUG
    // auto new_traj_test = test_interpolator_->interpolate(N, getPosition(), getVelocity(), move_time);
    // // For debugging interpolation differences:
    // std::cout << "Interpolating to target over " << move_time << " seconds with " << N << " steps:" << std::endl;
    // std::cout << "Initial position: [";
    // for (size_t j = 0; j < NUM_JOINTS; ++j) {
    //     std::cout << std::fixed << std::setprecision(4) << getPosition()[j] << (j + 1 < NUM_JOINTS ? ", " : "");
    // }
    // std::cout << "], Initial velocity: [";
    // for (size_t j = 0; j < NUM_JOINTS; ++j) {
    //     std::cout << std::fixed << std::setprecision(4) << getVelocity()[j] << (j + 1 < NUM_JOINTS ? ", " : "");
    // }
    // std::cout << "]" << std::endl;
    // for (size_t step = 0; step < N; ++step) {
    //     std::cout << "Step " << step << ": Interpolator = [";
    //     for (size_t j = 0; j < NUM_JOINTS; ++j) {
    //         std::cout << std::fixed << std::setprecision(4) << new_traj[step][j] << (j < NUM_JOINTS - 1 ? ", " : "");
    //     }
    //     std::cout << "], Test Interpolator = [";
    //     for (size_t j = 0; j < NUM_JOINTS; ++j) {
    //         std::cout << std::fixed << std::setprecision(4) << new_traj_test[step][j] << (j < NUM_JOINTS - 1 ? ", " : "");
    //     }
    //     std::cout << "]" << std::endl;
    //     auto difference = new_traj[step] - new_traj_test[step];
    //     std::cout << "Difference = [";
    //     for (size_t j = 0; j < NUM_JOINTS; ++j) {
    //         std::cout << std::fixed << std::setprecision(4) << difference[j] << (j < NUM_JOINTS - 1 ? ", " : "");
    //     }
    //     std::cout << "]" << std::endl;
    // }
    std::deque<TrajectoryFrame> new_traj_deque;
    for (size_t step = 0; step < N; ++step) {
        CmdArray group{};
        JointArray qi = new_traj[step];
        qi[GRIPPER_INDEX] = gripper_target;
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            float dqi = 0.0f;
            if (j != GRIPPER_INDEX && new_traj.size() > 1) {
                const JointArray& adjacent_q = step == 0
                    ? new_traj[1]
                    : new_traj[step - 1];
                const float direction = step == 0 ? 1.0f : -1.0f;
                dqi = direction * (adjacent_q[j] - qi[j]) / control_cycle;
                dqi = std::clamp(dqi, -joint_vmax_[j], joint_vmax_[j]);
            }
            group[j] = MITCmd{kp[j], kd[j], qi[j], dqi, tau[j]};
            assert(std::isfinite(group[j].q));
            assert(std::fabs(group[j].q) < 100);
        }
        new_traj_deque.push_back(TrajectoryFrame{group, apply_delay_smoothing});
    }
    std::lock_guard<std::mutex> lk(traj_mutex_);
    traj_queue_ = std::move(new_traj_deque);
    traj_queue_updated_ = true;
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
    state_log_.open(state_log_path_);
    std::cout << "Motor state log: " << state_log_path_ << std::endl;
    state_log_ << "timestamp_ms";
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_ext_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_cmd_" << j << ",dq_cmd_" << j << ",tau_cmd_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_snd_" << j << ",dq_snd_" << j << ",tau_snd_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",dq_plan_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",q_" << j << ",dq_" << j << ",tau_" << j;
    for (size_t j = 0; j < NUM_JOINTS; ++j)
        state_log_ << ",p_mech_" << j << ",p_elec_" << j;
    state_log_ << ",ok,keyframe" << std::endl;
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

MotorManager::CmdArray MotorManager::shape_input(
    const CmdArray& curr,
    bool apply_delay_smoothing) {
    if (!motion_filter_config_.enabled ||
        !motion_filter_config_.input_shaper.enabled ||
        !apply_delay_smoothing) {
        input_shaper_history_.clear();
        input_shaper_history_.push_back(curr);
        for (IIRNotchState& state : iir_notch_states_) {
            state.initialized = false;
        }
        return curr;
    }

    if (motion_filter_config_.input_shaper.type == "iir_notch") {
        CmdArray result = curr;
        const auto filter_sample = [this](float input,
                                          float& x1, float& x2,
                                          float& y1, float& y2) {
            const float output =
                iir_b0_ * input + iir_b1_ * x1 + iir_b2_ * x2 -
                iir_a1_ * y1 - iir_a2_ * y2;
            x2 = x1;
            x1 = input;
            y2 = y1;
            y1 = output;
            return output;
        };

        for (size_t joint = 0; joint < GRIPPER_INDEX; ++joint) {
            IIRNotchState& state = iir_notch_states_[joint];
            if (!state.initialized) {
                state.q_x1 = state.q_x2 = state.q_y1 = state.q_y2 =
                    curr[joint].q;
                state.dq_x1 = state.dq_x2 = state.dq_y1 = state.dq_y2 =
                    curr[joint].dq;
                state.initialized = true;
            }
            result[joint].q = filter_sample(
                curr[joint].q, state.q_x1, state.q_x2,
                state.q_y1, state.q_y2);
            result[joint].dq = filter_sample(
                curr[joint].dq, state.dq_x1, state.dq_x2,
                state.dq_y1, state.dq_y2);
        }
        // The gripper bypasses the notch. The following Hann stage still
        // applies its configured pure delay to all seven command channels.
        return result;
    }

    input_shaper_history_.push_back(curr);
    while (input_shaper_history_.size() > input_shaper_max_delay_ + 1) {
        input_shaper_history_.pop_front();
    }

    const auto delayed_sample = [this](std::size_t delay_frames)
        -> const CmdArray& {
        if (delay_frames >= input_shaper_history_.size()) {
            return input_shaper_history_.front();
        }
        return input_shaper_history_[input_shaper_history_.size() - 1 -
                                     delay_frames];
    };

    CmdArray result = curr;
    for (size_t joint = 0; joint < GRIPPER_INDEX; ++joint) {
        float shaped_q = 0.0f;
        float shaped_dq = 0.0f;
        for (const InputShaperTap& tap : input_shaper_taps_) {
            const MITCmd& sample = delayed_sample(tap.delay_frames)[joint];
            shaped_q += tap.weight * sample.q;
            shaped_dq += tap.weight * sample.dq;
        }
        result[joint].q = shaped_q;
        result[joint].dq = shaped_dq;
    }

    // The gripper keeps its independent command law. Delay it by the shaper's
    // weighted centroid so arm and gripper actions remain time-aligned.
    result[GRIPPER_INDEX] =
        delayed_sample(input_shaper_gripper_delay_)[GRIPPER_INDEX];
    return result;
}

MotorManager::CmdArray MotorManager::smooth_delayed_input(
    const CmdArray& curr,
    bool apply_delay_smoothing) {
    if (!motion_filter_config_.enabled ||
        motion_filter_config_.delay_frames == 0 ||
        !apply_delay_smoothing) {
        delay_smoothing_history_.clear();
        delay_smoothing_history_.push_back(curr);
        return curr;
    }

    const size_t window_size = delay_smoothing_weights_.size();
    delay_smoothing_history_.push_back(curr);
    while (delay_smoothing_history_.size() > window_size) {
        delay_smoothing_history_.pop_front();
    }

    CmdArray result = curr;
    const size_t left_padding = window_size - delay_smoothing_history_.size();
    for (size_t joint = 0; joint < GRIPPER_INDEX; ++joint) {
        float smoothed_q = 0.0f;
        float smoothed_dq = 0.0f;
        for (size_t index = 0; index < window_size; ++index) {
            const CmdArray& sample = index < left_padding
                ? delay_smoothing_history_.front()
                : delay_smoothing_history_[index - left_padding];
            const float weight = delay_smoothing_weights_[index];
            smoothed_q += weight * sample[joint].q;
            smoothed_dq += weight * sample[joint].dq;
        }
        result[joint].q = smoothed_q;
        result[joint].dq = smoothed_dq;
    }

    // Keep the gripper synchronized with the arm's group delay without
    // smoothing its independent position/torque command.
    const size_t delayed_index = motion_filter_config_.delay_frames;
    const CmdArray& delayed_sample = delayed_index < left_padding
        ? delay_smoothing_history_.front()
        : delay_smoothing_history_[delayed_index - left_padding];
    result[GRIPPER_INDEX] = delayed_sample[GRIPPER_INDEX];
    return result;
}

MITCmd MotorManager::filter_motion_frame(size_t joint, const MITCmd& prev, const MITCmd& curr) {
    const float dt = control_cycle;
    const float max_velocity = joint_vmax_[joint];
    const float max_acceleration = joint_amax_[joint];
    const float current_position = motion_filter_q_[joint];
    const float current_velocity = motion_filter_dq_[joint];
    const float distance = curr.q - current_position;
    const float max_velocity_step = max_acceleration * dt;
    const float target_velocity = std::clamp(curr.dq, -max_velocity, max_velocity);
    const bool target_is_moving =
        std::fabs(target_velocity) > motion_filter_config_.velocity_tolerance;

    // Snap to a nearly reached interpolated frame to eliminate small command
    // oscillations around a target that has actually stopped.
    if (!target_is_moving &&
        std::fabs(distance) <= motion_filter_config_.position_tolerance &&
        std::fabs(current_velocity) <= motion_filter_config_.velocity_tolerance) {
        motion_filter_q_[joint] = curr.q;
        motion_filter_dq_[joint] = 0.0f;
        return MITCmd{
            curr.kp,
            curr.kd,
            curr.q,
            0.0f,
            filter_alpha * prev.tau + (1.0f - filter_alpha) * curr.tau
        };
    }

    const float direction = distance > 0.0f ? 1.0f
                          : distance < 0.0f ? -1.0f
                                            : 0.0f;

    float desired_velocity = 0.0f;
    if (target_is_moving) {
        // Track a moving trajectory with velocity feedforward. The position
        // correction removes accumulated lag without forcing every
        // interpolated frame to become a rest-to-rest target.
        constexpr float position_correction_horizon = 0.05f;
        desired_velocity = std::clamp(
            target_velocity + distance / position_correction_horizon,
            -max_velocity,
            max_velocity);
    } else {
        // Preserve the original distance-based braking behavior for a target
        // that has stopped.
        const float braking_velocity =
            std::sqrt(2.0f * max_acceleration * std::fabs(distance));
        desired_velocity =
            direction * std::min(max_velocity, braking_velocity);
    }

    // Unlike the original pseudocode, retain the previous frame velocity so
    // that max_acceleration is an actual per-frame acceleration limit.
    float next_velocity = std::clamp(
        desired_velocity,
        current_velocity - max_velocity_step,
        current_velocity + max_velocity_step);
    // Keep the same position update used by the original pseudocode.
    float next_position = current_position + next_velocity * dt;

    // Only a stationary target is a stopping point. Crossing a moving target
    // frame must not reset velocity, otherwise every new frame creates a
    // stop-and-go pulse.
    if (!target_is_moving &&
        distance * (curr.q - next_position) <= 0.0f &&
        std::fabs(next_velocity) <= max_velocity_step) {
        next_position = curr.q;
        next_velocity = 0.0f;
    }

    motion_filter_q_[joint] = next_position;
    motion_filter_dq_[joint] = next_velocity;

    return MITCmd{
        curr.kp,
        curr.kd,
        next_position,
        motion_filter_config_.send_velocity ? next_velocity : 0.0f,
        filter_alpha * prev.tau + (1.0f - filter_alpha) * curr.tau
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
        motion_filter_q_[i] = last_q_[i];
        motion_filter_dq_[i] = 0.0f;
    }
    interpolator_->fill(last_q_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        mc_->refresh_motor_status(*motors_[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        last_q_[i] = motors_[i]->Get_Position();
        last_cmd_[i] = MITCmd{kp_[i], kd_[i], last_q_[i], 0.0f, 0.0f};
        last_dq_[i] = 0.0f;
        motion_filter_q_[i] = last_q_[i];
        motion_filter_dq_[i] = 0.0f;
    }
    interpolator_->fill(last_q_);
    if (compute_full_dynamic_torque || compute_dynamic_torque) {
        std::cout << "Dynamics Feedforward Enabled." << std::endl;
    }

    filtered_cmd_ = last_cmd_;
    while (running_) {
        auto t0 = std::chrono::steady_clock::now();

        TrajectoryFrame trajectory_frame{};
        bool ok = false;
        bool is_keyframe = false;

        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            if (!traj_queue_.empty()) {
                trajectory_frame = traj_queue_.front();
                traj_queue_.pop_front();
                ok = true;
            }
            if (traj_queue_updated_) {
                is_keyframe = true;
                traj_queue_updated_ = false;
            }
        }
        CmdArray input_cmd{};
        input_cmd = ok ? trajectory_frame.command : last_cmd_;
        const bool apply_delay_smoothing = ok
            ? trajectory_frame.apply_delay_smoothing
            : last_delay_smoothing_enabled_;
        if (!ok) {
            // A consumed trajectory must not leave a stale feedforward
            // velocity active while waiting for the next command.
            for (auto& cmd : input_cmd) {
                cmd.dq = 0.0f;
            }
        }
        input_cmd = shape_input(input_cmd, apply_delay_smoothing);
        input_cmd = smooth_delayed_input(input_cmd, apply_delay_smoothing);
        CmdArray send_cmd;

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& prev = filtered_cmd_[j];
            const auto& curr = input_cmd[j];
            if (j == GRIPPER_INDEX) {
                // The gripper is torque-controlled from its unsmoothed position
                // error. Preserve the synchronized delayed target and use zero
                // velocity demand without applying the arm motion filter.
                send_cmd[j] = curr;
                send_cmd[j].dq = 0.0f;
                motion_filter_q_[j] = curr.q;
                motion_filter_dq_[j] = 0.0f;
            } else {
                send_cmd[j] = motion_filter_config_.enabled
                    ? filter_motion_frame(j, prev, curr)
                    : filter_cmd(prev, curr, filter_alpha);
            }
        }


        // Finalize the exact velocity that will be sent before computing the
        // rigid-body feedforward terms.
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            if (!motion_filter_config_.enabled && j != GRIPPER_INDEX) {
                send_cmd[j].dq =
                    (send_cmd[j].q - filtered_cmd_[j].q) / control_cycle;
                send_cmd[j].dq = std::clamp(
                    send_cmd[j].dq, -joint_vmax_[j], joint_vmax_[j]);
            }
        }

        JointArray target_q = {};
        JointArray target_v = {};
        JointArray target_a = {};
        JointArray target_t = {};
        JointArray target_kp = {};
        JointArray target_kd = {};
        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            target_q[j] = send_cmd[j].q;
            target_v[j] = send_cmd[j].dq;
            target_a[j] =
                (send_cmd[j].dq - filtered_cmd_[j].dq) / control_cycle;
        }

        const bool dynamic_torque_enabled =
            static_cast<bool>(compute_full_dynamic_torque) ||
            static_cast<bool>(compute_dynamic_torque);
        auto tau_computed = JointArray{};
        if (compute_full_dynamic_torque) {
            tau_computed = compute_full_dynamic_torque(
                getPosition(), target_q, target_v, target_a);
        }
        else if (compute_dynamic_torque) {
            tau_computed = compute_dynamic_torque(getPosition(), target_q);
        }
        for (size_t j=0; j < NUM_JOINTS; ++j) {
            try {
                const auto& c = send_cmd[j];
                if (dynamic_torque_enabled) {
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

        if (ok) {
            last_cmd_ = trajectory_frame.command;
            last_delay_smoothing_enabled_ =
                trajectory_frame.apply_delay_smoothing;
        }
        
        auto t_now = std::chrono::steady_clock::now();
        long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - t_start_).count();
        auto q = getPosition();
        auto dq = getVelocity();
        auto tau = getTau();

        const CmdArray& cmd_group = input_cmd;

        state_log_ << ts;

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << external_target_[j];

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& c = cmd_group[j];
            state_log_ << "," << c.q << "," << c.dq << "," << (dynamic_torque_enabled ? tau_computed[j] : c.tau) ;
        }

        for (size_t j = 0; j < NUM_JOINTS; ++j) {
            const auto& c = send_cmd[j];
            state_log_ << "," << c.q << "," << c.dq << "," << (dynamic_torque_enabled ? tau_computed[j] : c.tau) ;
        }

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << motion_filter_dq_[j];

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << q[j] << "," << dq[j] << "," << tau[j];

        for (size_t j = 0; j < NUM_JOINTS; ++j)
            state_log_ << "," << motors_[j]->estimate_mechanical_power()
                       << "," << motors_[j]->estimate_electrical_power();

        state_log_ << "," << ok << "," << is_keyframe << std::endl;
        
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

void MotorManager::setPosition(const JointArray& q) {
    interpolator_->fill(q);
    move_mit(q, uniform(0.0f), uniform(0.0f), kp_, kd_, false);
}
