#include "OpenEAIArm.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// Switch between simulation and real hardware by changing this one line.
//   true  -> protocol-level simulation, runs without a serial device
//   false -> real hardware, parameters are loaded from configs/default_right.yml
static constexpr bool kUseSimulation = false;

static constexpr size_t NUM_JOINTS = 7;
static constexpr double kPi = 3.14159265358979323846;

namespace {

// The post-interpolation motion filter and the fallback exponential blend would
// turn a step into a first-order response, so we write a temporary config with
// both smoothing stages disabled.
std::string make_damping_config(const std::string& src, const std::string& dst) {
    YAML::Node cfg = YAML::LoadFile(src);
    cfg["controller"]["motion_filter"]["enabled"] = false;
    cfg["controller"]["filter_alpha"] = 0.0f;
    std::ofstream out(dst);
    out << cfg;
    return dst;
}

std::vector<size_t> local_extrema(const std::vector<double>& x) {
    std::vector<size_t> idx;
    for (size_t i = 1; i + 1 < x.size(); ++i) {
        if ((x[i] - x[i - 1]) * (x[i + 1] - x[i]) < 0.0) {
            idx.push_back(i);
        }
    }
    return idx;
}

bool estimate_zeta(const std::vector<double>& t, const std::vector<double>& q,
                   double steady, double& zeta, double& wd, double& wn) {
    std::vector<double> x(q.size());
    for (size_t i = 0; i < q.size(); ++i) x[i] = q[i] - steady;

    std::vector<size_t> ext = local_extrema(x);
    if (ext.size() < 3) return false;

    // Use same-sign peaks, preferring the dominant side.
    std::vector<size_t> peaks;
    for (size_t i : ext) if (x[i] > 1e-4) peaks.push_back(i);
    if (peaks.size() < 2) {
        peaks.clear();
        for (size_t i : ext) if (x[i] < -1e-4) peaks.push_back(i);
    }
    if (peaks.size() < 2) return false;

    double delta = 0.0;
    int count = 0;
    for (size_t i = 1; i < peaks.size(); ++i) {
        double d = std::log(std::fabs(x[peaks[i - 1]]) / std::fabs(x[peaks[i]]));
        if (d > 0.0) {
            delta += d;
            ++count;
        }
    }
    if (count == 0) return false;
    delta /= count;
    zeta = delta / std::sqrt(4.0 * kPi * kPi + delta * delta);

    double period = 0.0;
    int pc = 0;
    for (size_t i = 1; i < peaks.size(); ++i) {
        period += t[peaks[i]] - t[peaks[i - 1]];
        ++pc;
    }
    if (pc > 0) period /= pc;
    if (period > 1e-6) {
        wd = 2.0 * kPi / period;
        if (zeta < 1.0) wn = wd / std::sqrt(1.0 - zeta * zeta);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    int joint = 0;
    double step = 0.02;
    double record_time = 3.0;
    double sample_dt = 0.005;
    double settle_time = 1.0;
    bool all = false;
    std::string config_path = "configs/default_right.yml";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() { return (i + 1 < argc) ? std::string(argv[++i]) : ""; };
        if (arg == "--joint") joint = std::stoi(next());
        else if (arg == "--step") step = std::stod(next());
        else if (arg == "--record-time") record_time = std::stod(next());
        else if (arg == "--sample-dt") sample_dt = std::stod(next());
        else if (arg == "--settle-time") settle_time = std::stod(next());
        else if (arg == "--all") all = true;
        else if (arg == "--config") config_path = next();
        else if (arg == "--help") {
            std::cout << "usage: test_damping [--joint 0..5] [--step 0.02] "
                         "[--record-time 3] [--sample-dt 0.005] [--all] [--config path]\n";
            return 0;
        }
    }

    if (joint < 0 || joint >= 6) {
        std::cerr << "joint must be in 0..5 (gripper is not an MIT impedance joint)\n";
        return 1;
    }

    OpenEAIArm::ControlMode mode = kUseSimulation
        ? OpenEAIArm::ControlMode::SIM
        : OpenEAIArm::ControlMode::MIT_MIX;

    const std::string tmp_config = "damping_config_tmp.yml";
    make_damping_config(config_path, tmp_config);

    OpenEAIArm arm(tmp_config, mode);

    try {
        auto pose = arm.get_joint_positions();
        std::cout << "mode: " << (kUseSimulation ? "SIM" : "REAL") << "\n";
        std::cout << "current pose:";
        for (size_t j = 0; j < NUM_JOINTS; ++j) std::cout << " " << pose[j];
        std::cout << "\nstep=" << step << " rad, record=" << record_time << "s\n";

        std::vector<int> joints;
        if (all) for (int j = 0; j < 6; ++j) joints.push_back(j);
        else joints.push_back(joint);

        for (int j : joints) {
            pose = arm.get_joint_positions();
            auto target = pose;
            target[j] += step;

            // A damping test needs a real step, not an interpolated profile.
            // move() only takes the direct MIT path when the command interval
            // is short, so first send the current pose to refresh the command
            // timestamp, then immediately send the step target.
            arm.set_joint_targets(pose, 0.0f, {}, {}, false);
            arm.set_joint_targets(target, 0.0f, {}, {}, false);

            std::vector<double> t, q;
            auto t0 = std::chrono::steady_clock::now();
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < record_time) {
                t.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
                q.push_back(arm.get_joint_positions()[j]);
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(sample_dt * 1e6)));
            }

            // Steady-state is approximated by the tail average.
            size_t tail = std::max<size_t>(20, q.size() / 10);
            double steady = 0.0;
            for (size_t k = q.size() - tail; k < q.size(); ++k) steady += q[k];
            steady /= tail;

            double zeta = -1.0, wd = 0.0, wn = 0.0;
            bool ok = estimate_zeta(t, q, steady, zeta, wd, wn);
            std::cout << "--- joint " << j << " ---\n";
            if (!ok) {
                std::cout << "  no clear decaying oscillation detected\n";
            } else {
                std::cout << "  zeta=" << zeta << "  wd=" << wd << " rad/s  wn=" << wn << " rad/s\n";
            }

            // Restore directly as well, then wait for residual settling.
            arm.set_joint_targets(pose, 0.0f, {}, {}, false);
            arm.set_joint_targets(pose, 0.0f, {}, {}, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(settle_time * 1000)));
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
    }

    arm.disable_all();
    std::remove(tmp_config.c_str());
    return 0;
}
