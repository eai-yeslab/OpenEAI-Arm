#pragma once
#include <cstddef>
#include <cmath>
#include <vector>
#include <memory>
#include <array>
#include <algorithm>

template<typename T, size_t N>
class HistoryQueue {
private:
    std::array<T, N> data_;
    size_t head_ = 0;  // data_[0] always is latest
    bool filled_ = false;

public:
    HistoryQueue() { fill(T{}); }

    // Insert at front
    void push_front(const T& value) {
        for (size_t i = N-1; i > 0; --i) data_[i] = data_[i-1];
        data_[0] = value;
        filled_ = filled_ || (head_ == N-1);
        if (head_ < N-1) ++head_;
    }

    void fill(const T& value) { data_.fill(value); head_ = 0; filled_ = false; }

    T& operator[](size_t i) { return data_[i]; }
    const T& operator[](size_t i) const { return data_[i]; }

    const std::array<T, N>& array() const { return data_; }

    T getVel() const {
        if (head_ < 1) return T{};
        return data_[0] - data_[1];
    }
    size_t size() const { return filled_ ? N : head_+1; }
};

template<typename T, size_t N>
class Interpolator {
protected:
    HistoryQueue<T, N> history_;
    size_t history_length_;

public:
    Interpolator() : history_length_(N), history_() {}

    virtual ~Interpolator() = default;

    void push_command(const T& cmd) { history_.push_front(cmd); }


    void fill(const T& cmd) { history_.fill(cmd); }
    const HistoryQueue<T, N>& getHistory() const { return history_; }
    HistoryQueue<T, N>& getHistory() { return history_; }
    T getVel() const { return history_.getVel(); }
    T& operator[](size_t idx) { return history_[idx]; }
    const T& operator[](size_t idx) const { return history_[idx]; }
    virtual std::vector<T> interpolate(size_t n_steps) {
        std::vector<T> traj(n_steps);
        for (size_t step = 1; step <= n_steps; ++step) {
            float s = compute_s(step, n_steps);
            traj[step-1] = interpolate_impl(s);
        }
        return traj;
    }

    virtual float compute_s(size_t step, size_t total_step) const { return float(step)/total_step; }

    virtual T interpolate_impl(float s) const {
        return history_[1] + (history_[0] - history_[1]) * s;
    }

    size_t length() const { return history_length_; }
};

template<typename T, size_t N>
class LinearInterpolator : public Interpolator<T, N> {
public:
    using Interpolator<T,N>::Interpolator;
    float compute_s(size_t step, size_t n_steps) const override {
        return float(step) / n_steps;
    }

};

template<typename T, size_t N>
class CubicSInterpolator : public Interpolator<T, N> {
public:
    using Interpolator<T,N>::Interpolator;
    float compute_s(size_t step, size_t n_steps) const override {
        float s = float(step) / n_steps;
        return 3*s*s - 2*s*s*s;
    }
};

template<typename T, size_t N>
class MinJerkInterpolator : public Interpolator<T, N> {
public:
    using Interpolator<T,N>::Interpolator;
    float compute_s(size_t step, size_t n_steps) const override {
        float s = float(step) / n_steps;
        return 10*std::pow(s,3) - 15*std::pow(s,4) + 6*std::pow(s,5);
    }
};

