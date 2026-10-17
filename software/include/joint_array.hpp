#pragma once

#include <array>
#include <cstddef>
#include <ostream>

template <size_t N>
inline std::ostream& operator<<(std::ostream& os, const std::array<float, N>& values) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        if (i) os << ", ";
        os << values[i];
    }
    return os << "]";
}

template <size_t N>
inline std::array<float, N> operator+(
    const std::array<float, N>& a, const std::array<float, N>& b) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) out[i] = a[i] + b[i];
    return out;
}

template <size_t N>
inline std::array<float, N> operator-(
    const std::array<float, N>& a, const std::array<float, N>& b) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) out[i] = a[i] - b[i];
    return out;
}

template <size_t N>
inline std::array<float, N> operator*(
    const std::array<float, N>& a, float value) {
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i) out[i] = a[i] * value;
    return out;
}
