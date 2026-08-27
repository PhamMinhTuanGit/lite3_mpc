#pragma once

#include <algorithm>

template <typename T>
constexpr T EstimatorContactPhase(bool standing, T gait_phase) noexcept
{
    return standing ? T(0.5) : gait_phase;
}

template <typename T>
constexpr T LinearKfContactTrust(T phase, T trust_window = T(0.2)) noexcept
{
    phase = std::clamp(phase, T(0), T(1));
    if (phase < trust_window) {
        return phase / trust_window;
    }
    if (phase > T(1) - trust_window) {
        return (T(1) - phase) / trust_window;
    }
    return T(1);
}
