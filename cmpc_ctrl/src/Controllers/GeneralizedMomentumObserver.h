#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

#include "Lite3DynamicModel.hpp"

struct GMOConfig {
    bool enabled = true;
    double gain = 30.0;
    double force_damping = 1e-4;
    std::uint32_t warmup_samples = 100;
    double force_filter_cutoff_hz = 50.0;
    double max_abs_force_n = 1000.0;
    double min_jacobian_quality = 1e-3;
    double max_sample_gap_s = 0.01;
    std::array<Eigen::Vector3d, Lite3Dynamics::kNumLegs> force_bias_world{};

    GMOConfig()
    {
        for (auto& bias : force_bias_world) {
            bias.setZero();
        }
    }
};

enum class GMOInvalidReason : std::uint8_t {
    None = 0,
    Disabled,
    Warmup,
    NonFiniteInput,
    DynamicsFailure,
    NonFiniteOutput,
    SampleDiscontinuity,
    ManualReset
};

struct GMOResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    bool initialized = false;
    bool ready = false;
    GMOInvalidReason invalid_reason = GMOInvalidReason::None;
    std::uint32_t samples_since_reset = 0;
    std::uint64_t reset_count = 0;
    Lite3Dynamics::Velocity momentum = Lite3Dynamics::Velocity::Zero();
    Lite3Dynamics::Velocity momentum_hat = Lite3Dynamics::Velocity::Zero();
    Lite3Dynamics::Velocity residual = Lite3Dynamics::Velocity::Zero();
};

class GeneralizedMomentumObserver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GeneralizedMomentumObserver(
        Lite3Dynamics& dynamics,
        double dt,
        const GMOConfig& config = GMOConfig{});

    const GMOResult& update(
        const Lite3Dynamics::Configuration& q,
        const Lite3Dynamics::Velocity& v,
        const Eigen::Matrix<double, Lite3Dynamics::kNumActuatedJoints, 1>&
            motor_torque);

    void reset(GMOInvalidReason reason = GMOInvalidReason::ManualReset) noexcept;
    const GMOResult& result() const noexcept { return result_; }
    const GMOConfig& config() const noexcept { return config_; }
    std::uint64_t resetCount() const noexcept { return reset_count_; }

private:
    Lite3Dynamics& dynamics_;
    double dt_;
    GMOConfig config_;
    bool initialized_ = false;
    std::uint32_t samples_since_reset_ = 0;
    std::uint64_t reset_count_ = 0;
    Lite3Dynamics::Velocity momentum_hat_ = Lite3Dynamics::Velocity::Zero();
    GMOResult result_;
};
