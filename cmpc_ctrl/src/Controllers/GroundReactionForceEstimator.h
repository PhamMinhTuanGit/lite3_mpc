#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

#include "Lite3DynamicModel.hpp"
#include "Controllers/GeneralizedMomentumObserver.h"

enum class GRFInvalidReason : std::uint8_t {
    None = 0,
    ObserverWarmup,
    NonFiniteInput,
    IllConditionedJacobian,
    SolverFailure,
    NonFiniteOutput,
    ForceLimitExceeded
};

struct GRFResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    bool ready = false;
    GRFInvalidReason invalid_reason = GRFInvalidReason::None;
    std::array<std::uint8_t, Lite3Dynamics::kNumLegs> leg_valid{};
    std::array<double, Lite3Dynamics::kNumLegs> jacobian_quality{};
    std::array<Eigen::Vector3d, Lite3Dynamics::kNumLegs> raw_force_world{};
    // Bias-corrected, low-pass-filtered force. Kept under the original name
    // so existing read-only telemetry consumers remain source-compatible.
    std::array<Eigen::Vector3d, Lite3Dynamics::kNumLegs> force_world{};

    GRFResult()
    {
        for (std::size_t leg = 0; leg < force_world.size(); ++leg) {
            raw_force_world[leg].setZero();
            force_world[leg].setZero();
        }
    }
};

class GroundReactionForceEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GroundReactionForceEstimator(Lite3Dynamics& dynamics, double damping);
    GroundReactionForceEstimator(
        Lite3Dynamics& dynamics,
        double dt,
        const GMOConfig& config);

    const GRFResult& update(
        const Lite3Dynamics::Configuration& q,
        const Lite3Dynamics::Velocity& residual,
        bool observer_ready = true);

    void reset() noexcept;

private:
    Lite3Dynamics& dynamics_;
    double dt_ = 0.001;
    GMOConfig config_;
    double filter_alpha_ = 1.0;
    bool filter_initialized_ = false;
    std::array<Eigen::Vector3d, Lite3Dynamics::kNumLegs> filtered_force_world_{};
    GRFResult result_;
};
