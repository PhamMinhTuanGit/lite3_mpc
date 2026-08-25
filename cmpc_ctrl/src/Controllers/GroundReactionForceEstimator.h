#pragma once

#include <array>

#include <Eigen/Core>

#include "Lite3DynamicModel.hpp"

struct GRFResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    std::array<Eigen::Vector3d, Lite3Dynamics::kNumLegs> force_world{};

    GRFResult()
    {
        for (auto& force : force_world) {
            force.setZero();
        }
    }
};

class GroundReactionForceEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GroundReactionForceEstimator(Lite3Dynamics& dynamics, double damping);

    const GRFResult& update(
        const Lite3Dynamics::Configuration& q,
        const Lite3Dynamics::Velocity& residual);

    void reset() noexcept { result_ = GRFResult{}; }

private:
    Lite3Dynamics& dynamics_;
    double damping_;
    GRFResult result_;
};
