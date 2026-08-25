#pragma once

#include <Eigen/Core>

#include "Lite3DynamicModel.hpp"

struct GMOConfig {
    bool enabled = true;
    double gain = 30.0;
    double force_damping = 1e-4;
};

struct GMOResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    bool initialized = false;
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

    void reset() noexcept;
    const GMOResult& result() const noexcept { return result_; }

private:
    Lite3Dynamics& dynamics_;
    double dt_;
    GMOConfig config_;
    bool initialized_ = false;
    Lite3Dynamics::Velocity momentum_hat_ = Lite3Dynamics::Velocity::Zero();
    GMOResult result_;
};
