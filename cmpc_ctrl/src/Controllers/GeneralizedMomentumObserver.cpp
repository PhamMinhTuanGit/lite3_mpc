#include "Controllers/GeneralizedMomentumObserver.h"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

constexpr std::array<Lite3Dynamics::Leg, Lite3Dynamics::kNumLegs> kLegs{{
    Lite3Dynamics::Leg::FR,
    Lite3Dynamics::Leg::FL,
    Lite3Dynamics::Leg::HR,
    Lite3Dynamics::Leg::HL
}};

constexpr std::array<Lite3Dynamics::LegJoint, Lite3Dynamics::kJointsPerLeg>
    kJoints{{
        Lite3Dynamics::LegJoint::HipX,
        Lite3Dynamics::LegJoint::HipY,
        Lite3Dynamics::LegJoint::Knee
    }};

}  // namespace

GeneralizedMomentumObserver::GeneralizedMomentumObserver(
    Lite3Dynamics& dynamics,
    double dt,
    const GMOConfig& config)
    : dynamics_(dynamics), dt_(dt), config_(config)
{
    if (!(dt_ > 0.0) || !(config_.gain >= 0.0)
        || !(config_.force_damping > 0.0)
        || !(config_.force_filter_cutoff_hz >= 0.0)
        || !(config_.max_abs_force_n > 0.0)
        || !(config_.min_jacobian_quality >= 0.0)
        || !(config_.min_jacobian_quality <= 1.0)
        || !(config_.max_sample_gap_s > 0.0)) {
        throw std::invalid_argument("Invalid GMO configuration");
    }
    for (const auto& bias : config_.force_bias_world) {
        if (!bias.allFinite()) {
            throw std::invalid_argument("GMO force bias must be finite");
        }
    }
}

const GMOResult& GeneralizedMomentumObserver::update(
    const Lite3Dynamics::Configuration& q,
    const Lite3Dynamics::Velocity& v,
    const Eigen::Matrix<double, Lite3Dynamics::kNumActuatedJoints, 1>&
        motor_torque)
{
    result_ = GMOResult{};
    result_.reset_count = reset_count_;
    result_.samples_since_reset = samples_since_reset_;
    if (!config_.enabled) {
        result_.invalid_reason = GMOInvalidReason::Disabled;
        return result_;
    }
    if (!q.allFinite() || !v.allFinite() || !motor_torque.allFinite()) {
        reset(GMOInvalidReason::NonFiniteInput);
        return result_;
    }

    Lite3Dynamics::MassMatrix mass_matrix;
    Lite3Dynamics::MassMatrix coriolis;
    Lite3Dynamics::Velocity gravity;
    try {
        mass_matrix = dynamics_.computeMassMatrix(q);
        coriolis = dynamics_.computeCoriolisMatrix(q, v);
        gravity = dynamics_.computeGravity(q);
    } catch (const std::exception&) {
        reset(GMOInvalidReason::DynamicsFailure);
        return result_;
    }

    const Lite3Dynamics::Velocity momentum = mass_matrix * v;
    const Lite3Dynamics::Velocity beta = gravity - coriolis.transpose() * v;
    Lite3Dynamics::Velocity generalized_torque = Lite3Dynamics::Velocity::Zero();

    const auto& model = dynamics_.model();
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto joint_id = dynamics_.jointId(kLegs[leg], kJoints[joint]);
            const int v_index = model.joints[joint_id].idx_v();
            generalized_torque[v_index] =
                motor_torque[leg * Lite3Dynamics::kJointsPerLeg + joint];
        }
    }

    if (!initialized_) {
        momentum_hat_ = momentum;
        initialized_ = true;
    }

    ++samples_since_reset_;

    const Lite3Dynamics::Velocity residual =
        config_.gain * (momentum - momentum_hat_);
    result_.valid = momentum.allFinite() && beta.allFinite()
        && residual.allFinite() && momentum_hat_.allFinite();
    result_.initialized = initialized_;
    result_.samples_since_reset = samples_since_reset_;
    result_.reset_count = reset_count_;
    result_.ready = result_.valid
        && samples_since_reset_ >= config_.warmup_samples;
    result_.invalid_reason = result_.ready
        ? GMOInvalidReason::None : GMOInvalidReason::Warmup;
    result_.momentum = momentum;
    result_.momentum_hat = momentum_hat_;
    result_.residual = residual;

    if (!result_.valid) {
        reset(GMOInvalidReason::NonFiniteOutput);
        return result_;
    }

    momentum_hat_.noalias() +=
        dt_ * (generalized_torque - beta + residual);
    if (!momentum_hat_.allFinite()) {
        reset(GMOInvalidReason::NonFiniteOutput);
    }
    return result_;
}

void GeneralizedMomentumObserver::reset(GMOInvalidReason reason) noexcept
{
    if (initialized_ || samples_since_reset_ != 0) {
        ++reset_count_;
    }
    initialized_ = false;
    samples_since_reset_ = 0;
    momentum_hat_.setZero();
    result_ = GMOResult{};
    result_.invalid_reason = reason;
    result_.reset_count = reset_count_;
}
