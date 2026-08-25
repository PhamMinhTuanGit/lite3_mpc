#include "Controllers/GeneralizedMomentumObserver.h"

#include <array>
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
    if (!(dt_ > 0.0) || !(config_.gain >= 0.0)) {
        throw std::invalid_argument("GMO dt must be positive and gain non-negative");
    }
}

const GMOResult& GeneralizedMomentumObserver::update(
    const Lite3Dynamics::Configuration& q,
    const Lite3Dynamics::Velocity& v,
    const Eigen::Matrix<double, Lite3Dynamics::kNumActuatedJoints, 1>&
        motor_torque)
{
    result_ = GMOResult{};
    if (!config_.enabled || !q.allFinite() || !v.allFinite()
        || !motor_torque.allFinite()) {
        reset();
        return result_;
    }

    const Lite3Dynamics::MassMatrix mass_matrix =
        dynamics_.computeMassMatrix(q);
    const Lite3Dynamics::MassMatrix coriolis =
        dynamics_.computeCoriolisMatrix(q, v);
    const Lite3Dynamics::Velocity gravity = dynamics_.computeGravity(q);

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

    const Lite3Dynamics::Velocity residual =
        config_.gain * (momentum - momentum_hat_);
    result_.valid = momentum.allFinite() && beta.allFinite()
        && residual.allFinite() && momentum_hat_.allFinite();
    result_.initialized = initialized_;
    result_.momentum = momentum;
    result_.momentum_hat = momentum_hat_;
    result_.residual = residual;

    if (!result_.valid) {
        reset();
        return result_;
    }

    momentum_hat_.noalias() +=
        dt_ * (generalized_torque - beta + residual);
    if (!momentum_hat_.allFinite()) {
        reset();
    }
    return result_;
}

void GeneralizedMomentumObserver::reset() noexcept
{
    initialized_ = false;
    momentum_hat_.setZero();
    result_ = GMOResult{};
}
