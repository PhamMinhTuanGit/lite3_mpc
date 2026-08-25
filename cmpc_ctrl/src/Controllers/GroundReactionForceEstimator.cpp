#include "Controllers/GroundReactionForceEstimator.h"

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

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

GroundReactionForceEstimator::GroundReactionForceEstimator(
    Lite3Dynamics& dynamics,
    double damping)
    : dynamics_(dynamics)
{
    config_.force_damping = damping;
    if (!(config_.force_damping > 0.0)) {
        throw std::invalid_argument("GRF damping must be positive");
    }
    filter_alpha_ = 1.0;
    reset();
}

GroundReactionForceEstimator::GroundReactionForceEstimator(
    Lite3Dynamics& dynamics,
    double dt,
    const GMOConfig& config)
    : dynamics_(dynamics), dt_(dt), config_(config)
{
    if (!(dt_ > 0.0) || !(config_.force_damping > 0.0)
        || !(config_.force_filter_cutoff_hz >= 0.0)
        || !(config_.max_abs_force_n > 0.0)
        || !(config_.min_jacobian_quality >= 0.0)
        || !(config_.min_jacobian_quality <= 1.0)) {
        throw std::invalid_argument("Invalid GRF configuration");
    }
    constexpr double kTwoPi = 6.28318530717958647692;
    filter_alpha_ = config_.force_filter_cutoff_hz == 0.0
        ? 1.0
        : 1.0 - std::exp(-kTwoPi * config_.force_filter_cutoff_hz * dt_);
    reset();
}

const GRFResult& GroundReactionForceEstimator::update(
    const Lite3Dynamics::Configuration& q,
    const Lite3Dynamics::Velocity& residual,
    bool observer_ready)
{
    result_ = GRFResult{};
    if (!q.allFinite() || !residual.allFinite()) {
        result_.invalid_reason = GRFInvalidReason::NonFiniteInput;
        return result_;
    }

    const auto& jacobians = dynamics_.computeFootJacobians(q);
    const auto& model = dynamics_.model();
    bool all_valid = true;

    for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
        Eigen::Matrix3d joint_jacobian;
        Eigen::Vector3d joint_residual;
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto joint_id = dynamics_.jointId(kLegs[leg], kJoints[joint]);
            const int v_index = model.joints[joint_id].idx_v();
            joint_jacobian.col(joint) = jacobians[leg].col(v_index);
            joint_residual[joint] = residual[v_index];
        }

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
            joint_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Vector3d singular_values = svd.singularValues();
        const double sigma_max = singular_values.maxCoeff();
        const double sigma_min = singular_values.minCoeff();
        const double quality = sigma_max > 1e-12 ? sigma_min / sigma_max : 0.0;
        result_.jacobian_quality[leg] = quality;
        if (!std::isfinite(quality) || quality < config_.min_jacobian_quality) {
            result_.invalid_reason = GRFInvalidReason::IllConditionedJacobian;
            all_valid = false;
            continue;
        }

        const Eigen::Matrix3d normal_matrix =
            joint_jacobian * joint_jacobian.transpose()
            + config_.force_damping * Eigen::Matrix3d::Identity();
        const Eigen::LDLT<Eigen::Matrix3d> solver(normal_matrix);
        if (solver.info() != Eigen::Success) {
            result_.invalid_reason = GRFInvalidReason::SolverFailure;
            all_valid = false;
            continue;
        }
        result_.raw_force_world[leg] =
            solver.solve(joint_jacobian * joint_residual);
        if (solver.info() != Eigen::Success
            || !result_.raw_force_world[leg].allFinite()) {
            result_.raw_force_world[leg].setZero();
            result_.invalid_reason = GRFInvalidReason::NonFiniteOutput;
            all_valid = false;
            continue;
        }

        const Eigen::Vector3d calibrated_force =
            result_.raw_force_world[leg] - config_.force_bias_world[leg];
        if (calibrated_force.norm() > config_.max_abs_force_n) {
            result_.invalid_reason = GRFInvalidReason::ForceLimitExceeded;
            all_valid = false;
            continue;
        }

        if (!filter_initialized_) {
            filtered_force_world_[leg] = calibrated_force;
        } else {
            filtered_force_world_[leg] +=
                filter_alpha_ * (calibrated_force - filtered_force_world_[leg]);
        }
        result_.force_world[leg] = filtered_force_world_[leg];
        result_.leg_valid[leg] = 1;
    }

    if (!filter_initialized_ && all_valid) {
        filter_initialized_ = true;
    }
    result_.valid = all_valid;
    result_.ready = all_valid && observer_ready;
    if (all_valid && !observer_ready) {
        result_.invalid_reason = GRFInvalidReason::ObserverWarmup;
    } else if (all_valid) {
        result_.invalid_reason = GRFInvalidReason::None;
    }
    return result_;
}

void GroundReactionForceEstimator::reset() noexcept
{
    filter_initialized_ = false;
    for (auto& force : filtered_force_world_) {
        force.setZero();
    }
    result_ = GRFResult{};
}
