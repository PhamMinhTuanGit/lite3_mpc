#include "Controllers/GroundReactionForceEstimator.h"

#include <array>
#include <stdexcept>

#include <Eigen/Cholesky>

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
    : dynamics_(dynamics), damping_(damping)
{
    if (!(damping_ > 0.0)) {
        throw std::invalid_argument("GRF damping must be positive");
    }
}

const GRFResult& GroundReactionForceEstimator::update(
    const Lite3Dynamics::Configuration& q,
    const Lite3Dynamics::Velocity& residual)
{
    result_ = GRFResult{};
    if (!q.allFinite() || !residual.allFinite()) {
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

        const Eigen::Matrix3d normal_matrix =
            joint_jacobian * joint_jacobian.transpose()
            + damping_ * Eigen::Matrix3d::Identity();
        const Eigen::LDLT<Eigen::Matrix3d> solver(normal_matrix);
        if (solver.info() != Eigen::Success) {
            all_valid = false;
            continue;
        }
        result_.force_world[leg] =
            solver.solve(joint_jacobian * joint_residual);
        if (!result_.force_world[leg].allFinite()) {
            result_.force_world[leg].setZero();
            all_valid = false;
        }
    }

    result_.valid = all_valid;
    return result_;
}
