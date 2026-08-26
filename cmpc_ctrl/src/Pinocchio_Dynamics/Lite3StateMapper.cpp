#include "Lite3StateMapper.hpp"

#include <array>
#include <cmath>

#include <Eigen/Geometry>

namespace {

constexpr std::array<double, Lite3Dynamics::kJointsPerLeg> kJointSigns{{
    -1.0, 1.0, 1.0
}};

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

Lite3StateMapStatus Lite3StateMapper::map(
    const StateEstimate<float>& state,
    const LegControllerData<float> (&legs)[Lite3Dynamics::kNumLegs],
    const Lite3Dynamics& dynamics,
    Lite3MappedState& output) const noexcept
{
    output.q.setZero();
    output.v.setZero();
    output.motor_torque.setZero();

    if (!state.position.allFinite() || !state.vBody.allFinite()
        || !state.omegaBody.allFinite() || !state.rBody.allFinite()) {
        return Lite3StateMapStatus::NonFiniteInput;
    }

    const Eigen::Matrix3d world_R_body =
        state.rBody.template cast<double>().transpose();
    const Eigen::Matrix3d orthogonality_error =
        world_R_body.transpose() * world_R_body - Eigen::Matrix3d::Identity();
    if (!world_R_body.allFinite() || orthogonality_error.norm() > 1e-3
        || world_R_body.determinant() < 0.0) {
        return Lite3StateMapStatus::InvalidRotation;
    }

    Eigen::Quaterniond quaternion(world_R_body);
    if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9) {
        return Lite3StateMapStatus::InvalidRotation;
    }
    quaternion.normalize();

    output.q.template head<3>() = state.position.template cast<double>();
    output.q.template segment<4>(3) = quaternion.coeffs();  // x, y, z, w
    output.v.template head<3>() = state.vBody.template cast<double>();
    output.v.template segment<3>(3) = state.omegaBody.template cast<double>();

    const auto& model = dynamics.model();
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
        if (!legs[leg].q.allFinite() || !legs[leg].qd.allFinite()
            || !legs[leg].tauEstimate.allFinite()) {
            output.q.setZero();
            output.v.setZero();
            output.motor_torque.setZero();
            return Lite3StateMapStatus::NonFiniteInput;
        }

        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto joint_id = dynamics.jointId(kLegs[leg], kJoints[joint]);
            const int q_index = model.joints[joint_id].idx_q();
            const int v_index = model.joints[joint_id].idx_v();
            const double sign = kJointSigns[joint];
            output.q[q_index] = sign * static_cast<double>(legs[leg].q[joint]);
            output.v[v_index] = sign * static_cast<double>(legs[leg].qd[joint]);
            output.motor_torque[leg * Lite3Dynamics::kJointsPerLeg + joint] =
                sign * static_cast<double>(legs[leg].tauEstimate[joint]);
        }
    }

    return Lite3StateMapStatus::Ok;
}
