#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Geometry>

#include "Controllers/GeneralizedMomentumObserver.h"
#include "Controllers/GroundReactionForceEstimator.h"
#include "Lite3StateMapper.hpp"

namespace {
constexpr std::array<Lite3Dynamics::Leg, 4> kLegs{{
    Lite3Dynamics::Leg::FR, Lite3Dynamics::Leg::FL,
    Lite3Dynamics::Leg::HR, Lite3Dynamics::Leg::HL
}};
constexpr std::array<Lite3Dynamics::LegJoint, 3> kJoints{{
    Lite3Dynamics::LegJoint::HipX,
    Lite3Dynamics::LegJoint::HipY,
    Lite3Dynamics::LegJoint::Knee
}};
}  // namespace

int main()
{
    Lite3Dynamics dynamics;
    Lite3StateMapper mapper;
    StateEstimate<float> state{};
    LegControllerData<float> legs[4];
    Lite3MappedState mapped;

    state.position << 0.1f, -0.2f, 0.35f;
    state.vBody << 0.2f, -0.1f, 0.05f;
    state.omegaBody << -0.3f, 0.15f, 0.1f;
    const Eigen::Matrix3f world_R_body =
        (Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitZ())
         * Eigen::AngleAxisf(-0.1f, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(0.05f, Eigen::Vector3f::UnitX())).matrix();
    state.rBody = world_R_body.transpose();
    for (int leg = 0; leg < 4; ++leg) {
        legs[leg].q << 0.08f, -1.0f, 2.0f;
        legs[leg].qd << 0.1f + leg, 0.2f + leg, 0.3f + leg;
        legs[leg].tauEstimate << 1.0f + leg, 2.0f + leg, 3.0f + leg;
    }

    if (mapper.map(state, legs, dynamics, mapped) != Lite3StateMapStatus::Ok) return 1;
    if ((mapped.q.head<3>() - state.position.cast<double>()).norm() > 1e-12
        || (mapped.v.head<3>() - state.vBody.cast<double>()).norm() > 1e-12
        || (mapped.v.segment<3>(3) - state.omegaBody.cast<double>()).norm() > 1e-12) return 2;
    const Eigen::Quaterniond quaternion(mapped.q[6], mapped.q[3], mapped.q[4], mapped.q[5]);
    if ((quaternion.toRotationMatrix() - world_R_body.cast<double>()).norm() > 1e-6) return 3;

    const auto& model = dynamics.model();
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const double sign = joint == 0 ? -1.0 : 1.0;
            const auto id = dynamics.jointId(kLegs[leg], kJoints[joint]);
            if (std::abs(mapped.q[model.joints[id].idx_q()] - sign * legs[leg].q[joint]) > 1e-12
                || std::abs(mapped.v[model.joints[id].idx_v()] - sign * legs[leg].qd[joint]) > 1e-12
                || std::abs(mapped.motor_torque[leg * 3 + joint]
                            - sign * legs[leg].tauEstimate[joint]) > 1e-12) return 4;
        }
    }

    GMOConfig config;
    GeneralizedMomentumObserver observer(dynamics, 0.001, config);
    const GMOResult first = observer.update(mapped.q, mapped.v, mapped.motor_torque);
    if (!first.valid || !first.initialized || first.residual.norm() > 1e-12
        || (first.momentum - first.momentum_hat).norm() > 1e-12) return 5;

    const Lite3Dynamics::MassMatrix coriolis = dynamics.computeCoriolisMatrix(mapped.q, mapped.v);
    const Lite3Dynamics::Velocity gravity = dynamics.computeGravity(mapped.q);
    const Lite3Dynamics::Velocity beta = gravity - coriolis.transpose() * mapped.v;
    Lite3Dynamics::Velocity tau = Lite3Dynamics::Velocity::Zero();
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg)
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto id = dynamics.jointId(kLegs[leg], kJoints[joint]);
            tau[model.joints[id].idx_v()] = mapped.motor_torque[leg * 3 + joint];
        }
    const Lite3Dynamics::Velocity expected_hat = first.momentum + 0.001 * (tau - beta);
    const GMOResult second = observer.update(mapped.q, mapped.v, mapped.motor_torque);
    if (!second.valid || (second.momentum_hat - expected_hat).norm() > 1e-9) return 6;

    GroundReactionForceEstimator grf(dynamics, 1e-10);
    Lite3Dynamics::Velocity residual = Lite3Dynamics::Velocity::Zero();
    const auto jacobians = dynamics.computeFootJacobians(mapped.q);
    const std::array<Eigen::Vector3d, 4> expected_forces{{
        Eigen::Vector3d(2.0, -1.0, 25.0), Eigen::Vector3d(-1.0, 0.5, 30.0),
        Eigen::Vector3d(0.5, 1.0, 27.0), Eigen::Vector3d(-0.5, -1.5, 29.0)
    }};
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg) {
        Eigen::Matrix3d joint_jacobian;
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto id = dynamics.jointId(kLegs[leg], kJoints[joint]);
            joint_jacobian.col(joint) = jacobians[leg].col(model.joints[id].idx_v());
        }
        const Eigen::Vector3d joint_residual = joint_jacobian.transpose() * expected_forces[leg];
        for (std::size_t joint = 0; joint < kJoints.size(); ++joint) {
            const auto id = dynamics.jointId(kLegs[leg], kJoints[joint]);
            residual[model.joints[id].idx_v()] = joint_residual[joint];
        }
    }
    const GRFResult forces = grf.update(mapped.q, residual);
    if (!forces.valid) return 7;
    for (std::size_t leg = 0; leg < kLegs.size(); ++leg)
        if ((forces.force_world[leg] - expected_forces[leg]).norm() > 1e-4) return 8;

    state.position[0] = std::numeric_limits<float>::quiet_NaN();
    if (mapper.map(state, legs, dynamics, mapped) != Lite3StateMapStatus::NonFiniteInput) return 9;
    observer.reset();
    if (observer.result().valid || observer.result().initialized) return 10;

    std::cout << "Lite3 GMO/GRF OK: mapper, observer, damped GRF\n";
    return 0;
}
