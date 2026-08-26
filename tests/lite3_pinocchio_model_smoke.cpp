#include <array>
#include <cmath>
#include <iostream>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "Lite3DynamicModel.hpp"

int main()
{
    Lite3Dynamics dynamics;
    const auto& model = dynamics.model();

    if (model.nq != Lite3Dynamics::kNq || model.nv != Lite3Dynamics::kNv) {
        return 1;
    }

    double total_mass = 0.0;
    for (const auto& inertia : model.inertias) {
        total_mass += inertia.mass();
    }
    if (std::abs(total_mass - 11.9376) > 1e-10) {
        return 2;
    }

    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    pinocchio::forwardKinematics(model, dynamics.data(), q, v);
    pinocchio::updateFramePlacements(model, dynamics.data());

    const std::array<Lite3Dynamics::Leg, 4> legs{{
        Lite3Dynamics::Leg::FR,
        Lite3Dynamics::Leg::FL,
        Lite3Dynamics::Leg::HR,
        Lite3Dynamics::Leg::HL
    }};
    const std::array<Eigen::Vector3d, 4> expected_positions{{
        Eigen::Vector3d( 0.1745, -0.15935, -0.41012),
        Eigen::Vector3d( 0.1745,  0.15935, -0.41012),
        Eigen::Vector3d(-0.1745, -0.15935, -0.41012),
        Eigen::Vector3d(-0.1745,  0.15935, -0.41012)
    }};

    for (std::size_t i = 0; i < legs.size(); ++i) {
        const auto frame_id = dynamics.footFrameId(legs[i]);
        if (frame_id >= model.nframes || model.frames[frame_id].parentFrame == frame_id) {
            return 3;
        }
        const Eigen::Vector3d position = dynamics.data().oMf[frame_id].translation();
        if ((position - expected_positions[i]).norm() > 1e-12) {
            return 4;
        }
    }

    const Eigen::MatrixXd& mass_matrix = dynamics.computeMassMatrix(q);
    if (!mass_matrix.allFinite()
        || (mass_matrix - mass_matrix.transpose()).norm() > 1e-12) {
        return 5;
    }

    const Eigen::VectorXd gravity = dynamics.computeGravity(q);
    const Eigen::VectorXd nonlinear = dynamics.computeNonlinearEffects(q, v);
    if ((gravity - nonlinear).norm() > 1e-12) {
        return 6;
    }

    // Exercise C(q,v) with non-zero velocity. The core smoke uses v=0, which
    // cannot detect a broken Coriolis implementation.
    for (int sample = 0; sample < 8; ++sample) {
        q = pinocchio::neutral(model);
        v.setZero();
        for (int joint = 2; joint < static_cast<int>(model.njoints); ++joint) {
            const int iq = model.joints[joint].idx_q();
            const int iv = model.joints[joint].idx_v();
            q[iq] = -0.9 + 0.13 * sample + 0.04 * joint;
            v[iv] = -0.7 + 0.09 * sample + 0.03 * joint;
        }
        v.head<6>() << 0.2, -0.1, 0.05, -0.3, 0.15, 0.1;
        const Eigen::MatrixXd mass = dynamics.computeMassMatrix(q);
        const Eigen::MatrixXd coriolis = dynamics.computeCoriolisMatrix(q, v);
        const Eigen::VectorXd g = dynamics.computeGravity(q);
        const Eigen::VectorXd h = dynamics.computeNonlinearEffects(q, v);
        if (!mass.allFinite() || !coriolis.allFinite() || !g.allFinite()
            || !h.allFinite() || (mass - mass.transpose()).norm() > 1e-10
            || (h - coriolis * v - g).norm() > 1e-9) {
            return 7;
        }
    }

    // Validate every actuated Jacobian column against finite differences.
    q = pinocchio::neutral(model);
    for (std::size_t leg = 0; leg < legs.size(); ++leg) {
        const std::array<double, 3> angles{{0.08, -1.0, 2.0}};
        for (std::size_t joint = 0; joint < angles.size(); ++joint) {
            const auto joint_id = dynamics.jointId(
                legs[leg], static_cast<Lite3Dynamics::LegJoint>(joint));
            q[model.joints[joint_id].idx_q()] = angles[joint];
        }
    }
    const auto jacobians = dynamics.computeFootJacobians(q);
    constexpr double epsilon = 1e-7;
    for (std::size_t leg = 0; leg < legs.size(); ++leg) {
        for (std::size_t joint = 0; joint < 3; ++joint) {
            const auto joint_id = dynamics.jointId(
                legs[leg], static_cast<Lite3Dynamics::LegJoint>(joint));
            const int iq = model.joints[joint_id].idx_q();
            const int iv = model.joints[joint_id].idx_v();
            Eigen::VectorXd q_plus = q;
            Eigen::VectorXd q_minus = q;
            q_plus[iq] += epsilon;
            q_minus[iq] -= epsilon;
            pinocchio::forwardKinematics(model, dynamics.data(), q_plus);
            pinocchio::updateFramePlacements(model, dynamics.data());
            const Eigen::Vector3d p_plus =
                dynamics.data().oMf[dynamics.footFrameId(legs[leg])].translation();
            pinocchio::forwardKinematics(model, dynamics.data(), q_minus);
            pinocchio::updateFramePlacements(model, dynamics.data());
            const Eigen::Vector3d p_minus =
                dynamics.data().oMf[dynamics.footFrameId(legs[leg])].translation();
            const Eigen::Vector3d numeric = (p_plus - p_minus) / (2.0 * epsilon);
            if ((numeric - jacobians[leg].col(iv)).norm() > 1e-6) {
                return 8;
            }
        }
    }

    std::cout << "Lite3 Pinocchio model OK: nq=" << model.nq
              << " nv=" << model.nv
              << " mass=" << total_mass << '\n';
    return 0;
}
