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

    std::cout << "Lite3 Pinocchio model OK: nq=" << model.nq
              << " nv=" << model.nv
              << " mass=" << total_mass << '\n';
    return 0;
}
