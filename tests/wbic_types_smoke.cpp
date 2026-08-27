#include <cassert>
#include <limits>
#include <iostream>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include "RobotModel.hpp"
#include "WbcType.h"

int main()
{
    std::cout << "[TEST] wbic_types_smoke starting..." << std::endl;

    RobotModel robot_model;
    RobotModelConfig cfg;
    std::string err;
    if (!robot_model.build(cfg, &err)) {
        std::cerr << "Failed to build RobotModel: " << err << std::endl;
        return 1;
    }

    const auto& model = robot_model.model();

    wbic::FootFrameIds foot_ids{{
        model.getFrameId("FR_FOOT"),
        model.getFrameId("FL_FOOT"),
        model.getFrameId("HR_FOOT"),
        model.getFrameId("HL_FOOT"),
    }};

    const wbic::Configuration q = pinocchio::neutral(model);
    const wbic::GeneralizedVelocity v = wbic::GeneralizedVelocity::Zero();
    wbic::DynamicsInput dynamics_input(model, q, v, foot_ids);

    if (dynamics_input.validate() != wbic::InputStatus::Ok) {
        std::cerr << "DynamicsInput validation failed on neutral" << std::endl;
        return 1;
    }

    dynamics_input.q[6] = 2.0;
    if (dynamics_input.validate() != wbic::InputStatus::InvalidBaseQuaternion) {
        std::cerr << "Expected InvalidBaseQuaternion" << std::endl;
        return 2;
    }
    dynamics_input.q = q;
    dynamics_input.foot_ids[1] = dynamics_input.foot_ids[0];
    if (dynamics_input.validate() != wbic::InputStatus::DuplicateFootFrame) {
        std::cerr << "Expected DuplicateFootFrame" << std::endl;
        return 3;
    }

    wbic::ContactAssemblyInput contact_input;
    if (contact_input.validate() != wbic::InputStatus::Ok) {
        std::cerr << "ContactAssemblyInput default validation failed" << std::endl;
        return 4;
    }
    for (std::size_t leg = 0; leg < wbic::kNumLegs; ++leg) {
        if (!contact_input.f_mpc[leg].isZero()
            || !contact_input.Jf[leg].isZero()
            || !contact_input.dJdq_f[leg].isZero()) {
            std::cerr << "Contact input defaults not zero" << std::endl;
            return 5;
        }
    }

    contact_input.phase[0] = 1.01;
    if (contact_input.validate() != wbic::InputStatus::InvalidContactPhase) {
        std::cerr << "Expected InvalidContactPhase" << std::endl;
        return 6;
    }
    contact_input.phase[0] = 0.5;
    contact_input.f_mpc[2][1] = std::numeric_limits<double>::quiet_NaN();
    if (contact_input.validate() != wbic::InputStatus::NonFiniteValue) {
        std::cerr << "Expected NonFiniteValue on NaN force" << std::endl;
        return 7;
    }

    // Verify ContactSet assembly
    wbic::ContactSet contact_set;
    wbic::ContactFlags flags{{true, false, true, false}};
    wbic::FootVectorArray f_mpc{};
    f_mpc[0] = Eigen::Vector3d(10, 20, 50);
    f_mpc[1] = Eigen::Vector3d(0, 0, 0);
    f_mpc[2] = Eigen::Vector3d(-10, -20, 60);
    f_mpc[3] = Eigen::Vector3d(0, 0, 0);

    wbic::FootJacobianArray Jf{};
    wbic::FootVectorArray dJdq_f{};
    contact_set.assemble(flags, f_mpc, Jf, dJdq_f);

    if (contact_set.nc != 2 || contact_set.cid[0] != 0 || contact_set.cid[1] != 2) {
        std::cerr << "ContactSet assembly cid/nc error" << std::endl;
        return 8;
    }

    // Verify WbicConfig defaults
    wbic::WbicConfig config;
    if (std::abs(config.kp_body_pos - 100.0) > 1e-6 ||
        std::abs(config.kd_body_pos - 20.0) > 1e-6 ||
        std::abs(config.kp_body_ori - 100.0) > 1e-6 ||
        std::abs(config.kd_body_ori - 20.0) > 1e-6 ||
        std::abs(config.kp_foot - 500.0) > 1e-6 ||
        std::abs(config.mu - 0.4) > 1e-6) {
        std::cerr << "WbicConfig default parameter mismatch" << std::endl;
        return 9;
    }

    std::cout << "[TEST] wbic_types_smoke PASSED!" << std::endl;
    return 0;
}
