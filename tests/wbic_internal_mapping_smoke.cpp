#include <array>
#include <iostream>

#include "KinWbc.hpp"
#include "RobotModel.hpp"
#include "WbcType.h"

int main()
{
    RobotModel model;
    std::string error;
    if (!model.build({}, &error)) {
        std::cerr << "RobotModel build failed: " << error << std::endl;
        return 1;
    }

    Vec12 q_joint;
    q_joint << -0.08, -0.73, 1.43,
                0.04, -0.81, 1.56,
               -0.03, -0.69, 1.38,
                0.09, -0.87, 1.67;
    Vec12 dq_joint;
    dq_joint << 0.01, -0.02, 0.03,
               -0.04,  0.05, -0.06,
                0.07, -0.08, 0.09,
               -0.10,  0.11, -0.12;

    const Vec3 p_world(0.0, 0.0, 0.30);
    const Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
    model.setState(p_world, q_wb, Vec3::Zero(), Vec3::Zero(), q_joint, dq_joint);
    model.update();

    wbic::DynamicsOutput dyn;
    dyn.M = model.M();
    dyn.h = model.h();
    dyn.M_inv = dyn.M.ldlt().solve(
        Eigen::Matrix<double, wbic::kVelocityDimension,
                      wbic::kVelocityDimension>::Identity());
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        dyn.Jf[leg] = model.Jc().middleRows<3>(3 * leg);
        dyn.dJdq_f[leg] = model.dJv().segment<3>(3 * leg);
        dyn.pf[leg] = model.footPos(leg);
        dyn.vf[leg] = model.footVel(leg);
    }
    dyn.Jb = model.Jb();
    dyn.dJdq_b = model.dJvb();
    dyn.R_wb = model.baseRot();
    dyn.p_b = model.basePos();

    wbic::WbicInput input;
    input.p_body = p_world;
    input.R_body = Eigen::Matrix3d::Identity();
    input.q_joint_raw = q_joint;
    input.qd_joint_raw = dq_joint;
    input.p_body_des = p_world;
    input.R_body_des = input.R_body;
    input.contact = {{false, true, true, true}};  // FR swing only
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        input.p_foot_des[leg] = dyn.pf[leg];
        input.v_foot_des[leg] = dyn.vf[leg];
        input.a_foot_des[leg].setZero();
        input.Fr_des[leg] = Eigen::Vector3d(0.0, 0.0, 30.0);
    }
    input.p_foot_des[static_cast<int>(wbic::Leg::FR)].x() += 0.01;

    wbic::ContactSet contact_set;
    contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);
    std::array<int, wbic::kNumJoints> idx_v{};
    for (int k = 0; k < wbic::kNumJoints; ++k) idx_v[k] = model.idxV(k);

    wbic::WbicConfig config;
    wbic::KinWbc kin_wbc(config.damping);
    wbic::KinWbcResult result;
    if (!kin_wbc.Compute(input, dyn, contact_set, idx_v, config, &result)) {
        std::cerr << "KinWbc failed" << std::endl;
        return 2;
    }

    const double fr_correction = result.delta_q.segment<3>(0).norm();
    const double fl_correction = result.delta_q.segment<3>(3).norm();
    if (fr_correction < 1e-5 || fr_correction <= 5.0 * fl_correction) {
        std::cerr << "FR correction mapped to wrong public leg: FR=" << fr_correction
                  << " FL=" << fl_correction << std::endl;
        return 3;
    }

    if ((result.q_des - input.q_joint_raw - result.delta_q).norm() > 1e-10) {
        std::cerr << "q_des does not use mapped delta_q" << std::endl;
        return 4;
    }

    std::cout << "[TEST] wbic_internal_mapping_smoke PASSED!" << std::endl;
    return 0;
}
