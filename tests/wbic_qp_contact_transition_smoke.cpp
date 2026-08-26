#include <array>
#include <cmath>
#include <iostream>

#include "KinWbc.hpp"
#include "RobotModel.hpp"
#include "WbcType.h"
#include "WbicQp.hpp"

namespace {

bool Near(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}

bool CompareResults(const wbic::WbicQpResult& persistent,
                    const wbic::WbicQpResult& fresh,
                    int pattern_index)
{
    constexpr double kVectorTolerance = 1e-7;
    constexpr double kResidualTolerance = 1e-8;
    if (persistent.status != fresh.status
        || (persistent.qddot - fresh.qddot).lpNorm<Eigen::Infinity>() > kVectorTolerance
        || (persistent.f_opt - fresh.f_opt).lpNorm<Eigen::Infinity>() > kVectorTolerance
        || (persistent.tau_ff - fresh.tau_ff).lpNorm<Eigen::Infinity>() > kVectorTolerance
        || !Near(persistent.residuals.eom_residual_norm,
                 fresh.residuals.eom_residual_norm, kResidualTolerance)
        || !Near(persistent.residuals.contact_acc_residual_norm,
                 fresh.residuals.contact_acc_residual_norm, kResidualTolerance)
        || !Near(persistent.residuals.inequality_violation_norm,
                 fresh.residuals.inequality_violation_norm, kResidualTolerance)
        || !Near(persistent.residuals.torque_limit_margin,
                 fresh.residuals.torque_limit_margin, kResidualTolerance)) {
        std::cerr << "Persistent/fresh mismatch at pattern " << pattern_index
                  << ": status " << static_cast<int>(persistent.status)
                  << "/" << static_cast<int>(fresh.status)
                  << ", qddot=" << (persistent.qddot - fresh.qddot).norm()
                  << ", force=" << (persistent.f_opt - fresh.f_opt).norm()
                  << ", torque=" << (persistent.tau_ff - fresh.tau_ff).norm()
                  << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    RobotModel model;
    std::string error;
    if (!model.build({}, &error)) {
        std::cerr << "RobotModel build failed: " << error << std::endl;
        return 1;
    }

    const Vec3 p_world(0.0, 0.0, 0.30);
    Vec12 q_joint;
    Vec12 dq_joint = Vec12::Zero();
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        q_joint[3 * leg] = (leg % 2 == 0) ? -0.05 : 0.05;
        q_joint[3 * leg + 1] = -0.8;
        q_joint[3 * leg + 2] = 1.6;
    }
    model.setState(p_world, Eigen::Quaterniond::Identity(), Vec3::Zero(),
                   Vec3::Zero(), q_joint, dq_joint);
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
    for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
        input.p_foot_des[leg] = dyn.pf[leg];
        input.v_foot_des[leg].setZero();
        input.a_foot_des[leg].setZero();
    }

    const std::array<wbic::ContactFlags, 6> patterns{{
        {{true, true, true, true}},
        {{true, false, false, true}},   // FR + HL
        {{false, true, true, false}},   // FL + HR
        {{false, false, false, false}}, // flight
        {{true, false, false, true}},   // FR + HL
        {{true, true, true, true}},
    }};

    std::array<int, wbic::kNumJoints> idx_v{};
    for (int k = 0; k < wbic::kNumJoints; ++k) idx_v[k] = model.idxV(k);

    wbic::WbicConfig config;
    config.max_wsr = 200;
    config.max_cpu_time = 0.01;
    wbic::KinWbc kin_wbc(config.damping);
    wbic::WbicQp persistent_solver(config);

    for (std::size_t i = 0; i < patterns.size(); ++i) {
        input.contact = patterns[i];
        int contact_count = 0;
        for (bool contact : input.contact) contact_count += contact ? 1 : 0;
        for (int leg = 0; leg < wbic::kNumLegs; ++leg) {
            input.Fr_des[leg].setZero();
            if (input.contact[leg]) {
                input.Fr_des[leg].z() = 11.9376 * 9.81 / contact_count;
            }
        }

        wbic::ContactSet contact_set;
        contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);
        wbic::KinWbcResult kin_result;
        if (!kin_wbc.Compute(input, dyn, contact_set, idx_v, config, &kin_result)) {
            std::cerr << "KinWbc failed at pattern " << i << std::endl;
            return 2;
        }

        wbic::WbicQpResult persistent_result;
        const bool persistent_ok = persistent_solver.Solve(
            input, dyn, contact_set, kin_result.qddot_cmd, kin_result.q_des,
            kin_result.dq_des, idx_v, config, &persistent_result);

        wbic::WbicQp fresh_solver(config);
        wbic::WbicQpResult fresh_result;
        const bool fresh_ok = fresh_solver.Solve(
            input, dyn, contact_set, kin_result.qddot_cmd, kin_result.q_des,
            kin_result.dq_des, idx_v, config, &fresh_result);

        if (persistent_ok != fresh_ok
            || !CompareResults(persistent_result, fresh_result, static_cast<int>(i))) {
            return 3;
        }
        if (!persistent_ok) {
            std::cerr << "QP failed at pattern " << i << " with matching status "
                      << static_cast<int>(persistent_result.status) << std::endl;
            return 4;
        }
    }

    std::cout << "[TEST] wbic_qp_contact_transition_smoke PASSED!" << std::endl;
    return 0;
}
