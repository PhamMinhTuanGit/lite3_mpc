#include <iostream>
#include <cmath>

#include "KinWbc.hpp"
#include "RobotModel.hpp"
#include "WbcType.h"
#include "WbicQp.hpp"

int main()
{
    std::cout << "[TEST] wbic_qp_smoke starting..." << std::endl;

    RobotModel model;
    RobotModelConfig cfg;
    std::string err;
    if (!model.build(cfg, &err)) {
        std::cerr << "RobotModel build failed: " << err << std::endl;
        return 1;
    }

    std::array<int, 12> idx_v{};
    for (int k = 0; k < 12; ++k) {
        idx_v[k] = model.idxV(k);
    }

    // Set a standing state
    Vec3 p_world(0.0, 0.0, 0.30);
    Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();
    Vec3 v_lin_world = Vec3::Zero();
    Vec3 omega_world = Vec3::Zero();

    Vec12 q_joint, dq_joint;
    for (int leg = 0; leg < 4; ++leg) {
        q_joint[leg * 3 + 0] = (leg % 2 == 0) ? -0.05 : 0.05;
        q_joint[leg * 3 + 1] = -0.8;
        q_joint[leg * 3 + 2] = 1.6;
        dq_joint[leg * 3 + 0] = 0.0;
        dq_joint[leg * 3 + 1] = 0.0;
        dq_joint[leg * 3 + 2] = 0.0;
    }

    model.setState(p_world, q_wb, v_lin_world, omega_world, q_joint, dq_joint);
    model.update();

    wbic::DynamicsOutput dyn;
    dyn.M = model.M();
    dyn.h = model.h();
    dyn.M_inv = dyn.M.ldlt().solve(Eigen::Matrix<double, 18, 18>::Identity());
    for (int leg = 0; leg < 4; ++leg) {
        dyn.Jf[leg] = model.Jc().middleRows<3>(3 * leg);
        dyn.dJdq_f[leg] = model.dJv().segment<3>(3 * leg);
        dyn.pf[leg] = model.footPos(leg);
        dyn.vf[leg] = model.footVel(leg);
    }
    dyn.Jb = model.Jb();
    dyn.dJdq_b = model.dJvb();
    dyn.R_wb = model.baseRot();
    dyn.p_b = model.basePos();
    dyn.v_b_world = v_lin_world;
    dyn.omega_b_world = omega_world;

    wbic::WbicConfig config;
    wbic::KinWbc kin_wbc(config.damping);
    wbic::WbicQp wbic_qp(config);

    wbic::WbicInput input;
    input.p_body = p_world;
    input.R_body = q_wb.toRotationMatrix();
    input.v_body_world = v_lin_world;
    input.omega_body_world = omega_world;
    input.q_joint_raw = q_joint;
    input.qd_joint_raw = dq_joint;

    input.p_body_des = p_world;
    input.v_body_des = v_lin_world;
    input.a_body_des = Vec3::Zero();
    input.R_body_des = input.R_body;
    input.omega_body_des = omega_world;
    input.domega_body_des = Vec3::Zero();

    for (int leg = 0; leg < 4; ++leg) {
        input.p_foot_des[leg] = dyn.pf[leg];
        input.v_foot_des[leg] = Vec3::Zero();
        input.a_foot_des[leg] = Vec3::Zero();
        input.Fr_des[leg] = Vec3(0.0, 0.0, 11.9376 * 9.81 / 4.0); // nominal gravity support ~29.3 N
    }

    // ─────────────────────────────────────────────────────────────────
    // Test Case 1: 4-Leg Stance
    // ─────────────────────────────────────────────────────────────────
    {
        input.contact = {{true, true, true, true}};
        wbic::ContactSet contact_set;
        contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);

        wbic::KinWbcResult kin_res;
        if (!kin_wbc.Compute(input, dyn, contact_set, config, &kin_res)) {
            std::cerr << "Case 1: KinWbc failed" << std::endl;
            return 2;
        }

        wbic::WbicQpResult qp_res;
        const bool solve_ok = wbic_qp.Solve(input, dyn, contact_set, kin_res.qddot_cmd,
                                            kin_res.q_des, kin_res.dq_des, idx_v, config, &qp_res);
        std::cout << "Case 1: solve_ok=" << solve_ok << ", status=" << static_cast<int>(qp_res.status)
                  << ", EoM=" << qp_res.residuals.eom_residual_norm
                  << ", ContactAcc=" << qp_res.residuals.contact_acc_residual_norm
                  << ", Ineq=" << qp_res.residuals.inequality_violation_norm << std::endl;
        if (!solve_ok) {
            std::cerr << "Case 1: WbicQp failed, status = " << static_cast<int>(qp_res.status) << std::endl;
            return 3;
        }

        if (qp_res.residuals.eom_residual_norm > 1e-3) {
            std::cerr << "Case 1: EoM residual too high: " << qp_res.residuals.eom_residual_norm << std::endl;
            return 4;
        }

        // Verify total vertical force is approximately mg
        double total_fz = 0.0;
        for (int leg = 0; leg < 4; ++leg) {
            total_fz += qp_res.f_opt[leg * 3 + 2];
            const double fx = qp_res.f_opt[leg * 3 + 0];
            const double fy = qp_res.f_opt[leg * 3 + 1];
            const double fz = qp_res.f_opt[leg * 3 + 2];
            if (std::abs(fx) > config.mu * fz + 1e-4 || std::abs(fy) > config.mu * fz + 1e-4) {
                std::cerr << "Case 1: Friction pyramid violated on leg " << leg << std::endl;
                return 5;
            }
        }
        const double expected_fz = 11.9376 * 9.81;
        if (std::abs(total_fz - expected_fz) > 1.0) {
            std::cerr << "Case 1: Total Fz (" << total_fz << ") does not match mg (" << expected_fz << ")" << std::endl;
            return 6;
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Test Case 2: 2-Leg Trot (FR + HL in stance, FL + HR in swing)
    // ─────────────────────────────────────────────────────────────────
    {
        input.contact = {{true, false, false, true}};
        input.Fr_des[0] = Vec3(0.0, 0.0, 11.9376 * 9.81 / 2.0);
        input.Fr_des[1] = Vec3(0.0, 0.0, 0.0);
        input.Fr_des[2] = Vec3(0.0, 0.0, 0.0);
        input.Fr_des[3] = Vec3(0.0, 0.0, 11.9376 * 9.81 / 2.0);

        wbic::ContactSet contact_set;
        contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);

        wbic::KinWbcResult kin_res;
        kin_wbc.Compute(input, dyn, contact_set, config, &kin_res);

        wbic::WbicQpResult qp_res;
        if (!wbic_qp.Solve(input, dyn, contact_set, kin_res.qddot_cmd,
                           kin_res.q_des, kin_res.dq_des, idx_v, config, &qp_res)) {
            std::cerr << "Case 2: 2-leg trot QP failed" << std::endl;
            return 7;
        }

        // Swing legs (FL, HR) must have strictly zero force
        if (qp_res.f_opt.segment<3>(1 * 3).norm() > 1e-6 || qp_res.f_opt.segment<3>(2 * 3).norm() > 1e-6) {
            std::cerr << "Case 2: Swing leg force non-zero!" << std::endl;
            return 8;
        }

        if (qp_res.residuals.eom_residual_norm > 1e-3) {
            std::cerr << "Case 2: EoM residual too high: " << qp_res.residuals.eom_residual_norm << std::endl;
            return 9;
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Test Case 3: Flight Mode (all 4 swing)
    // ─────────────────────────────────────────────────────────────────
    {
        input.contact = {{false, false, false, false}};
        for (int leg = 0; leg < 4; ++leg) input.Fr_des[leg].setZero();

        wbic::ContactSet contact_set;
        contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);

        wbic::KinWbcResult kin_res;
        kin_wbc.Compute(input, dyn, contact_set, config, &kin_res);

        wbic::WbicQpResult qp_res;
        if (!wbic_qp.Solve(input, dyn, contact_set, kin_res.qddot_cmd,
                           kin_res.q_des, kin_res.dq_des, idx_v, config, &qp_res)) {
            std::cerr << "Case 3: Flight mode QP failed" << std::endl;
            return 10;
        }

        if (qp_res.f_opt.norm() > 1e-6) {
            std::cerr << "Case 3: Flight mode has non-zero contact force" << std::endl;
            return 11;
        }
    }

    std::cout << "[TEST] wbic_qp_smoke PASSED!" << std::endl;
    return 0;
}
