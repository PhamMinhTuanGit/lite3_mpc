#include <iostream>
#include <vector>

#include "KinWbc.hpp"
#include "RobotModel.hpp"
#include "WbcType.h"

int main()
{
    std::cout << "[TEST] wbic_kin_smoke starting..." << std::endl;

    RobotModel model;
    RobotModelConfig cfg;
    std::string err;
    if (!model.build(cfg, &err)) {
        std::cerr << "RobotModel build failed: " << err << std::endl;
        return 1;
    }

    // Test SO(3) Rotation Error
    {
        const Eigen::Matrix3d R_cur = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d R_des_yaw = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d err_yaw = wbic::KinWbc::RotationErrorSO3(R_des_yaw, R_cur);
        if (std::abs(err_yaw[0]) > 1e-6 || std::abs(err_yaw[1]) > 1e-6 || std::abs(err_yaw[2] - 0.2) > 1e-6) {
            std::cerr << "SO(3) yaw rotation error calculation failed: " << err_yaw.transpose() << std::endl;
            return 2;
        }

        const Eigen::Matrix3d R_des_roll = Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Vector3d err_roll = wbic::KinWbc::RotationErrorSO3(R_des_roll, R_cur);
        if (std::abs(err_roll[0] - (-0.15)) > 1e-6 || std::abs(err_roll[1]) > 1e-6 || std::abs(err_roll[2]) > 1e-6) {
            std::cerr << "SO(3) roll rotation error calculation failed: " << err_roll.transpose() << std::endl;
            return 3;
        }
    }

    // Set a nominal state
    Vec3 p_world(0.0, 0.0, 0.28);
    Eigen::Quaterniond q_wb(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()));
    Vec3 v_lin_world(0.1, 0.0, 0.0);
    Vec3 omega_world(0.0, 0.0, 0.05);

    Vec12 q_joint, dq_joint;
    for (int leg = 0; leg < 4; ++leg) {
        q_joint[leg * 3 + 0] = 0.0;
        q_joint[leg * 3 + 1] = -0.7;
        q_joint[leg * 3 + 2] = 1.4;
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
    std::array<int, wbic::kNumJoints> idx_v{};
    for (int k = 0; k < wbic::kNumJoints; ++k) idx_v[k] = model.idxV(k);

    wbic::WbicInput input;
    input.p_body = p_world;
    input.R_body = q_wb.toRotationMatrix();
    input.v_body_world = v_lin_world;
    input.omega_body_world = omega_world;
    input.q_joint_raw = q_joint;
    input.qd_joint_raw = dq_joint;

    input.p_body_des = p_world + Vec3(0.01, 0.0, 0.0);
    input.v_body_des = v_lin_world;
    input.a_body_des = Vec3::Zero();
    input.R_body_des = input.R_body;
    input.omega_body_des = omega_world;
    input.domega_body_des = Vec3::Zero();

    for (int leg = 0; leg < 4; ++leg) {
        input.p_foot_des[leg] = dyn.pf[leg] + Vec3(0.0, 0.0, 0.05);
        input.v_foot_des[leg] = Vec3(0.1, 0.0, 0.0);
        input.a_foot_des[leg] = Vec3::Zero();
        input.Fr_des[leg] = Vec3(0.0, 0.0, 30.0);
    }

    // Test all 16 contact masks
    for (int mask = 0; mask < 16; ++mask) {
        for (int leg = 0; leg < 4; ++leg) {
            input.contact[leg] = ((mask >> leg) & 1) != 0;
        }

        wbic::ContactSet contact_set;
        contact_set.assemble(input.contact, input.Fr_des, dyn.Jf, dyn.dJdq_f);

        wbic::KinWbcResult result;
        const bool ok = kin_wbc.Compute(input, dyn, contact_set, idx_v, config, &result);
        if (!ok) {
            std::cerr << "KinWbc failed for contact mask " << mask << std::endl;
            return 4;
        }

        if (!result.qddot_cmd.allFinite() || !result.q_des.allFinite() || !result.dq_des.allFinite()) {
            std::cerr << "Non-finite output in KinWbc for mask " << mask << std::endl;
            return 5;
        }

        // For stance legs, contact acceleration residual must be small
        if (contact_set.nc > 0) {
            if (result.contact_acc_residual_norm > 1e-4) {
                std::cerr << "Contact acceleration residual too high ("
                          << result.contact_acc_residual_norm << ") for mask " << mask << std::endl;
                return 6;
            }
        }

        // Check joint limits satisfaction
        for (int k = 0; k < 12; ++k) {
            const int joint_type = k % 3;
            if (joint_type == 0) { // HipX
                if (result.q_des[k] < -0.523 - 1e-5 || result.q_des[k] > 0.523 + 1e-5) {
                    std::cerr << "HipX joint limit violated: " << result.q_des[k] << std::endl;
                    return 7;
                }
            } else if (joint_type == 1) { // HipY
                if (result.q_des[k] < -2.67 - 1e-5 || result.q_des[k] > 0.314 + 1e-5) {
                    std::cerr << "HipY joint limit violated: " << result.q_des[k] << std::endl;
                    return 8;
                }
            } else if (joint_type == 2) { // Knee
                if (result.q_des[k] < 0.524 - 1e-5 || result.q_des[k] > 2.792 + 1e-5) {
                    std::cerr << "Knee joint limit violated: " << result.q_des[k] << std::endl;
                    return 9;
                }
            }
        }
    }

    std::cout << "[TEST] wbic_kin_smoke PASSED!" << std::endl;
    return 0;
}
