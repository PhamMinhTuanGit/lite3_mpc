#include <cassert>
#include <cmath>
#include <iostream>
#include <numeric>

#include "CentroidalWrenchController.hpp"

namespace {

constexpr double kTol = 1e-3;

void Test1_SymmetricStanding()
{
    std::cout << ">>> Running Test 1: Symmetric Standing..." << std::endl;

    wbic::CentroidalWrenchConfig cfg;
    wbic::CentroidalWrenchController controller(cfg);

    wbic::CentroidalWrenchInput input;
    input.mass = 12.0; // kg
    input.com_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_des_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_vel_world.setZero();
    input.com_vel_des_world.setZero();
    input.com_acc_des_world.setZero();

    input.R_world_body = Eigen::Matrix3d::Identity();
    input.R_des_world_body = Eigen::Matrix3d::Identity();
    input.omega_world.setZero();
    input.omega_des_world.setZero();

    // Symmetric foot layout: FR, FL, HR, HL
    input.foot_pos_world[0] = Eigen::Vector3d( 0.20, -0.15, 0.0);
    input.foot_pos_world[1] = Eigen::Vector3d( 0.20,  0.15, 0.0);
    input.foot_pos_world[2] = Eigen::Vector3d(-0.20, -0.15, 0.0);
    input.foot_pos_world[3] = Eigen::Vector3d(-0.20,  0.15, 0.0);
    input.contact = {{true, true, true, true}};

    wbic::CentroidalWrenchOutput output;
    const bool ok = controller.Compute(input, &output);
    assert(ok && "Controller Compute failed in Test 1");

    const double expected_Fz = input.mass * cfg.gravity;
    std::cout << "  F_des: " << output.force_des_world.transpose() << std::endl;
    std::cout << "  F_grf: " << output.force_grf_world.transpose() << std::endl;
    std::cout << "  M_grf: " << output.moment_grf_world.transpose() << std::endl;
    for (int i = 0; i < 4; ++i) {
        std::cout << "  f[" << i << "]: " << output.grf_world[i].transpose() << std::endl;
    }

    assert(std::abs(output.force_grf_world.x()) < 0.1);
    assert(std::abs(output.force_grf_world.y()) < 0.1);
    assert(std::abs(output.force_grf_world.z() - expected_Fz) < 0.1);
    assert(output.moment_grf_world.norm() < 0.1);

    // Each foot should carry approximately 1/4 of total weight
    for (int i = 0; i < 4; ++i) {
        assert(std::abs(output.grf_world[i].z() - expected_Fz / 4.0) < 1.0);
        assert(std::abs(output.grf_world[i].x()) < 0.1);
        assert(std::abs(output.grf_world[i].y()) < 0.1);
    }
    std::cout << ">>> Test 1 PASSED.\n" << std::endl;
}

void Test2_PitchMoment()
{
    std::cout << ">>> Running Test 2: Pitch Moment Sign & Redistribution..." << std::endl;

    wbic::CentroidalWrenchConfig cfg;
    wbic::CentroidalWrenchController controller(cfg);

    wbic::CentroidalWrenchInput input;
    input.mass = 12.0;
    input.com_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_des_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_vel_world.setZero();
    input.com_vel_des_world.setZero();
    input.com_acc_des_world.setZero();

    // Robot pitched forward (+pitch = nose down around +y)
    const double pitch_angle = 0.05; // ~2.86 degrees
    input.R_world_body = (Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY())).toRotationMatrix();
    input.R_des_world_body = Eigen::Matrix3d::Identity(); // Desired pitch = 0
    input.omega_world.setZero();
    input.omega_des_world.setZero();

    input.foot_pos_world[0] = Eigen::Vector3d( 0.20, -0.15, 0.0);
    input.foot_pos_world[1] = Eigen::Vector3d( 0.20,  0.15, 0.0);
    input.foot_pos_world[2] = Eigen::Vector3d(-0.20, -0.15, 0.0);
    input.foot_pos_world[3] = Eigen::Vector3d(-0.20,  0.15, 0.0);
    input.contact = {{true, true, true, true}};

    wbic::CentroidalWrenchOutput output;
    const bool ok = controller.Compute(input, &output);
    assert(ok && "Controller Compute failed in Test 2");

    std::cout << "  M_des: " << output.moment_des_world.transpose() << std::endl;
    std::cout << "  M_grf: " << output.moment_grf_world.transpose() << std::endl;
    std::cout << "  Fz front [FR, FL]: " << output.grf_world[0].z() << ", " << output.grf_world[1].z() << std::endl;
    std::cout << "  Fz rear  [HR, HL]: " << output.grf_world[2].z() << ", " << output.grf_world[3].z() << std::endl;

    // Pitch error is negative (nose down, needs nose up restorative moment M_y < 0)
    assert(output.moment_des_world.y() < 0.0);
    assert(output.moment_grf_world.y() < 0.0);

    // Front feet must push harder than rear feet to create negative pitch torque
    const double Fz_front = output.grf_world[0].z() + output.grf_world[1].z();
    const double Fz_rear  = output.grf_world[2].z() + output.grf_world[3].z();
    assert(Fz_front > Fz_rear);

    std::cout << ">>> Test 2 PASSED.\n" << std::endl;
}

void Test3_CoMOffset()
{
    std::cout << ">>> Running Test 3: CoM Offset Equilibrium..." << std::endl;

    wbic::CentroidalWrenchConfig cfg;
    wbic::CentroidalWrenchController controller(cfg);

    wbic::CentroidalWrenchInput input;
    input.mass = 14.0; // Payload added
    // CoM shifted forward along +x by +0.02 m
    input.com_world = Eigen::Vector3d(0.02, 0.0, 0.30);
    input.com_des_world = Eigen::Vector3d(0.02, 0.0, 0.30);
    input.com_vel_world.setZero();
    input.com_vel_des_world.setZero();
    input.com_acc_des_world.setZero();

    input.R_world_body = Eigen::Matrix3d::Identity();
    input.R_des_world_body = Eigen::Matrix3d::Identity();
    input.omega_world.setZero();
    input.omega_des_world.setZero();

    input.foot_pos_world[0] = Eigen::Vector3d( 0.20, -0.15, 0.0);
    input.foot_pos_world[1] = Eigen::Vector3d( 0.20,  0.15, 0.0);
    input.foot_pos_world[2] = Eigen::Vector3d(-0.20, -0.15, 0.0);
    input.foot_pos_world[3] = Eigen::Vector3d(-0.20,  0.15, 0.0);
    input.contact = {{true, true, true, true}};

    wbic::CentroidalWrenchOutput output;
    const bool ok = controller.Compute(input, &output);
    assert(ok && "Controller Compute failed in Test 3");

    std::cout << "  M_des: " << output.moment_des_world.transpose() << std::endl;
    std::cout << "  M_grf: " << output.moment_grf_world.transpose() << std::endl;
    std::cout << "  Fz front [FR, FL]: " << output.grf_world[0].z() << ", " << output.grf_world[1].z() << std::endl;
    std::cout << "  Fz rear  [HR, HL]: " << output.grf_world[2].z() << ", " << output.grf_world[3].z() << std::endl;

    // Desired moment is zero (in equilibrium at shifted CoM)
    assert(output.moment_des_world.norm() < 1e-6);
    assert(output.moment_grf_world.norm() < 0.1);

    // Front feet have arm rx = 0.20 - 0.02 = 0.18 m
    // Rear feet have arm rx = -0.20 - 0.02 = -0.22 m
    // To have zero moment: Fz_front * 0.18 = Fz_rear * 0.22 => Fz_front / Fz_rear = 0.22 / 0.18 = 1.222
    const double Fz_front = output.grf_world[0].z() + output.grf_world[1].z();
    const double Fz_rear  = output.grf_world[2].z() + output.grf_world[3].z();
    const double ratio = Fz_front / Fz_rear;
    std::cout << "  Fz_front / Fz_rear = " << ratio << " (expected ~ 1.222)" << std::endl;
    assert(std::abs(ratio - (0.22 / 0.18)) < 0.05);

    std::cout << ">>> Test 3 PASSED.\n" << std::endl;
}

void Test4_SwingForce()
{
    std::cout << ">>> Running Test 4: Swing Foot Zero Force..." << std::endl;

    wbic::CentroidalWrenchConfig cfg;
    wbic::CentroidalWrenchController controller(cfg);

    wbic::CentroidalWrenchInput input;
    input.mass = 12.0;
    input.com_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_des_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input.com_vel_world.setZero();
    input.com_vel_des_world.setZero();
    input.com_acc_des_world.setZero();

    input.R_world_body = Eigen::Matrix3d::Identity();
    input.R_des_world_body = Eigen::Matrix3d::Identity();
    input.omega_world.setZero();
    input.omega_des_world.setZero();

    input.foot_pos_world[0] = Eigen::Vector3d( 0.20, -0.15, 0.0);
    input.foot_pos_world[1] = Eigen::Vector3d( 0.20,  0.15, 0.0);
    input.foot_pos_world[2] = Eigen::Vector3d(-0.20, -0.15, 0.0);
    input.foot_pos_world[3] = Eigen::Vector3d(-0.20,  0.15, 0.0);
    
    // Leg 1 (FL) is in swing
    input.contact = {{true, false, true, true}};

    wbic::CentroidalWrenchOutput output;
    const bool ok = controller.Compute(input, &output);
    assert(ok && "Controller Compute failed in Test 4");

    std::cout << "  f[FL (swing)]: " << output.grf_world[1].transpose() << std::endl;
    assert(output.grf_world[1].norm() < 1e-6);

    const double expected_Fz = input.mass * cfg.gravity;
    std::cout << "  F_grf total z: " << output.force_grf_world.z() << " (expected: " << expected_Fz << ")" << std::endl;
    assert(std::abs(output.force_grf_world.z() - expected_Fz) < 0.5);

    std::cout << ">>> Test 4 PASSED.\n" << std::endl;
}

void Test5_ForceRateRegularization()
{
    std::cout << ">>> Running Test 5: Force-Rate Regularization Penalty..." << std::endl;

    wbic::CentroidalWrenchInput input1;
    input1.mass = 12.0;
    input1.com_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input1.com_des_world = Eigen::Vector3d(0.0, 0.0, 0.30);
    input1.foot_pos_world[0] = Eigen::Vector3d( 0.20, -0.15, 0.0);
    input1.foot_pos_world[1] = Eigen::Vector3d( 0.20,  0.15, 0.0);
    input1.foot_pos_world[2] = Eigen::Vector3d(-0.20, -0.15, 0.0);
    input1.foot_pos_world[3] = Eigen::Vector3d(-0.20,  0.15, 0.0);
    input1.contact = {{true, true, true, true}};

    // Sudden disturbance input
    wbic::CentroidalWrenchInput input2 = input1;
    input2.R_world_body = (Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())).toRotationMatrix();

    // 1. Controller without force-rate penalty (w_force_rate = 0.0)
    wbic::CentroidalWrenchConfig cfg_unpenalized;
    cfg_unpenalized.w_force_rate = 0.0;
    wbic::CentroidalWrenchController ctrl_unpenalized(cfg_unpenalized);

    wbic::CentroidalWrenchOutput out1_unpen, out2_unpen;
    ctrl_unpenalized.Compute(input1, &out1_unpen);
    ctrl_unpenalized.Compute(input2, &out2_unpen);
    const double df_unpenalized = out2_unpen.df_norm;

    // 2. Controller with force-rate penalty (w_force_rate = 10.0)
    wbic::CentroidalWrenchConfig cfg_penalized;
    cfg_penalized.w_force_rate = 10.0;
    wbic::CentroidalWrenchController ctrl_penalized(cfg_penalized);

    wbic::CentroidalWrenchOutput out1_pen, out2_pen;
    ctrl_penalized.Compute(input1, &out1_pen);
    ctrl_penalized.Compute(input2, &out2_pen);
    const double df_penalized = out2_pen.df_norm;

    std::cout << "  ||delta_f|| without force-rate penalty: " << df_unpenalized << " N" << std::endl;
    std::cout << "  ||delta_f|| with force-rate penalty:    " << df_penalized << " N" << std::endl;
    std::cout << "  Telemetry breakdown: Fz_gravity=" << out2_pen.Fz_gravity 
              << ", My_P=" << out2_pen.My_P << ", df_norm=" << out2_pen.df_norm << std::endl;

    assert(df_penalized < df_unpenalized && "Force-rate penalty should reduce ||delta f||");
    assert(out2_pen.df_norm > 0.0);
    assert(out2_pen.valid);

    std::cout << ">>> Test 5 PASSED.\n" << std::endl;
}

} // namespace

int main()
{
    std::cout << "==================================================" << std::endl;
    std::cout << " CENTROIDAL WRENCH + GRF QP SMOKE TESTS" << std::endl;
    std::cout << "==================================================" << std::endl;

    Test1_SymmetricStanding();
    Test2_PitchMoment();
    Test3_CoMOffset();
    Test4_SwingForce();
    Test5_ForceRateRegularization();

    std::cout << "==================================================" << std::endl;
    std::cout << " ALL CENTROIDAL WRENCH TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "==================================================" << std::endl;
    return 0;
}
