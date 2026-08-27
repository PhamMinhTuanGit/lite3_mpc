#pragma once

#include <array>
#include <memory>
#include <Eigen/Dense>

#include "CentroidalModel.hpp"
#include "WbcType.h"

namespace wbic {

/** Configuration parameters for Centroidal Wrench Controller and GRF QP. */
struct CentroidalWrenchConfig {
    // Translational gains
    Eigen::Vector3d kp_pos = Eigen::Vector3d(100.0, 100.0, 100.0);
    Eigen::Vector3d kd_pos = Eigen::Vector3d(20.0, 20.0, 20.0);

    // Rotational gains (SO(3) orientation error)
    Eigen::Vector3d kp_ori = Eigen::Vector3d(100.0, 100.0, 100.0);
    Eigen::Vector3d kd_ori = Eigen::Vector3d(10.0, 10.0, 10.0);

    // QP objective weights
    double w_force = 1.0;     // Weight for tracking F_des
    double w_moment = 100.0;  // Weight for tracking M_des
    double w_reg = 1e-4;      // Regularization weight ||f||^2
    double w_force_rate = 0.0;// Force-rate regularization weight lambda_df * ||f - f_prev||^2

    // Physical constraints
    double mu = 0.4;          // Friction coefficient
    double fz_min = 5.0;      // Minimum normal force per stance leg [N]
    double fz_max = 120.0;    // Maximum normal force per leg [N]
    double gravity = 9.81;    // Gravity magnitude [m/s^2]

    // QP solver settings
    int max_wsr = 100;
    double max_cpu_time = 0.005; // 5 ms
};

/** Input to the Centroidal Wrench Controller. */
struct CentroidalWrenchInput {
    double mass = 11.9376;
    Eigen::Vector3d com_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_des_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_vel_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_vel_des_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_acc_des_world = Eigen::Vector3d::Zero();

    Eigen::Matrix3d R_world_body = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_des_world_body = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_des_world = Eigen::Vector3d::Zero();

    std::array<Eigen::Vector3d, 4> foot_pos_world{};
    std::array<bool, 4> contact{{true, true, true, true}};
};

/** Output from the Centroidal Wrench Controller and GRF QP. */
struct CentroidalWrenchOutput {
    Eigen::Vector3d force_des_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d moment_des_world = Eigen::Vector3d::Zero();

    // Wrench controller breakdown telemetry
    double Fz_gravity = 0.0;
    double Fz_pos_P = 0.0;
    double Fz_vel_D = 0.0;
    double My_P = 0.0;
    double My_D = 0.0;
    double My_ff = 0.0;

    std::array<Eigen::Vector3d, 4> grf_world{};
    Eigen::Vector3d force_grf_world = Eigen::Vector3d::Zero();  // sum f_i
    Eigen::Vector3d moment_grf_world = Eigen::Vector3d::Zero(); // sum (p_i - p_com) x f_i

    // Force-rate tracking telemetry
    double df_norm = 0.0;
    std::array<double, 4> df_norm_leg{{0.0, 0.0, 0.0, 0.0}};

    double qp_cost = 0.0;
    double qp_solve_time_us = 0.0;
    int qp_status = 0; // 0 = OK, 1 = MaxIter, 2 = Infeasible/Error
    bool valid = false;

    void reset() noexcept
    {
        force_des_world.setZero();
        moment_des_world.setZero();
        Fz_gravity = 0.0;
        Fz_pos_P = 0.0;
        Fz_vel_D = 0.0;
        My_P = 0.0;
        My_D = 0.0;
        My_ff = 0.0;
        for (auto& f : grf_world) f.setZero();
        force_grf_world.setZero();
        moment_grf_world.setZero();
        df_norm = 0.0;
        df_norm_leg.fill(0.0);
        qp_cost = 0.0;
        qp_solve_time_us = 0.0;
        qp_status = 0;
        valid = false;
    }
};

/**
 * Centroidal Wrench Controller with qpOASES-based GRF Distribution.
 */
class CentroidalWrenchController {
public:
    explicit CentroidalWrenchController(const CentroidalWrenchConfig& config = CentroidalWrenchConfig());
    ~CentroidalWrenchController();

    CentroidalWrenchController(const CentroidalWrenchController&) = delete;
    CentroidalWrenchController& operator=(const CentroidalWrenchController&) = delete;
    CentroidalWrenchController(CentroidalWrenchController&&) noexcept;
    CentroidalWrenchController& operator=(CentroidalWrenchController&&) noexcept;

    void Reset() noexcept;

    /**
     * Computes desired centroidal wrench (F_des, M_des) and solves the GRF distribution QP.
     */
    bool Compute(const CentroidalWrenchInput& input, CentroidalWrenchOutput* output) noexcept;

    const CentroidalWrenchConfig& GetConfig() const noexcept;
    void SetConfig(const CentroidalWrenchConfig& config) noexcept;
    void SetForceRateWeight(double w) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace wbic
