#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "RobotModel.hpp"
#include "WbcType.h"

int main()
{
    std::cout << "[TEST] wbic_dynamics_smoke starting..." << std::endl;

    RobotModel model;
    RobotModelConfig cfg;
    std::string err;
    if (!model.build(cfg, &err)) {
        std::cerr << "RobotModel build failed: " << err << std::endl;
        return 1;
    }

    // Set a typical standing configuration
    Vec3 p_world(0.0, 0.0, 0.30);
    Eigen::Quaterniond q_wb(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()));
    Vec3 v_lin_world(0.2, -0.1, 0.0);
    Vec3 omega_world(0.05, -0.02, 0.1);

    Vec12 q_joint, dq_joint;
    // Typical quadruped standing angles for Lite3
    for (int leg = 0; leg < 4; ++leg) {
        q_joint[leg * 3 + 0] = (leg % 2 == 0) ? -0.05 : 0.05; // HipX
        q_joint[leg * 3 + 1] = -0.8;                           // HipY
        q_joint[leg * 3 + 2] = 1.6;                            // Knee
        dq_joint[leg * 3 + 0] = 0.1 * (leg + 1);
        dq_joint[leg * 3 + 1] = -0.2 * (leg + 1);
        dq_joint[leg * 3 + 2] = 0.3 * (leg + 1);
    }

    model.setState(p_world, q_wb, v_lin_world, omega_world, q_joint, dq_joint);
    model.update();

    const auto& M = model.M();
    const auto& h = model.h();
    const auto& Jc = model.Jc();
    const auto& dJv = model.dJv();
    const auto& Jb = model.Jb();
    const auto& dJvb = model.dJvb();

    // 1. Check dimensions and finite
    if (M.rows() != 18 || M.cols() != 18 || h.size() != 18) {
        std::cerr << "Invalid M or h dimensions" << std::endl;
        return 2;
    }
    if (!M.allFinite() || !h.allFinite() || !Jc.allFinite() || !dJv.allFinite() || !Jb.allFinite()) {
        std::cerr << "Non-finite dynamics matrices" << std::endl;
        return 3;
    }

    // 2. Check Mass matrix symmetry: M == M^T
    const double sym_err = (M - M.transpose()).lpNorm<Eigen::Infinity>();
    if (sym_err > 1e-8) {
        std::cerr << "Mass matrix not symmetric, error: " << sym_err << std::endl;
        return 4;
    }

    // 3. Check Mass matrix positive-definiteness via LLT (Cholesky)
    Eigen::LLT<Eigen::MatrixXd> llt(M);
    if (llt.info() != Eigen::Success) {
        std::cerr << "Mass matrix is not positive definite (LLT failed)" << std::endl;
        return 5;
    }

    // 4. Check Inverse consistency: M * M_inv approx I
    const Eigen::MatrixXd M_inv = llt.solve(Eigen::MatrixXd::Identity(18, 18));
    const double inv_err = (M * M_inv - Eigen::MatrixXd::Identity(18, 18)).lpNorm<Eigen::Infinity>();
    if (inv_err > 1e-6) {
        std::cerr << "M * M_inv != I, error: " << inv_err << std::endl;
        return 6;
    }

    // 5. Check Foot Jacobians
    if (Jc.rows() != 12 || Jc.cols() != 18) {
        std::cerr << "Invalid Jc dimensions" << std::endl;
        return 7;
    }
    for (int leg = 0; leg < 4; ++leg) {
        const auto J_leg = Jc.middleRows<3>(3 * leg);
        if (J_leg.norm() < 1e-3) {
            std::cerr << "Foot Jacobian norm too small for leg " << leg << std::endl;
            return 8;
        }
    }

    // 6. Check Base Jacobian (top 3 lin, bottom 3 ang)
    if (Jb.rows() != 6 || Jb.cols() != 18) {
        std::cerr << "Invalid Jb dimensions" << std::endl;
        return 9;
    }
    // Base translational and rotational Jacobians should have rank 3 each
    if (Jb.topRows<3>().leftCols<6>().norm() < 1e-3 || Jb.bottomRows<3>().leftCols<6>().norm() < 1e-3) {
        std::cerr << "Base Jacobian null in base DoFs" << std::endl;
        return 10;
    }

    std::cout << "[TEST] wbic_dynamics_smoke PASSED!" << std::endl;
    return 0;
}
