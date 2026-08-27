#include "WbicQp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>

namespace wbic {

namespace {
constexpr double kBigNumber = 1e10;
}

WbicQp::WbicQp(const WbicConfig& config)
{
    solver_ = std::make_unique<qpOASES::SQProblem>(kNumVars, kMaxConstraints, qpOASES::HST_SEMIDEF);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    options.enableRegularisation = qpOASES::BT_TRUE;
    options.epsRegularisation = config.regularization;
    options.enableDropInfeasibles = qpOASES::BT_TRUE;
    options.enableInertiaCorrection = qpOASES::BT_TRUE;
    solver_->setOptions(options);
    is_initialized_ = false;
}

WbicQp::~WbicQp() = default;

void WbicQp::Reset() noexcept
{
    if (solver_) {
        solver_->reset();
    }
    is_initialized_ = false;
    prev_contact_ = {{false, false, false, false}};
}

bool WbicQp::Solve(const WbicInput& input,
                   const DynamicsOutput& dyn,
                   const ContactSet& contact_set,
                   const GeneralizedAcceleration& qddot_cmd,
                   const JointVector& q_des,
                   const JointVector& dq_des,
                   const std::array<int, kNumJoints>& idx_v,
                   const WbicConfig& config,
                   WbicQpResult* result) noexcept
{
    if (result == nullptr) return false;

    auto start_time = std::chrono::high_resolution_clock::now();

    result->status = WbicStatus::Ok;
    result->qddot.setZero();
    result->delta_qddot_u.setZero();
    result->f_opt.setZero();
    result->tau_ff.setZero();
    result->residuals = WbicResiduals{};
    result->wsr_performed = 0;
    result->cpu_time_ms = 0.0;

    // ══════════════════════════════════════════════════════════════════════
    // 1. Construct Hessian H (18x18) and Gradient g (18x1)
    // ══════════════════════════════════════════════════════════════════════
    H_mat_.setZero();
    g_vec_.setZero();

    // Base acceleration weights (indices 0..5)
    H_mat_.block<6, 6>(0, 0).diagonal().setConstant(config.w_acc + config.regularization);

    // Force weights (indices 6..17)
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const int f_idx = 6 + leg * 3;
        if (input.contact[static_cast<std::size_t>(leg)]) {
            H_mat_(f_idx + 0, f_idx + 0) = config.w_force[0] + config.regularization;
            H_mat_(f_idx + 1, f_idx + 1) = config.w_force[1] + config.regularization;
            H_mat_(f_idx + 2, f_idx + 2) = config.w_force[2] + config.regularization;

            const Eigen::Vector3d& f_des = input.Fr_des[static_cast<std::size_t>(leg)];
            g_vec_[f_idx + 0] = -config.w_force[0] * f_des[0];
            g_vec_[f_idx + 1] = -config.w_force[1] * f_des[1];
            g_vec_[f_idx + 2] = -config.w_force[2] * f_des[2];
        } else {
            H_mat_.block<3, 3>(f_idx, f_idx).diagonal().setConstant(1.0 + config.regularization);
            g_vec_.segment<3>(f_idx).setZero();
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // 2. Variable Bounds lb <= z <= ub
    // ══════════════════════════════════════════════════════════════════════
    for (int i = 0; i < 3; ++i) {
        lb_mem_[i] = -config.delta_qddot_lin_max;
        ub_mem_[i] = config.delta_qddot_lin_max;
    }
    for (int i = 3; i < 6; ++i) {
        lb_mem_[i] = -config.delta_qddot_ang_max;
        ub_mem_[i] = config.delta_qddot_ang_max;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const int f_idx = 6 + leg * 3;
        if (input.contact[static_cast<std::size_t>(leg)]) {
            lb_mem_[f_idx + 0] = -kBigNumber;
            ub_mem_[f_idx + 0] = kBigNumber;
            lb_mem_[f_idx + 1] = -kBigNumber;
            ub_mem_[f_idx + 1] = kBigNumber;
            lb_mem_[f_idx + 2] = config.fz_min;
            ub_mem_[f_idx + 2] = config.fz_max;
        } else {
            lb_mem_[f_idx + 0] = 0.0;
            ub_mem_[f_idx + 0] = 0.0;
            lb_mem_[f_idx + 1] = 0.0;
            ub_mem_[f_idx + 1] = 0.0;
            lb_mem_[f_idx + 2] = 0.0;
            ub_mem_[f_idx + 2] = 0.0;
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // 3. Constraints Matrix A and Bounds lbA <= A*z <= ubA
    // ══════════════════════════════════════════════════════════════════════
    A_mat_.setZero();
    lbA_vec_.setZero();
    ubA_vec_.setZero();

    int row = 0;

    // Constraint 1: Floating-base EoM (6 equality constraints)
    // M_u,u * delta_qddot_u - sum_i(J_f,u[i]^T * F_i) = -(M_u * qddot_cmd + h_u)
    const auto M_u = dyn.M.topRows<6>();
    const auto h_u = dyn.h.head<6>();
    const Eigen::Matrix<double, 6, 1> eom_bias = -(M_u * qddot_cmd + h_u);

    for (int r = 0; r < 6; ++r) {
        A_mat_.block<1, 6>(row, 0) = M_u.block<1, 6>(r, 0);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int f_idx = 6 + leg * 3;
            A_mat_(row, f_idx + 0) = -dyn.Jf[leg](0, r);
            A_mat_(row, f_idx + 1) = -dyn.Jf[leg](1, r);
            A_mat_(row, f_idx + 2) = -dyn.Jf[leg](2, r);
        }
        lbA_vec_[row] = eom_bias[r];
        ubA_vec_[row] = eom_bias[r];
        ++row;
    }

    // Constraint 2: Friction Pyramid (16 inequalities: 4 per leg, rows 6..21)
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const int f_idx = 6 + leg * 3;
        const int row_base = 6 + leg * 4;

        // 1. Fx - mu * Fz <= 0
        A_mat_(row_base + 0, f_idx + 0) = 1.0;
        A_mat_(row_base + 0, f_idx + 2) = -config.mu;

        // 2. -Fx - mu * Fz <= 0
        A_mat_(row_base + 1, f_idx + 0) = -1.0;
        A_mat_(row_base + 1, f_idx + 2) = -config.mu;

        // 3. Fy - mu * Fz <= 0
        A_mat_(row_base + 2, f_idx + 1) = 1.0;
        A_mat_(row_base + 2, f_idx + 2) = -config.mu;

        // 4. -Fy - mu * Fz <= 0
        A_mat_(row_base + 3, f_idx + 1) = -1.0;
        A_mat_(row_base + 3, f_idx + 2) = -config.mu;

        if (input.contact[static_cast<std::size_t>(leg)]) {
            lbA_vec_[row_base + 0] = -kBigNumber;
            ubA_vec_[row_base + 0] = 0.0;
            lbA_vec_[row_base + 1] = -kBigNumber;
            ubA_vec_[row_base + 1] = 0.0;
            lbA_vec_[row_base + 2] = -kBigNumber;
            ubA_vec_[row_base + 2] = 0.0;
            lbA_vec_[row_base + 3] = -kBigNumber;
            ubA_vec_[row_base + 3] = 0.0;
        } else {
            // For swing legs, contact forces are already constrained to 0 via simple bounds lb=ub=0.
            // Relaxing these rows to [-kBigNumber, kBigNumber] prevents degenerate zero-equality rows.
            lbA_vec_[row_base + 0] = -kBigNumber;
            ubA_vec_[row_base + 0] = kBigNumber;
            lbA_vec_[row_base + 1] = -kBigNumber;
            ubA_vec_[row_base + 1] = kBigNumber;
            lbA_vec_[row_base + 2] = -kBigNumber;
            ubA_vec_[row_base + 2] = kBigNumber;
            lbA_vec_[row_base + 3] = -kBigNumber;
            ubA_vec_[row_base + 3] = kBigNumber;
        }
    }
    row = 6 + kNumLegs * 4; // 22

    // Constraint 3: Total Actuated Torque Limits (12 two-sided inequalities, rows 22..33)
    // -tau_max <= tau_total <= tau_max
    const auto M_a = dyn.M.bottomRows<12>();
    const auto h_a = dyn.h.tail<12>();
    const Eigen::Matrix<double, 12, 1> tau_bias_pin = M_a * qddot_cmd + h_a;

    for (int k = 0; k < kNumJoints; ++k) {
        const int pin_joint_idx = idx_v[static_cast<std::size_t>(k)] - 6; // actuated joint index in Pinocchio [0..11]
        const int pin_v_idx = idx_v[static_cast<std::size_t>(k)];

        // Compute PD torque for joint k
        const int joint_in_leg = k % 3;
        const double kp = config.kp_joint[joint_in_leg];
        const double kd = config.kd_joint[joint_in_leg];
        const double tau_pd = kp * (q_des[k] - input.q_joint_raw[k]) +
                              kd * (dq_des[k] - input.qd_joint_raw[k]);
        const double bias_k = tau_bias_pin[pin_joint_idx] + tau_pd;
        const double max_tau = config.tau_max[joint_in_leg];

        A_mat_.block<1, 6>(row, 0) = M_a.block<1, 6>(pin_joint_idx, 0);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const int f_idx = 6 + leg * 3;
            A_mat_(row, f_idx + 0) = -dyn.Jf[leg](0, pin_v_idx);
            A_mat_(row, f_idx + 1) = -dyn.Jf[leg](1, pin_v_idx);
            A_mat_(row, f_idx + 2) = -dyn.Jf[leg](2, pin_v_idx);
        }

        lbA_vec_[row] = -max_tau - bias_k;
        ubA_vec_[row] = max_tau - bias_k;
        ++row;
    }

    // ══════════════════════════════════════════════════════════════════════
    // 4. Copy to Row-Major Memory Buffers
    // ══════════════════════════════════════════════════════════════════════
    std::memcpy(H_mem_, H_mat_.data(), kNumVars * kNumVars * sizeof(qpOASES::real_t));
    std::memcpy(g_mem_, g_vec_.data(), kNumVars * sizeof(qpOASES::real_t));
    std::memcpy(A_mem_, A_mat_.data(), kMaxConstraints * kNumVars * sizeof(qpOASES::real_t));
    std::memcpy(lbA_mem_, lbA_vec_.data(), kMaxConstraints * sizeof(qpOASES::real_t));
    std::memcpy(ubA_mem_, ubA_vec_.data(), kMaxConstraints * sizeof(qpOASES::real_t));

    // ══════════════════════════════════════════════════════════════════════
    // 5. Solve QP using qpOASES SQProblem
    // ══════════════════════════════════════════════════════════════════════
    int nWSR = config.max_wsr;
    qpOASES::real_t cputime = config.max_cpu_time;
    qpOASES::returnValue status_qp = qpOASES::TERMINAL_LIST_ELEMENT;

    bool contact_changed = false;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        if (prev_contact_[leg] != input.contact[leg]) {
            contact_changed = true;
            break;
        }
    }

    if (contact_changed) {
        is_initialized_ = false;
        solver_->reset();
        prev_contact_ = input.contact;
    }

    if (is_initialized_) {
        status_qp = solver_->hotstart(H_mem_, g_mem_, A_mem_, lb_mem_, ub_mem_,
                                     lbA_mem_, ubA_mem_, nWSR, &cputime);
    }

    if (!is_initialized_ || status_qp != qpOASES::SUCCESSFUL_RETURN) {
        solver_->reset();
        nWSR = config.max_wsr;
        cputime = config.max_cpu_time;
        status_qp = solver_->init(H_mem_, g_mem_, A_mem_, lb_mem_, ub_mem_,
                                  lbA_mem_, ubA_mem_, nWSR, &cputime);
    }

    if (status_qp == qpOASES::SUCCESSFUL_RETURN) {
        is_initialized_ = true;
        solver_->getPrimalSolution(z_opt_);
        result->status = WbicStatus::Ok;
    } else if (status_qp == qpOASES::RET_MAX_NWSR_REACHED) {
        solver_->getPrimalSolution(z_opt_);
        result->status = WbicStatus::QpMaxIter;
    } else if (status_qp == qpOASES::RET_INIT_FAILED_INFEASIBILITY ||
               status_qp == qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY) {
        result->status = WbicStatus::QpInfeasible;
        is_initialized_ = false;
        return false;
    } else {
        result->status = WbicStatus::QpSolverError;
        is_initialized_ = false;
        return false;
    }

    result->wsr_performed = nWSR;

    // ══════════════════════════════════════════════════════════════════════
    // 6. Extract Solution & Reconstruct Accelerations, Forces and Torques
    // ══════════════════════════════════════════════════════════════════════
    for (int i = 0; i < 6; ++i) {
        result->delta_qddot_u[i] = z_opt_[i];
    }
    for (int i = 0; i < 12; ++i) {
        result->f_opt[i] = z_opt_[6 + i];
    }

    result->qddot = qddot_cmd;
    result->qddot.head<6>() += result->delta_qddot_u;

    // Check finite values
    if (!result->qddot.allFinite() || !result->f_opt.allFinite()) {
        result->status = WbicStatus::InvalidInput;
        is_initialized_ = false;
        return false;
    }

    // Reconstruct Pinocchio actuated torque: tau_pin = M_a * qddot + h_a - J_a^T * F
    Eigen::Matrix<double, 12, 1> J_a_T_F = Eigen::Matrix<double, 12, 1>::Zero();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Eigen::Vector3d f_leg = result->f_opt.segment<3>(leg * 3);
        J_a_T_F += dyn.Jf[leg].rightCols<12>().transpose() * f_leg;
    }
    const Eigen::Matrix<double, 12, 1> tau_pin = M_a * result->qddot + h_a - J_a_T_F;

    // Reorder from Pinocchio to hardware order [FR, FL, HR, HL]
    for (int k = 0; k < kNumJoints; ++k) {
        const int pin_idx = idx_v[static_cast<std::size_t>(k)] - 6;
        result->tau_ff[k] = tau_pin[pin_idx];
    }

    if (!result->tau_ff.allFinite()) {
        result->status = WbicStatus::InvalidInput;
        is_initialized_ = false;
        return false;
    }

    // ══════════════════════════════════════════════════════════════════════
    // 7. Calculate Residuals & Check Thresholds
    // ══════════════════════════════════════════════════════════════════════
    // EoM residual: M_u * qddot + h_u - J_u^T * F
    Eigen::Matrix<double, 6, 1> J_u_T_F = Eigen::Matrix<double, 6, 1>::Zero();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Eigen::Vector3d f_leg = result->f_opt.segment<3>(leg * 3);
        J_u_T_F += dyn.Jf[leg].leftCols<6>().transpose() * f_leg;
    }
    const Eigen::Matrix<double, 6, 1> eom_res = M_u * result->qddot + h_u - J_u_T_F;
    result->residuals.eom_residual_norm = eom_res.lpNorm<Eigen::Infinity>();

    // Contact acceleration residual (KinWBC kinematic contact constraint satisfaction)
    if (contact_set.nc > 0) {
        const int dc = contact_set.nc * 3;
        const auto Jc = contact_set.Jc.topRows(dc);
        const auto dJc_qdot = contact_set.dJdq_c.head(dc);
        const Eigen::VectorXd contact_acc = Jc * result->qddot + dJc_qdot;
        result->residuals.contact_acc_residual_norm = contact_acc.lpNorm<Eigen::Infinity>();
    } else {
        result->residuals.contact_acc_residual_norm = 0.0;
    }

    // Inequality violation residual
    double ineq_max = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (input.contact[static_cast<std::size_t>(leg)]) {
            const int f_idx = leg * 3;
            const double fx = result->f_opt[f_idx + 0];
            const double fy = result->f_opt[f_idx + 1];
            const double fz = result->f_opt[f_idx + 2];
            ineq_max = std::max(ineq_max, std::abs(fx) - config.mu * fz);
            ineq_max = std::max(ineq_max, std::abs(fy) - config.mu * fz);
            if (fz < config.fz_min) ineq_max = std::max(ineq_max, config.fz_min - fz);
            if (fz > config.fz_max) ineq_max = std::max(ineq_max, fz - config.fz_max);
        }
    }
    result->residuals.inequality_violation_norm = ineq_max;

    // Torque limits margin
    double min_margin = kBigNumber;
    for (int k = 0; k < kNumJoints; ++k) {
        const int joint_in_leg = k % 3;
        const double max_tau = config.tau_max[joint_in_leg];
        const double tau_total = result->tau_ff[k] +
                                 config.kp_joint[joint_in_leg] * (q_des[k] - input.q_joint_raw[k]) +
                                 config.kd_joint[joint_in_leg] * (dq_des[k] - input.qd_joint_raw[k]);
        min_margin = std::min(min_margin, max_tau - std::abs(tau_total));
    }
    result->residuals.torque_limit_margin = min_margin;

    auto end_time = std::chrono::high_resolution_clock::now();
    result->cpu_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Final contact acceleration is diagnostic only: the reduced QP changes the
    // floating-base acceleration without enforcing the KinWBC contact task.
    // Verify thresholds
    if (result->residuals.eom_residual_norm > config.eom_residual_threshold ||
        result->residuals.inequality_violation_norm > config.inequality_residual_threshold) {
        result->status = WbicStatus::ResidualExceeded;
    }

    return (result->status == WbicStatus::Ok || result->status == WbicStatus::QpMaxIter);
}

}  // namespace wbic
