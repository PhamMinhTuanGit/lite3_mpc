#include "KinWbc.hpp"

#include <algorithm>
#include <cmath>

namespace wbic {

KinWbc::KinWbc(double damping)
    : damping_(damping)
{
    // Default joint limits for Lite3
    // Order [FR, FL, HR, HL], [HipX, HipY, Knee]
    JointVector q_min, q_max, qd_max;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        // HipX (Abad)
        q_min[leg * 3 + 0] = -0.523;
        q_max[leg * 3 + 0] = 0.523;
        qd_max[leg * 3 + 0] = 26.2;

        // HipY (Thigh)
        q_min[leg * 3 + 1] = -2.67;
        q_max[leg * 3 + 1] = 0.314;
        qd_max[leg * 3 + 1] = 26.2;

        // Knee (Shank)
        q_min[leg * 3 + 2] = 0.524;
        q_max[leg * 3 + 2] = 2.792;
        qd_max[leg * 3 + 2] = 17.3;
    }
    SetJointLimits(q_min, q_max, qd_max);
}

void KinWbc::SetJointLimits(const JointVector& q_min,
                            const JointVector& q_max,
                            const JointVector& qd_max) noexcept
{
    q_min_ = q_min;
    q_max_ = q_max;
    qd_max_ = qd_max;
}

Eigen::Vector3d KinWbc::RotationErrorSO3(const Eigen::Matrix3d& R_des,
                                        const Eigen::Matrix3d& R_cur) noexcept
{
    const Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    const Eigen::AngleAxisd aa(R_err);
    double angle = aa.angle();

    if (!std::isfinite(angle) || std::abs(angle) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }

    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;

    const Eigen::Vector3d axis = aa.axis();
    if (!axis.allFinite()) {
        return Eigen::Vector3d::Zero();
    }

    return angle * axis;
}

bool KinWbc::Compute(const WbicInput& input,
                     const DynamicsOutput& dyn,
                     const ContactSet& contact_set,
                     const WbicConfig& config,
                     KinWbcResult* result) noexcept
{
    if (result == nullptr) return false;

    result->task_stack.reset();
    result->qddot_cmd.setZero();
    result->delta_q.setZero();
    result->q_des.setZero();
    result->dq_des.setZero();
    result->contact_acc_residual_norm = 0.0;
    result->success = true;

    // Reset internal buffers
    N_cur_.setIdentity();
    N_kin_.setIdentity();
    qddot_cur_.setZero();
    delta_v_cur_.setZero();
    delta_q_cur_.setZero();

    const double lambda_sq = std::max(damping_ * damping_, 1e-12);
    const double lambda_kin_sq = 1e-6;
    const auto& A_inv = dyn.M_inv;

    // ══════════════════════════════════════════════════════════════════════
    // Priority 1: Contact Constraint Task
    // J_c * qddot = - dJ_c/dt * qdot
    // ══════════════════════════════════════════════════════════════════════
    if (contact_set.nc > 0) {
        const int dc = contact_set.nc * 3;
        const auto Jc = contact_set.Jc.topRows(dc);
        const auto dJc_qdot = contact_set.dJdq_c.head(dc);

        KinTask contact_task;
        contact_task.type = KinTask::Type::Contact;
        contact_task.dim = dc;
        contact_task.J.topRows(dc) = Jc;
        contact_task.dJdq.head(dc) = dJc_qdot;
        contact_task.x_ddot_des.head(dc).setZero();
        contact_task.err.head(dc).setZero();
        contact_task.derr.head(dc).setZero();
        result->task_stack.push(contact_task);

        // Dynamically consistent inverse: J_bar = A_inv * Jc^T * (Jc * A_inv * Jc^T + lambda^2 * I)^(-1)
        Eigen::MatrixXd inner = Jc * A_inv * Jc.transpose();
        inner.diagonal().array() += lambda_sq;
        const Eigen::MatrixXd inner_inv = inner.ldlt().solve(Eigen::MatrixXd::Identity(dc, dc));
        const Eigen::MatrixXd Jc_bar = A_inv * Jc.transpose() * inner_inv;

        qddot_cur_ = Jc_bar * (-dJc_qdot);
        N_cur_ = Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity() - Jc_bar * Jc;

        // Kinematics nullspace
        Eigen::MatrixXd inner_kin = Jc * Jc.transpose();
        inner_kin.diagonal().array() += lambda_kin_sq;
        const Eigen::MatrixXd inner_kin_inv = inner_kin.ldlt().solve(Eigen::MatrixXd::Identity(dc, dc));
        const Eigen::MatrixXd Jc_pinv = Jc.transpose() * inner_kin_inv;
        N_kin_ = Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity() - Jc_pinv * Jc;
        delta_v_cur_.setZero();
        delta_q_cur_.setZero();

        const Eigen::VectorXd contact_acc = Jc * qddot_cur_ + dJc_qdot;
        result->contact_acc_residual_norm = contact_acc.lpNorm<Eigen::Infinity>();
    }

    // Helper lambda for hierarchical projection of a task
    auto addTask = [&](KinTask& task, const Eigen::Matrix<double, Eigen::Dynamic, kVelocityDimension>& J_task,
                       const Eigen::VectorXd& dJ_task_qdot, const Eigen::VectorXd& x_ddot_des,
                       const Eigen::VectorXd& err, const Eigen::VectorXd& derr) {
        const int d = task.dim;
        task.J.topRows(d) = J_task;
        task.dJdq.head(d) = dJ_task_qdot;
        task.x_ddot_des.head(d) = x_ddot_des;
        task.err.head(d) = err;
        task.derr.head(d) = derr;
        result->task_stack.push(task);

        // Projected Jacobian: J_tilde = J_task * N_cur
        const Eigen::MatrixXd J_tilde = J_task * N_cur_;

        // Dynamically consistent inverse
        Eigen::MatrixXd inner = J_tilde * A_inv * J_tilde.transpose();
        inner.diagonal().array() += lambda_sq;
        const Eigen::MatrixXd inner_inv = inner.ldlt().solve(Eigen::MatrixXd::Identity(d, d));
        const Eigen::MatrixXd J_bar = A_inv * J_tilde.transpose() * inner_inv;

        const Eigen::VectorXd dddot_err = x_ddot_des - dJ_task_qdot - J_task * qddot_cur_;
        qddot_cur_ += J_bar * dddot_err;
        N_cur_ = N_cur_ * (Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity() - J_bar * J_tilde);

        // Kinematics updates
        const Eigen::MatrixXd J_tilde_kin = J_task * N_kin_;
        Eigen::MatrixXd inner_kin = J_tilde_kin * J_tilde_kin.transpose();
        inner_kin.diagonal().array() += lambda_kin_sq;
        const Eigen::MatrixXd inner_kin_inv = inner_kin.ldlt().solve(Eigen::MatrixXd::Identity(d, d));
        const Eigen::MatrixXd J_pinv = J_tilde_kin.transpose() * inner_kin_inv;

        delta_v_cur_ += J_pinv * (derr - J_task * delta_v_cur_);
        delta_q_cur_ += J_pinv * (err - J_task * delta_q_cur_);
        N_kin_ = N_kin_ * (Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity() - J_pinv * J_tilde_kin);
    };

    // ══════════════════════════════════════════════════════════════════════
    // Priority 2: Body Orientation Task
    // ══════════════════════════════════════════════════════════════════════
    {
        KinTask ori_task;
        ori_task.type = KinTask::Type::BodyOrientation;
        ori_task.dim = 3;

        const Eigen::Vector3d e_ori = RotationErrorSO3(input.R_body_des, dyn.R_wb);
        const Eigen::Vector3d de_ori = input.omega_body_des - dyn.omega_b_world;
        const Eigen::Vector3d x_ddot_ori = input.domega_body_des + config.kp_body_ori * e_ori + config.kd_body_ori * de_ori;

        const Eigen::Matrix<double, 3, kVelocityDimension> J_ori = dyn.Jb.bottomRows<3>();
        const Eigen::Vector3d dJ_ori_qdot = dyn.dJdq_b.tail<3>();

        addTask(ori_task, J_ori, dJ_ori_qdot, x_ddot_ori, e_ori, de_ori);
    }

    // ══════════════════════════════════════════════════════════════════════
    // Priority 3: Body Position Task
    // ══════════════════════════════════════════════════════════════════════
    {
        KinTask pos_task;
        pos_task.type = KinTask::Type::BodyPosition;
        pos_task.dim = 3;

        const Eigen::Vector3d e_pos = input.p_body_des - dyn.p_b;
        const Eigen::Vector3d de_pos = input.v_body_des - dyn.v_b_world;
        const Eigen::Vector3d x_ddot_pos = input.a_body_des + config.kp_body_pos * e_pos + config.kd_body_pos * de_pos;

        const Eigen::Matrix<double, 3, kVelocityDimension> J_pos = dyn.Jb.topRows<3>();
        const Eigen::Vector3d dJ_pos_qdot = dyn.dJdq_b.head<3>();

        addTask(pos_task, J_pos, dJ_pos_qdot, x_ddot_pos, e_pos, de_pos);
    }

    // ══════════════════════════════════════════════════════════════════════
    // Priority 4..n: Swing Foot Tasks (in order [FR, FL, HR, HL])
    // ══════════════════════════════════════════════════════════════════════
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!input.contact[static_cast<std::size_t>(leg)]) {
            KinTask foot_task;
            foot_task.type = KinTask::Type::SwingFoot;
            foot_task.dim = 3;
            foot_task.leg_idx = leg;

            const Eigen::Vector3d e_foot = input.p_foot_des[leg] - dyn.pf[leg];
            const Eigen::Vector3d de_foot = input.v_foot_des[leg] - dyn.vf[leg];
            const Eigen::Vector3d x_ddot_foot =
                input.a_foot_des[leg] + config.kp_foot * e_foot + config.kd_foot * de_foot;

            const Eigen::Matrix<double, 3, kVelocityDimension>& J_foot = dyn.Jf[leg];
            const Eigen::Vector3d& dJ_foot_qdot = dyn.dJdq_f[leg];

            addTask(foot_task, J_foot, dJ_foot_qdot, x_ddot_foot, e_foot, de_foot);
        }
    }

    result->qddot_cmd = qddot_cur_;

    // Extract actuated joints: generalized coordinates 6..17 map to actuated joints
    const JointVector delta_q_act = delta_q_cur_.tail<kNumJoints>();
    const JointVector delta_v_act = delta_v_cur_.tail<kNumJoints>();

    result->delta_q = delta_q_act;
    result->dq_des = input.qd_joint_raw + delta_v_act;
    result->q_des = input.q_joint_raw + delta_q_act;

    // Joint limit clamping
    for (int i = 0; i < kNumJoints; ++i) {
        result->q_des[i] = std::clamp(result->q_des[i], q_min_[i], q_max_[i]);
        result->dq_des[i] = std::clamp(result->dq_des[i], -qd_max_[i], qd_max_[i]);
    }

    return result->success;
}

}  // namespace wbic
