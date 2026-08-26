#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pinocchio/multibody/model.hpp>

/**
 * Common types and data structures for the Lite3 WBIC pipeline.
 *
 * All public leg-indexed data uses the controller order [FR, FL, HR, HL].
 * All joint-indexed data per leg uses [HipX, HipY, Knee].
 * Quantities consumed by Pinocchio/WBIC use double precision and fixed-size
 * storage so constructing these structures does not allocate on the heap.
 */
namespace wbic {

inline constexpr int kNumLegs = 4;
inline constexpr int kJointsPerLeg = 3;
inline constexpr int kNumJoints = 12;
inline constexpr int kContactDimension = 3;
inline constexpr int kConfigurationDimension = 19;
inline constexpr int kVelocityDimension = 18;
inline constexpr int kMaxTasks = 6;

enum class Leg : std::size_t {
    FR = 0,
    FL = 1,
    HR = 2,
    HL = 3,
};

using Configuration = Eigen::Matrix<double, kConfigurationDimension, 1>;
using GeneralizedVelocity = Eigen::Matrix<double, kVelocityDimension, 1>;
using GeneralizedAcceleration = Eigen::Matrix<double, kVelocityDimension, 1>;
using JointVector = Eigen::Matrix<double, kNumJoints, 1>;
using JointMatrix = Eigen::Matrix<double, kNumJoints, kNumJoints>;
using FootVector = Eigen::Vector3d;
using ContactPhase = Eigen::Matrix<double, kNumLegs, 1>;
using FootJacobian = Eigen::Matrix<double, kContactDimension, kVelocityDimension>;
using BaseJacobian = Eigen::Matrix<double, 6, kVelocityDimension>;
using ContactFlags = std::array<bool, kNumLegs>;
using FootFrameIds = std::array<pinocchio::FrameIndex, kNumLegs>;
using FootVectorArray = std::array<FootVector, kNumLegs>;
using FootJacobianArray = std::array<FootJacobian, kNumLegs>;

enum class InputStatus {
    Ok = 0,
    MissingModel,
    InvalidModelDimension,
    NonFiniteValue,
    InvalidBaseQuaternion,
    InvalidFootFrame,
    DuplicateFootFrame,
    InvalidContactPhase,
};

enum class CommandSource : uint8_t {
    Wbic = 0,
    FallbackLegacyCmpc = 1,
    SafeZero = 2,
};

enum class WbicStatus : uint8_t {
    Ok = 0,
    InvalidInput,
    DynamicsError,
    KinWbcError,
    QpInfeasible,
    QpMaxIter,
    QpSolverError,
    ResidualExceeded,
    TorqueLimitViolated,
    SafetyViolation,
};

inline Configuration neutralConfiguration()
{
    Configuration q = Configuration::Zero();
    // Pinocchio free-flyer convention: [x, y, z, qx, qy, qz, qw].
    q[6] = 1.0;
    return q;
}

/** Inputs of the Dynamics module described in document/IPO.md. */
struct DynamicsInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Configuration q = neutralConfiguration();
    GeneralizedVelocity v = GeneralizedVelocity::Zero();

    // Non-owning: the model must outlive this input object.
    const pinocchio::Model* model = nullptr;
    FootFrameIds foot_ids{{
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
    }};

    DynamicsInput() = default;

    DynamicsInput(const pinocchio::Model& model_in,
                  const Configuration& q_in,
                  const GeneralizedVelocity& v_in,
                  const FootFrameIds& foot_ids_in) noexcept
        : q(q_in), v(v_in), model(&model_in), foot_ids(foot_ids_in)
    {
    }

    InputStatus validate(double quaternion_tolerance = 1e-6) const noexcept
    {
        if (model == nullptr) {
            return InputStatus::MissingModel;
        }
        if (model->nq != kConfigurationDimension
            || model->nv != kVelocityDimension) {
            return InputStatus::InvalidModelDimension;
        }
        if (!q.allFinite() || !v.allFinite()) {
            return InputStatus::NonFiniteValue;
        }

        const double quaternion_norm = q.template segment<4>(3).norm();
        if (!std::isfinite(quaternion_norm)
            || std::abs(quaternion_norm - 1.0) > quaternion_tolerance) {
            return InputStatus::InvalidBaseQuaternion;
        }

        for (std::size_t leg = 0; leg < foot_ids.size(); ++leg) {
            if (foot_ids[leg] >= model->nframes) {
                return InputStatus::InvalidFootFrame;
            }
            for (std::size_t previous = 0; previous < leg; ++previous) {
                if (foot_ids[leg] == foot_ids[previous]) {
                    return InputStatus::DuplicateFootFrame;
                }
            }
        }
        return InputStatus::Ok;
    }
};

/** Outputs of the Dynamics module. */
struct DynamicsOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, kVelocityDimension, kVelocityDimension> M =
        Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Zero();
    Eigen::Matrix<double, kVelocityDimension, kVelocityDimension> M_inv =
        Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Zero();
    GeneralizedVelocity h = GeneralizedVelocity::Zero();

    FootJacobianArray Jf{};
    FootVectorArray dJdq_f{};
    FootVectorArray pf{};
    FootVectorArray vf{};

    BaseJacobian Jb = BaseJacobian::Zero();
    Eigen::Matrix<double, 6, 1> dJdq_b = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix3d R_wb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_b_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_b_world = Eigen::Vector3d::Zero();

    DynamicsOutput()
    {
        for (auto& J : Jf) J.setZero();
        for (auto& dJ : dJdq_f) dJ.setZero();
        for (auto& p : pf) p.setZero();
        for (auto& v : vf) v.setZero();
    }
};

/** Inputs of the Contact Assembly module. */
struct ContactAssemblyInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactFlags contact{{false, false, false, false}};
    FootVectorArray f_mpc{};
    ContactPhase phase = ContactPhase::Zero();
    FootJacobianArray Jf{};
    FootVectorArray dJdq_f{};

    ContactAssemblyInput()
    {
        for (auto& force : f_mpc) force.setZero();
        for (auto& jacobian : Jf) jacobian.setZero();
        for (auto& bias_acc : dJdq_f) bias_acc.setZero();
    }

    InputStatus validate(double phase_tolerance = 1e-9) const noexcept
    {
        if (!phase.allFinite()) {
            return InputStatus::NonFiniteValue;
        }
        if ((phase.array() < -phase_tolerance).any()
            || (phase.array() > 1.0 + phase_tolerance).any()) {
            return InputStatus::InvalidContactPhase;
        }

        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (!f_mpc[leg].allFinite() || !Jf[leg].allFinite()
                || !dJdq_f[leg].allFinite()) {
                return InputStatus::NonFiniteValue;
            }
        }
        return InputStatus::Ok;
    }
};

/** Contact set assembly output. */
struct ContactSet {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::array<int, kNumLegs> cid{{-1, -1, -1, -1}};
    int nc = 0;

    Eigen::Matrix<double, 12, kVelocityDimension> Jc =
        Eigen::Matrix<double, 12, kVelocityDimension>::Zero();
    Eigen::Matrix<double, 12, 1> dJdq_c = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> f_mpc_compact = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> f_mpc_full = Eigen::Matrix<double, 12, 1>::Zero();

    void reset() noexcept
    {
        cid = {{-1, -1, -1, -1}};
        nc = 0;
        Jc.setZero();
        dJdq_c.setZero();
        f_mpc_compact.setZero();
        f_mpc_full.setZero();
    }

    void assemble(const ContactFlags& contact,
                  const FootVectorArray& f_mpc,
                  const FootJacobianArray& Jf,
                  const FootVectorArray& dJdq_f) noexcept
    {
        reset();
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (contact[static_cast<std::size_t>(leg)]) {
                cid[static_cast<std::size_t>(nc)] = leg;
                Jc.middleRows<3>(3 * nc) = Jf[static_cast<std::size_t>(leg)];
                dJdq_c.segment<3>(3 * nc) = dJdq_f[static_cast<std::size_t>(leg)];
                f_mpc_compact.segment<3>(3 * nc) = f_mpc[static_cast<std::size_t>(leg)];
                f_mpc_full.segment<3>(3 * leg) = f_mpc[static_cast<std::size_t>(leg)];
                ++nc;
            } else {
                f_mpc_full.segment<3>(3 * leg).setZero();
            }
        }
    }
};

/** Kinematic Task representation. */
struct KinTask {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class Type {
        Contact = 0,
        BodyOrientation,
        BodyPosition,
        SwingFoot,
        Generic,
    };

    Type type = Type::Generic;
    int dim = 0;
    int leg_idx = -1;

    Eigen::Matrix<double, 6, kVelocityDimension> J =
        Eigen::Matrix<double, 6, kVelocityDimension>::Zero();
    Eigen::Matrix<double, 6, 1> dJdq = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> x_ddot_des = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> err = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> derr = Eigen::Matrix<double, 6, 1>::Zero();

    void reset() noexcept
    {
        type = Type::Generic;
        dim = 0;
        leg_idx = -1;
        J.setZero();
        dJdq.setZero();
        x_ddot_des.setZero();
        err.setZero();
        derr.setZero();
    }
};

/** Fixed-size Task Stack (up to 6 tasks). */
struct TaskStack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::array<KinTask, kMaxTasks> tasks{};
    int task_count = 0;

    void reset() noexcept
    {
        for (auto& task : tasks) task.reset();
        task_count = 0;
    }

    bool push(const KinTask& task) noexcept
    {
        if (task_count >= kMaxTasks) return false;
        tasks[static_cast<std::size_t>(task_count++)] = task;
        return true;
    }
};

/** Full input snapshot for WBIC. */
struct WbicInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Base state (World frame)
    Eigen::Vector3d p_body = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_body = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_body_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_body_world = Eigen::Vector3d::Zero();

    // Actuated joints (order [FR, FL, HR, HL], [HipX, HipY, Knee])
    JointVector q_joint_raw = JointVector::Zero();
    JointVector qd_joint_raw = JointVector::Zero();

    // Desired base trajectory (World frame)
    Eigen::Vector3d p_body_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_body_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_body_des = Eigen::Vector3d::Zero();

    Eigen::Matrix3d R_body_des = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_body_des = Eigen::Vector3d::Zero();
    Eigen::Vector3d domega_body_des = Eigen::Vector3d::Zero();

    // Desired foot trajectories (World frame)
    FootVectorArray p_foot_des{};
    FootVectorArray v_foot_des{};
    FootVectorArray a_foot_des{};

    // Ground reaction forces from MPC (World frame)
    FootVectorArray Fr_des{};

    // Stance/Swing flags
    ContactFlags contact{{false, false, false, false}};
    ContactPhase phase = ContactPhase::Zero();

    // Metadata
    double timestamp = 0.0;
    uint64_t sequence = 0;

    WbicInput()
    {
        for (auto& p : p_foot_des) p.setZero();
        for (auto& v : v_foot_des) v.setZero();
        for (auto& a : a_foot_des) a.setZero();
        for (auto& f : Fr_des) f.setZero();
    }

    bool isValid() const noexcept
    {
        if (!p_body.allFinite() || !R_body.allFinite()
            || !v_body_world.allFinite() || !omega_body_world.allFinite()) {
            return false;
        }
        if (!q_joint_raw.allFinite() || !qd_joint_raw.allFinite()) {
            return false;
        }
        if (!p_body_des.allFinite() || !v_body_des.allFinite()
            || !a_body_des.allFinite() || !R_body_des.allFinite()
            || !omega_body_des.allFinite() || !domega_body_des.allFinite()) {
            return false;
        }
        for (std::size_t i = 0; i < kNumLegs; ++i) {
            if (!p_foot_des[i].allFinite() || !v_foot_des[i].allFinite()
                || !a_foot_des[i].allFinite() || !Fr_des[i].allFinite()) {
                return false;
            }
        }
        return true;
    }
};

/** Residual and diagnostic telemetry. */
struct WbicResiduals {
    double eom_residual_norm = 0.0;
    double contact_acc_residual_norm = 0.0;
    double inequality_violation_norm = 0.0;
    double torque_limit_margin = 0.0;
};

/** Full output generated by WBIC. */
struct WbicOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GeneralizedAcceleration qddot = GeneralizedAcceleration::Zero();
    JointVector delta_q = JointVector::Zero();
    JointVector q_des = JointVector::Zero();
    JointVector dq_des = JointVector::Zero();

    // Full 12x1 ground reaction forces [FR, FL, HR, HL] (World frame)
    Eigen::Matrix<double, 12, 1> f_opt = Eigen::Matrix<double, 12, 1>::Zero();

    // Joint feedforward torque [FR, FL, HR, HL]
    JointVector tau_ff = JointVector::Zero();

    // Hybrid PD gains
    JointVector joint_kp = JointVector::Zero();
    JointVector joint_kd = JointVector::Zero();

    // Status and telemetry
    WbicStatus status = WbicStatus::Ok;
    WbicResiduals residuals{};
    double solve_time_us = 0.0;
    CommandSource command_source = CommandSource::Wbic;

    void reset() noexcept
    {
        qddot.setZero();
        delta_q.setZero();
        q_des.setZero();
        dq_des.setZero();
        f_opt.setZero();
        tau_ff.setZero();
        joint_kp.setZero();
        joint_kd.setZero();
        status = WbicStatus::Ok;
        residuals = WbicResiduals{};
        solve_time_us = 0.0;
        command_source = CommandSource::Wbic;
    }
};

/** Configuration parameters for WBIC. */
struct WbicConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Task PD gains
    double kp_body_pos = 100.0;
    double kd_body_pos = 10.0;
    double kp_body_ori = 100.0;
    double kd_body_ori = 10.0;
    double kp_foot = 500.0;
    double kd_foot = 10.0;

    // Joint hybrid gains [HipX, HipY, Knee]
    Eigen::Vector3d kp_joint{3.0, 3.0, 3.0};
    Eigen::Vector3d kd_joint{1.0, 0.2, 0.2};

    // QP weights and regularization
    double mu = 0.4;
    double damping = 1e-6;
    double regularization = 1e-8;
    double w_acc = 0.1;
    Eigen::Vector3d w_force{1.0, 1.0, 0.01};  // [Fx, Fy, Fz]

    // Physical limits
    double fz_min = 0.0;
    double fz_max = 120.0;
    double delta_qddot_lin_max = 30.0;   // m/s^2
    double delta_qddot_ang_max = 100.0;  // rad/s^2
    Eigen::Vector3d tau_max{40.0, 40.0, 65.0};  // Nm

    // Solver settings
    int max_wsr = 50;
    double max_cpu_time = 0.0005;  // 0.5 ms

    // Residual thresholds
    double eom_residual_threshold = 1e-4;
    double contact_acc_residual_threshold = 1e-3;
    double inequality_residual_threshold = 1e-6;

    // Failure policy
    int max_consecutive_failures = 3;
    double blend_time_s = 0.1;  // 100 ms legacy -> wbic ramp
};

/** Final 5-column joint command POD [Kp, q_des, Kd, qd_des, tau_ff]. */
struct JointHybridCommand {
    double kp[kNumJoints] = {};
    double q_des[kNumJoints] = {};
    double kd[kNumJoints] = {};
    double qd_des[kNumJoints] = {};
    double tau_ff[kNumJoints] = {};

    void setZero() noexcept
    {
        for (int i = 0; i < kNumJoints; ++i) {
            kp[i] = 0.0;
            q_des[i] = 0.0;
            kd[i] = 0.0;
            qd_des[i] = 0.0;
            tau_ff[i] = 0.0;
        }
    }
};

}  // namespace wbic
