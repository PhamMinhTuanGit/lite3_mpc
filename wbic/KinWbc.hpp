#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "WbcType.h"

namespace wbic {

struct KinWbcResult {
    GeneralizedAcceleration qddot_cmd = GeneralizedAcceleration::Zero();
    JointVector delta_q = JointVector::Zero();
    JointVector q_des = JointVector::Zero();
    JointVector dq_des = JointVector::Zero();
    TaskStack task_stack{};
    double contact_acc_residual_norm = 0.0;
    bool success = true;
};

class KinWbc {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit KinWbc(double damping = 1e-6);
    ~KinWbc() = default;

    void SetDamping(double damping) noexcept { damping_ = damping; }
    double GetDamping() const noexcept { return damping_; }

    void SetJointLimits(const JointVector& q_min,
                        const JointVector& q_max,
                        const JointVector& qd_max) noexcept;

    /**
     * Compute hierarchical kinWBC reference acceleration qddot_cmd,
     * desired joint positions q_des, and desired joint velocities dq_des.
     */
    bool Compute(const WbicInput& input,
                 const DynamicsOutput& dyn,
                 const ContactSet& contact_set,
                 const WbicConfig& config,
                 KinWbcResult* result) noexcept;

    /**
     * Compute rotation error on SO(3) via matrix logarithm: log3(R_des * R^T).
     */
    static Eigen::Vector3d RotationErrorSO3(const Eigen::Matrix3d& R_des,
                                           const Eigen::Matrix3d& R_cur) noexcept;

private:
    double damping_ = 1e-6;

    JointVector q_min_ = JointVector::Zero();
    JointVector q_max_ = JointVector::Zero();
    JointVector qd_max_ = JointVector::Zero();

    // Work buffers to avoid heap allocations
    Eigen::Matrix<double, kVelocityDimension, kVelocityDimension> N_cur_ =
        Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity();
    Eigen::Matrix<double, kVelocityDimension, kVelocityDimension> N_kin_ =
        Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity();
    GeneralizedAcceleration qddot_cur_ = GeneralizedAcceleration::Zero();
    GeneralizedVelocity delta_v_cur_ = GeneralizedVelocity::Zero();
    GeneralizedVelocity delta_q_cur_ = GeneralizedVelocity::Zero();

    Eigen::Matrix<double, 6, kVelocityDimension> J_tilde_ =
        Eigen::Matrix<double, 6, kVelocityDimension>::Zero();
    Eigen::Matrix<double, kVelocityDimension, 6> J_bar_ =
        Eigen::Matrix<double, kVelocityDimension, 6>::Zero();
    Eigen::Matrix<double, 6, 6> J_inv_inner_ = Eigen::Matrix<double, 6, 6>::Zero();
};

}  // namespace wbic
