#pragma once

#include <array>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <qpOASES/SQProblem.hpp>

#include "WbcType.h"

namespace wbic {

struct WbicQpResult {
    GeneralizedAcceleration qddot = GeneralizedAcceleration::Zero();
    Eigen::Matrix<double, 6, 1> delta_qddot_u = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 12, 1> f_opt = Eigen::Matrix<double, 12, 1>::Zero();
    JointVector tau_ff = JointVector::Zero();
    WbicResiduals residuals{};
    WbicStatus status = WbicStatus::Ok;
    int wsr_performed = 0;
    double cpu_time_ms = 0.0;
};

class WbicQp {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit WbicQp(const WbicConfig& config = WbicConfig());
    ~WbicQp();

    void Reset() noexcept;

    /**
     * Solve the reduced 18-variable WBIC QP problem.
     */
    bool Solve(const WbicInput& input,
               const DynamicsOutput& dyn,
               const ContactSet& contact_set,
               const GeneralizedAcceleration& qddot_cmd,
               const JointVector& q_des,
               const JointVector& dq_des,
               const std::array<int, kNumJoints>& idx_v,
               const WbicConfig& config,
               WbicQpResult* result) noexcept;

private:
    static constexpr int kNumVars = 18;
    static constexpr int kMaxConstraints = 34; // 6 (EoM) + 16 (Friction) + 12 (Torque)

    std::unique_ptr<qpOASES::SQProblem> solver_;
    bool is_initialized_ = false;
    ContactFlags prev_contact_{{false, false, false, false}};

    // Fixed-size row-major buffers for qpOASES (zero heap alloc at runtime)
    qpOASES::real_t H_mem_[kNumVars * kNumVars] = {};
    qpOASES::real_t g_mem_[kNumVars] = {};
    qpOASES::real_t A_mem_[kMaxConstraints * kNumVars] = {};
    qpOASES::real_t lb_mem_[kNumVars] = {};
    qpOASES::real_t ub_mem_[kNumVars] = {};
    qpOASES::real_t lbA_mem_[kMaxConstraints] = {};
    qpOASES::real_t ubA_mem_[kMaxConstraints] = {};
    qpOASES::real_t z_opt_[kNumVars] = {};

    Eigen::Matrix<double, kNumVars, kNumVars, Eigen::RowMajor> H_mat_ =
        Eigen::Matrix<double, kNumVars, kNumVars, Eigen::RowMajor>::Zero();
    Eigen::Matrix<double, kNumVars, 1> g_vec_ = Eigen::Matrix<double, kNumVars, 1>::Zero();
    Eigen::Matrix<double, kMaxConstraints, kNumVars, Eigen::RowMajor> A_mat_ =
        Eigen::Matrix<double, kMaxConstraints, kNumVars, Eigen::RowMajor>::Zero();
    Eigen::Matrix<double, kMaxConstraints, 1> lbA_vec_ = Eigen::Matrix<double, kMaxConstraints, 1>::Zero();
    Eigen::Matrix<double, kMaxConstraints, 1> ubA_vec_ = Eigen::Matrix<double, kMaxConstraints, 1>::Zero();
};

}  // namespace wbic
