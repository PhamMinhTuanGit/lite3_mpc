#include "CentroidalWrenchController.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

#include <qpOASES/SQProblem.hpp>
#include "KinWbc.hpp"

namespace wbic {

namespace {
constexpr double kBigNumber = 1e10;

inline Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& r) noexcept
{
    Eigen::Matrix3d m;
    m <<     0.0, -r(2),  r(1),
            r(2),   0.0, -r(0),
          -r(1),  r(0),   0.0;
    return m;
}

} // namespace

class CentroidalWrenchController::Impl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Impl(const CentroidalWrenchConfig& config)
        : config_(config)
    {
        solver_ = std::make_unique<qpOASES::SQProblem>(kNumVars, kNumConstraints, qpOASES::HST_SEMIDEF);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        options.enableRegularisation = qpOASES::BT_TRUE;
        options.epsRegularisation = config.w_reg;
        options.enableDropInfeasibles = qpOASES::BT_TRUE;
        options.enableInertiaCorrection = qpOASES::BT_TRUE;
        solver_->setOptions(options);
        is_initialized_ = false;
    }

    void Reset() noexcept
    {
        if (solver_) {
            solver_->reset();
        }
        is_initialized_ = false;
        prev_contact_ = {{false, false, false, false}};
        has_f_prev_ = false;
        f_prev_.setZero();
    }

    bool Compute(const CentroidalWrenchInput& input, CentroidalWrenchOutput* output) noexcept
    {
        if (output == nullptr) return false;
        output->reset();

        const auto start_time = std::chrono::high_resolution_clock::now();

        // ══════════════════════════════════════════════════════════════════
        // 1. Compute Desired Translational Force F_des
        // F_des = m * (a_des + Kp * (p_des - p) + Kd * (v_des - v) - g)
        // ══════════════════════════════════════════════════════════════════
        const Eigen::Vector3d g_world(0.0, 0.0, -config_.gravity);
        const Eigen::Vector3d p_err = input.com_des_world - input.com_world;
        const Eigen::Vector3d v_err = input.com_vel_des_world - input.com_vel_world;

        const Eigen::Vector3d a_cmd = input.com_acc_des_world
                                    + config_.kp_pos.cwiseProduct(p_err)
                                    + config_.kd_pos.cwiseProduct(v_err)
                                    - g_world;

        const Eigen::Vector3d F_des = input.mass * a_cmd;
        output->force_des_world = F_des;

        // Breakdown telemetry
        output->Fz_gravity = input.mass * config_.gravity;
        output->Fz_pos_P = input.mass * config_.kp_pos.z() * p_err.z();
        output->Fz_vel_D = input.mass * config_.kd_pos.z() * v_err.z();

        // ══════════════════════════════════════════════════════════════════
        // 2. Compute Desired Rotational Moment M_des
        // M_des = Kp_M * e_R + Kd_M * (omega_des - omega)
        // ══════════════════════════════════════════════════════════════════
        const Eigen::Vector3d e_R = KinWbc::RotationErrorSO3(input.R_des_world_body, input.R_world_body);
        const Eigen::Vector3d e_omega = input.omega_des_world - input.omega_world;

        output->My_P = config_.kp_ori.y() * e_R.y();
        output->My_D = config_.kd_ori.y() * e_omega.y();
        output->My_ff = 0.0;

        const Eigen::Vector3d M_des = config_.kp_ori.cwiseProduct(e_R)
                                    + config_.kd_ori.cwiseProduct(e_omega);
        output->moment_des_world = M_des;

        // Initialize nominal static f_prev if not set
        if (!has_f_prev_) {
            const double fz_nom = (input.mass * config_.gravity) / 4.0;
            for (int i = 0; i < 4; ++i) {
                f_prev_.segment<3>(3 * i) << 0.0, 0.0, (input.contact[static_cast<std::size_t>(i)] ? fz_nom : 0.0);
            }
        }

        // ══════════════════════════════════════════════════════════════════
        // 3. Assemble QP Matrices
        // Decision variable f = [f_0, f_1, f_2, f_3] in R^12
        // A_F = [I_3, I_3, I_3, I_3] (3x12)
        // A_M = [[r_0]x, [r_1]x, [r_2]x, [r_3]x] (3x12) where r_i = p_foot_i - p_com
        // ══════════════════════════════════════════════════════════════════
        Eigen::Matrix<double, 3, 12> A_F;
        Eigen::Matrix<double, 3, 12> A_M;

        for (int i = 0; i < 4; ++i) {
            A_F.block<3, 3>(0, 3 * i) = Eigen::Matrix3d::Identity();
            const Eigen::Vector3d r_i = input.foot_pos_world[static_cast<std::size_t>(i)] - input.com_world;
            A_M.block<3, 3>(0, 3 * i) = SkewSymmetric(r_i);
        }

        // H = w_F * A_F^T * A_F + w_M * A_M^T * A_M + w_reg * I_12
        H_mat_ = config_.w_force * (A_F.transpose() * A_F)
               + config_.w_moment * (A_M.transpose() * A_M);
        H_mat_.diagonal().array() += config_.w_reg;

        // g = - w_F * A_F^T * F_des - w_M * A_M^T * M_des
        g_vec_ = -config_.w_force * (A_F.transpose() * F_des)
                 - config_.w_moment * (A_M.transpose() * M_des);

        // Force-rate regularization penalty: lambda_df * ||f - f_prev||^2
        if (config_.w_force_rate > 0.0) {
            H_mat_.diagonal().array() += config_.w_force_rate;
            g_vec_ -= config_.w_force_rate * f_prev_;
        }

        // Bounds and Friction Cone Constraints
        A_mat_.setZero();
        lbA_vec_.setConstant(-kBigNumber);
        ubA_vec_.setZero();

        for (int i = 0; i < 4; ++i) {
            const int f_idx = 3 * i;
            const int row_base = 4 * i;

            if (input.contact[static_cast<std::size_t>(i)]) {
                // Variable bounds
                lb_mem_[f_idx + 0] = -kBigNumber;
                ub_mem_[f_idx + 0] = kBigNumber;
                lb_mem_[f_idx + 1] = -kBigNumber;
                ub_mem_[f_idx + 1] = kBigNumber;
                lb_mem_[f_idx + 2] = config_.fz_min;
                ub_mem_[f_idx + 2] = config_.fz_max;

                // 1. fx - mu * fz <= 0
                A_mat_(row_base + 0, f_idx + 0) = 1.0;
                A_mat_(row_base + 0, f_idx + 2) = -config_.mu;
                ubA_vec_[row_base + 0] = 0.0;

                // 2. -fx - mu * fz <= 0
                A_mat_(row_base + 1, f_idx + 0) = -1.0;
                A_mat_(row_base + 1, f_idx + 2) = -config_.mu;
                ubA_vec_[row_base + 1] = 0.0;

                // 3. fy - mu * fz <= 0
                A_mat_(row_base + 2, f_idx + 1) = 1.0;
                A_mat_(row_base + 2, f_idx + 2) = -config_.mu;
                ubA_vec_[row_base + 2] = 0.0;

                // 4. -fy - mu * fz <= 0
                A_mat_(row_base + 3, f_idx + 1) = -1.0;
                A_mat_(row_base + 3, f_idx + 2) = -config_.mu;
                ubA_vec_[row_base + 3] = 0.0;
            } else {
                // Swing foot: f = 0
                lb_mem_[f_idx + 0] = 0.0;
                ub_mem_[f_idx + 0] = 0.0;
                lb_mem_[f_idx + 1] = 0.0;
                ub_mem_[f_idx + 1] = 0.0;
                lb_mem_[f_idx + 2] = 0.0;
                ub_mem_[f_idx + 2] = 0.0;

                // Relax inequalities
                ubA_vec_[row_base + 0] = kBigNumber;
                ubA_vec_[row_base + 1] = kBigNumber;
                ubA_vec_[row_base + 2] = kBigNumber;
                ubA_vec_[row_base + 3] = kBigNumber;
            }
        }

        // Copy matrices to Row-Major buffers for qpOASES
        std::memcpy(H_mem_, H_mat_.data(), kNumVars * kNumVars * sizeof(qpOASES::real_t));
        std::memcpy(g_mem_, g_vec_.data(), kNumVars * sizeof(qpOASES::real_t));
        std::memcpy(A_mem_, A_mat_.data(), kNumConstraints * kNumVars * sizeof(qpOASES::real_t));
        std::memcpy(lbA_mem_, lbA_vec_.data(), kNumConstraints * sizeof(qpOASES::real_t));
        std::memcpy(ubA_mem_, ubA_vec_.data(), kNumConstraints * sizeof(qpOASES::real_t));

        // ══════════════════════════════════════════════════════════════════
        // 4. Solve QP
        // ══════════════════════════════════════════════════════════════════
        int nWSR = config_.max_wsr;
        qpOASES::real_t cputime = config_.max_cpu_time;
        qpOASES::returnValue status_qp = qpOASES::TERMINAL_LIST_ELEMENT;

        bool contact_changed = false;
        for (std::size_t leg = 0; leg < 4; ++leg) {
            if (prev_contact_[leg] != input.contact[leg]) {
                contact_changed = true;
                break;
            }
        }

        if (!is_initialized_ || contact_changed) {
            solver_->reset();
            status_qp = solver_->init(H_mem_, g_mem_, A_mem_, lb_mem_, ub_mem_,
                                      lbA_mem_, ubA_mem_, nWSR, &cputime);
            is_initialized_ = (status_qp == qpOASES::SUCCESSFUL_RETURN);
        } else {
            status_qp = solver_->hotstart(H_mem_, g_mem_, A_mem_, lb_mem_, ub_mem_,
                                          lbA_mem_, ubA_mem_, nWSR, &cputime);
            if (status_qp != qpOASES::SUCCESSFUL_RETURN && status_qp != qpOASES::RET_MAX_NWSR_REACHED) {
                solver_->reset();
                status_qp = solver_->init(H_mem_, g_mem_, A_mem_, lb_mem_, ub_mem_,
                                          lbA_mem_, ubA_mem_, nWSR, &cputime);
                is_initialized_ = (status_qp == qpOASES::SUCCESSFUL_RETURN);
            }
        }

        prev_contact_ = input.contact;

        qpOASES::real_t f_sol[kNumVars] = {};
        solver_->getPrimalSolution(f_sol);

        Eigen::Matrix<double, kNumVars, 1> f_sol_vec;
        for (int i = 0; i < kNumVars; ++i) {
            f_sol_vec[i] = f_sol[i];
        }

        for (int i = 0; i < 4; ++i) {
            output->grf_world[static_cast<std::size_t>(i)] =
                Eigen::Vector3d(f_sol[3 * i + 0], f_sol[3 * i + 1], f_sol[3 * i + 2]);
            output->force_grf_world += output->grf_world[static_cast<std::size_t>(i)];

            const Eigen::Vector3d r_i = input.foot_pos_world[static_cast<std::size_t>(i)] - input.com_world;
            output->moment_grf_world += r_i.cross(output->grf_world[static_cast<std::size_t>(i)]);
        }

        output->df_norm = (f_sol_vec - f_prev_).norm();
        for (int i = 0; i < 4; ++i) {
            output->df_norm_leg[static_cast<std::size_t>(i)] =
                (f_sol_vec.segment<3>(3 * i) - f_prev_.segment<3>(3 * i)).norm();
        }

        output->qp_cost = solver_->getObjVal();

        const auto end_time = std::chrono::high_resolution_clock::now();
        output->qp_solve_time_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        if (status_qp == qpOASES::SUCCESSFUL_RETURN) {
            output->qp_status = 0;
            output->valid = true;
            f_prev_ = f_sol_vec;
            has_f_prev_ = true;
        } else if (status_qp == qpOASES::RET_MAX_NWSR_REACHED) {
            output->qp_status = 1;
            output->valid = true; // Still usable suboptimal solution
            f_prev_ = f_sol_vec;
            has_f_prev_ = true;
        } else {
            output->qp_status = 2;
            output->valid = false;
        }

        return output->valid;
    }

    const CentroidalWrenchConfig& GetConfig() const noexcept { return config_; }
    void SetConfig(const CentroidalWrenchConfig& config) noexcept
    {
        config_ = config;
    }
    void SetForceRateWeight(double w) noexcept
    {
        config_.w_force_rate = w;
    }

private:
    static constexpr int kNumVars = 12;
    static constexpr int kNumConstraints = 16;

    CentroidalWrenchConfig config_;
    std::unique_ptr<qpOASES::SQProblem> solver_;
    bool is_initialized_ = false;
    std::array<bool, 4> prev_contact_{{false, false, false, false}};

    Eigen::Matrix<double, kNumVars, 1> f_prev_ = Eigen::Matrix<double, kNumVars, 1>::Zero();
    bool has_f_prev_ = false;

    // Fixed-size row-major buffers
    qpOASES::real_t H_mem_[kNumVars * kNumVars] = {};
    qpOASES::real_t g_mem_[kNumVars] = {};
    qpOASES::real_t A_mem_[kNumConstraints * kNumVars] = {};
    qpOASES::real_t lb_mem_[kNumVars] = {};
    qpOASES::real_t ub_mem_[kNumVars] = {};
    qpOASES::real_t lbA_mem_[kNumConstraints] = {};
    qpOASES::real_t ubA_mem_[kNumConstraints] = {};

    Eigen::Matrix<double, kNumVars, kNumVars, Eigen::RowMajor> H_mat_{};
    Eigen::Matrix<double, kNumVars, 1> g_vec_{};
    Eigen::Matrix<double, kNumConstraints, kNumVars, Eigen::RowMajor> A_mat_{};
    Eigen::Matrix<double, kNumConstraints, 1> lbA_vec_{};
    Eigen::Matrix<double, kNumConstraints, 1> ubA_vec_{};
};

CentroidalWrenchController::CentroidalWrenchController(const CentroidalWrenchConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

CentroidalWrenchController::~CentroidalWrenchController() = default;

CentroidalWrenchController::CentroidalWrenchController(CentroidalWrenchController&&) noexcept = default;
CentroidalWrenchController& CentroidalWrenchController::operator=(CentroidalWrenchController&&) noexcept = default;

void CentroidalWrenchController::Reset() noexcept
{
    if (impl_) impl_->Reset();
}

bool CentroidalWrenchController::Compute(const CentroidalWrenchInput& input, CentroidalWrenchOutput* output) noexcept
{
    if (!impl_) return false;
    return impl_->Compute(input, output);
}

const CentroidalWrenchConfig& CentroidalWrenchController::GetConfig() const noexcept
{
    return impl_->GetConfig();
}

void CentroidalWrenchController::SetConfig(const CentroidalWrenchConfig& config) noexcept
{
    if (impl_) impl_->SetConfig(config);
}

void CentroidalWrenchController::SetForceRateWeight(double w) noexcept
{
    if (impl_) impl_->SetForceRateWeight(w);
}

} // namespace wbic
