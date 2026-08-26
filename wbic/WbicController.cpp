#include "WbicController.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

#include "KinWbc.hpp"
#include "RobotModel.hpp"
#include "WbicQp.hpp"

namespace wbic {

class WbicController::Impl {
public:
    explicit Impl(const WbicConfig& config)
        : config_(config), kin_wbc_(config.damping), wbic_qp_(config)
    {
        RobotModelConfig rm_cfg;
        rm_cfg.base_frame = "TORSO";
        rm_cfg.foot_frames = {"FR_FOOT", "FL_FOOT", "HR_FOOT", "HL_FOOT"};
        rm_cfg.joint_names = {
            "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
            "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
            "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
            "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint"
        };
        rm_cfg.foot_radius = 0.0;
        rm_cfg.armature = 0.0;

        std::string err;
        if (!robot_model_.build(rm_cfg, &err)) {
            std::cerr << "[WbicController] Error building RobotModel: " << err << std::endl;
        }

        for (int k = 0; k < kNumJoints; ++k) {
            idx_v_[static_cast<std::size_t>(k)] = robot_model_.idxV(k);
        }
    }

    void Reset() noexcept
    {
        wbic_qp_.Reset();
        consecutive_failures_ = 0;
        is_latched_ = false;
        blend_timer_s_ = 0.0;
    }

    WbicStatus RecordFailure(WbicStatus status, WbicOutput* output) noexcept
    {
        ++consecutive_failures_;
        if (consecutive_failures_ >= config_.max_consecutive_failures) {
            is_latched_ = true;
        }
        if (output) {
            output->command_source = CommandSource::FallbackLegacyCmpc;
            output->status = status;
        }
        return status;
    }

    WbicStatus Run(const WbicInput& input, WbicOutput* output) noexcept
    {
        if (output == nullptr) return WbicStatus::InvalidInput;
        output->reset();

        if (is_latched_ || consecutive_failures_ >= config_.max_consecutive_failures) {
            is_latched_ = true;
            output->command_source = CommandSource::FallbackLegacyCmpc;
            output->status = WbicStatus::SafetyViolation;
            return WbicStatus::SafetyViolation;
        }

        if (!input.isValid()) {
            return RecordFailure(WbicStatus::InvalidInput, output);
        }

        const auto start_time = std::chrono::high_resolution_clock::now();

        // ══════════════════════════════════════════════════════════════════
        // 1. Update RobotModel Dynamics
        // ══════════════════════════════════════════════════════════════════
        const Eigen::Quaterniond q_wb(input.R_body);
        robot_model_.setState(input.p_body, q_wb, input.v_body_world, input.omega_body_world,
                              input.q_joint_raw, input.qd_joint_raw);
        robot_model_.update();

        // Populate DynamicsOutput
        dyn_output_.M = robot_model_.M();
        dyn_output_.h = robot_model_.h();

        // Compute M_inv
        Eigen::Matrix<double, kVelocityDimension, kVelocityDimension> M_damped = dyn_output_.M;
        M_damped.diagonal().array() += config_.regularization;
        dyn_output_.M_inv = M_damped.ldlt().solve(
            Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>::Identity());

        if (!dyn_output_.M_inv.allFinite()) {
            return RecordFailure(WbicStatus::DynamicsError, output);
        }

        for (int leg = 0; leg < kNumLegs; ++leg) {
            dyn_output_.Jf[leg] = robot_model_.Jc().middleRows<3>(3 * leg);
            dyn_output_.dJdq_f[leg] = robot_model_.dJv().segment<3>(3 * leg);
            dyn_output_.pf[leg] = robot_model_.footPos(leg);
            dyn_output_.vf[leg] = robot_model_.footVel(leg);
        }

        dyn_output_.Jb = robot_model_.Jb();
        dyn_output_.dJdq_b = robot_model_.dJvb();
        dyn_output_.R_wb = robot_model_.baseRot();
        dyn_output_.p_b = robot_model_.basePos();
        dyn_output_.v_b_world = input.v_body_world;
        dyn_output_.omega_b_world = input.omega_body_world;

        // ══════════════════════════════════════════════════════════════════
        // 2. Assemble Contact Set
        // ══════════════════════════════════════════════════════════════════
        contact_set_.assemble(input.contact, input.Fr_des, dyn_output_.Jf, dyn_output_.dJdq_f);

        // ══════════════════════════════════════════════════════════════════
        // 3. Hierarchical KinWBC
        // ══════════════════════════════════════════════════════════════════
        KinWbcResult kin_res;
        if (!kin_wbc_.Compute(input, dyn_output_, contact_set_, config_, &kin_res)) {
            return RecordFailure(WbicStatus::KinWbcError, output);
        }

        // ══════════════════════════════════════════════════════════════════
        // 4. Reduced WBIC QP
        // ══════════════════════════════════════════════════════════════════
        WbicQpResult qp_res;
        if (!wbic_qp_.Solve(input, dyn_output_, contact_set_, kin_res.qddot_cmd,
                            kin_res.q_des, kin_res.dq_des, idx_v_, config_, &qp_res)) {
            return RecordFailure(qp_res.status, output);
        }

        // ══════════════════════════════════════════════════════════════════
        // 5. Populate Output & Handle Blending
        // ══════════════════════════════════════════════════════════════════
        consecutive_failures_ = 0;

        output->qddot = qp_res.qddot;
        output->delta_q = kin_res.delta_q;
        output->q_des = kin_res.q_des;
        output->dq_des = kin_res.dq_des;
        output->f_opt = qp_res.f_opt;
        output->tau_ff = qp_res.tau_ff;
        output->residuals = qp_res.residuals;
        output->status = qp_res.status;
        output->command_source = CommandSource::Wbic;

        // Smooth gain ramp during blend period (100 ms)
        blend_timer_s_ += 0.002; // nominal 500 Hz dt
        const double blend_ratio = std::clamp(blend_timer_s_ / std::max(config_.blend_time_s, 1e-4), 0.0, 1.0);

        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int j = 0; j < 3; ++j) {
                output->joint_kp[leg * 3 + j] = blend_ratio * config_.kp_joint[j];
                output->joint_kd[leg * 3 + j] = blend_ratio * config_.kd_joint[j];
            }
        }

        const auto end_time = std::chrono::high_resolution_clock::now();
        output->solve_time_us =
            std::chrono::duration<double, std::micro>(end_time - start_time).count();

        return output->status;
    }

    const WbicConfig& GetConfig() const noexcept { return config_; }
    void SetConfig(const WbicConfig& config) noexcept
    {
        config_ = config;
        kin_wbc_.SetDamping(config.damping);
    }

    bool IsLatched() const noexcept { return is_latched_; }
    void Unlatch() noexcept
    {
        is_latched_ = false;
        consecutive_failures_ = 0;
        blend_timer_s_ = 0.0;
    }

    int GetConsecutiveFailures() const noexcept { return consecutive_failures_; }

private:
    WbicConfig config_;
    RobotModel robot_model_;
    KinWbc kin_wbc_;
    WbicQp wbic_qp_;

    std::array<int, kNumJoints> idx_v_{};
    DynamicsOutput dyn_output_{};
    ContactSet contact_set_{};

    int consecutive_failures_ = 0;
    bool is_latched_ = false;
    double blend_timer_s_ = 0.0;
};

WbicController::WbicController(const WbicConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

WbicController::~WbicController() = default;

WbicController::WbicController(WbicController&&) noexcept = default;
WbicController& WbicController::operator=(WbicController&&) noexcept = default;

void WbicController::Reset() noexcept
{
    if (impl_) impl_->Reset();
}

WbicStatus WbicController::Run(const WbicInput& input, WbicOutput* output) noexcept
{
    if (!impl_) return WbicStatus::InvalidInput;
    return impl_->Run(input, output);
}

const WbicConfig& WbicController::GetConfig() const noexcept
{
    return impl_->GetConfig();
}

void WbicController::SetConfig(const WbicConfig& config) noexcept
{
    if (impl_) impl_->SetConfig(config);
}

bool WbicController::IsLatched() const noexcept
{
    return impl_ ? impl_->IsLatched() : false;
}

void WbicController::Unlatch() noexcept
{
    if (impl_) impl_->Unlatch();
}

int WbicController::GetConsecutiveFailures() const noexcept
{
    return impl_ ? impl_->GetConsecutiveFailures() : 0;
}

void WbicController::PopulateHybridCommand(const WbicOutput& output,
                                           JointHybridCommand* cmd) noexcept
{
    if (cmd == nullptr) return;
    for (int i = 0; i < kNumJoints; ++i) {
        cmd->kp[i] = output.joint_kp[i];
        cmd->q_des[i] = output.q_des[i];
        cmd->kd[i] = output.joint_kd[i];
        cmd->qd_des[i] = output.dq_des[i];
        cmd->tau_ff[i] = output.tau_ff[i];
    }
}

}  // namespace wbic
