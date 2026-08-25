/**
 * @file cmpc_state.hpp
 * @brief Quadruped robot walking via Convex MPC (MIT Cheetah controller)
 */

#ifndef CMPC_HPP_
#define CMPC_HPP_

#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <chrono>
#include <iostream>

#include "state_base.h"
#include "cmpc_bridge.h"
#include "cmpc_logger.hpp"

class CMPCState : public StateBase {
private:
    std::unique_ptr<CMPCBridge> gait_ctrl_;
    CmpcLogger cmpc_logger_;
    Eigen::Matrix<float, 12, 5> joint_cmd_;
    double run_time_ = 0.0;
    double time_stamp_record_ = 0.0;

    // ── Worker thread (giống PolicyRunner trong rl_control_state) ──────────
    std::thread          cmpc_thread_;
    std::atomic<bool>    start_flag_{false};
    std::atomic<int>     state_run_cnt_{-1};
    std::atomic<int64_t> last_calc_time_ms_{0};
    std::atomic<bool>    has_first_calc_{false};
    static constexpr int64_t kCmpcDeadlineMs = 100; // Deadline 100ms cho mỗi chu kỳ tính torque
    std::mutex           data_mtx_;
    // Bộ đệm quan sát: Run() (vòng FSM) ghi, thread nền đọc (q: 12, qd: 12, tau: 12, vel: 3)
    double imu_data_[10]   = {};
    double motor_data_[36] = {};
    double vel_cmd_[3]     = {};
    interface::GroundTruthContactData ground_truth_contact_;
    // Chạy TorqueCalculator mỗi `kDecimation` lần Run() (1 = mỗi tick)
    static constexpr int kDecimation = 1;

    // 1000 Hz loop frequency (dt = 0.001s), matching hardware packet rate & LinearKF
    static constexpr double freq       = 1000.0; // freq
    static constexpr double kStandKp   = 100.0;
    static constexpr double kStandKd   = 1.0;
    static constexpr double kJointKp   = 0.0;
    static constexpr double kJointKd   = 0.05;

    // ri_ptr_ joint order: [FL, FR, HL, HR]
    // GaitCtrller order:   [FR, FL, HR, HL]
    static constexpr int kLegRemap[4] = {1, 0, 3, 2}; // cmpc_leg → ri_leg index

    void BuildImuData(double* imuData) {
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f omg = ri_ptr_->GetImuOmega();

        imuData[0] = acc(0);
        imuData[1] = acc(1);
        imuData[2] = acc(2);

        // RPY → quaternion (ZYX convention)
        Eigen::Quaternionf q =
            Eigen::AngleAxisf(rpy(2), Vec3f::UnitZ()) *
            Eigen::AngleAxisf(rpy(1), Vec3f::UnitY()) *
            Eigen::AngleAxisf(rpy(0), Vec3f::UnitX());
        imuData[3] = q.x();
        imuData[4] = q.y();
        imuData[5] = q.z();
        imuData[6] = q.w();

        imuData[7] = omg(0);
        imuData[8] = omg(1);
        imuData[9] = omg(2);
    }

    // Reorder joints from ri_ptr_ [FL,FR,HL,HR] to GaitCtrller [FR,FL,HR,HL]
    void BuildMotorData(double* motorData) {
        VecXf q   = ri_ptr_->GetJointPosition();
        VecXf qd  = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();

        // GaitCtrller leg 0=FR, 1=FL, 2=HR, 3=HL
        // ri_ptr_ leg   0=FL, 1=FR, 2=HL, 3=HR   (each leg occupies 3 joints)
        const int ri_order[4] = {1, 0, 3, 2}; // cmpc leg i → ri_ptr_ leg ri_order[i]
        for (int i = 0; i < 4; i++) {
            int ri_leg = ri_order[i];
            for (int j = 0; j < 3; j++) {
                motorData[i * 3 + j]      = q(ri_leg * 3 + j);
                motorData[12 + i * 3 + j] = qd(ri_leg * 3 + j);
                motorData[24 + i * 3 + j] = tau(ri_leg * 3 + j);
            }
        }
    }

    // ── Alpha Blending khi chuyển từ StandUpState sang CMPCState ─────────
    VecXf                                 init_stand_joint_pos_;
    VecXf                                 stand_kp_, stand_kd_;
    double                                blend_start_time_ = 0.0;
    std::chrono::steady_clock::time_point blend_start_steady_time_;
    static constexpr float                kBlendDuration = 0.5f; // Thời gian hòa trộn lực 0.5s

    // Map effort[12] from GaitCtrller [FR,FL,HR,HL] → joint_cmd_ [FL,FR,HL,HR]
    void ApplyEffort(const double* effort) {
        joint_cmd_.setZero();

        // Tính hệ số alpha blend từ 0.0 (100% Joint PD) -> 1.0 (100% MPC Torque)
        float elapsed_sim  = static_cast<float>(ri_ptr_->GetInterfaceTimeStamp() - blend_start_time_);
        float elapsed_wall = std::chrono::duration<float>(std::chrono::steady_clock::now() - blend_start_steady_time_).count();
        float elapsed      = (elapsed_sim > 0.0f) ? elapsed_sim : elapsed_wall;
        float alpha        = std::max(0.0f, std::min(1.0f, elapsed / kBlendDuration));

        const int ri_order[4] = {1, 0, 3, 2};
        for (int i = 0; i < 4; i++) {
            int ri_leg = ri_order[i];
            for (int j = 0; j < 3; j++) {
                int joint_idx = ri_leg * 3 + j;
                int cmpc_idx  = i * 3 + j;

                // Alpha blending: Giảm dần Joint PD từ StandUp và tăng dần Mô-men từ MPC
                joint_cmd_(joint_idx, 0) = (1.0f - alpha) * stand_kp_(joint_idx); // Kp
                joint_cmd_(joint_idx, 1) = init_stand_joint_pos_(joint_idx);       // q_des
                joint_cmd_(joint_idx, 2) = std::max(1.0f, (1.0f - alpha)) * stand_kd_(joint_idx); // Kd
                joint_cmd_(joint_idx, 3) = 0.0f;                                   // qd_des
                joint_cmd_(joint_idx, 4) = alpha * static_cast<float>(effort[cmpc_idx]); // tau_ff
            }
        }
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    void BuildVelocityCommand(double* vel) {
        auto cmd = uc_ptr_->GetUserCommand();
        vel[0] = cmd.forward_vel_scale  * 0.5;   // vx: max 0.5 m/s
        vel[1] = cmd.side_vel_scale     * 0.3;   // vy: max 0.3 m/s
        vel[2] = cmd.turnning_vel_scale * 0.8;   // yaw rate: max 0.8 rad/s
    }

    // Thread nền: chạy bộ điều khiển nặng (TorqueCalculator) độc lập với vòng FSM
    void CmpcRunner() {
        int run_cnt_record = -1;
        while (start_flag_) {
            int cnt = state_run_cnt_.load();
            if (cnt >= 0 && cnt % kDecimation == 0 && cnt != run_cnt_record) {
                // Lấy bản sao quan sát + lệnh vận tốc đồng bộ hoàn toàn (Atomic Snapshot)
                double imu[10], motor[36], vel[3], effort[12] = {};
                interface::GroundTruthContactData ground_truth;
                {
                    std::lock_guard<std::mutex> lk(data_mtx_);
                    std::memcpy(imu,   imu_data_,   sizeof(imu));
                    std::memcpy(motor, motor_data_, sizeof(motor));
                    std::memcpy(vel,   vel_cmd_,    sizeof(vel));
                    ground_truth = ground_truth_contact_;
                }
                gait_ctrl_->SetRobotVel(vel);
                CmpcTelemetryData telem = {};
                gait_ctrl_->TorqueCalculator(imu, motor, effort, &telem);
                telem.gt_valid = static_cast<uint8_t>(ground_truth.valid);
                for (int leg = 0; leg < 4; ++leg) {
                    telem.gt_contact[leg] = ground_truth.contact[leg];
                    for (int axis = 0; axis < 3; ++axis) {
                        telem.gt_force_world[leg][axis] =
                            ground_truth.force_world[leg][axis];
                    }
                }
                ApplyEffort(effort);

                double time_ms = (ri_ptr_) ? (ri_ptr_->GetInterfaceTimeStamp() * 1000.0) : 0.0;
                cmpc_logger_.Log(time_ms, telem);

                if (ds_ptr_) {
                    ds_ptr_->InsertCmpcTelemetry(telem);
                    ds_ptr_->InsertScopeData(0, telem.cmd_vx);
                    ds_ptr_->InsertScopeData(1, telem.des_vx);
                    ds_ptr_->InsertScopeData(2, telem.kf_vel_body[0]);
                    ds_ptr_->InsertScopeData(3, telem.kf_rpy[1]); // pitch
                    ds_ptr_->InsertScopeData(4, telem.kf_pos[2]); // height z
                    ds_ptr_->InsertScopeData(5, telem.est_mass_filtered);
                    ds_ptr_->InsertScopeData(6, telem.est_com_body[0]); // dx
                    ds_ptr_->InsertScopeData(7, telem.est_com_body[1]); // dy
                    ds_ptr_->InsertScopeData(8, telem.total_support_force_z);
                    ds_ptr_->InsertScopeData(9, telem.t_total_ms);
                }

                auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                last_calc_time_ms_.store(now_ms);
                has_first_calc_.store(true);

                run_cnt_record = cnt;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    CMPCState(const RobotType& robot_type, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        init_stand_joint_pos_ = VecXf::Zero(12);
        stand_kp_ = cp_ptr_->swing_leg_kp_.replicate(4, 1);
        stand_kd_ = cp_ptr_->swing_leg_kd_.replicate(4, 1);
    }
    ~CMPCState() {}

    virtual void OnEnter() override {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::CMPC);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        time_stamp_record_ = run_time_;

        // Khởi tạo Alpha Blending với góc khớp và gain giữ từ StandUpState
        init_stand_joint_pos_     = ri_ptr_->GetJointPosition();
        stand_kp_                 = cp_ptr_->swing_leg_kp_.replicate(4, 1);
        stand_kd_                 = cp_ptr_->swing_leg_kd_.replicate(4, 1);
        blend_start_time_         = run_time_;
        blend_start_steady_time_  = std::chrono::steady_clock::now();

        double pidParam[4] = {kStandKp, kStandKd, kJointKp, kJointKd};
        gait_ctrl_ = std::make_unique<CMPCBridge>(freq, pidParam);
        gait_ctrl_->SetGaitType(0);   // 0 = trot
        // gait_ctrl_->SetGaitType(10);   // 10 = walking
        gait_ctrl_->SetRobotMode(0);  // 0 = follow user velocity command

        // Khởi tạo CmpcLogger
        cmpc_logger_.Init();

        // Khởi động thread nền SAU khi gait_ctrl_ đã sẵn sàng
        has_first_calc_.store(false);
        last_calc_time_ms_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        state_run_cnt_ = -1;
        start_flag_    = true;
        cmpc_thread_   = std::thread(std::bind(&CMPCState::CmpcRunner, this));
    }

    virtual void OnExit() override {
        start_flag_ = false;
        if (cmpc_thread_.joinable()) cmpc_thread_.join();
        state_run_cnt_ = -1;
        has_first_calc_.store(false);
        gait_ctrl_.reset();
        cmpc_logger_.Close();
    }

    virtual void Run() override {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();

        // Vòng FSM chỉ cập nhật quan sát + lệnh vận tốc + tăng bộ đếm (nhẹ);
        // tính mô-men nặng do CmpcRunner() lo ở thread riêng.
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            BuildImuData(imu_data_);
            BuildMotorData(motor_data_);
            BuildVelocityCommand(vel_cmd_);
            ground_truth_contact_ = ri_ptr_->GetGroundTruthContactData();
        }
        ++state_run_cnt_;
    }

    virtual bool LoseControlJudge() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping))
            return true;
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if (fabs(rpy(0)) > 25. / 180 * M_PI || fabs(rpy(1)) > 30. / 180 * M_PI) {
            std::cout << "[CMPC Safety] Posture limit exceeded: " << 180. / M_PI * rpy.transpose() << std::endl;
            return true;
        }

        // Kiểm tra deadline / timeout cho CMPC worker thread
        if (has_first_calc_.load()) {
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            int64_t elapsed_ms = now_ms - last_calc_time_ms_.load();
            if (elapsed_ms > kCmpcDeadlineMs) {
                std::cerr << "[CMPC Safety] CMPC worker thread missed deadline ("
                          << elapsed_ms << " ms > " << kCmpcDeadlineMs << " ms)! Switching to damping..." << std::endl;
                return true;
            }
        }
        return false;
    }

    virtual StateName GetNextStateName() override {
        if (run_time_ - time_stamp_record_ > 1.0 &&
            uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::SitDown)) {
            return StateName::kSitDown;
        }
        return StateName::kCMPC;
    }
};

#endif
