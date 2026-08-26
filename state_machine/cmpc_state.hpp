/**
 * @file cmpc_state.hpp
 * @brief Quadruped robot walking via Convex MPC + WBIC
 */

#ifndef CMPC_HPP_
#define CMPC_HPP_

#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <chrono>

#include "state_base.h"
#include "cmpc_bridge.h"
#include "WbcType.h"

class CMPCState : public StateBase {
private:
    std::unique_ptr<CMPCBridge> gait_ctrl_;
    double run_time_ = 0.0;
    double time_stamp_record_ = 0.0;

    // ── Worker thread (500 Hz, dt = 0.002s) ──────────────────────────────────
    std::thread       cmpc_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<int>  state_run_cnt_{-1};

    // Observation Buffer: FSM thread writes, Worker thread reads
    std::mutex        obs_mtx_;
    double imu_data_[10]   = {};
    double motor_data_[24] = {};
    double user_vel_[3]    = {};

    // Command Double-Buffer: Worker thread writes, FSM thread reads
    std::mutex        cmd_mtx_;
    struct CommandSnapshot {
        Eigen::Matrix<float, 12, 5> joint_cmd = Eigen::Matrix<float, 12, 5>::Zero();
        double timestamp = 0.0;
        uint64_t sequence = 0;
        bool valid = false;
    } latest_cmd_;

    // Must match the simulation YAML config (freq tuning for MPC timestep)
    static constexpr double freq       = 500.0; // 500 Hz
    static constexpr double kDt        = 1.0 / freq; // 0.002 s
    static constexpr double kStandKp   = 100.0;
    static constexpr double kStandKd   = 1.0;
    static constexpr double kJointKp   = 0.0;
    static constexpr double kJointKd   = 0.05;

    // Default WBIC policy: enabled in simulation, disabled by default on hardware
#ifdef BUILD_SIMULATION
    static constexpr bool kDefaultWbicEnabled = true;
#else
    static constexpr bool kDefaultWbicEnabled = false;
#endif

    // ri_ptr_ joint order: [FL, FR, HL, HR]
    // GaitCtrller order:   [FR, FL, HR, HL]
    static constexpr int kCmpcToRiLeg[4] = {1, 0, 3, 2}; // cmpc_leg → ri_leg index

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
        VecXf q  = ri_ptr_->GetJointPosition();
        VecXf qd = ri_ptr_->GetJointVelocity();

        const int ri_order[4] = {1, 0, 3, 2}; // cmpc leg i → ri_ptr_ leg ri_order[i]
        for (int i = 0; i < 4; i++) {
            int ri_leg = ri_order[i];
            for (int j = 0; j < 3; j++) {
                motorData[i * 3 + j]      = q(ri_leg * 3 + j);
                motorData[12 + i * 3 + j] = qd(ri_leg * 3 + j);
            }
        }
    }

    void BuildUserVelocity(double* vel) {
        auto cmd = uc_ptr_->GetUserCommand();
        vel[0] = cmd.forward_vel_scale  * 1.0;   // vx: max 3 m/s
        vel[1] = cmd.side_vel_scale     * 1.0;   // vy: max 2 m/s
        vel[2] = cmd.turnning_vel_scale * 1.0;   // yaw rate: max 2.5 rad/s
    }

    // Worker thread: executes heavy CMPC + WBIC at 500 Hz
    void CmpcRunner() {
        int run_cnt_record = -1;
        uint64_t worker_seq = 0;

        while (start_flag_) {
            int cnt = state_run_cnt_.load();
            if (cnt >= 0 && cnt != run_cnt_record) {
                double imu[10], motor[24], vel[3], effort[12] = {};
                {
                    std::lock_guard<std::mutex> lk(obs_mtx_);
                    std::memcpy(imu,   imu_data_,   sizeof(imu));
                    std::memcpy(motor, motor_data_, sizeof(motor));
                    std::memcpy(vel,   user_vel_,   sizeof(vel));
                }

                gait_ctrl_->SetRobotVel(vel);

                wbic::JointHybridCommand hybrid_cmd;
                gait_ctrl_->TorqueCalculator(imu, motor, effort, &hybrid_cmd);

                // Remap hybrid command [FR, FL, HR, HL] → RI [FL, FR, HL, HR]
                Eigen::Matrix<float, 12, 5> joint_cmd;
                joint_cmd.setZero();

                for (int cmpc_leg = 0; cmpc_leg < 4; ++cmpc_leg) {
                    const int ri_leg = kCmpcToRiLeg[cmpc_leg];
                    for (int j = 0; j < 3; ++j) {
                        joint_cmd(ri_leg * 3 + j, 0) = static_cast<float>(hybrid_cmd.kp[cmpc_leg * 3 + j]);
                        joint_cmd(ri_leg * 3 + j, 1) = static_cast<float>(hybrid_cmd.q_des[cmpc_leg * 3 + j]);
                        joint_cmd(ri_leg * 3 + j, 2) = static_cast<float>(hybrid_cmd.kd[cmpc_leg * 3 + j]);
                        joint_cmd(ri_leg * 3 + j, 3) = static_cast<float>(hybrid_cmd.qd_des[cmpc_leg * 3 + j]);
                        joint_cmd(ri_leg * 3 + j, 4) = static_cast<float>(hybrid_cmd.tau_ff[cmpc_leg * 3 + j]);
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(cmd_mtx_);
                    latest_cmd_.joint_cmd = joint_cmd;
                    latest_cmd_.timestamp = ri_ptr_->GetInterfaceTimeStamp();
                    latest_cmd_.sequence = ++worker_seq;
                    latest_cmd_.valid = true;
                }

                run_cnt_record = cnt;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    CMPCState(const RobotType& robot_type, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {}
    ~CMPCState() = default;

    virtual void OnEnter() override {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::CMPC);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        time_stamp_record_ = run_time_;

        double pidParam[4] = {kStandKp, kStandKd, kJointKp, kJointKd};
        gait_ctrl_ = std::make_unique<CMPCBridge>(freq, pidParam, kDefaultWbicEnabled);
        gait_ctrl_->SetGaitType(0);   // 0 = trot
        gait_ctrl_->SetRobotMode(0);  // 0 = follow user velocity command
        gait_ctrl_->Reset();

        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            latest_cmd_.joint_cmd.setZero();
            latest_cmd_.timestamp = run_time_;
            latest_cmd_.sequence = 0;
            latest_cmd_.valid = false;
        }

        state_run_cnt_ = -1;
        start_flag_    = true;
        cmpc_thread_   = std::thread(std::bind(&CMPCState::CmpcRunner, this));
    }

    virtual void OnExit() override {
        start_flag_ = false;
        if (cmpc_thread_.joinable()) cmpc_thread_.join();
        state_run_cnt_ = -1;
        gait_ctrl_.reset();
    }

    virtual void Run() override {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();

        // 1. Update observation buffer
        {
            std::lock_guard<std::mutex> lk(obs_mtx_);
            BuildImuData(imu_data_);
            BuildMotorData(motor_data_);
            BuildUserVelocity(user_vel_);
        }
        ++state_run_cnt_;

        // 2. Fetch and apply latest joint command (FSM thread is sole caller of SetJointCommand)
        CommandSnapshot cmd_snap;
        {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            cmd_snap = latest_cmd_;
        }

        // Staleness check: if command older than 3 control periods (6 ms), output safe damping
        const double cmd_age = run_time_ - cmd_snap.timestamp;
        if (cmd_snap.valid && cmd_age <= 3.0 * kDt) {
            ri_ptr_->SetJointCommand(cmd_snap.joint_cmd);
        } else {
            // Safe damping command
            Eigen::Matrix<float, 12, 5> safe_cmd;
            safe_cmd.setZero();
            for (int k = 0; k < 12; ++k) {
                safe_cmd(k, 2) = 1.0f; // Kd = 1.0 damping
            }
            ri_ptr_->SetJointCommand(safe_cmd);
        }
    }

    virtual bool LoseControlJudge() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping))
            return true;
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if (std::abs(rpy(0)) > 25. / 180 * M_PI || std::abs(rpy(1)) > 30. / 180 * M_PI) {
            std::cout << "posture value: " << 180. / M_PI * rpy.transpose() << std::endl;
            return true;
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
