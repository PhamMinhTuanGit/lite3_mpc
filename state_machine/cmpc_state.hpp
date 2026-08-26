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

#include "state_base.h"
#include "cmpc_bridge.h"

class CMPCState : public StateBase {
private:
    std::unique_ptr<CMPCBridge> gait_ctrl_;
    Eigen::Matrix<float, 12, 5> joint_cmd_;
    double run_time_ = 0.0;
    double time_stamp_record_ = 0.0;

    
    
    // ── Worker thread (giống PolicyRunner trong rl_control_state) ──────────
    std::thread       cmpc_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<int>  state_run_cnt_{-1};
    std::mutex        data_mtx_;
    // Bộ đệm quan sát: Run() (vòng FSM) ghi, thread nền đọc
    double imu_data_[10]   = {};
    double motor_data_[24] = {};
    // Chạy TorqueCalculator mỗi `kDecimation` lần Run() (1 = mỗi tick)
    static constexpr int kDecimation = 1;

    // Must match the simulation YAML config (freq tuning for MPC timestep)
    static constexpr double freq       = 500.0; // freq
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
        VecXf q  = ri_ptr_->GetJointPosition();
        VecXf qd = ri_ptr_->GetJointVelocity();

        // GaitCtrller leg 0=FR, 1=FL, 2=HR, 3=HL
        // ri_ptr_ leg   0=FL, 1=FR, 2=HL, 3=HR   (each leg occupies 3 joints)
        const int ri_order[4] = {1, 0, 3, 2}; // cmpc leg i → ri_ptr_ leg ri_order[i]
        for (int i = 0; i < 4; i++) {
            int ri_leg = ri_order[i];
            for (int j = 0; j < 3; j++) {
                motorData[i * 3 + j]      = q(ri_leg * 3 + j);
                motorData[12 + i * 3 + j] = qd(ri_leg * 3 + j);
            }
        }
    }

    // ── Cập nhật RobotModel từ imu_data_ + motor_data_ (đã build ở Run(), trong mutex) ──
    void BuildRobotModelState() {
        // imu_data_ = [accX,accY,accZ, qx,qy,qz,qw, omgX,omgY,omgZ]
        Eigen::Quaterniond quat(
            imu_data_[6],   // w
            imu_data_[3],   // x
            imu_data_[4],   // y
            imu_data_[5]    // z
        );
        Vec3 omega_world(imu_data_[7], imu_data_[8], imu_data_[9]);

        // motor_data_[ 0..11] = pos [FR,FL,HR,HL] = RobotModel order
        // motor_data_[12..23] = vel
        Vec12 q, dq;
        for (int i = 0; i < 12; ++i) {
            q[i]  = motor_data_[i];         // pos
            dq[i] = motor_data_[12 + i];    // vel
        }

        const Vec3 p_world = Vec3::Zero();    // cần state estimator → tạm 0
        const Vec3 v_lin   = Vec3::Zero();     // cần state estimator → tạm 0

        auto& rm = *data_ptr_->robot_model_ptr;
        rm.setState(p_world, quat, v_lin, omega_world, q, dq);
        rm.update();   // M, h, Jc, J̇v ready
    }

    // Map effort for each leg from GaitCtrller [FR,FL,HR,HL] → joint_cmd_ [FL,FR,HL,HR]
    void ApplyEffort(const double* effort) {
        joint_cmd_.setZero();
        const int ri_order[4] = {1, 0, 3, 2};
        for (int i = 0; i < 4; i++) {
            int ri_leg = ri_order[i];
            for (int j = 0; j < 3; j++) {
                joint_cmd_(ri_leg * 3 + j, 4) = static_cast<float>(effort[i * 3 + j]);
            }
        }
        ri_ptr_->SetJointCommand(joint_cmd_);  // gửi full torque (ko phân ra swing(position) và stance (torque))
    }

    void UpdateVelocityCommand() {
        auto cmd = uc_ptr_->GetUserCommand();
        double vel[3] = {
            cmd.forward_vel_scale  * 1.0,   // vx: max 3 m/s
            cmd.side_vel_scale     * 1.0,   // vy: max 2 m/s
            cmd.turnning_vel_scale * 1.0    // yaw rate: max 2.5 rad/s
        };
        // std::cout << "vel :" << vel[0] << " ; " << vel[1] << " ; " << vel[2] << std::endl;
        gait_ctrl_->SetRobotVel(vel);
    }

    



    // Thread nền: chạy bộ điều khiển nặng (TorqueCalculator) độc lập với vòng FSM
    void CmpcRunner() {
        int run_cnt_record = -1;
        while (start_flag_) {
            int cnt = state_run_cnt_.load();
            if (cnt >= 0 && cnt % kDecimation == 0 && cnt != run_cnt_record) {
                // Lấy bản sao quan sát mới nhất (tránh đọc rách giữa lúc Run() ghi)
                double imu[10], motor[24], effort[12] = {};
                {
                    std::lock_guard<std::mutex> lk(data_mtx_);
                    std::memcpy(imu,   imu_data_,   sizeof(imu));
                    std::memcpy(motor, motor_data_, sizeof(motor));
                }
                gait_ctrl_->TorqueCalculator(imu, motor, effort);
                ApplyEffort(effort);
                run_cnt_record = cnt;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    CMPCState(const RobotType& robot_type, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {}
    ~CMPCState() {}

    virtual void OnEnter() override {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::CMPC);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        time_stamp_record_ = run_time_;

        double pidParam[4] = {kStandKp, kStandKd, kJointKp, kJointKd};
        gait_ctrl_ = std::make_unique<CMPCBridge>(freq, pidParam);
        gait_ctrl_->SetGaitType(0);   // 0 = trot
        // gait_ctrl_->SetGaitType(10);   // 10 = walking
        gait_ctrl_->SetRobotMode(0);  // 0 = follow user velocity command

        // Khởi động thread nền SAU khi gait_ctrl_ đã sẵn sàng
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

        // Vòng FSM chỉ cập nhật quan sát + tăng bộ đếm (nhẹ);
        // tính mô-men nặng do CmpcRunner() lo ở thread riêng.
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            BuildImuData(imu_data_);
            BuildMotorData(motor_data_);
            // RobotModel được cập nhật ngay sau sensor data, trong cùng lock
            // để thread nền CmpcRunner() luôn thấy model đồng bộ với sensor.
            BuildRobotModelState();
        }
        UpdateVelocityCommand();
        ++state_run_cnt_;
    }

    virtual bool LoseControlJudge() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping))
            return true;
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if (fabs(rpy(0)) > 25. / 180 * M_PI || fabs(rpy(1)) > 30. / 180 * M_PI) {
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
