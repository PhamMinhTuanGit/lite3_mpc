#include "GaitCtrller.h"
#include "Utilities/Timer.h"
#include "RobotModel.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <limits>

namespace {

const char* StandingTestModeName(wbic::StandingTestMode mode)
{
    switch (mode) {
    case wbic::StandingTestMode::TestALegacy: return "TEST_A_LEGACY";
    case wbic::StandingTestMode::TestBWbicNormal: return "TEST_B_WBIC_NORMAL";
    case wbic::StandingTestMode::TestCWbicLockBase: return "TEST_C_WBIC_LOCK_BASE";
    }
    return "UNKNOWN";
}

const char* WbicStatusName(wbic::WbicStatus status)
{
    switch (status) {
    case wbic::WbicStatus::Ok: return "OK";
    case wbic::WbicStatus::InvalidInput: return "INVALID_INPUT";
    case wbic::WbicStatus::DynamicsError: return "DYNAMICS_ERROR";
    case wbic::WbicStatus::KinWbcError: return "KIN_WBC_ERROR";
    case wbic::WbicStatus::QpInfeasible: return "QP_INFEASIBLE";
    case wbic::WbicStatus::QpMaxIter: return "QP_MAX_ITER";
    case wbic::WbicStatus::QpSolverError: return "QP_SOLVER_ERROR";
    case wbic::WbicStatus::ResidualExceeded: return "RESIDUAL_EXCEEDED";
    case wbic::WbicStatus::TorqueLimitViolated: return "TORQUE_LIMIT_VIOLATED";
    case wbic::WbicStatus::SafetyViolation: return "SAFETY_VIOLATION";
    }
    return "UNKNOWN";
}

}  // namespace

GaitCtrller::GaitCtrller(double freq, double *PIDParam, bool wbic_enabled)
    : _wbic_enabled(wbic_enabled),
      freq_(freq),
      dt_(1.0 / freq),
      cheaterState(nullptr),
      controlParameters(std::make_unique<RobotControlParameters>()),
      _quadruped{buildMiniCheetah<float>()},
      _model{_quadruped.buildModel()},
      _legController{std::make_unique<LegController<float>>(_quadruped)},
      _stateEstimator{std::make_unique<StateEstimatorContainer<float>>(
          cheaterState.get(),
          &_vectorNavData,
          _legController->datas,
          &_stateEstimate,
          controlParameters.get())},
      convexMPC{std::make_unique<ConvexMPCLocomotion>(1.0 / freq, 30)},
      _desiredStateCommand{std::make_unique<DesiredStateCommand<float>>(1.0 / freq)},
      safetyChecker{std::make_unique<SafetyChecker<float>>()},
      _wbicController{std::make_unique<wbic::WbicController>()},
      centroidal_wrench_ctrl_{std::make_unique<wbic::CentroidalWrenchController>()},
      pinocchio_robot_model_{std::make_unique<RobotModel>()}
{
    controlParameters->controller_dt = 1.0 / freq;

    RobotModelConfig rm_cfg;
    rm_cfg.base_frame = "TORSO";
    rm_cfg.foot_frames = {"FR_FOOT", "FL_FOOT", "HR_FOOT", "HL_FOOT"};
    rm_cfg.joint_names = {
        "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
        "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
        "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
        "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint"
    };
    std::string err;
    if (!pinocchio_robot_model_->build(rm_cfg, &err)) {
        std::cerr << "[GaitCtrller] Error building Pinocchio RobotModel: " << err << std::endl;
    }

    for (int i = 0; i < 4; i++)
    {
        ctrlParam(i) = PIDParam[i];
    }
    _gamepadCommand.resize(4);

    // Reset state estimator container
    _stateEstimator->removeAllEstimators();
    _stateEstimator->addEstimator<ContactEstimator<float>>();
    Vec4<float> contactDefault;
    contactDefault << 0.5f, 0.5f, 0.5f, 0.5f;
    _stateEstimator->setContactPhase(contactDefault);

    // Sensor-based estimators
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

    SetStandingTestMode(wbic_enabled
                            ? wbic::StandingTestMode::TestCWbicLockBase
                            : wbic::StandingTestMode::TestALegacy);

    std::cout << "[GaitCtrller] Initialized controller (freq=" << freq_
              << " Hz, dt=" << dt_ << " s, wbic_enabled=" << _wbic_enabled << ")" << std::endl;
}

GaitCtrller::~GaitCtrller() = default;

void GaitCtrller::SetStandingTestMode(wbic::StandingTestMode mode) noexcept
{
    standing_test_mode_ = mode;
    _wbic_enabled = mode != wbic::StandingTestMode::TestALegacy;

    if (_wbicController) _wbicController->Reset();

    blend_progress_ = 0.0;
    diagnostic_csv_.close();
    diagnostic_csv_path_.clear();
}

void GaitCtrller::SetStandingDiagnosticCsvPath(const std::string& path)
{
    diagnostic_csv_.close();
    diagnostic_csv_path_ = path;
}

void GaitCtrller::SetForceRateWeight(double w) noexcept
{
    force_rate_weight_ = w;
    if (centroidal_wrench_ctrl_) {
        centroidal_wrench_ctrl_->SetForceRateWeight(w);
    }
}

void GaitCtrller::OpenDiagnosticCsvIfNeeded()
{
    if (diagnostic_csv_.is_open()) return;

    if (diagnostic_csv_path_.empty()) {
        diagnostic_csv_path_ =
            std::string("/tmp/lite3_standing_") + StandingTestModeName(standing_test_mode_) + ".csv";
    }

    const std::filesystem::path path(diagnostic_csv_path_);
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }

    diagnostic_csv_.open(diagnostic_csv_path_, std::ios::out | std::ios::trunc);
    if (!diagnostic_csv_) {
        std::cerr << "[StandingDiagnostic] Cannot open " << diagnostic_csv_path_ << std::endl;
        return;
    }

    diagnostic_csv_
        << "time,test_mode,roll,pitch,yaw,omega_x,omega_y,omega_z,"
        << "body_x,body_y,body_z,vx,vy,vz,"
        << "pitch_des,pitch_actual,pitch_error,orientation_error_y,x_ddot_ori_y,"
        << "qddot_cmd_z,qddot_cmd_pitch,"
        << "kin_qddot_cmd_angular_y,final_qddot_angular_y,"
        << "contact_FR,contact_FL,contact_HR,contact_HL,"
        << "contact_phase_FR,contact_phase_FL,contact_phase_HR,contact_phase_HL,"
        << "Fr_des_FR_x,Fr_des_FR_y,Fr_des_FR_z,"
        << "Fr_des_FL_x,Fr_des_FL_y,Fr_des_FL_z,"
        << "Fr_des_HR_x,Fr_des_HR_y,Fr_des_HR_z,"
        << "Fr_des_HL_x,Fr_des_HL_y,Fr_des_HL_z,"
        << "f_opt_FR_x,f_opt_FR_y,f_opt_FR_z,"
        << "f_opt_FL_x,f_opt_FL_y,f_opt_FL_z,"
        << "f_opt_HR_x,f_opt_HR_y,f_opt_HR_z,"
        << "f_opt_HL_x,f_opt_HL_y,f_opt_HL_z,"
        << "Fz_front,Fz_rear,Fz_total,Fz_f_opt,"
        << "delta_qddot_u_0,delta_qddot_u_1,delta_qddot_u_2,"
        << "delta_qddot_u_3,delta_qddot_u_4,delta_qddot_u_5,"
        << "norm_delta_qddot_u,kin_contact_residual,final_contact_acc_residual,"
        << "eom_residual,inequality_residual,torque_margin,"
        << "wbic_status,command_source,blend_progress,fallback_count,nWSR,"
        << "tau_ff_0,tau_ff_1,tau_ff_2,tau_ff_3,tau_ff_4,tau_ff_5,"
        << "tau_ff_6,tau_ff_7,tau_ff_8,tau_ff_9,tau_ff_10,tau_ff_11,"
        << "My,My_MPC,My_WBIC,My_f_opt,r_FR_x,r_FR_y,r_FR_z,r_FL_x,r_FL_y,r_FL_z,"
        << "r_HR_x,r_HR_y,r_HR_z,r_HL_x,r_HL_y,r_HL_z,"
        << "controller_mode,mass_used,com_x,com_y,com_z,payload_mass_used,payload_com_offset_x,"
        << "Fdes_x,Fdes_y,Fdes_z,Mdes_x,Mdes_y,Mdes_z,My_des,My_grf,GRF_QP_status,GRF_QP_cost,GRF_QP_solve_time_us,"
        << "Fz_gravity,Fz_pos_P,Fz_vel_D,Fz_des_total,Fz_des,My_P,My_D,My_ff,df_norm,df_norm_FR,df_norm_FL,df_norm_HR,df_norm_HL,"
        << "Fz_FR,Fz_FL,Fz_HR,Fz_HL,"
        << "z_error,vz_error,z_acc_P,z_acc_D,z_acc_ff,x_ddot_pos_z,"
        << "omega_y_error,pitch_acc_P,pitch_acc_D,pitch_acc_ff,"
        << "wbic_start_wall_ns,wbic_done_wall_ns";
    constexpr const char* leg_names[4] = {"FR", "FL", "HR", "HL"};
    constexpr const char* joint_names[2] = {"HipY", "Knee"};
    constexpr const char* joint_fields[9] = {
        "q_actual", "q_des", "q_error", "qd_actual", "qd_des",
        "tau_ff", "tau_PD", "tau_total", "torque_limit"
    };
    for (const char* leg_name : leg_names) {
        for (const char* joint_name : joint_names) {
            for (const char* field : joint_fields) {
                diagnostic_csv_ << ',' << leg_name << '_' << joint_name << '_' << field;
            }
        }
    }
    diagnostic_csv_ << '\n';
    diagnostic_csv_ << std::setprecision(17);
    std::cout << "[StandingDiagnostic] CSV: " << diagnostic_csv_path_ << std::endl;
}

void GaitCtrller::LogStandingDiagnostic(const wbic::WbicOutput& wbic_out,
                                        bool wbic_attempted,
                                        bool wbic_success,
                                        const double* selected_tau_ff,
                                        const wbic::JointHybridCommand* selected_command,
                                        uint64_t tick_sequence,
                                        uint64_t wbic_start_wall_ns)
{
    if (!convexMPC || !convexMPC->IsStanding()) return;
    OpenDiagnosticCsvIfNeeded();
    if (!diagnostic_csv_) return;

    const auto& se = _stateEstimator->getResult();
    const bool use_wbic_force = wbic_success
        && standing_test_mode_ != wbic::StandingTestMode::TestALegacy;

    Eigen::Matrix<double, 12, 1> selected_force =
        Eigen::Matrix<double, 12, 1>::Zero();
    for (int leg = 0; leg < 4; ++leg) {
        if (use_wbic_force) {
            selected_force.segment<3>(3 * leg) = wbic_out.f_opt.segment<3>(3 * leg);
        } else {
            selected_force.segment<3>(3 * leg) = convexMPC->Fr_des[leg].cast<double>();
        }
    }

    const double fz_front = selected_force[2] + selected_force[5];
    const double fz_rear = selected_force[8] + selected_force[11];
    double my = 0.0;
    double my_mpc = 0.0;
    double my_wbic = 0.0;
    double my_f_opt = 0.0;
    for (int leg = 0; leg < 4; ++leg) {
        const auto r = convexMPC->mpc_moment_arms[leg].cast<double>();
        const auto f = selected_force.segment<3>(3 * leg);
        my += r[2] * f[0] - r[0] * f[2];
        const auto f_mpc = convexMPC->Fr_des[leg].cast<double>();
        my_mpc += r[2] * f_mpc[0] - r[0] * f_mpc[2];
        if (wbic_attempted) {
            const auto f_wbic = wbic_out.f_opt.segment<3>(3 * leg);
            my_wbic += r[2] * f_wbic[0] - r[0] * f_wbic[2];
            const Eigen::Vector3d r_com =
                r + se.position.cast<double>() - last_centroidal_state_.com_world;
            my_f_opt += r_com[2] * f_wbic[0] - r_com[0] * f_wbic[2];
        }
    }

    const char* wbic_status = "DISABLED";
    if (standing_test_mode_ != wbic::StandingTestMode::TestALegacy) {
        wbic_status = WbicStatusName(wbic_out.status);
    }

    const char* command_source = "LEGACY_CMPC";
    if (!_safetyCheck) {
        command_source = "SAFE_ZERO";
    } else if (wbic_success) {
        command_source = "WBIC";
    } else if (_wbic_enabled) {
        command_source = "FALLBACK_LEGACY_CMPC";
    }

    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    const double pitch_des = convexMPC->pBody_RPY_des[1];
    const double pitch_actual = se.rpy[1];
    diagnostic_csv_ << tick_sequence * dt_ << ',' << StandingTestModeName(standing_test_mode_)
                    << ',' << se.rpy[0] << ',' << se.rpy[1] << ',' << se.rpy[2]
                    << ',' << se.omegaWorld[0] << ',' << se.omegaWorld[1] << ',' << se.omegaWorld[2]
                    << ',' << se.position[0] << ',' << se.position[1] << ',' << se.position[2]
                    << ',' << se.vWorld[0] << ',' << se.vWorld[1] << ',' << se.vWorld[2]
                    << ',' << pitch_des << ',' << pitch_actual << ',' << (pitch_des - pitch_actual)
                    << ',' << (wbic_attempted ? wbic_out.orientation_error[1] : unavailable)
                    << ',' << (wbic_attempted ? wbic_out.x_ddot_ori[1] : unavailable)
                    << ',' << (wbic_attempted ? wbic_out.qddot_cmd[2] : unavailable)
                    << ',' << (wbic_attempted ? wbic_out.qddot_cmd[4] : unavailable)
                    << ',' << (wbic_attempted ? wbic_out.qddot_cmd[4] : unavailable)
                    << ',' << (wbic_attempted ? wbic_out.qddot[4] : unavailable);

    for (int leg = 0; leg < 4; ++leg) diagnostic_csv_ << ',' << (convexMPC->planned_contact[leg] ? 1 : 0);
    for (int leg = 0; leg < 4; ++leg) diagnostic_csv_ << ',' << convexMPC->contact_state[leg];
    for (int leg = 0; leg < 4; ++leg) {
        for (int axis = 0; axis < 3; ++axis) diagnostic_csv_ << ',' << convexMPC->Fr_des[leg][axis];
    }
    for (int i = 0; i < 12; ++i) {
        diagnostic_csv_ << ',' << (wbic_attempted ? wbic_out.f_opt[i] : unavailable);
    }

    diagnostic_csv_ << ',' << fz_front << ',' << fz_rear << ',' << (fz_front + fz_rear)
                    << ',' << (fz_front + fz_rear);
    for (int i = 0; i < 6; ++i) {
        diagnostic_csv_ << ',' << (wbic_attempted ? wbic_out.delta_qddot_u[i] : unavailable);
    }
    diagnostic_csv_
        << ',' << (wbic_attempted ? wbic_out.residuals.delta_qddot_u_norm : unavailable)
        << ',' << (wbic_attempted ? wbic_out.residuals.kin_contact_acc_residual_norm : unavailable)
        << ',' << (wbic_attempted ? wbic_out.residuals.contact_acc_residual_norm : unavailable)
        << ',' << (wbic_attempted ? wbic_out.residuals.eom_residual_norm : unavailable)
        << ',' << (wbic_attempted ? wbic_out.residuals.inequality_violation_norm : unavailable)
        << ',' << (wbic_attempted ? wbic_out.residuals.torque_limit_margin : unavailable)
        << ',' << wbic_status << ',' << command_source << ',' << blend_progress_
        << ',' << wbic_fallback_count_ << ',' << (wbic_attempted ? wbic_out.n_wsr : 0);

    for (int i = 0; i < 12; ++i) diagnostic_csv_ << ',' << selected_tau_ff[i];
    diagnostic_csv_ << ',' << my << ',' << my_mpc
                    << ',' << (wbic_attempted ? my_wbic : unavailable)
                    << ',' << (wbic_attempted ? my_f_opt : unavailable);
    for (int leg = 0; leg < 4; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            diagnostic_csv_ << ',' << convexMPC->mpc_moment_arms[leg][axis];
        }
    }

    const char* controller_mode = "CONVEX_MPC_WBIC";
    if (standing_test_mode_ == wbic::StandingTestMode::TestALegacy) {
        controller_mode = "LEGACY_CMPC";
    } else if (use_centroidal_wrench_) {
        controller_mode = payload_config_.enabled ? "PAYLOAD_CENTROIDAL_WRENCH_WBIC" : "NOMINAL_CENTROIDAL_WRENCH_WBIC";
    }

    const double mass_used = last_centroidal_state_.mass;
    const double com_x = last_centroidal_state_.com_world.x();
    const double com_y = last_centroidal_state_.com_world.y();
    const double com_z = last_centroidal_state_.com_world.z();
    const double payload_mass_used = last_centroidal_state_.payload_mass;
    const double payload_com_offset_x = last_centroidal_state_.payload_offset_body.x();

    const double Fdes_x = last_centroidal_output_.force_des_world.x();
    const double Fdes_y = last_centroidal_output_.force_des_world.y();
    const double Fdes_z = last_centroidal_output_.force_des_world.z();

    const double Mdes_x = last_centroidal_output_.moment_des_world.x();
    const double Mdes_y = last_centroidal_output_.moment_des_world.y();
    const double Mdes_z = last_centroidal_output_.moment_des_world.z();

    const double my_des = last_centroidal_output_.moment_des_world.y();
    const double my_grf = last_centroidal_output_.moment_grf_world.y();

    const int grf_qp_status = last_centroidal_output_.qp_status;
    const double grf_qp_cost = last_centroidal_output_.qp_cost;
    const double grf_qp_solve_time = last_centroidal_output_.qp_solve_time_us;

    const double Fz_gravity = last_centroidal_output_.Fz_gravity;
    const double Fz_pos_P = last_centroidal_output_.Fz_pos_P;
    const double Fz_vel_D = last_centroidal_output_.Fz_vel_D;
    const double Fz_des_total = last_centroidal_output_.force_des_world.z();
    const double My_P = last_centroidal_output_.My_P;
    const double My_D = last_centroidal_output_.My_D;
    const double My_ff = last_centroidal_output_.My_ff;
    const double df_norm = last_centroidal_output_.df_norm;
    const double df_norm_FR = last_centroidal_output_.df_norm_leg[0];
    const double df_norm_FL = last_centroidal_output_.df_norm_leg[1];
    const double df_norm_HR = last_centroidal_output_.df_norm_leg[2];
    const double df_norm_HL = last_centroidal_output_.df_norm_leg[3];
    const double Fz_FR = last_centroidal_output_.grf_world[0].z();
    const double Fz_FL = last_centroidal_output_.grf_world[1].z();
    const double Fz_HR = last_centroidal_output_.grf_world[2].z();
    const double Fz_HL = last_centroidal_output_.grf_world[3].z();

    const wbic::WbicConfig wbic_config = _wbicController
        ? _wbicController->GetConfig()
        : wbic::WbicConfig{};
    const double z_error = convexMPC->pBody_des.z() - se.position.z();
    const double vz_error = convexMPC->vBody_des.z() - se.vWorld.z();
    const double z_acc_P = wbic_config.kp_body_pos * z_error;
    const double z_acc_D = wbic_config.kd_body_pos * vz_error;
    const double z_acc_ff = convexMPC->aBody_des.z();
    const double x_ddot_pos_z = z_acc_ff + z_acc_P + z_acc_D;

    const double omega_y_error = convexMPC->vBody_Ori_des.y() - se.omegaWorld.y();
    const double pitch_acc_P = wbic_config.kp_body_ori * wbic_out.orientation_error.y();
    const double pitch_acc_D = wbic_config.kd_body_ori * omega_y_error;
    const double pitch_acc_ff = 0.0;

    diagnostic_csv_
        << ',' << controller_mode << ',' << mass_used
        << ',' << com_x << ',' << com_y << ',' << com_z
        << ',' << payload_mass_used << ',' << payload_com_offset_x
        << ',' << Fdes_x << ',' << Fdes_y << ',' << Fdes_z
        << ',' << Mdes_x << ',' << Mdes_y << ',' << Mdes_z
        << ',' << my_des << ',' << my_grf
        << ',' << grf_qp_status << ',' << grf_qp_cost << ',' << grf_qp_solve_time
        << ',' << Fz_gravity << ',' << Fz_pos_P << ',' << Fz_vel_D << ',' << Fz_des_total
        << ',' << Fz_des_total
        << ',' << My_P << ',' << My_D << ',' << My_ff
        << ',' << df_norm << ',' << df_norm_FR << ',' << df_norm_FL << ',' << df_norm_HR << ',' << df_norm_HL
        << ',' << Fz_FR << ',' << Fz_FL << ',' << Fz_HR << ',' << Fz_HL
        << ',' << z_error << ',' << vz_error
        << ',' << z_acc_P << ',' << z_acc_D << ',' << z_acc_ff << ',' << x_ddot_pos_z
        << ',' << omega_y_error
        << ',' << pitch_acc_P << ',' << pitch_acc_D << ',' << pitch_acc_ff
        << ',' << wbic_start_wall_ns
        << ',' << std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
    for (int leg = 0; leg < 4; ++leg) {
        for (int joint = 1; joint <= 2; ++joint) {
            const int k = leg * 3 + joint;
            const double q_actual = joint == 1 ? _legdata.q_hip[leg] : _legdata.q_knee[leg];
            const double qd_actual = joint == 1 ? _legdata.qd_hip[leg] : _legdata.qd_knee[leg];
            const bool command_available = wbic_success && selected_command != nullptr;
            const double q_des = command_available ? selected_command->q_des[k] : unavailable;
            const double qd_des = command_available ? selected_command->qd_des[k] : unavailable;
            const double q_error = command_available ? q_des - q_actual : unavailable;
            const double tau_pd = command_available
                ? selected_command->kp[k] * q_error
                    + selected_command->kd[k] * (qd_des - qd_actual)
                : unavailable;
            const double tau_ff = selected_tau_ff[k];
            const double tau_total = command_available ? tau_ff + tau_pd : unavailable;
            const double torque_limit = joint == 2 ? 36.0 : 24.0;
            diagnostic_csv_ << ',' << q_actual << ',' << q_des << ',' << q_error
                            << ',' << qd_actual << ',' << qd_des << ',' << tau_ff
                            << ',' << tau_pd << ',' << tau_total << ',' << torque_limit;
        }
    }
    diagnostic_csv_ << '\n';
    if ((tick_sequence % 100U) == 0U) diagnostic_csv_.flush();
}

void GaitCtrller::Reset() noexcept
{
    if (_wbicController) {
        _wbicController->Reset();
    }
    if (centroidal_wrench_ctrl_) {
        centroidal_wrench_ctrl_->Reset();
    }
    last_centroidal_output_.reset();
    blend_progress_ = 0.0;
    seq_cnt_ = 0;
    last_wbic_status_ = wbic::WbicStatus::Ok;
    wbic_fallback_count_ = 0;
}

void GaitCtrller::SetIMUData(double *imuData)
{
    _vectorNavData.accelerometer(0, 0) = imuData[0];
    _vectorNavData.accelerometer(1, 0) = imuData[1];
    _vectorNavData.accelerometer(2, 0) = imuData[2];
    _vectorNavData.quat(0, 0) = imuData[3];
    _vectorNavData.quat(1, 0) = imuData[4];
    _vectorNavData.quat(2, 0) = imuData[5];
    _vectorNavData.quat(3, 0) = imuData[6];
    _vectorNavData.gyro(0, 0) = imuData[7];
    _vectorNavData.gyro(1, 0) = imuData[8];
    _vectorNavData.gyro(2, 0) = imuData[9];
}

void GaitCtrller::SetLegData(double *motorData)
{
    for (int i = 0; i < 4; i++)
    {
        _legdata.q_abad[i] = -motorData[i * 3]; // especially for Lite3 legacy CMPC
        _legdata.q_hip[i] = motorData[i * 3 + 1];
        _legdata.q_knee[i] = motorData[i * 3 + 2];

        _legdata.qd_abad[i] = -motorData[12 + i * 3]; // especially for Lite3 legacy CMPC
        _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
        _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];
    }
}

void GaitCtrller::PreWork(double *imuData, double *motorData)
{
    SetIMUData(imuData);
    SetLegData(motorData);

    // ★ Eliminate 1-tick delay: update leg data BEFORE state estimator runs
    _legController->updateData(&_legdata);
    _stateEstimator->run();
}

void GaitCtrller::SetGaitType(int gaitType)
{
    _gaitType = gaitType;
    std::cout << "set gait type to: " << _gaitType << std::endl;
}

void GaitCtrller::SetRobotMode(int mode)
{
    _robotMode = mode;
    std::cout << "set robot mode to: " << _robotMode << std::endl;
}

void GaitCtrller::SetRobotVel(double *vel)
{
    if (std::abs(vel[0]) < 0.03)
    {
        _gamepadCommand[0] = 0.0;
    }
    else
    {
        _gamepadCommand[0] = vel[0] * 1.0;
    }

    if (std::abs(vel[1]) < 0.03)
    {
        _gamepadCommand[1] = 0.0;
    }
    else
    {
        _gamepadCommand[1] = vel[1] * 1.0;
    }

    if (std::abs(vel[2]) < 0.03)
    {
        _gamepadCommand[2] = 0.0;
    }
    else
    {
        _gamepadCommand[2] = vel[2] * 1.0;
    }
}

void GaitCtrller::TorqueCalculator(double *imuData,
                                  double *motorData,
                                  double *effort,
                                  wbic::JointHybridCommand *hybridCmd)
{
    const uint64_t wbic_start_wall_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    Timer t_total;
    const uint64_t tick_sequence = seq_cnt_++;

    Timer t_est;
    PreWork(imuData, motorData);
    double t_est_ms = t_est.getMs();

    // Setup leg controller
    _legController->zeroCommand();
    _legController->setEnabled(true);

    // Desired state command
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);

    // Safety checks
    _safetyCheck = true;
    if (!safetyChecker->checkSafeOrientation(*_stateEstimator))
    {
        _safetyCheck = false;
        std::cout << "broken: Orientation Safety Check FAIL" << std::endl;
    }
    else if (!safetyChecker->checkPDesFoot(_quadruped, *_legController))
    {
        _safetyCheck = false;
        std::cout << "broken: Foot Position Safety Check FAIL" << std::endl;
    }
    else if (!safetyChecker->checkForceFeedForward(*_legController))
    {
        _safetyCheck = false;
        std::cout << "broken: Force FeedForward Safety Check FAIL" << std::endl;
    }
    else if (!safetyChecker->checkJointLimit(*_legController))
    {
        _safetyCheck = false;
        std::cout << "broken: Joint Limit Safety Check FAIL" << std::endl;
    }

    // Run Convex MPC
    Timer t_mpc;
    convexMPC->run(_quadruped,
                   *_legController,
                   *_stateEstimator,
                   *_desiredStateCommand,
                   _gamepadCommand,
                   _gaitType,
                   _robotMode);
    double t_mpc_ms = t_mpc.getMs();

    // Compute legacy CMPC commands for same-tick fallback readiness
    Timer t_leg;
    _legController->updateCommand(&legcommand, ctrlParam);
    double t_leg_ms = t_leg.getMs();

    double legacy_effort[12] = {};
    if (_safetyCheck)
    {
        for (int i = 0; i < 4; i++)
        {
            legacy_effort[i * 3 + 0] = -legcommand.tau_abad_ff[i]; // especially for Lite3 legacy
            legacy_effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
            legacy_effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    // WBIC Execution & Arbitration Pipeline
    // ══════════════════════════════════════════════════════════════════════
    bool wbic_success = false;
    bool wbic_attempted = false;
    wbic::WbicOutput wbic_out;

    if (_safetyCheck && _wbic_enabled && _wbicController && !_wbicController->IsLatched())
    {
        wbic_attempted = true;
        const auto& seResult = _stateEstimator->getResult();

        wbic::WbicInput wbic_in;
        // Base state (World frame)
        wbic_in.R_body = seResult.rBody.transpose().cast<double>();
        wbic_in.p_body = seResult.position.cast<double>();
        wbic_in.v_body_world = seResult.vWorld.cast<double>();
        wbic_in.omega_body_world = seResult.omegaWorld.cast<double>();

        // Actuated joints (order [FR, FL, HR, HL])
        for (int i = 0; i < 12; ++i)
        {
            wbic_in.q_joint_raw[i] = motorData[i];
            wbic_in.qd_joint_raw[i] = motorData[12 + i];
        }

        // Desired base trajectory
        wbic_in.p_body_des = convexMPC->pBody_des.cast<double>();
        wbic_in.v_body_des = convexMPC->vBody_des.cast<double>();
        wbic_in.a_body_des = convexMPC->aBody_des.cast<double>();

        // Desired body orientation: RPY from convexMPC
        const float roll_des = convexMPC->pBody_RPY_des[0];
        const float pitch_des = convexMPC->pBody_RPY_des[1];
        const float yaw_des = convexMPC->pBody_RPY_des[2];
        wbic_in.R_body_des = (Eigen::AngleAxisd(yaw_des, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(pitch_des, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(roll_des, Eigen::Vector3d::UnitX())).toRotationMatrix();
        wbic_in.omega_body_des = convexMPC->vBody_Ori_des.cast<double>();
        wbic_in.domega_body_des.setZero();

        // Desired feet & contact flags
        for (int leg = 0; leg < 4; ++leg)
        {
            wbic_in.p_foot_des[leg] = convexMPC->pFoot_des[leg].cast<double>();
            wbic_in.v_foot_des[leg] = convexMPC->vFoot_des[leg].cast<double>();
            wbic_in.a_foot_des[leg] = convexMPC->aFoot_des[leg].cast<double>();

            wbic_in.contact[leg] = convexMPC->planned_contact[leg];
            if (wbic_in.contact[leg])
            {
                wbic_in.Fr_des[leg] = convexMPC->Fr_des[leg].cast<double>();
            }
            else
            {
                wbic_in.Fr_des[leg].setZero();
            }
            wbic_in.phase[leg] = convexMPC->contact_state[leg];
        }

        wbic_in.standing_mode = convexMPC->IsStanding();
        wbic_in.payload_config = payload_config_;
        const bool full_stance = std::all_of(wbic_in.contact.begin(), wbic_in.contact.end(),
                                             [](bool contact) { return contact; });

        // Update Pinocchio robot model kinematics & centroidal state
        std::array<Eigen::Vector3d, 4> pin_foot_pos_world{};
        if (pinocchio_robot_model_)
        {
            const Eigen::Quaterniond q_wb(wbic_in.R_body);
            pinocchio_robot_model_->setState(wbic_in.p_body, q_wb, wbic_in.v_body_world, wbic_in.omega_body_world,
                                             wbic_in.q_joint_raw, wbic_in.qd_joint_raw);
            pinocchio_robot_model_->updateKinematics();

            for (int leg = 0; leg < 4; ++leg)
            {
                pin_foot_pos_world[leg] = pinocchio_robot_model_->footPos(leg);
            }

            last_centroidal_state_ = wbic::ComputeCentroidalState(
                pinocchio_robot_model_->mass(),
                pinocchio_robot_model_->comPos(),
                pinocchio_robot_model_->comVel(),
                wbic_in.p_body,
                wbic_in.R_body,
                wbic_in.v_body_world,
                wbic_in.omega_body_world,
                payload_config_);
        }

        // ══════════════════════════════════════════════════════════════════
        // Standing Routing: Centroidal Wrench Controller vs Convex MPC
        // ══════════════════════════════════════════════════════════════════
        const bool run_centroidal_wrench = wbic_in.standing_mode && full_stance && use_centroidal_wrench_;
        if (run_centroidal_wrench && centroidal_wrench_ctrl_ && pinocchio_robot_model_)
        {
            wbic::CentroidalWrenchInput c_in;
            c_in.mass = last_centroidal_state_.mass;
            c_in.com_world = last_centroidal_state_.com_world;
            c_in.com_des_world = wbic_in.p_body_des + wbic_in.R_body_des * last_centroidal_state_.com_offset_body;
            c_in.com_vel_world = last_centroidal_state_.com_vel_world;
            c_in.com_vel_des_world = wbic_in.v_body_des + wbic_in.omega_body_des.cross(wbic_in.R_body_des * last_centroidal_state_.com_offset_body);
            c_in.com_acc_des_world = wbic_in.a_body_des;
            c_in.R_world_body = wbic_in.R_body;
            c_in.R_des_world_body = wbic_in.R_body_des;
            c_in.omega_world = wbic_in.omega_body_world;
            c_in.omega_des_world = wbic_in.omega_body_des;
            c_in.foot_pos_world = pin_foot_pos_world;
            c_in.contact = wbic_in.contact;

            if (centroidal_wrench_ctrl_->Compute(c_in, &last_centroidal_output_) && last_centroidal_output_.valid)
            {
                for (int leg = 0; leg < 4; ++leg)
                {
                    wbic_in.Fr_des[leg] = last_centroidal_output_.grf_world[leg];
                }
            }
        }
        else
        {
            last_centroidal_output_.reset();
        }

        wbic_in.timestamp = tick_sequence * dt_;
        wbic_in.sequence = tick_sequence;

        const wbic::WbicStatus status = _wbicController->Run(wbic_in, &wbic_out);
        last_wbic_status_ = status;
        wbic_success = (status == wbic::WbicStatus::Ok || status == wbic::WbicStatus::QpMaxIter);

        if (full_stance && (wbic_in.sequence % 100U) == 0U)
        {
            const bool lock = wbic_in.standing_mode && full_stance;
            std::printf(
                "[WBC_STANCE] lock=%d | kin_contact=%.6e | final_contact=%.6e | "
                "delta_qddot_u=%.6e | Fz=[%.3f %.3f %.3f %.3f] | "
                "rpy=[%.4f %.4f] | p=[%.4f %.4f %.4f] | v=[%.4f %.4f %.4f]\n",
                lock ? 1 : 0,
                wbic_out.residuals.kin_contact_acc_residual_norm,
                wbic_out.residuals.contact_acc_residual_norm,
                wbic_out.residuals.delta_qddot_u_norm,
                wbic_out.f_opt[2], wbic_out.f_opt[5], wbic_out.f_opt[8], wbic_out.f_opt[11],
                seResult.rpy[0], seResult.rpy[1],
                seResult.position[0], seResult.position[1], seResult.position[2],
                seResult.vWorld[0], seResult.vWorld[1], seResult.vWorld[2]);
        }
    }
    else if (_wbic_enabled && _wbicController && _wbicController->IsLatched())
    {
        wbic_out.status = wbic::WbicStatus::SafetyViolation;
        wbic_out.command_source = wbic::CommandSource::FallbackLegacyCmpc;
        last_wbic_status_ = wbic_out.status;
    }

    // ══════════════════════════════════════════════════════════════════════
    // Output Selection, Blending and Fallback
    // ══════════════════════════════════════════════════════════════════════
    if (!_safetyCheck)
    {
        for (int i = 0; i < 12; ++i)
        {
            effort[i] = 0.0;
        }
        if (hybridCmd != nullptr)
        {
            hybridCmd->setZero();
        }
    }
    else if (wbic_success)
    {
        // 100 ms blend ramp
        blend_progress_ = std::min(1.0, blend_progress_ + dt_ / blend_time_);

        if (hybridCmd != nullptr)
        {
            wbic::WbicController::PopulateHybridCommand(wbic_out, hybridCmd);
            for (int k = 0; k < 12; ++k)
            {
                hybridCmd->tau_ff[k] =
                    (1.0 - blend_progress_) * legacy_effort[k] + blend_progress_ * wbic_out.tau_ff[k];
            }
        }

        for (int k = 0; k < 12; ++k)
        {
            effort[k] =
                (1.0 - blend_progress_) * legacy_effort[k] + blend_progress_ * wbic_out.tau_ff[k];
        }
    }
    else
    {
        // Same-tick instant fallback to Legacy CMPC
        if (_wbic_enabled) {
            ++wbic_fallback_count_;
        }
        blend_progress_ = 0.0;

        if (hybridCmd != nullptr)
        {
            hybridCmd->setZero();
            for (int k = 0; k < 12; ++k)
            {
                hybridCmd->tau_ff[k] = legacy_effort[k];
            }
        }

        for (int k = 0; k < 12; ++k)
        {
            effort[k] = legacy_effort[k];
        }
    }

    if (convexMPC->IsStanding())
    {
        LogStandingDiagnostic(wbic_out, wbic_attempted, wbic_success, effort, hybridCmd,
                              tick_sequence, wbic_start_wall_ns);
    }

    double t_total_ms = t_total.getMs();
    static int log_cnt = 0;
    if ((log_cnt++ % 100) == 0)
    {
        printf("[time] est: %.3f ms | mpc+swing: %.3f ms | legctrl: %.3f ms | total: %.3f ms | wbic: %s\n",
               t_est_ms, t_mpc_ms, t_leg_ms, t_total_ms, wbic_success ? "ON" : "OFF");
    }
}
