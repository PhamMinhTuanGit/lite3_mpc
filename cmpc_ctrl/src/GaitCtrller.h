#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Controllers/ControlFSMData.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/SafetyChecker.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/MiniCheetah.h"
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "StateEstimator/ContactEstimator.h"
#include "StateEstimator/OrientationEstimator.h"
#include "StateEstimator/PositionVelocityEstimator.h"
#include "Utilities/IMUTypes.h"
#include "WbcType.h"
#include "WbicController.hpp"
#include "CentroidalModel.hpp"
#include "CentroidalWrenchController.hpp"
#include "calculateTool.h"

class RobotModel;

struct JointEff
{
    double eff[12];
};

class GaitCtrller
{
public:
    GaitCtrller(double freq, double *PIDParam, bool wbic_enabled = true);
    ~GaitCtrller();

    void SetIMUData(double *imuData);
    void SetLegData(double *motorData);
    void PreWork(double *imuData, double *motorData);
    void SetGaitType(int gaitType);
    void SetRobotMode(int mode);
    void SetRobotVel(double *vel);

    void SetWbicEnabled(bool enabled) noexcept { _wbic_enabled = enabled; }
    void SetStandingTestMode(wbic::StandingTestMode mode) noexcept;
    wbic::StandingTestMode GetStandingTestMode() const noexcept { return standing_test_mode_; }
    void SetStandingDiagnosticCsvPath(const std::string& path);
    const std::string& StandingDiagnosticCsvPath() const noexcept { return diagnostic_csv_path_; }
    bool IsWbicEnabled() const noexcept { return _wbic_enabled; }
    wbic::WbicStatus LastWbicStatus() const noexcept { return last_wbic_status_; }
    uint64_t WbicFallbackCount() const noexcept { return wbic_fallback_count_; }

    void SetUseCentroidalWrench(bool enable) noexcept { use_centroidal_wrench_ = enable; }
    bool IsUseCentroidalWrench() const noexcept { return use_centroidal_wrench_; }
    void SetPayloadConfig(const wbic::PayloadConfig& cfg) noexcept { payload_config_ = cfg; }
    const wbic::PayloadConfig& GetPayloadConfig() const noexcept { return payload_config_; }
    void SetForceRateWeight(double w) noexcept;
    double GetForceRateWeight() const noexcept { return force_rate_weight_; }

    void Reset() noexcept;

    void TorqueCalculator(double *imuData,
                          double *motorData,
                          double *effort,
                          wbic::JointHybridCommand *hybridCmd = nullptr);

private:
    int _gaitType = 0;
    int _robotMode = 0;
    bool _safetyCheck = true;
    bool _wbic_enabled = true;
    wbic::StandingTestMode standing_test_mode_ = wbic::StandingTestMode::TestCWbicLockBase;
    std::vector<double> _gamepadCommand;
    Vec4<float> ctrlParam;

    // Member declaration order: independent parameters and containers first
    VectorNavData _vectorNavData;
    std::unique_ptr<CheaterState<double>> cheaterState;
    StateEstimate<float> _stateEstimate;
    std::unique_ptr<RobotControlParameters> controlParameters;

    Quadruped<float> _quadruped;
    FloatingBaseModel<float> _model;
    std::unique_ptr<LegController<float>> _legController;
    std::unique_ptr<StateEstimatorContainer<float>> _stateEstimator;
    std::unique_ptr<ConvexMPCLocomotion> convexMPC;
    std::unique_ptr<DesiredStateCommand<float>> _desiredStateCommand;
    std::unique_ptr<SafetyChecker<float>> safetyChecker;
    std::unique_ptr<wbic::WbicController> _wbicController;
    std::unique_ptr<wbic::CentroidalWrenchController> centroidal_wrench_ctrl_;
    std::unique_ptr<RobotModel> pinocchio_robot_model_;

    wbic::PayloadConfig payload_config_{};
    bool use_centroidal_wrench_ = false;
    double force_rate_weight_ = 0.0;
    wbic::CentroidalWrenchOutput last_centroidal_output_{};
    wbic::CentroidalState last_centroidal_state_{};

    LegData _legdata;
    LegCommand legcommand;
    ControlFSMData<float> control_data;

    // Timing and blending
    double dt_ = 0.002;
    double freq_ = 500.0;
    uint64_t seq_cnt_ = 0;
    double blend_time_ = 0.1;      // 100 ms blend
    double blend_progress_ = 0.0;  // 0.0 -> 1.0
    wbic::WbicStatus last_wbic_status_ = wbic::WbicStatus::Ok;
    uint64_t wbic_fallback_count_ = 0;

    std::ofstream diagnostic_csv_;
    std::string diagnostic_csv_path_;
    void OpenDiagnosticCsvIfNeeded();
    void LogStandingDiagnostic(const wbic::WbicOutput& wbic_out,
                               bool wbic_attempted,
                               bool wbic_success,
                               const double* selected_tau_ff,
                               const wbic::JointHybridCommand* selected_command,
                               uint64_t tick_sequence,
                               uint64_t wbic_start_wall_ns);
};

#endif
