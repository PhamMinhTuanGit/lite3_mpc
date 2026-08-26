#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <cmath>
#include <ctime>
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
#include "calculateTool.h"

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
    bool IsWbicEnabled() const noexcept { return _wbic_enabled; }
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

    LegData _legdata;
    LegCommand legcommand;
    ControlFSMData<float> control_data;

    // Timing and blending
    double dt_ = 0.002;
    double freq_ = 500.0;
    uint64_t seq_cnt_ = 0;
    double blend_time_ = 0.1;      // 100 ms blend
    double blend_progress_ = 0.0;  // 0.0 -> 1.0
};

#endif
