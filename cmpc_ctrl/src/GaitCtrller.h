#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <math.h>
#include <time.h>

#include <iostream>
#include <memory>
#include <string>

#include "Controllers/ContactEstimator.h"
#include "Controllers/GeneralizedMomentumObserver.h"
#include "Controllers/GroundReactionForceEstimator.h"
#include "Controllers/ControlFSMData.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/SafetyChecker.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/MiniCheetah.h"          // original (no payload)
// #include "Dynamics/MiniCheetahWithPayload.h"  // with 2 kg payload
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "MPC_Ctrl/CmpcTelemetry.h"
#include "Utilities/IMUTypes.h"
#include "Lite3StateMapper.hpp"
#include "calculateTool.h"

struct JointEff
{
    double eff[12];
};

class GaitCtrller
{
public:
    GaitCtrller(double freq, double *PIDParam, const GMOConfig& gmoConfig = GMOConfig{});
    ~GaitCtrller();
    void SetIMUData(double *imuData);
    void SetLegData(double *motorData);
    void PreWork(double *imuData, double *motorData);
    void SetGaitType(int gaitType);
    void SetRobotMode(int mode);
    void SetRobotVel(double *vel);
    void TorqueCalculator(double *imuData, double *motorData, double *effort, CmpcTelemetryData *telem = nullptr);

private:
    int _gaitType = 0;
    int _robotMode = 0;
    bool _safetyCheck = true;
    std::vector<double> _gamepadCommand;
    Vec4<float> ctrlParam;

    Quadruped<float> _quadruped;
    FloatingBaseModel<float> _model;
    std::unique_ptr<ConvexMPCLocomotion> convexMPC;
    std::unique_ptr<LegController<float>> _legController;
    std::unique_ptr<StateEstimatorContainer<float>> _stateEstimator;
    LegData _legdata;
    LegCommand legcommand;
    ControlFSMData<float> control_data;
    VectorNavData _vectorNavData;
    std::unique_ptr<CheaterState<double>> cheaterState;
    StateEstimate<float> _stateEstimate;
    std::unique_ptr<RobotControlParameters> controlParameters;
    std::unique_ptr<DesiredStateCommand<float>> _desiredStateCommand;
    std::unique_ptr<SafetyChecker<float>> safetyChecker;
    GMOConfig _gmoConfig;
    Lite3Dynamics _pinocchioDynamics;
    Lite3StateMapper _stateMapper;
    Lite3MappedState _mappedState;
    std::unique_ptr<GeneralizedMomentumObserver> _gmo;
    std::unique_ptr<GroundReactionForceEstimator> _grfEstimator;
    GMOResult _gmoResult;
    GRFResult _grfResult;
};

extern "C"
{

    GaitCtrller *gCtrller = NULL;
    JointEff jointEff;

    // first step, init the controller
    void init_controller_with_gmo(
        double freq,
        double PIDParam[],
        int enabled,
        double gain,
        double forceDamping)
    {
        if (NULL != gCtrller)
        {
            delete gCtrller;
        }
        GMOConfig config;
        config.enabled = enabled != 0;
        config.gain = gain;
        config.force_damping = forceDamping;
        gCtrller = new GaitCtrller(freq, PIDParam, config);
    }

    void init_controller(double freq, double PIDParam[])
    {
        init_controller_with_gmo(freq, PIDParam, 1, 30.0, 1e-4);
    }

    // the kalman filter need to work second
    void pre_work(double imuData[], double legData[])
    {
        gCtrller->PreWork(imuData, legData);
    }

    // gait type can be set in any time
    void set_gait_type(int gaitType)
    {
        gCtrller->SetGaitType(gaitType);
    }

    // set robot mode, 0: High performance model, 1: Low power mode
    void set_robot_mode(int mode)
    {
        gCtrller->SetRobotMode(mode);
    }

    // robot vel can be set in any time
    void set_robot_vel(double vel[])
    {
        gCtrller->SetRobotVel(vel);
    }

    // after init controller and pre work, the mpc calculator can work
    JointEff *torque_calculator(double imuData[], double motorData[])
    {
        double eff[12] = {0.0};
        gCtrller->TorqueCalculator(imuData, motorData, eff, nullptr);
        for (int i = 0; i < 12; i++)
        {
            jointEff.eff[i] = eff[i];
        }
        return &jointEff;
    }

    JointEff *torque_calculator_with_telem(double imuData[], double motorData[], CmpcTelemetryData *telem)
    {
        double eff[12] = {0.0};
        gCtrller->TorqueCalculator(imuData, motorData, eff, telem);
        for (int i = 0; i < 12; i++)
        {
            jointEff.eff[i] = eff[i];
        }
        return &jointEff;
    }
}

#endif
