#include "GaitCtrller.h"
#include "Utilities/Timer.h"

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
      _wbicController{std::make_unique<wbic::WbicController>()}
{
    controlParameters->controller_dt = 1.0 / freq;

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

    std::cout << "[GaitCtrller] Initialized controller (freq=" << freq_
              << " Hz, dt=" << dt_ << " s, wbic_enabled=" << _wbic_enabled << ")" << std::endl;
}

GaitCtrller::~GaitCtrller() = default;

void GaitCtrller::Reset() noexcept
{
    if (_wbicController) {
        _wbicController->Reset();
    }
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
    Timer t_total;

    Timer t_est;
    PreWork(imuData, motorData);
    double t_est_ms = t_est.getMs();

    // Setup leg controller
    _legController->zeroCommand();
    _legController->setEnabled(true);

    // Desired state command
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);

    // Safety checks
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
    wbic::WbicOutput wbic_out;

    if (_safetyCheck && _wbic_enabled && _wbicController && !_wbicController->IsLatched())
    {
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

        wbic_in.timestamp = seq_cnt_ * dt_;
        wbic_in.sequence = seq_cnt_++;

        const wbic::WbicStatus status = _wbicController->Run(wbic_in, &wbic_out);
        last_wbic_status_ = status;
        wbic_success = (status == wbic::WbicStatus::Ok || status == wbic::WbicStatus::QpMaxIter);
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

    double t_total_ms = t_total.getMs();
    static int log_cnt = 0;
    if ((log_cnt++ % 100) == 0)
    {
        printf("[time] est: %.3f ms | mpc+swing: %.3f ms | legctrl: %.3f ms | total: %.3f ms | wbic: %s\n",
               t_est_ms, t_mpc_ms, t_leg_ms, t_total_ms, wbic_success ? "ON" : "OFF");
    }
}
