#include "GaitCtrller.h"
#include "Utilities/Timer.h"

#include <algorithm>

GaitCtrller::GaitCtrller(
    double freq,
    double *PIDParam,
    const GMOConfig& gmoConfig)
    : _quadruped{buildMiniCheetah<float>()}, _model{_quadruped.buildModel()},        // original (no payload)
	    // : _quadruped{buildMiniCheetahWithPayload<float>()}, _model{_quadruped.buildModel()}, // with 2 kg payload
      // iterationsBetweenMPC = 28: chu kỳ bước ~0.784s (stance=swing~0.392s),
      // bước thưa hơn, tránh dậm chân liên tục gần nhau.
      convexMPC{std::make_unique<ConvexMPCLocomotion>(1.0 / freq, 30)},
      _legController{std::make_unique<LegController<float>>(_quadruped)},
      _stateEstimator{std::make_unique<StateEstimatorContainer<float>>(cheaterState.get(),
                                                                       &_vectorNavData,
                                                                       _legController->datas,
                                                                       &_stateEstimate,
                                                                       controlParameters.get())},
      _desiredStateCommand{std::make_unique<DesiredStateCommand<float>>(1.0 / freq)},
      safetyChecker{std::make_unique<SafetyChecker<float>>()},
      _gmoConfig{gmoConfig}
{
    _gmo = std::make_unique<GeneralizedMomentumObserver>(
        _pinocchioDynamics, 1.0 / freq, _gmoConfig);
    _grfEstimator = std::make_unique<GroundReactionForceEstimator>(
        _pinocchioDynamics, 1.0 / freq, _gmoConfig);
    for (int i = 0; i < 4; i++)
    {
        ctrlParam(i) = PIDParam[i];
    }
    _gamepadCommand.resize(4);

    // reset the state estimator
    _stateEstimator->removeAllEstimators();
    _stateEstimator->addEstimator<ContactEstimator<float>>();
    Vec4<float> contactDefault;
    contactDefault << 0.5, 0.5, 0.5, 0.5;
    _stateEstimator->setContactPhase(contactDefault);

    // dùng sensor-based estimation
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();

    std::cout << "finish init controller" << std::endl;
}

GaitCtrller::~GaitCtrller()
{
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
        _legdata.q_abad[i] = -motorData[i * 3]; // especially for Lite3
        _legdata.q_hip[i] = motorData[i * 3 + 1];
        _legdata.q_knee[i] = motorData[i * 3 + 2];

        _legdata.qd_abad[i] = -motorData[12 + i * 3]; // especially for Lite3
        _legdata.qd_hip[i] = motorData[12 + i * 3 + 1];
        _legdata.qd_knee[i] = motorData[12 + i * 3 + 2];

        _legdata.tau_abad[i] = -motorData[24 + i * 3]; // especially for Lite3
        _legdata.tau_hip[i] = motorData[24 + i * 3 + 1];
        _legdata.tau_knee[i] = motorData[24 + i * 3 + 2];
    }
}

void GaitCtrller::PreWork(double *imuData, double *motorData)
{
    SetIMUData(imuData);
    SetLegData(motorData);
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
    if (abs(vel[0]) < 0.03)
    {
        _gamepadCommand[0] = 0.0;
    }
    else
    {
        _gamepadCommand[0] = vel[0] * 1.0;
    }

    if (abs(vel[1]) < 0.03)
    {
        _gamepadCommand[1] = 0.0;
    }
    else
    {
        _gamepadCommand[1] = vel[1] * 1.0;
    }

    if (abs(vel[2]) < 0.03)
    {
        _gamepadCommand[2] = 0.0;
    }
    else
    {
        _gamepadCommand[2] = vel[2] * 1.0;
    }
}

void GaitCtrller::SetObservationTimestamp(double timestampSeconds)
{
    _observationTimestamp = timestampSeconds;
}

void GaitCtrller::TorqueCalculator(double *imuData, double *motorData, double *effort, CmpcTelemetryData *telem)
{
    Timer t_total;

    Timer t_est;
    PreWork(imuData, motorData);
    double t_est_ms = t_est.getMs(); // state estimation + leg data update

    double t_gmo_ms = 0.0;
    double t_grf_ms = 0.0;
    if (_gmoConfig.enabled)
    {
        if (_observationTimestamp >= 0.0 && _lastObservationTimestamp >= 0.0)
        {
            const double sample_gap =
                _observationTimestamp - _lastObservationTimestamp;
            if (!(sample_gap > 0.0)
                || sample_gap > _gmoConfig.max_sample_gap_s)
            {
                _gmo->reset(GMOInvalidReason::SampleDiscontinuity);
                _grfEstimator->reset();
            }
        }
        if (_observationTimestamp >= 0.0)
        {
            _lastObservationTimestamp = _observationTimestamp;
        }
        try
        {
            const Lite3StateMapStatus map_status = _stateMapper.map(
                _stateEstimator->getResult(), _legController->datas,
                _pinocchioDynamics, _mappedState);
            if (map_status == Lite3StateMapStatus::Ok)
            {
                Timer t_gmo;
                _gmoResult = _gmo->update(
                    _mappedState.q, _mappedState.v, _mappedState.motor_torque);
                t_gmo_ms = t_gmo.getMs();

                if (_gmoResult.valid)
                {
                    Timer t_grf;
                    _grfResult = _grfEstimator->update(
                        _mappedState.q, _gmoResult.residual,
                        _gmoResult.ready);
                    t_grf_ms = t_grf.getMs();
                }
                else
                {
                    _grfEstimator->reset();
                    _grfResult = GRFResult{};
                }
            }
            else
            {
                _gmo->reset(GMOInvalidReason::NonFiniteInput);
                _grfEstimator->reset();
                _gmoResult = GMOResult{};
                _grfResult = GRFResult{};
            }
        }
        catch (const std::exception&)
        {
            _gmo->reset(GMOInvalidReason::DynamicsFailure);
            _grfEstimator->reset();
            _gmoResult = GMOResult{};
            _grfResult = GRFResult{};
        }
    }

    const float evidence_timing_ms =
        static_cast<float>(t_gmo_ms + t_grf_ms);
    _evidenceTimingMs[_evidenceTimingCount % _evidenceTimingMs.size()] =
        evidence_timing_ms;
    ++_evidenceTimingCount;
    if ((_evidenceTimingCount % _evidenceTimingMs.size()) == 0)
    {
        auto sorted_timing = _evidenceTimingMs;
        const std::size_t p99_index =
            static_cast<std::size_t>(0.99 * (sorted_timing.size() - 1));
        std::nth_element(
            sorted_timing.begin(), sorted_timing.begin() + p99_index,
            sorted_timing.end());
        _evidenceTimingP99Ms = sorted_timing[p99_index];
    }

    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);

    //safety check
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

    float fz_gmo[4];
    bool gmo_valid[4];
    for (int i = 0; i < 4; ++i)
    {
        fz_gmo[i] = (float)_grfResult.force_world[i].z();
        gmo_valid[i] = _grfResult.ready && (_grfResult.leg_valid[i] != 0);
    }
    convexMPC->setContactEvidence(fz_gmo, gmo_valid);

    Timer t_mpc;
    convexMPC->run(_quadruped,
                   *_legController,
                   *_stateEstimator,
                   *_desiredStateCommand,
                   _gamepadCommand,
                   _gaitType,
                   _robotMode);
    double t_mpc_ms = t_mpc.getMs(); // swing planning + MPC (solve only when scheduled)

    Timer t_leg;
    _legController->updateCommand(&legcommand, ctrlParam);
    double t_leg_ms = t_leg.getMs(); // leg torque computation

    double t_total_ms = t_total.getMs();

    if (telem != nullptr)
    {
        telem->cmd_vx = (float)_gamepadCommand[0];
        telem->cmd_vy = (float)_gamepadCommand[1];
        telem->cmd_yaw_rate = (float)_gamepadCommand[2];
        telem->des_vx = convexMPC->getDesVx();
        telem->des_vy = convexMPC->getDesVy();
        telem->des_yaw_rate = convexMPC->getDesYawRate();

        const auto &se = _stateEstimator->getResult();
        for (int i = 0; i < 3; i++)
        {
            telem->kf_pos[i] = se.position[i];
            telem->kf_vel_world[i] = se.vWorld[i];
            telem->kf_vel_body[i] = se.vBody[i];
            telem->kf_rpy[i] = se.rpy[i];
            telem->kf_omega_body[i] = se.omegaBody[i];
            telem->kf_acc_body[i] = se.aBody[i];
        }
        for (int i = 0; i < 4; i++)
        {
            telem->kf_contact_prob[i] = se.contactEstimate[i];
        }

        const auto &comEst = convexMPC->getCoMEstimator();
        telem->est_mass_raw = comEst.getEstimatedMassRaw();
        telem->est_mass_filtered = comEst.getEstimatedMass();
        for (int i = 0; i < 3; i++)
        {
            telem->est_com_body[i] = comEst.getCoMOffset()[i];
        }
        telem->total_support_force_z = comEst.getTotalSupportForce();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                telem->p_feet_des_world[i][j] = convexMPC->pFoot_des[i][j];
                telem->p_feet_actual_body[i][j] = _legController->datas[i].p[j];
                telem->f_mpc_des_world[i][j] = convexMPC->Fr_des[i][j];
                telem->f_act_est_world[i][j] = _legController->commands[i].forceFeedForward[j];
            }
        }

        telem->gmo_valid = static_cast<uint8_t>(_gmoResult.valid);
        telem->gmo_initialized = static_cast<uint8_t>(_gmoResult.initialized);
        telem->gmo_ready = static_cast<uint8_t>(_gmoResult.ready);
        telem->gmo_invalid_reason =
            static_cast<uint8_t>(_gmoResult.invalid_reason);
        telem->gmo_samples_since_reset = _gmoResult.samples_since_reset;
        telem->gmo_reset_count = _gmoResult.reset_count;
        for (int i = 0; i < Lite3Dynamics::kNv; ++i)
        {
            telem->gmo_momentum[i] = static_cast<float>(_gmoResult.momentum[i]);
            telem->gmo_momentum_hat[i] =
                static_cast<float>(_gmoResult.momentum_hat[i]);
            telem->gmo_residual[i] = static_cast<float>(_gmoResult.residual[i]);
        }
        telem->grf_valid = static_cast<uint8_t>(_grfResult.valid);
        telem->grf_ready = static_cast<uint8_t>(_grfResult.ready);
        telem->grf_invalid_reason =
            static_cast<uint8_t>(_grfResult.invalid_reason);
        for (int leg = 0; leg < Lite3Dynamics::kNumLegs; ++leg)
        {
            telem->grf_leg_valid[leg] = _grfResult.leg_valid[leg];
            telem->grf_jacobian_quality[leg] =
                static_cast<float>(_grfResult.jacobian_quality[leg]);
            for (int axis = 0; axis < 3; ++axis)
            {
                telem->gmo_force_raw_world[leg][axis] =
                    static_cast<float>(_grfResult.raw_force_world[leg][axis]);
                telem->gmo_force_world[leg][axis] =
                    static_cast<float>(_grfResult.force_world[leg][axis]);
            }
            telem->gmo_force_norm[leg] =
                static_cast<float>(_grfResult.force_world[leg].norm());
            telem->gmo_fz[leg] =
                static_cast<float>(_grfResult.force_world[leg].z());
            telem->scheduled_contact[leg] = convexMPC->contact_state[leg];
            telem->gmo_contact_binary[leg] = convexMPC->getGmoContactBinary(leg);
            telem->lost_contact_time_ms[leg] = convexMPC->getLostContactTimeMs(leg);
            telem->stance_force_scale[leg] = convexMPC->getStanceForceScale(leg);
            telem->q_abad[leg] = static_cast<float>(_legdata.q_abad[leg]);
            telem->tau_abad_cmd[leg] = static_cast<float>(legcommand.tau_abad_ff[leg]);
        }

        telem->t_est_ms = (float)t_est_ms;
        telem->t_gmo_ms = (float)t_gmo_ms;
        telem->t_grf_ms = (float)t_grf_ms;
        telem->t_evidence_p99_ms = _evidenceTimingP99Ms;
        telem->t_mpc_ms = (float)t_mpc_ms;
        telem->t_total_ms = (float)t_total_ms;
    }
    static int log_cnt = 0;
    if ((log_cnt++ % 100) == 0) // in mỗi 100 tick để tránh spam
    {
        printf("[time] est: %.3f ms | mpc+swing: %.3f ms | legctrl: %.3f ms | total: %.3f ms\n",
               t_est_ms, t_mpc_ms, t_leg_ms, t_total_ms);

        // Ab/ad diagnostic in the convention actually sent to MuJoCo.
        // Leg order here is [FR, FL, HR, HL]. An inward-closing command has
        // tau_sim signs [-, +, -, +] for this model/order.
        printf("[abad] order=FR,FL,HR,HL q_sim="
               "[%+.3f,%+.3f,%+.3f,%+.3f] tau_sim="
               "[%+.3f,%+.3f,%+.3f,%+.3f]\n",
               -_legController->datas[0].q(0),
               -_legController->datas[1].q(0),
               -_legController->datas[2].q(0),
               -_legController->datas[3].q(0),
               -legcommand.tau_abad_ff[0],
               -legcommand.tau_abad_ff[1],
               -legcommand.tau_abad_ff[2],
               -legcommand.tau_abad_ff[3]);

        printf("[abad-src] fy_mpc="
               "[%+.2f,%+.2f,%+.2f,%+.2f] fz_mpc="
               "[%+.2f,%+.2f,%+.2f,%+.2f] py_err="
               "[%+.4f,%+.4f,%+.4f,%+.4f]\n",
               _legController->commands[0].forceFeedForward(1),
               _legController->commands[1].forceFeedForward(1),
               _legController->commands[2].forceFeedForward(1),
               _legController->commands[3].forceFeedForward(1),
               _legController->commands[0].forceFeedForward(2),
               _legController->commands[1].forceFeedForward(2),
               _legController->commands[2].forceFeedForward(2),
               _legController->commands[3].forceFeedForward(2),
               _legController->commands[0].pDes(1) - _legController->datas[0].p(1),
               _legController->commands[1].pDes(1) - _legController->datas[1].p(1),
               _legController->commands[2].pDes(1) - _legController->datas[2].p(1),
               _legController->commands[3].pDes(1) - _legController->datas[3].p(1));
    }

    if (_safetyCheck)
    {
        for (int i = 0; i < 4; i++)
        {
            effort[i * 3] = -legcommand.tau_abad_ff[i]; // especially for Lite3
            effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
            effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
            // std::cout << "effort: " << i * 3 << " "<< effort[i * 3] << std::endl;
            // std::cout << "effort: " << i * 3 + 1 << " "<< effort[i * 3 + 1] << std::endl;
            // std::cout << "effort: " << i * 3 + 2 << " "<< effort[i * 3 + 2] << std::endl;
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            effort[i * 3] = 0.0;
            effort[i * 3 + 1] = 0.0;
            effort[i * 3 + 2] = 0.0;
        }
    }

    // return effort;
}
