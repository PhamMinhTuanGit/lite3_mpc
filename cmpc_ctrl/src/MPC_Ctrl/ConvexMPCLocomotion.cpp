#include "ConvexMPCLocomotion.h"

#include <iostream>

#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"
#include "convexMPC_interface.h"
#include "RobotConfig.h"
// #include "../../../../common/FootstepPlanner/GraphSearch.h"

// #include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

////////////////////
// Controller
// Refer to Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive
// Control One gait cycle consists of horizonLength(10) MPC cycles. The gait is processed at 1kHz.

// Tổng thời gian một chu kì (swing -> stance)
// The MPC counting interval is about 30. (được cài tại `convexMPC{std::make_unique<ConvexMPCLocomotion>(1.0 / freq, 30)},`)
// It counts once every millisecond to control the frequency.
// That is, one MPC cycle is 30ms. Therefore, the gait cycle is 10*30 = 300ms

// dt = 1.0 / 1000 = 0.001s = 1ms    ← control loop chạy ở 1kHz
// iterationsBetweenMPC = 30         ← MPC chạy mỗi 30 iterations
// horizonLength = 10                ← MPC nhìn trước 10 bước

// Chi tiết:
// iterationsBetweenMPC = 30:
//   → MPC chỉ được gọi MỖI 30 ticks
//   → Tức là MPC chạy mỗi 30 × 1ms = 30ms

// Chỉ số cụ thể:
// dt = 1ms
// (1kHz loop)
//     │
//     × iterationsBetweenMPC (30)
//     │
// dtMPC = 30ms
// (MPC frequency ≈ 33Hz)
//     │
//     × horizonLength (10)
//     │
// T_gait = 300ms
// (gait frequency ≈ 3.33Hz)
//     │
// Stance = T_gait/2 = 150ms  (trot, duty=50%)
// Swing  = T_gait/2 = 150ms
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc)
    : dt(_dt),                                       // 0.001
      iterationsBetweenMPC(_iterations_between_mpc), // Frequency control
      horizonLength(14),                             //
      trotting(horizonLength,
               Vec4<int>(0, horizonLength / 2.0, horizonLength / 2.0, 0),
               Vec4<int>(horizonLength / 2.0, horizonLength / 2.0, horizonLength / 2.0, horizonLength / 2.0),
               "Trotting"), // trot buoc nhu vay
    //   trotting(horizonLength, Vec4<int>(0, 10, 10, 0), Vec4<int>(10, 10, 10, 10), "Trotting"),
      bounding(horizonLength, Vec4<int>(7, 7, 0, 0), Vec4<int>(6, 6, 6, 6), "Bounding"),
      pronking(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(6, 6, 6, 6), "Pronking"),
      jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(3, 3, 3, 3), "Jumping"),
      galloping(horizonLength, Vec4<int>(0, 4, 7, 11), Vec4<int>(7, 7, 7, 7), "Galloping"),
      standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(14, 14, 14, 14), "Standing"),
      trotRunning(horizonLength, Vec4<int>(0, 7, 7, 0), Vec4<int>(6, 6, 6, 6), "Trot Running"),
      walking(horizonLength,
              Vec4<int>(0, horizonLength / 2.0, horizonLength / 4.0, 3.0 * horizonLength / 4.0),
              Vec4<int>(3.0 * horizonLength / 4.0,
                        3.0 * horizonLength / 4.0,
                        3.0 * horizonLength / 4.0,
                        3.0 * horizonLength / 4.0),
              "Walking"),
      walking2(horizonLength, Vec4<int>(0, 7, 7, 0), Vec4<int>(10, 10, 10, 10), "Walking2"),
      pacing(horizonLength, Vec4<int>(7, 0, 7, 0), Vec4<int>(7, 7, 7, 7), "Pacing"),
      aio(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(14, 14, 14, 14), "aio")
{
    dtMPC = dt * iterationsBetweenMPC; // 0.03
    default_iterations_between_mpc = iterationsBetweenMPC;
    printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC); // 0.002, 15, 0.03
    // setup_problem(dtMPC, horizonLength, 0.4, 120);
    // setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
    rpy_comp[0] = 0;
    rpy_comp[1] = 0;
    rpy_comp[2] = 0;
    rpy_int[0] = 0;
    rpy_int[1] = 0;
    rpy_int[2] = 0;

    for (int i = 0; i < 4; i++)
        firstSwing[i] = true;

    initSparseMPC();

    pBody_des.setZero();
    vBody_des.setZero();
    aBody_des.setZero();
    for (int i = 0; i < 4; i++)
        f_ff[i].setZero();
}

void ConvexMPCLocomotion::initialize()
{
    for (int i = 0; i < 4; i++)
        firstSwing[i] = true;
    firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
    iterationsBetweenMPC = iterations_per_mpc;
    dtMPC = dt * iterations_per_mpc;
}

//设置期望值
void ConvexMPCLocomotion::_SetupCommand(StateEstimatorContainer<float> &_stateEstimator,
                                        std::vector<double> gamepadCommand)
{
    _body_height = RobotConfig::BODY_HEIGHT;

    // ── 1. Đọc và kẹp lệnh thô từ tay cầm ────────────────────────────────
    float x_cmd = std::max(-0.3f, std::min(0.5f, static_cast<float>(gamepadCommand[0])));
    float y_cmd = std::max(-0.3f, std::min(0.3f, static_cast<float>(gamepadCommand[1])));
    float yaw_cmd = std::max(-0.8f, std::min(0.8f, static_cast<float>(gamepadCommand[2])));

    // ── 2. Bộ định hình biên dạng S-Curve (Giới hạn chặt Gia tốc & Jerk) ──
    const float dt_ctrl = (dt > 1e-4f) ? dt : 0.002f;

    // Trục X (Tiến/Lùi): V_max = 0.5 m/s, A_max = 0.8 m/s^2, Jerk_max = 3.0 m/s^3
    {
        const float a_max = 0.8f;
        const float jerk_max = 3.0f;
        const float tau = 0.15f; // Thời gian đáp ứng

        float a_target = (x_cmd - _x_vel_des) / tau;
        a_target = std::max(-a_max, std::min(a_max, a_target));

        float j_raw = (a_target - _x_acc_des) / dt_ctrl;
        float j_clamped = std::max(-jerk_max, std::min(jerk_max, j_raw));

        _x_acc_des += j_clamped * dt_ctrl;
        _x_acc_des = std::max(-a_max, std::min(a_max, _x_acc_des));

        _x_vel_des += _x_acc_des * dt_ctrl;
        _x_vel_des = std::max(-0.3f, std::min(0.5f, _x_vel_des));
    }

    // Trục Y (Ngang): V_max = 0.3 m/s, A_max = 0.5 m/s^2, Jerk_max = 2.0 m/s^3
    {
        const float a_max = 0.5f;
        const float jerk_max = 2.0f;
        const float tau = 0.15f;

        float a_target = (y_cmd - _y_vel_des) / tau;
        a_target = std::max(-a_max, std::min(a_max, a_target));

        float j_raw = (a_target - _y_acc_des) / dt_ctrl;
        float j_clamped = std::max(-jerk_max, std::min(jerk_max, j_raw));

        _y_acc_des += j_clamped * dt_ctrl;
        _y_acc_des = std::max(-a_max, std::min(a_max, _y_acc_des));

        _y_vel_des += _y_acc_des * dt_ctrl;
        _y_vel_des = std::max(-0.3f, std::min(0.3f, _y_vel_des));
    }

    // Trục Yaw (Quay): W_max = 0.8 rad/s, A_max = 1.5 rad/s^2, Jerk_max = 5.0 rad/s^3
    {
        const float a_max = 1.5f;
        const float jerk_max = 5.0f;
        const float tau = 0.10f;

        float a_target = (yaw_cmd - _yaw_turn_rate) / tau;
        a_target = std::max(-a_max, std::min(a_max, a_target));

        float j_raw = (a_target - _yaw_acc_des) / dt_ctrl;
        float j_clamped = std::max(-jerk_max, std::min(jerk_max, j_raw));

        _yaw_acc_des += j_clamped * dt_ctrl;
        _yaw_acc_des = std::max(-a_max, std::min(a_max, _yaw_acc_des));

        _yaw_turn_rate += _yaw_acc_des * dt_ctrl;
        _yaw_turn_rate = std::max(-0.8f, std::min(0.8f, _yaw_turn_rate));
    }

    _yaw_des = _stateEstimator.getResult().rpy[2] + dt * _yaw_turn_rate;

    if ((std::fabs(_stateEstimator.getResult().rpy[2] - _yaw_des_true) > 5.0f))
    {
        _yaw_des_true = _stateEstimator.getResult().rpy[2];
    }
    _yaw_des_true = _yaw_des_true + dt * _yaw_turn_rate;

    _roll_des = 0.0f;
    _pitch_des = 0.0f;
}

template <>
void ConvexMPCLocomotion::run(Quadruped<float> &_quadruped,
                              LegController<float> &_legController,
                              StateEstimatorContainer<float> &_stateEstimator,
                              DesiredStateCommand<float> & /*_desiredStateCommand*/,
                              std::vector<double> gamepadCommand,
                              int gaitType,
                              int robotMode)
{
    bool omniMode = false;
    // Command Setup
    _SetupCommand(_stateEstimator, gamepadCommand);

    gaitNumber = gaitType; // data.userParameters->cmpc_gait; 步态默认为trot

    if (gaitNumber >= 20)
    {
        gaitNumber -= 20;
        omniMode = true;
    }

    // Chế độ tay: nếu vận tốc lệnh ≈ 0, chuyển standing (MPC all 4 feet contact)
    if (robotMode == 0 &&
        fabs(_x_vel_des) < 0.01f &&
        fabs(_y_vel_des) < 0.01f &&
        fabs(_yaw_turn_rate) < 0.01f)
    {
        gaitNumber = 4;
    }

    auto &seResult = _stateEstimator.getResult(); //状态估计器

    // Capture the transition before current_gait is updated below. The flag is
    // also used to latch the four current foot positions as standing anchors.
    const bool enteringStanding =
        (gaitNumber == 4) && (current_gait != 4 || firstRun);

    // Check if transition to standing 检查是否过渡到站立
    if (enteringStanding || firstRun)
    {
        stand_traj[0] = seResult.position[0];
        stand_traj[1] = seResult.position[1];
        stand_traj[2] = 0.29;
        stand_traj[3] = 0;
        stand_traj[4] = 0;
        stand_traj[5] = seResult.rpy[2];
        world_position_desired[0] = stand_traj[0];
        world_position_desired[1] = stand_traj[1];
    }

    // pick gait
    Gait *gait = &trotting;
    if (robotMode == 0) // chế độ theo gait người dùng tùy chọn
    {
        if (gaitNumber == 1)
            gait = &bounding;
        else if (gaitNumber == 2)
            gait = &pronking;
        // else if(gaitNumber == 3)
        //   gait = &random;
        else if (gaitNumber == 4)
            gait = &standing;
        else if (gaitNumber == 5)
            gait = &trotRunning;
        // else if(gaitNumber == 6)
        //   gait = &random2;
        else if (gaitNumber == 7)
            gait = &galloping;
        else if (gaitNumber == 8)
            gait = &pacing;
        else if (gaitNumber == 9)
            gait = &trotting;
        else if (gaitNumber == 10)
            gait = &walking;
        else if (gaitNumber == 11)
            gait = &walking2;
    }
    else if (robotMode == 1) // chuyển chế độ theo vận tốc thân
    {
        int h = 10;
        double vBody = sqrt(_x_vel_des * _x_vel_des) + (_y_vel_des * _y_vel_des);
        gait = &aio;
        gaitNumber = 9; // Trotting
        if (gait->getCurrentGaitPhase() == 0)
        {
            if (vBody < 0.002)
            {
                if (abs(_yaw_turn_rate) < 0.01)
                {
                    gaitNumber = 4; // Standing
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h, Vec4<int>(0, 0, 0, 0), Vec4<int>(h, h, h, h), "Standing");
                }
                else
                {
                    h = 10;
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2),
                                       "trotting");
                }
            }
            else
            {
                if (vBody <= 0.2)
                {
                    h = 16;
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, 1 * h / 2, 1 * h / 4, 3 * h / 4),
                                       Vec4<int>(3 * h / 4, 3 * h / 4, 3 * h / 4, 3 * h / 4),
                                       "Walking");
                }
                else if (vBody > 0.2 && vBody <= 0.4)
                {
                    h = 16;
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(
                        h,
                        Vec4<int>(0, 1 * h / 2, h * ((5.0 / 4.0) * vBody), h * ((5.0 / 4.0) * vBody + (1.0 / 2.0))),
                        Vec4<int>(h * ((-5.0 / 4.0) * vBody + 1.0),
                                  h * ((-5.0 / 4.0) * vBody + 1.0),
                                  h * ((-5.0 / 4.0) * vBody + 1.0),
                                  h * ((-5.0 / 4.0) * vBody + 1.0)),
                        "Walking2trotting");
                }
                else if (vBody > 0.4 && vBody <= 1.4)
                {
                    h = 14;
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2),
                                       "trotting");
                }
                else
                {
                    // h = 10;
                    h = -20.0 * vBody + 42.0;
                    if (h < 10)
                        h = 10;
                    if (gait->getGaitHorizon() != h)
                    {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2),
                                       "trotting");

                    // std::cout << vBody << " " << h << " " << h / 2 << std::endl;
                }
            }
        }
        horizonLength = h;
    }
    else
    {
        std::cout << "err robot mode!!!" << std::endl;
    }

    current_gait = gaitNumber;
    gait->setIterations(iterationsBetweenMPC, iterationCounter); // Gait period calculation

    // integrate position setpoint
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,
                            0); // Desired linear velocity in body coordinate system
    Vec3<float> v_des_world =
        omniMode ? v_des_robot
                 : seResult.rBody.transpose() * v_des_robot; //Desired linear velocity in world coordinate system
    Vec3<float> v_robot = seResult.vWorld;                   //The robot's actual speed in the world coordinate system

    // Integral-esque pitch and roll compensation
    // Points reach compensation value*******************************
    // To keep the body parallel to the ground during exercise
    if (fabs(v_robot[0]) > 0.02f) // avoid dividing by zero
    {
        rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
    }
    if (fabs(v_robot[1]) > 0.02f)
    {
        rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
    }

    //Initial angle limiting
    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25); //-0.25~0.25
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    rpy_comp[1] = v_robot[0] * rpy_int[1]; // compensation value
    rpy_comp[0] = v_robot[1] * rpy_int[0]; // turn off for pronking

    // Obtain the foot position in the world coordinate system
    // Fuselage coordinates + fuselage rotation matrix ^T * (coordinates of the side swing joint under the fuselage
    // + coordinates of the foot under the side swing joint)
    for (int i = 0; i < 4; i++)
    {
        pFoot[i] =
            seResult.position + seResult.rBody.transpose() * (_quadruped.getHipLocation(i) + _legController.datas[i].p);
        // pFoot[i] = _legController.datas[i].p;
    }

    Vec3<float> error;
    if (gait != &standing)
    { //The desired position when not standing is achieved by accumulating the desired velocity.
        world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    }

    // some first time initialization
    if (firstRun)
    {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        world_position_desired[2] = seResult.rpy[2];

        for (int i = 0; i < 4; i++) // Foot swing trajectory
        {
            footSwingTrajectories[i].setHeight(0.15);
            footSwingTrajectories[i].setInitialPosition(pFoot[i]); // set p0
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);   // set pf
        }
        firstRun = false;
    }

    // Standing uses fixed world-frame foot anchors. Without this latch the
    // stance controller can reuse a stale sample from the previous swing.
    if (enteringStanding)
    {
        standingStiffnessRamp = 0.0f;
        for (int i = 0; i < 4; i++)
        {
            standingFootPositions[i] = pFoot[i];
            footSwingTrajectories[i].setInitialPosition(pFoot[i]);
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);
        }
    }
    else if (gaitNumber == 4)
    {
        // Avoid an instantaneous Cartesian-force step when all four legs become
        // stance legs. Reach full lateral stiffness after about 1.0 second.
        standingStiffnessRamp =
            fminf(1.0f, standingStiffnessRamp + dt / 1.0f);
    }
    else
    {
        standingStiffnessRamp = 0.0f;
    }

    // foot placement
    for (int l = 0; l < 4; l++)
    {
        swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l); // return dtMPC * _stance  0.026 * 5 = 0.13
        // The value of dtMPC changed to 0.026, an external assignment that modified the value.
    }

    // ── Side sign ─────────────────────────────────────────────────────────────
    // +1 cho right legs (FR, HR): abad link lệch về phía +Y (right)
    // -1 cho left  legs (FL, HL): abad link lệch về phía -Y (left)
    float side_sign[4] = {-1, 1, -1, 1};

    // ── Interleave offset ──────────────────────────────────────────────────────
    // Đặt interleave_gain = 0 để 4 chân đặt hoàn toàn đối xứng dưới hip, loại bỏ lắc lư
    float interleave_y[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float interleave_gain = 0.0f;

    // Tốc độ tuyến tính mong muốn theo trục x (body frame), dùng scale interleave
    float v_abs = std::fabs(v_des_robot[0]);

    for (int i = 0; i < 4; i++)
    {
        // ── 1. Cập nhật thời gian còn lại của swing phase ─────────────────────
        // firstSwing[i] = true khi chân vừa lift-off → reset về swingTimes[i]
        // Các bước tiếp theo: đếm ngược xuống 0
        if (firstSwing[i])
            swingTimeRemaining[i] = swingTimes[i];
        else
            swingTimeRemaining[i] -= dt;

        // ── 2. Swing height ───────────────────────────────────────────────────
        // Chiều cao tối đa của Bezier swing trajectory (m).
        // Lite3: tăng lên 0.15m để tránh va chạm chân-chân ở tốc độ cao
        // và để đủ clearance trên địa hình rough.
        footSwingTrajectories[i].setHeight(0.15);

        // ── 3. Hip location trong body frame ──────────────────────────────────
        // Offset ngang = abadLinkLength (khoảng cách abad joint → hip joint theo Y).
        // Dùng _abadLinkLength thực (Lite3 = 0.09735m) thay vì giá trị cứng 0.065m
        // của Mini Cheetah, tránh chân bị thu vào trong và thu nhỏ support polygon.
        Vec3<float> offset(0, side_sign[i] * _quadruped._abadLinkLength, 0);
        Vec3<float> pRobotFrame = _quadruped.getHipLocation(i) + offset;

        // Cộng thêm interleave offset theo tốc độ tiến
        pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

        // ── 4. Yaw correction ─────────────────────────────────────────────────
        // Rotate hip location quanh trục Z một góc = -yaw_rate * stance_time/2
        // để dự đoán vị trí hip khi chân chạm đất (body đã quay thêm).
        // Nửa stance time vì chân chạm đất ở giữa stance phase.
        float stance_time = gait->getCurrentStanceTime(dtMPC, i);
        Vec3<float> pYawCorrected =
            coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

        // ── 5. Extrapolate vị trí đặt chân trong world frame ──────────────────
        // Giả định body di chuyển đều trong thời gian swingTimeRemaining còn lại:
        //   Pf = p_body + R^T * (pYawCorrected + v_des * swingTimeRemaining)
        // R^T: rotate từ world → body frame (seResult.rBody là R_body_to_world)
        Vec3<float> des_vel(_x_vel_des, _y_vel_des, 0.0);
        Vec3<float> Pf =
            seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

        float p_rel_max = 0.15f; // giới hạn step offset tối đa [m] cho tốc độ 0.5 m/s

        // ── 6. Raibert symmetry + capture point correction ────────────────────

        /**
     * @brief pfx_rel — foot placement offset theo trục X (world frame)
     *
     * Term 1: vx * 0.5 * T_stance
     *   → Raibert symmetry: đặt chân tại midstance position
     *   → Đảm bảo touchdown và liftoff velocity đối xứng
     *
     * Term 2: 0.03 * (vx_actual - vx_des)
     *   → Velocity error feedback: nếu đang chạy nhanh hơn mong muốn
     *     → đặt chân xa hơn về phía trước để hãm lại
     *
     * Term 3: 0.5*sqrt(z/g) * vy * yaw_rate
     *   → Capture point: bù centrifugal force khi quay
     *   → sqrt(z/g) = time constant của inverted pendulum
     */
        float pfx_rel = seResult.vWorld[0] * 0.5f * stance_time + 0.03f * (seResult.vWorld[0] - v_des_world[0]) +
                        (0.5f * sqrtf(seResult.position[2] / 9.81f)) * (seResult.vWorld[1] * _yaw_turn_rate);

        /**
     * @brief pfy_rel — foot placement offset theo trục Y (world frame)
     *
     * Cấu trúc tương tự pfx_rel nhưng cho chiều ngang:
     *   Term 3 dùng (-vx * yaw_rate) thay vì (vy * yaw_rate)
     *   → Bù moment quay theo chiều ngược lại
     */
        float pfy_rel = seResult.vWorld[1] * 0.5f * stance_time + 0.06f * (seResult.vWorld[1] - v_des_world[1]) +
                        (0.5f * sqrtf(seResult.position[2] / 9.81f)) * (-seResult.vWorld[0] * _yaw_turn_rate);

        // ── 7. Clamp offset ───────────────────────────────────────────────────
        // Giới hạn step offset trong [-p_rel_max, p_rel_max] = [-0.3, 0.3] m
        // Tránh lệnh đặt chân quá xa gây mất thăng bằng hoặc QP infeasible
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        // ── 8. Final foot position ────────────────────────────────────────────
        // Cộng Raibert offset vào vị trí hip world đã extrapolate
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
        Pf[2] = 0.0f; // đặt chân trên mặt phẳng z=0 (flat ground assumption)

        // Set endpoint cho Bezier swing trajectory
        // Trajectory sẽ interpolate từ lift-off position → Pf
        footSwingTrajectories[i].setFinalPosition(Pf);
    }
    // std::cout << std::endl;

    // calc gait
    iterationCounter++; // +1 mỗi tick (1 kHz)

    // load LCM leg swing gains
    // Kp << 700, 0, 0, 0, 700, 0, 0, 0, 50;
    Kp << 700, 0, 0, 0, 700, 0, 0, 0, 200;
    // Use a conservative Y-only anchor. At the 3 cm error limit this produces
    // at most 2.4 N per foot; vertical support remains entirely with MPC.
    Kp_stance << 80, 0, 0,
                 0, 80, 0,
                 0, 0, 80;
    Kp_stance *= standingStiffnessRamp;

    Kd << 10, 0, 0, 0, 10, 0, 0, 0, 10;
    // Kd_stance = 1.0 * Kd;
    // Kp_stance << 0,  0,   0,
    //              0,  0,   0,
    //              0,  0, 80.0;  // Bắt đầu thử từ 50.0 đến 100.0 N/m

    // // Kd cho Stance:
    Kd_stance << 10.0,    0,    0,
                    0, 10.0,    0,
                    0,    0, 10.0;

    // gait
    Vec4<float> contactStates = gait->getContactState();
    Vec4<float> swingStates = gait->getSwingState();
    int *mpcTable = gait->getMpcTable();
    updateMPCIfNeeded(mpcTable, _stateEstimator, _legController, omniMode); // cứ 30 tick mới giải 1 lần (MPC -> QP)

    //  StateEstimator* se = hw_i->state_estimator;
    Vec4<float> se_contactState(0, 0, 0, 0);

    bool use_wbc = false;

    for (int foot = 0; foot < 4; foot++)
    {
        float contactState = contactStates[foot];
        float swingState = swingStates[foot];

        if (swingState > 0) // foot is in swing
        {
            if (firstSwing[foot])
            {
                firstSwing[foot] = false;
                footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
            }

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

            Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
            Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

            Vec3<float> pDesLeg =
                seResult.rBody *
                    (pDesFootWorld - seResult.position) //Foot coordinates in the lateral joint coordinate system
                //(This section should be changed to foot coordinates in the body coordinate system.)
                - _quadruped.getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // Update for WBC
            pFoot_des[foot] = pDesFootWorld;
            vFoot_des[foot] = vDesFootWorld;
            aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

            if (!use_wbc)
            {
                // Update leg control command regardless of the usage of WBIC
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;
                if (foot == 1 || foot == 3)
                {
                    _legController.commands[foot].kpCartesian = Kp;
                    _legController.commands[foot].kdCartesian = Kd;
                }
                else
                {
                    _legController.commands[foot].kpCartesian = 1 * Kp;
                    _legController.commands[foot].kdCartesian = 1 * Kd;
                }
            }
        }
        else // foot is in stance
        {
            firstSwing[foot] = true;

            const bool standingNow = (gaitNumber == 4);
            Vec3<float> pDesFootWorld = standingNow
                                            ? standingFootPositions[foot]
                                            : footSwingTrajectories[foot].getPosition();
            Vec3<float> vDesFootWorld = standingNow
                                            ? Vec3<float>::Zero()
                                            : footSwingTrajectories[foot].getVelocity();
            Vec3<float> pDesLeg =
                seResult.rBody * (pDesFootWorld - seResult.position) - _quadruped.getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            if (standingNow)
            {
                // Bound lateral feedback to about 2.4 N at full stiffness. This
                // preserves the world-frame anchor without allowing estimator
                // transients to create a large sideways impulse.
                const float maxLateralError = 0.03f;
                const float lateralError =
                    pDesLeg[1] - _legController.datas[foot].p[1];
                pDesLeg[1] = _legController.datas[foot].p[1] +
                             fminf(fmaxf(lateralError, -maxLateralError),
                                   maxLateralError);
            }

            if (!use_wbc)
            {
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;
                Mat3<float> Kp_trot_stance = Mat3<float>::Zero();
                Kp_trot_stance(0, 0) = 40.0f;
                Kp_trot_stance(1, 1) = 40.0f;
                Kp_trot_stance(2, 2) = 80.0f;
                _legController.commands[foot].kpCartesian =
                    standingNow ? Kp_stance : Kp_trot_stance;

                if (foot == 1 || foot == 3)
                {
                    _legController.commands[foot].kdCartesian = Kd_stance;
                }
                else
                {
                    _legController.commands[foot].kdCartesian = 1 * Kd_stance;
                }

                _legController.commands[foot].forceFeedForward = f_ff[foot];
                _legController.commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
            }
            else
            { // Stance foot damping
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;
                _legController.commands[foot].kpCartesian =
                    standingNow ? Kp_stance : Mat3<float>::Zero();
                _legController.commands[foot].kdCartesian = Kd_stance;
            }
            se_contactState[foot] = contactState;

            // Update for WBC
            // Fr_des[foot] = -f_ff[foot];
        }
    }
    // se->set_contact_state(se_contactState); todo removed
    _stateEstimator.setContactPhase(se_contactState);

    // Update For WBC
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = _body_height;

    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();

    pBody_RPY_des[0] = 0.;
    pBody_RPY_des[1] = 0.;
    pBody_RPY_des[2] = _yaw_des;

    vBody_Ori_des[0] = 0.;
    vBody_Ori_des[1] = 0.;
    vBody_Ori_des[2] = _yaw_turn_rate;

    // contact_state = gait->getContactState();
    contact_state = gait->getContactState();
    // END of WBC Update
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable,
                                            StateEstimatorContainer<float> &_stateEstimator,
                                            LegController<float> &_legController,
                                            bool omniMode)
{
    // iterationsBetweenMPC = 30;
    if ((iterationCounter % iterationsBetweenMPC) == 0) // cứ 30 tick mới giải 1 lần
    {
        auto seResult = _stateEstimator.getResult();
        float *p = seResult.position.data();

        Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
        Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
        // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

        // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
        // x_comp_integral);

        if (current_gait == 4)
        {
            float trajInitial[12] = {_roll_des,
                                     _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
                                     (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
                                     (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
                                     (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
                                     (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

            for (int i = 0; i < horizonLength; i++)
                for (int j = 0; j < 12; j++)
                    trajAll[12 * i + j] = trajInitial[j];
        }

        else
        {
            const float max_pos_error = .1;
            float xStart = world_position_desired[0];
            float yStart = world_position_desired[1];

            if (xStart - p[0] > max_pos_error)
                xStart = p[0] + 0.1;
            if (p[0] - xStart > max_pos_error)
                xStart = p[0] - 0.1;

            if (yStart - p[1] > max_pos_error)
                yStart = p[1] + 0.1;
            if (p[1] - yStart > max_pos_error)
                yStart = p[1] - 0.1;

            world_position_desired[0] = xStart;
            world_position_desired[1] = yStart;

            float trajInitial[12] = {(float)rpy_comp[0], // 0
                                     (float)rpy_comp[1], // 1
                                     _yaw_des_true,      // 2
                                     // yawStart,    // 2
                                     xStart,              // 3
                                     yStart,              // 4
                                     (float)_body_height, // 5
                                     0,                   // 6
                                     0,                   // 7
                                     _yaw_turn_rate,      // 8
                                     v_des_world[0],      // 9
                                     v_des_world[1],      // 10
                                     0};                  // 11

            for (int i = 0; i < horizonLength; i++)
            {
                for (int j = 0; j < 12; j++)
                    trajAll[12 * i + j] = trajInitial[j];

                if (i == 0) // start at current position  TODO consider not doing this
                {
                    // trajAll[2] = seResult.rpy[2];
                    trajAll[2] = _yaw_des_true;
                }
                else
                {
                    trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
                    trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
                    trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
                }
            }
        }

        Timer solveTimer;

        int cmpc_use_sparse = 0.0;

        if (cmpc_use_sparse > 0.5)
        {
            solveSparseMPC(mpcTable, _stateEstimator);
        }
        else
        {
            solveDenseMPC(mpcTable, _stateEstimator, _legController);
        }
        // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
    }
}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable,
                                        StateEstimatorContainer<float> &_stateEstimator,
                                        LegController<float> &_legController)
{
    auto seResult = _stateEstimator.getResult();

    // Q matrix weights: [Roll, Pitch, Yaw, X, Y, Z, dRoll, dPitch, dYaw, Vx, Vy, Vz]
    float Q[12] = {25.0f, 30.0f, 80.0f, 5.0f, 5.0f, 350.0f, 1.0f, 1.2f, 1.5f, 0.5f, 2.0f, 0.2f};
    float yaw = seResult.rpy[2];
    float *weights = Q;
    float alpha = RobotConfig::MPC_ALPHA;
    float *p = seResult.position.data();
    float *v = seResult.vWorld.data();
    float *w = seResult.omegaWorld.data();
    float *q = seResult.orientation.data();

    // ── Cập nhật bộ ước lượng trọng tâm & khối lượng ──────────────────
    Eigen::Matrix<float, 3, 4> p_feet_body;
    Eigen::Matrix<float, 3, 4> f_feet_mpc_body;
    Eigen::Matrix<float, 3, 4> f_feet_actual_world;
    for (int i = 0; i < 4; i++)
    {
        p_feet_body.col(i) = seResult.rBody * (pFoot[i] - seResult.position);
        f_feet_mpc_body.col(i) = seResult.rBody * Fr_des[i];

        // Tính phản lực tiếp xúc thực tế từ mô-men động cơ đo được: f = (J^T)^-1 * (tau_meas - tau_fric)
        Eigen::Vector3f f_act_body = CoMEstimator::computeFootForceFromJointTorque(
            _legController.datas[i].J,
            _legController.datas[i].tauEstimate,
            _legController.datas[i].qd);
        f_feet_actual_world.col(i) = seResult.rBody.transpose() * f_act_body;
    }
    Eigen::Matrix3f I_body = Eigen::Matrix3f::Zero();
    I_body.diagonal() << RobotConfig::IXX, RobotConfig::IYY, RobotConfig::IZZ;

    Eigen::Vector4f contact_state_vec;
    contact_state_vec << (float)mpcTable[0], (float)mpcTable[1], (float)mpcTable[2], (float)mpcTable[3];

    _comEstimator.update(p_feet_body, f_feet_mpc_body, f_feet_actual_world,
                         seResult.omegaBody, dtMPC, I_body, contact_state_vec);
    Vec3<float> r_com_body = _comEstimator.getCoMOffset();
    Vec3<float> r_com_world = seResult.rBody.transpose() * r_com_body;
    float est_mass = _comEstimator.getEstimatedMass();
    set_robot_mass(est_mass); // Tự động đồng bộ khối lượng thực tế vào MPC

    if (iterationCounter % 100 == 0)
    {
        printf("[CoM & Mass Estimator] CoM: x=%+.3f m | y=%+.3f m | z=%+.3f m || Mass: %.2f kg (raw: %.2f kg | Fz: %.1f N)\n",
               r_com_body[0], r_com_body[1], r_com_body[2], est_mass,
               _comEstimator.getEstimatedMassRaw(), _comEstimator.getTotalSupportForce());
    }

    // ── Cánh tay đòn MPC có bù độ lệch CoM ước lượng ──────────────────
    float r[12];
    for (int i = 0; i < 12; i++)
        r[i] = pFoot[i % 4][i / 4] - (seResult.position[i / 4] + r_com_world[i / 4]);

    // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if (alpha > 1e-4)
    {
        std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
        alpha = 1e-5;
    }

    Vec3<float> pxy_act(p[0], p[1], 0);
    Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
    // Vec3<float> pxy_err = pxy_act - pxy_des;
    float pz_err = p[2] - _body_height;
    Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

    Timer t1;
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, RobotConfig::FRICTION_MU, RobotConfig::MAX_FORCE);
    update_x_drag(x_comp_integral);

    float cmpc_x_drag = 3.0;

    if (vxy[0] > 0.3 || vxy[0] < -0.3)
    {
        // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC /
        // vxy[0];
        x_comp_integral += cmpc_x_drag * pz_err * dtMPC / vxy[0];
    }

    // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

    int jcqp_max_iter = 10000;
    double jcqp_rho = 0.0000001;
    double jcqp_sigma = 0.00000001;
    double jcqp_alpha = 1.5;
    double jcqp_terminate = 0.1;
    double use_jcqp = 0.0;

    update_solver_settings(jcqp_max_iter, jcqp_rho, jcqp_sigma, jcqp_alpha, jcqp_terminate, use_jcqp);
    // t1.stopPrint("Setup MPC");

    Timer t2;
    // cout << "dtMPC: " << dtMPC << "\n";
    // std::cout << "pFoot = " << std::endl;
    // for(int i=0; i<4; i++) {
    //   for(int j=0; j<3; j++) {
    //     std::cout << pFoot[i][j] << " ";
    //   }
    // }
    // std::cout << std::endl;
    update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);

    // std::cout << "the value is " << mpcTable << std::endl;

    printf("[time]   MPC setup: %.3f ms | MPC solve: %.3f ms\n", t1.getMs() - t2.getMs(), t2.getMs());

    for (int leg = 0; leg < 4; leg++)
    {
        Vec3<float> f;
        for (int axis = 0; axis < 3; axis++)
            f[axis] = get_solution(leg * 3 + axis);

        // if(myflags < 50){
        // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);
        // }

        f_ff[leg] = -seResult.rBody * f;
        // std::cout << "Foot " << leg << " force: " << f.transpose() << "\n";
        // std::cout << "Foot " << leg << " force: " << f_ff[leg].transpose() << "\n"; // Update for WBC
        Fr_des[leg] = f;
    }
    myflags = myflags + 1;
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, StateEstimatorContainer<float> &_stateEstimator)
{
    // X0, contact trajectory, state trajectory, feet, get result!
    (void)mpcTable;
    auto seResult = _stateEstimator.getResult();

    std::vector<ContactState> contactStates;
    for (int i = 0; i < horizonLength; i++)
    {
        contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
    }

    for (int i = 0; i < horizonLength; i++)
    {
        for (u32 j = 0; j < 12; j++)
        {
            _sparseTrajectory[i][j] = trajAll[i * 12 + j];
        }
    }

    Vec3<float> CoM_body = RobotConfig::isComOffset ? Vec3<float>{0.05f, 0.0f, 0.05f} : Vec3<float>::Zero();
    Vec3<float> CoM_world = seResult.rBody.transpose() * CoM_body;
    Vec12<float> feet;
    for (u32 foot = 0; foot < 4; foot++)
    {
        for (u32 axis = 0; axis < 3; axis++)
        {
            feet[foot * 3 + axis] = pFoot[foot][axis] - (seResult.position[axis] + CoM_world[axis]);
        }
    }

    _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
    _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
    _sparseCMPC.setStateTrajectory(_sparseTrajectory);
    _sparseCMPC.setFeet(feet);
    _sparseCMPC.run();

    Vec12<float> resultForce = _sparseCMPC.getResult();

    for (u32 foot = 0; foot < 4; foot++)
    {
        Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
        printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
        f_ff[foot] = -seResult.rBody * force;
        Fr_des[foot] = force;
    }
}

void ConvexMPCLocomotion::initSparseMPC()
{
    Mat3<double> baseInertia;
    baseInertia << RobotConfig::IXX, 0, 0,
                   0, RobotConfig::IYY, 0,
                   0, 0, RobotConfig::IZZ;
    double mass = RobotConfig::MASS;
    double maxForce = RobotConfig::MAX_FORCE;

    std::vector<double> dtTraj;
    for (int i = 0; i < horizonLength; i++)
    {
        dtTraj.push_back(dtMPC);
    }

    Vec12<double> weights;
    weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
    // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

    _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
    _sparseCMPC.setFriction(1.0);
    _sparseCMPC.setWeights(weights, 4e-5);
    _sparseCMPC.setDtTrajectory(dtTraj);

    _sparseTrajectory.resize(horizonLength);
}
