/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "PositionVelocityEstimator.h"

#include <fstream>

/**
 * @brief Khởi tạo Linear Kalman Filter cho ước lượng vị trí và vận tốc.
 *
 * Kalman Filter ước lượng đồng thời:
 *   - Vị trí body trong world frame
 *   - Vận tốc body trong world frame
 *   - Vị trí 4 bàn chân trong world frame (assumed stationary khi stance)
 *
 * @par State vector xhat ∈ R_18:
 * @code
 *   xhat[0:3]   = p      (body position,    world frame)
 *   xhat[3:6]   = v      (body velocity,    world frame)
 *   xhat[6:9]   = pf_FL  (FL foot position, world frame)
 *   xhat[9:12]  = pf_FR  (FR foot position, world frame)
 *   xhat[12:15] = pf_HL  (HL foot position, world frame)
 *   xhat[15:18] = pf_HR  (HR foot position, world frame)
 * @endcode
 *
 * @par Process model (discrete time):
 * @code
 *   x[k+1] = A·x[k] + B·u[k]
 *
 *   A = [I3   dt·I3   0     ]    B = [0    ]
 *       [0    I3      0     ]        [dt·I3]
 *       [0    0       I12   ]        [0    ]
 *
 *   u[k] = a_imu  (IMU linear acceleration, world frame)
 *
 *   Nghĩa là:
 *     p[k+1] = p[k] + dt·v[k]           (kinematics)
 *     v[k+1] = v[k] + dt·a_imu          (Newton's 2nd law)
 *     pf[k+1] = pf[k]                   (chân assumed đứng yên)
 * @endcode
 *
 * @par Observation vector
 *
 * z ∈ R_28:
 * @code
 *   z[0:3]   = p - pf_FL   (body pos relative to FL foot)
 *   z[3:6]   = p - pf_FR
 *   z[6:9]   = p - pf_HL
 *   z[9:12]  = p - pf_HR
 *   z[12:15] = v            (body vel, measured via FL contact kinematics)
 *   z[15:18] = v            (via FR)
 *   z[18:21] = v            (via HL)
 *   z[21:24] = v            (via HR)
 *   z[24]    = pf_FL_z      (FL foot height)
 *   z[25]    = pf_FR_z
 *   z[26]    = pf_HL_z
 *   z[27]    = pf_HR_z
 * @endcode
 *
 * @par Observation matrix C ∈ R_28x18:
 * @code
 *   C = [I3   0    -I3  0    0    0   ]   ← z[0:3]  = p - pf_FL
 *       [I3   0    0    -I3  0    0   ]   ← z[3:6]  = p - pf_FR
 *       [I3   0    0    0    -I3  0   ]   ← z[6:9]  = p - pf_HL
 *       [I3   0    0    0    0    -I3 ]   ← z[9:12] = p - pf_HR
 *       [0    I3   0    0    0    0   ]   ← z[12:15]= v (×4)
 *       [0    I3   0    0    0    0   ]
 *       [0    I3   0    0    0    0   ]
 *       [0    I3   0    0    0    0   ]
 *       [0    0    e3ᵀ  0    0    0   ]   ← z[24] = pf_FL_z
 *       [0    0    0    e3ᵀ  0    0   ]   ← z[25] = pf_FR_z
 *       [0    0    0    0    e3ᵀ  0   ]   ← z[26] = pf_HL_z
 *       [0    0    0    0    0    e3ᵀ ]   ← z[27] = pf_HR_z
 * @endcode
 *
 * @par Noise covariances:
 * @code
 *   P0  = 100·I18    (initial state covariance, high uncertainty)
 *
 *   Q0 = diag(dt/20·I3,        ← position process noise
 *             dt·9.8/20·I3,    ← velocity process noise (scaled by g)
 *             dt·I12)          ← foot position process noise
 *
 *   R0  = I28        (measurement noise, tuned per contact state in run())
 * @endcode
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup()
{
    // ── Timestep ───────────────────────────────────────────────────────────
    T dt = 0.002; // default 500Hz
    if (this->_stateEstimatorData.parameters != nullptr &&
        this->_stateEstimatorData.parameters->controller_dt > 0.0) {
        dt = static_cast<T>(this->_stateEstimatorData.parameters->controller_dt);
    }

    // ── Khởi tạo state vector và foot positions về 0 ──────────────────────
    _xhat.setZero(); // state estimate [p; v; pf_FL; pf_FR; pf_HL; pf_HR]
    _ps.setZero();   // foot positions measured từ kinematics (buffer)
    _vs.setZero();   // foot velocities measured từ kinematics (buffer)

    // ══════════════════════════════════════════════════════════════════════
    // Process matrix A (18×18) - Kinematics
    // x[k+1] = A·x[k] + B·u[k]
    // ══════════════════════════════════════════════════════════════════════
    _A.setZero();

    // p[k+1] = p[k] + dt·v[k]
    // => dt là bước tích phân của Euler integration cho position.
    _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();      // p → p
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity(); // v → p

    // v[k+1] = v[k]  (acceleration từ IMU là control input u)
    _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity(); // v → v

    // pf[k+1] = pf[k]  (chân assumed đứng yên khi stance)
    _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity(); // pf → pf

    // ══════════════════════════════════════════════════════════════════════
    // Input matrix B (18×3) - IMU integration
    // u[k] = a_imu (IMU linear acceleration trong world frame)
    // v[k+1] = v[k] + dt·a_imu
    // => dt scale IMU acceleration thành velocity increment.
    // ══════════════════════════════════════════════════════════════════════
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity(); // a → v

    // ══════════════════════════════════════════════════════════════════════
    // Observation matrix C (28×18)
    // z = C·x
    // ══════════════════════════════════════════════════════════════════════

    // C1 = [I3 | 0]: trích xuất position từ state [p; v]
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();

    // C2 = [0 | I3]: trích xuất velocity từ state [p; v]
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();

    _C.setZero();

    // z[0:12] = p - pf[0..3]  (relative position body→foot, 4 chân)
    // Phần position: +C1 cho p
    _C.block(0, 0, 3, 6) = C1; // z[0:3]   += p  (FL)
    _C.block(3, 0, 3, 6) = C1; // z[3:6]   += p  (FR)
    _C.block(6, 0, 3, 6) = C1; // z[6:9]   += p  (HL)
    _C.block(9, 0, 3, 6) = C1; // z[9:12]  += p  (HR)

    // Phần foot: -I12 cho pf → z[0:12] = p - pf
    _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();

    // z[12:24] = v  (body velocity redundant từ 4 chân)
    _C.block(12, 0, 3, 6) = C2; // z[12:15] = v (FL)
    _C.block(15, 0, 3, 6) = C2; // z[15:18] = v (FR)
    _C.block(18, 0, 3, 6) = C2; // z[18:21] = v (HL)
    _C.block(21, 0, 3, 6) = C2; // z[21:24] = v (HR)

    // z[24:28] = foot heights pf[0..3]_z
    // Trích xuất thành phần z (index 8,11,14,17 trong state = pf_FL_z, pf_FR_z, ...)
    _C(24, 8) = T(1);  // z[24] = pf_FL_z  (xhat[8]  = FL foot z)
    _C(25, 11) = T(1); // z[25] = pf_FR_z  (xhat[11] = FR foot z)
    _C(26, 14) = T(1); // z[26] = pf_HL_z  (xhat[14] = HL foot z)
    _C(27, 17) = T(1); // z[27] = pf_HR_z  (xhat[17] = HR foot z)

    // ══════════════════════════════════════════════════════════════════════
    // Covariance matrices
    // ══════════════════════════════════════════════════════════════════════

    // P0 = 100·I18: initial state covariance
    // Giá trị lớn → uncertainty cao ban đầu → filter hội tụ nhanh
    _P.setIdentity();
    _P = T(100) * _P;

    // Q0: process noise covariance (18×18)
    // Phản ánh mức độ tin tưởng vào process model
    _Q0.setIdentity();
    // Process noise Q — position term
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity(); // position noise nhỏ
    // Process noise Q — velocity term
    _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity(); // velocity noise ~ g/20
    // Process noise Q — foot position term
    _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity(); // foot position noise

    // R0: measurement noise covariance (28×28)
    // Khởi tạo = I, được scale theo contact state trong run()
    // Chân swing: R tăng cao → ít tin vào measurement của chân đó
    // Chân stance: R giữ nhỏ → tin tưởng measurement
    _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator()
{
}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run()
{
    T process_noise_pimu = 0.02;          // imu_process_noise_position
    T process_noise_vimu = 0.02;          // imu_process_noise_velocity
    T process_noise_pfoot = 0.002;        // foot_process_noise_position
    T sensor_noise_pimu_rel_foot = 0.001; // foot_sensor_noise_position
    T sensor_noise_vimu_rel_foot = 0.1;   // foot_sensor_noise_velocity
    T sensor_noise_zfoot = 0.001;         // foot_height_sensor_noise

    Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
    Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
    Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
    Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

    Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
    R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
    R.block(12, 12, 12, 12) = _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
    R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    Vec3<T> g(0, 0, T(-9.81));
    Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
    // in old code, Rbod * se_acc + g
    Vec3<T> a = this->_stateEstimatorData.result->aWorld + g;

    // std::cout << "A WORLD" <<
    // this->_stateEstimatorData.result->aWorld.transpose() << std::endl;
    // std::ofstream fp;
    // fp.open("acceleration.txt", std::ofstream::app);
    // if(!fp){
    //   std::ofstream fpout("acceleration.txt");
    //   fpout << a(0) << "," << a(1) << "," << a(2) << ",";
    //   fp.close();
    //   fpout.close();
    // }else{
    //   fp << a(0) << "," << a(1) << "," << a(2) << std::endl;
    //   fp.close();
    // }

    Vec4<T> pzs = Vec4<T>::Zero();
    Vec4<T> trusts = Vec4<T>::Zero();
    Vec3<T> p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    for (int i = 0; i < 4; i++)
    {
        int i1 = 3 * i;
        Quadruped<T> &quadruped = *(this->_stateEstimatorData.legControllerData->quadruped);
        Vec3<T> ph = quadruped.getHipLocation(i); // hip positions relative to CoM

        // hw_i->leg_controller->leg_datas[i].p;
        Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p; //足端位置在机身坐标系中

        // hw_i->leg_controller->leg_datas[i].v;
        Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v; //足端速度在机身坐标系
        // std::cout << "leg v =" << dp_rel[0] << " " << dp_rel[1] << " " <<
        // dp_rel[2] << std::endl;
        Vec3<T> p_f = Rbod * p_rel; //足端位置在世界坐标系中
        Vec3<T> dp_f =
            Rbod * (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel); //足端速度在世界坐标系中

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        T trust = T(1);
        T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
        // T trust_window = T(0.25);
        T trust_window = T(0.2);

        if (phase < trust_window)
        {
            trust = phase / trust_window;
        }
        else if (phase > (T(1) - trust_window))
        {
            trust = (T(1) - phase) / trust_window;
        }
        // T high_suspect_number(1000);
        T high_suspect_number(100);

        // printf("Trust %d: %.3f\n", i, trust);
        Q.block(qindex, qindex, 3, 3) = (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
        R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block(rindex2, rindex2, 3, 3) =
            (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
        R(rindex3, rindex3) = (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

        trusts(i) = trust;

        _ps.segment(i1, 3) = -p_f;
        _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Eigen::Matrix<T, 28, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * a;
    Eigen::Matrix<T, 18, 18> At = _A.transpose();
    Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
    Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
    Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
    Eigen::Matrix<T, 28, 1> ey = y - yModel;
    Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;

    Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
    _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

    Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
    _P = (_P + Pt) / T(2);

    if (_P.block(0, 0, 2, 2).determinant() > T(0.000001))
    {
        _P.block(0, 2, 2, 16).setZero();
        _P.block(2, 0, 16, 2).setZero();
        _P.block(0, 0, 2, 2) /= T(10);
    }

    this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
    // this->_stateEstimatorData.result->position[0] =
    // this->_stateEstimatorData.vectorNavData->com_pos[0];
    // this->_stateEstimatorData.result->position[1] =
    // this->_stateEstimatorData.vectorNavData->com_pos[1];
    // this->_stateEstimatorData.result->position[2] =
    // this->_stateEstimatorData.vectorNavData->com_pos[2];
    this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
    // this->_stateEstimatorData.result->vWorld[0] =
    // this->_stateEstimatorData.vectorNavData->com_vel[0];
    // this->_stateEstimatorData.result->vWorld[1] =
    // this->_stateEstimatorData.vectorNavData->com_vel[1];
    // this->_stateEstimatorData.result->vWorld[2] =
    // this->_stateEstimatorData.vectorNavData->com_vel[2];
    this->_stateEstimatorData.result->vBody =
        this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;

    // std::cout << "pos = " <<
    // this->_stateEstimatorData.result->position.transpose() << std::endl;
    // std::cout << "vWorld = " <<
    // this->_stateEstimatorData.result->vWorld.transpose() << std::endl;
    // std::cout << "vBody = " <<
    // this->_stateEstimatorData.result->vBody.transpose() << std::endl;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;

/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run()
{
    this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
    this->_stateEstimatorData.result->vWorld = this->_stateEstimatorData.result->rBody.transpose().template cast<T>() *
                                               this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
    this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
