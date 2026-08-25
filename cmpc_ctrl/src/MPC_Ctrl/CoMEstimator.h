#ifndef _COM_ESTIMATOR_H_
#define _COM_ESTIMATOR_H_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include "RobotConfig.h"

/*!
 * @class CoMEstimator
 * @brief Combined Online Center of Mass (CoM) and Total Mass Estimator:
 *        1. CoM Estimator: Exact Original 3D RLS on MPC Ground Reaction Forces (Fr_des).
 *        2. Mass Estimator: Direct Joint-Torque Ground Reaction Force / Gravity (Newton's 2nd Law).
 */
class CoMEstimator {
public:
    /*!
     * @brief Constructor
     * @param lambda RLS forgetting factor for CoM (0.99 ~ 1.0)
     * @param filter_alpha CoM low-pass filter coefficient (0.01 ~ 0.1)
     * @param deadband_threshold CoM deadband threshold [m] (e.g. 0.002m = 2mm)
     * @param initial_mass Nominal robot mass [kg]
     */
    explicit CoMEstimator(float lambda = 0.998f,
                          float filter_alpha = 0.05f,
                          float deadband_threshold = 0.002f,
                          float initial_mass = RobotConfig::MASS)
        : _lambda(lambda),
          _filter_alpha(filter_alpha),
          _deadband(deadband_threshold),
          _mass_nominal(initial_mass),
          _mass_raw(initial_mass),
          _mass_filtered(initial_mass),
          _mass_output(initial_mass),
          _last_F_actual_z(initial_mass * 9.81f),
          _r_com_raw(Eigen::Vector3f::Zero()),
          _r_com_filtered(Eigen::Vector3f::Zero()),
          _r_com_output(Eigen::Vector3f::Zero()),
          _P(Eigen::Matrix3f::Identity() * 0.1f),
          _prev_omega(Eigen::Vector3f::Zero()),
          _domega(Eigen::Vector3f::Zero()),
          _initialized(false)
    {
        reset();
    }

    void reset() {
        _r_com_raw.setZero();
        _r_com_filtered.setZero();
        _r_com_output.setZero();
        _mass_raw = _mass_nominal;
        _mass_filtered = _mass_nominal;
        _mass_output = _mass_nominal;
        _last_F_actual_z = _mass_nominal * 9.81f;
        _P = Eigen::Matrix3f::Identity() * 0.1f;
        _prev_omega.setZero();
        _domega.setZero();
        _initialized = false;
    }

    /*!
     * @brief Compute Ground Reaction Force (GRF) in body/leg frame from joint torques and Jacobian:
     *        f_grf = - (J^T)^-1 * (tau_measured - tau_friction)
     * @param J 3x3 geometric Jacobian of the leg (v_foot = J * qdot)
     * @param tau_meas 3x1 measured joint torque [Abad, Hip, Knee] (Nm)
     * @param qd 3x1 joint velocity (rad/s)
     * @return 3x1 Ground Reaction Force vector in leg frame (N, fz > 0 upward)
     */
    static Eigen::Vector3f computeFootForceFromJointTorque(const Eigen::Matrix3f& J,
                                                          const Eigen::Vector3f& tau_meas,
                                                          const Eigen::Vector3f& qd)
    {
        // 1. Bù ma sát Coulomb nhỏ (~0.15 Nm) và ma sát nhớt
        Eigen::Vector3f tau_fric = Eigen::Vector3f::Zero();
        for (int i = 0; i < 3; i++) {
            float sgn = (qd[i] > 0.05f) ? 1.0f : ((qd[i] < -0.05f) ? -1.0f : 0.0f);
            tau_fric[i] = 0.15f * sgn + 0.01f * qd[i];
        }
        Eigen::Vector3f tau_clean = tau_meas - tau_fric;

        // 2. Tránh điểm kỳ dị
        float det = J.determinant();
        if (std::fabs(det) < 1e-4f) {
            return Eigen::Vector3f::Zero();
        }

        // 3. Tính lực chân: f_foot = (J^T)^-1 * tau_clean
        Eigen::Vector3f f_foot = J.transpose().colPivHouseholderQr().solve(tau_clean);

        // Ground Reaction Force (GRF hướng lên +Z): f_grf = -f_foot
        Eigen::Vector3f f_grf = -f_foot;
        if (f_grf[2] < 0.0f && f_foot[2] > 0.0f) {
            f_grf = f_foot;
        }
        return f_grf;
    }

    /*!
     * @brief Update both CoM and Mass estimates
     * @param p_feet_body 3x4 foot positions in body frame [m]
     * @param f_feet_mpc_body 3x4 MPC reaction forces in body frame [N] (from Fr_des)
     * @param f_feet_actual_world 3x4 actual reaction forces in world frame [N] (from joint torques)
     * @param omega_body 3x1 angular velocity in body frame [rad/s]
     * @param dt Time step [s]
     * @param I_body 3x3 body inertia matrix [kg.m^2]
     * @param contact_states 4x1 contact state (1 for stance, 0 for swing)
     */
    void update(const Eigen::Matrix<float, 3, 4>& p_feet_body,
                const Eigen::Matrix<float, 3, 4>& f_feet_mpc_body,
                const Eigen::Matrix<float, 3, 4>& f_feet_actual_world,
                const Eigen::Vector3f& omega_body,
                float dt,
                const Eigen::Matrix3f& I_body,
                const Eigen::Vector4f& contact_states)
    {
        if (!_initialized) {
            _prev_omega = omega_body;
            _initialized = true;
            return;
        }

        // ── Step 1: Cập nhật gia tốc góc ─────────────────────────────────
        updateAngularAcceleration(omega_body, dt);

        // ── Step 2: Ước lượng khối lượng từ phản lực khớp thực tế ─────────
        estimateTotalMassFromTorques(f_feet_actual_world, contact_states);

        // ── Step 3: Ước lượng trọng tâm CoM (Thuật toán RLS gốc với Fr_des) ──
        estimateCoMOriginalRLS(p_feet_body, f_feet_mpc_body, contact_states, omega_body, I_body);

        // ── Step 4: Lọc dao động và Deadband cho CoM ──────────────────────
        applyCoMFiltering();
    }

    /*!
     * @brief Update mass estimate only (CoM estimation disabled)
     * @param f_feet_actual_world 3x4 actual reaction forces in world frame [N] (from joint torques)
     * @param contact_states 4x1 contact state (1 for stance, 0 for swing)
     */
    void updateMassOnly(const Eigen::Matrix<float, 3, 4>& f_feet_actual_world,
                        const Eigen::Vector4f& contact_states)
    {
        estimateTotalMassFromTorques(f_feet_actual_world, contact_states);
        _r_com_raw.setZero();
        _r_com_filtered.setZero();
        _r_com_output.setZero();
    }

    // ── Getters (Encapsulation) ──────────────────────────────────────────
    const Eigen::Vector3f& getCoMOffset() const { return _r_com_output; }
    const Eigen::Vector3f& getCoMOffsetRaw() const { return _r_com_raw; }
    float getEstimatedMass() const { return _mass_output; }
    float getEstimatedMassRaw() const { return _mass_raw; }
    float getTotalSupportForce() const { return _last_F_actual_z; }

private:
    /*!
     * @brief Step 1: Update angular acceleration with EMA low-pass filter
     */
    void updateAngularAcceleration(const Eigen::Vector3f& omega_body, float dt) {
        Eigen::Vector3f raw_domega = (omega_body - _prev_omega) / std::max(dt, 1e-4f);
        _prev_omega = omega_body;
        _domega = 0.15f * raw_domega + 0.85f * _domega;
    }

    /*!
     * @brief Step 2: Estimate total robot mass using measured joint-torque forces
     */
    void estimateTotalMassFromTorques(const Eigen::Matrix<float, 3, 4>& f_feet_actual_world,
                                     const Eigen::Vector4f& contact_states)
    {
        float F_total_world_z = 0.0f;
        int stance_count = 0;

        for (int i = 0; i < 4; i++) {
            if (contact_states[i] > 0.5f) {
                F_total_world_z += std::fabs(f_feet_actual_world(2, i));
                stance_count++;
            }
        }

        _last_F_actual_z = F_total_world_z;

        if (stance_count >= 2 && F_total_world_z > 20.0f) {
            _mass_raw = F_total_world_z / 9.81f;
            // Giới hạn dải khối lượng an toàn cho Lite3 [9.0 kg, 25.0 kg]
            _mass_raw = std::max(9.0f, std::min(25.0f, _mass_raw));

            // Lọc thông thấp EMA (alpha = 0.02) để ổn định
            _mass_filtered = 0.98f * _mass_filtered + 0.02f * _mass_raw;

            // Deadband đầu ra 0.05 kg
            if (std::fabs(_mass_filtered - _mass_output) > 0.05f) {
                _mass_output = _mass_filtered;
            }
        }
    }

    /*!
     * @brief Step 3: Exact Original Recursive Least Squares (RLS) CoM Estimator
     */
    void estimateCoMOriginalRLS(const Eigen::Matrix<float, 3, 4>& p_feet_body,
                               const Eigen::Matrix<float, 3, 4>& f_feet_mpc_body,
                               const Eigen::Vector4f& contact_states,
                               const Eigen::Vector3f& omega_body,
                               const Eigen::Matrix3f& I_body)
    {
        Eigen::Vector3f F_total_body = Eigen::Vector3f::Zero();
        Eigen::Vector3f tau_feet = Eigen::Vector3f::Zero();
        int stance_count = 0;

        for (int i = 0; i < 4; i++) {
            if (contact_states[i] > 0.5f) {
                Eigen::Vector3f f_i = f_feet_mpc_body.col(i);
                Eigen::Vector3f p_i = p_feet_body.col(i);
                F_total_body += f_i;
                tau_feet += p_i.cross(f_i);
                stance_count++;
            }
        }

        // Chỉ cập nhật khi robot đang chống chân ổn định
        if (stance_count < 2 || F_total_body[2] < 20.0f) {
            return;
        }

        Eigen::Vector3f tau_inertial = I_body * _domega + omega_body.cross(I_body * omega_body);
        Eigen::Vector3f y = tau_feet - tau_inertial;

        Eigen::Matrix3f H;
        H <<              0.0f,  F_total_body[2], -F_total_body[1],
            -F_total_body[2],              0.0f,  F_total_body[0],
             F_total_body[1], -F_total_body[0],              0.0f;

        Eigen::Matrix3f S = H * _P * H.transpose() + Eigen::Matrix3f::Identity() * 0.1f;
        Eigen::Matrix3f K = _P * H.transpose() * S.inverse();

        _r_com_raw += K * (y - H * _r_com_raw);

        // Clamping theo kích thước Lite3 gốc
        _r_com_raw[0] = std::max(-0.08f, std::min(0.08f, _r_com_raw[0]));
        _r_com_raw[1] = std::max(-0.04f, std::min(0.04f, _r_com_raw[1]));
        _r_com_raw[2] = std::max(-0.05f, std::min(0.05f, _r_com_raw[2]));

        _P = (Eigen::Matrix3f::Identity() - K * H) * _P / _lambda;
        for (int i = 0; i < 3; i++) {
            _P(i, i) = std::max(1e-4f, std::min(10.0f, _P(i, i)));
        }
    }

    /*!
     * @brief Step 4: EMA Low-pass and Deadband filter for CoM
     */
    void applyCoMFiltering() {
        _r_com_filtered = (1.0f - _filter_alpha) * _r_com_filtered + _filter_alpha * _r_com_raw;
        for (int i = 0; i < 3; i++) {
            if (std::fabs(_r_com_filtered[i] - _r_com_output[i]) > _deadband) {
                _r_com_output[i] = _r_com_filtered[i];
            }
        }
    }

    // ── Member Variables ─────────────────────────────────────────────────────
    float _lambda;
    float _filter_alpha;
    float _deadband;
    float _mass_nominal;
    float _mass_raw;
    float _mass_filtered;
    float _mass_output;
    float _last_F_actual_z;

    Eigen::Vector3f _r_com_raw;
    Eigen::Vector3f _r_com_filtered;
    Eigen::Vector3f _r_com_output;

    Eigen::Matrix3f _P;
    Eigen::Vector3f _prev_omega;
    Eigen::Vector3f _domega;
    bool _initialized;
};

#endif // _COM_ESTIMATOR_H_
