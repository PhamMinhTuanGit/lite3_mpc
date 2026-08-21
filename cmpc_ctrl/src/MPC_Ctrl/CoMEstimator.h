#ifndef _COM_ESTIMATOR_H_
#define _COM_ESTIMATOR_H_

#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <cmath>
#include "RobotConfig.h"

/*!
 * @class CoMEstimator
 * @brief Combined Online Center of Mass (CoM) and Total Mass estimator using RLS
 *        with Low-Pass (EMA) and Deadband Filtering to reject small oscillations.
 */
class CoMEstimator {
public:
    /*!
     * @param lambda Forgetting factor for RLS (0.99 ~ 1.0)
     * @param filter_alpha Low-pass filter coefficient for CoM (0.01 ~ 0.1)
     * @param deadband_threshold Minimum change threshold [m] for CoM (e.g. 0.002m = 2mm)
     * @param initial_mass Nominal robot mass [kg]
     */
    CoMEstimator(float lambda = 0.998f, float filter_alpha = 0.05f, float deadband_threshold = 0.002f, float initial_mass = RobotConfig::MASS)
        : _lambda(lambda), _filter_alpha(filter_alpha), _deadband(deadband_threshold),
          _mass_nominal(initial_mass), _mass_raw(initial_mass), _mass_filtered(initial_mass), _mass_output(initial_mass) {
        reset();
    }

    void reset() {
        _r_com_raw.setZero();
        _r_com_filtered.setZero();
        _r_com_output.setZero();
        _mass_raw = _mass_nominal;
        _mass_filtered = _mass_nominal;
        _mass_output = _mass_nominal;
        _P = Eigen::Matrix3f::Identity() * 0.1f;
        _prev_omega.setZero();
        _domega.setZero();
        _initialized = false;
    }

    /*!
     * @brief Update CoM offset and Mass estimates using RLS and vertical force balance
     * @param p_feet_body 3x4 matrix of foot positions in body frame [m]
     * @param f_feet_body 3x4 matrix of ground reaction forces in body frame [N]
     * @param f_feet_world 3x4 matrix of ground reaction forces in world frame [N]
     * @param omega_body 3x1 angular velocity in body frame [rad/s]
     * @param a_world_z Vertical acceleration in world frame [m/s^2]
     * @param dt Time step [s]
     * @param I_body 3x3 body inertia matrix [kg.m^2]
     * @param contact_states 4x1 contact state (1 for stance, 0 for swing)
     */
    void update(const Eigen::Matrix<float, 3, 4>& p_feet_body,
                const Eigen::Matrix<float, 3, 4>& f_feet_body,
                const Eigen::Matrix<float, 3, 4>& f_feet_world,
                const Eigen::Vector3f& omega_body,
                float a_world_z,
                float dt,
                const Eigen::Matrix3f& I_body,
                const Eigen::Vector4f& contact_states)
    {
        if (!_initialized) {
            _prev_omega = omega_body;
            _initialized = true;
            return;
        }

        // 1. Tính và lọc gia tốc góc: domega = (omega - prev_omega)/dt
        Eigen::Vector3f raw_domega = (omega_body - _prev_omega) / std::max(dt, 1e-4f);
        _prev_omega = omega_body;
        _domega = 0.15f * raw_domega + 0.85f * _domega; // Low-pass filter

        // 2. Tổng hợp lực và mô-men từ các chân tiếp đất
        Eigen::Vector3f F_total_body = Eigen::Vector3f::Zero();
        Eigen::Vector3f tau_feet = Eigen::Vector3f::Zero();
        float F_total_world_z = 0.0f;
        int stance_count = 0;

        for (int i = 0; i < 4; i++) {
            if (contact_states[i] > 0.5f) {
                Eigen::Vector3f f_i_body = f_feet_body.col(i);
                Eigen::Vector3f p_i_body = p_feet_body.col(i);
                F_total_body += f_i_body;
                tau_feet += p_i_body.cross(f_i_body);
                F_total_world_z += f_feet_world(2, i);
                stance_count++;
            }
        }

        // Chỉ cập nhật khi robot đang chống chân ổn định
        if (stance_count < 2 || F_total_body[2] < 40.0f) {
            return;
        }

        // ── 3. ƯỚC LƯỢNG TỔNG KHỐI LƯỢNG (Mass Estimation) ──────────────
        // Theo định luật 2 Newton theo trục đứng: Fz_world = m * (g + az_world) = m * a_world_z
        // Lưu ý: a_world_z từ IMU accelerometer trong seResult đã là proper acceleration (g + az_world).
        float eff_g = a_world_z;
        if (eff_g > 5.0f && F_total_world_z > 40.0f) {
            _mass_raw = F_total_world_z / eff_g;
            _mass_raw = std::max(8.0f, std::min(25.0f, _mass_raw)); // Giới hạn dải khối lượng Lite3 [8kg, 25kg]

            // Lọc thông thấp cho khối lượng (lọc chậm, alpha = 0.02)
            _mass_filtered = 0.98f * _mass_filtered + 0.02f * _mass_raw;

            // Deadband cho khối lượng (chỉ cập nhật khi đổi > 0.15 kg)
            if (std::fabs(_mass_filtered - _mass_output) > 0.15f) {
                _mass_output = _mass_filtered;
            }
        }

        // ── 4. ƯỚC LƯỢNG VỊ TRÍ TRỌNG TÂM (CoM RLS Estimation) ───────────
        Eigen::Vector3f tau_inertial = I_body * _domega + omega_body.cross(I_body * omega_body);
        Eigen::Vector3f y = tau_feet - tau_inertial;

        Eigen::Matrix3f H;
        H <<              0.0f,  F_total_body[2], -F_total_body[1],
            -F_total_body[2],              0.0f,  F_total_body[0],
             F_total_body[1], -F_total_body[0],              0.0f;

        Eigen::Matrix3f S = H * _P * H.transpose() + Eigen::Matrix3f::Identity() * 0.1f;
        Eigen::Matrix3f K = _P * H.transpose() * S.inverse();

        _r_com_raw += K * (y - H * _r_com_raw);

        // Clamping theo kích thước Lite3
        _r_com_raw[0] = std::max(-0.08f, std::min(0.08f, _r_com_raw[0]));
        _r_com_raw[1] = std::max(-0.04f, std::min(0.04f, _r_com_raw[1]));
        _r_com_raw[2] = std::max(-0.05f, std::min(0.05f, _r_com_raw[2]));

        _P = (Eigen::Matrix3f::Identity() - K * H) * _P / _lambda;
        for (int i = 0; i < 3; i++) {
            _P(i, i) = std::max(1e-4f, std::min(10.0f, _P(i, i)));
        }

        // ── 5. Lọc dao động nhỏ CoM ─────────────────────────────────────
        _r_com_filtered = (1.0f - _filter_alpha) * _r_com_filtered + _filter_alpha * _r_com_raw;
        for (int i = 0; i < 3; i++) {
            if (std::fabs(_r_com_filtered[i] - _r_com_output[i]) > _deadband) {
                _r_com_output[i] = _r_com_filtered[i];
            }
        }
    }

    const Eigen::Vector3f& getCoMOffset() const { return _r_com_output; }
    const Eigen::Vector3f& getCoMOffsetRaw() const { return _r_com_raw; }
    float getEstimatedMass() const { return _mass_output; }
    float getEstimatedMassRaw() const { return _mass_raw; }

private:
    float _lambda;
    float _filter_alpha;
    float _deadband;
    float _mass_nominal;
    float _mass_raw;
    float _mass_filtered;
    float _mass_output;

    Eigen::Vector3f _r_com_raw;
    Eigen::Vector3f _r_com_filtered;
    Eigen::Vector3f _r_com_output;

    Eigen::Matrix3f _P;
    Eigen::Vector3f _prev_omega;
    Eigen::Vector3f _domega;
    bool _initialized;
};

#endif // _COM_ESTIMATOR_H_
