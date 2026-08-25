#pragma once

#ifndef CMPC_TELEMETRY_H
#define CMPC_TELEMETRY_H

#include <stdint.h>

/*!
 * @brief Plain Old Data (POD) struct for high-frequency CMPC & State Estimator telemetry.
 * Fixed size, zero dynamic allocations, safe for real-time robotic deployment.
 */
struct CmpcTelemetryData {
    // 1. Reference & Command (m/s, rad/s)
    float cmd_vx;
    float cmd_vy;
    float cmd_yaw_rate;
    float des_vx;
    float des_vy;
    float des_yaw_rate;

    // 2. Linear KF State Estimation (world frame & body frame)
    float kf_pos[3];            // x, y, z [m] (world frame)
    float kf_vel_world[3];      // vx, vy, vz [m/s] (world frame)
    float kf_vel_body[3];       // vx, vy, vz [m/s] (body frame)
    float kf_rpy[3];            // roll, pitch, yaw [rad]
    float kf_omega_body[3];     // wx, wy, wz [rad/s] (body frame)
    float kf_acc_body[3];       // ax, ay, az [m/s^2] (body frame)
    float kf_contact_prob[4];   // Scheduled contact/phase: FR, FL, HR, HL

    // 3. MPC & CoM / Mass Estimator
    float est_mass_raw;         // Instantaneous mass from joint torques [kg]
    float est_mass_filtered;    // Filtered mass used by MPC [kg]
    float est_com_body[3];      // CoM offset dx, dy, dz [m] (body frame)
    float total_support_force_z;// Sum of actual vertical support forces [N]

    // 4. Foot Kinematics & Reaction Forces (4 legs: 0=FR, 1=FL, 2=HR, 3=HL)
    float p_feet_des_world[4][3]; // Target touchdown position in world [m]
    float p_feet_actual_body[4][3];// Current foot position in body frame [m]
    float f_mpc_des_world[4][3];  // Optimal MPC reaction force in world [N]
    float f_act_est_world[4][3];  // Actual reaction force from motor torques in world [N]

    // 5. Read-only Generalized Momentum Observer (Pinocchio generalized order)
    uint8_t gmo_valid;
    uint8_t gmo_initialized;
    uint8_t gmo_ready;
    uint8_t gmo_invalid_reason;
    uint32_t gmo_samples_since_reset;
    uint64_t gmo_reset_count;
    float gmo_momentum[18];
    float gmo_momentum_hat[18];
    float gmo_residual[18];
    uint8_t grf_valid;
    uint8_t grf_ready;
    uint8_t grf_invalid_reason;
    uint8_t grf_leg_valid[4];
    float grf_jacobian_quality[4];
    float gmo_force_raw_world[4][3]; // Uncalibrated solve, FR, FL, HR, HL
    float gmo_force_world[4][3];     // Bias-corrected and filtered
    float gmo_force_norm[4];
    float gmo_fz[4];
    float scheduled_contact[4];

    // 6. Simulation-only contact ground truth (FR, FL, HR, HL)
    uint8_t gt_valid;
    float gt_contact[4];
    float gt_force_world[4][3];

    // 7. Computation Timing (ms)
    float t_est_ms;
    float t_gmo_ms;
    float t_grf_ms;
    float t_evidence_p99_ms;
    float t_mpc_ms;
    float t_total_ms;
};

#endif // CMPC_TELEMETRY_H
