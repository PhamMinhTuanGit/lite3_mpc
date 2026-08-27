#pragma once

#include <Eigen/Dense>

namespace wbic {

/** Configuration for payload model compensation. */
struct PayloadConfig {
    bool enabled = false;
    double mass = 2.0;                                          // Payload mass [kg]
    Eigen::Vector3d com_body = Eigen::Vector3d(0.10, 0.0, 0.0); // Payload CoM offset in TORSO frame [m]
};

/** Whole-body centroidal state in world frame. */
struct CentroidalState {
    double mass = 11.9376;                                  // Total mass (robot + payload) [kg]
    Eigen::Vector3d com_world = Eigen::Vector3d::Zero();     // Whole-body CoM position in World frame [m]
    Eigen::Vector3d com_vel_world = Eigen::Vector3d::Zero(); // Whole-body CoM velocity in World frame [m/s]
    Eigen::Vector3d p_payload_world = Eigen::Vector3d::Zero();
    double payload_mass = 0.0;
    Eigen::Vector3d payload_offset_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_offset_body = Eigen::Vector3d::Zero();       // Whole-body CoM offset in TORSO frame [m]
};

/**
 * Computes payload-aware total mass and whole-body CoM in World frame.
 *
 * @param robot_mass Nominal robot mass from Pinocchio model [kg]
 * @param com_robot_world Robot whole-body CoM from Pinocchio [m]
 * @param vcom_robot_world Robot whole-body CoM velocity from Pinocchio [m/s]
 * @param p_body_world TORSO base position in World frame [m]
 * @param R_wb Rotation matrix of TORSO in World frame
 * @param v_body_world TORSO linear velocity in World frame [m/s]
 * @param omega_body_world TORSO angular velocity in World frame [rad/s]
 * @param payload_cfg Payload configuration
 * @return CentroidalState structure containing total mass and combined CoM
 */
inline CentroidalState ComputeCentroidalState(
    double robot_mass,
    const Eigen::Vector3d& com_robot_world,
    const Eigen::Vector3d& vcom_robot_world,
    const Eigen::Vector3d& p_body_world,
    const Eigen::Matrix3d& R_wb,
    const Eigen::Vector3d& v_body_world,
    const Eigen::Vector3d& omega_body_world,
    const PayloadConfig& payload_cfg) noexcept
{
    CentroidalState state;

    if (!payload_cfg.enabled || payload_cfg.mass <= 0.0) {
        state.mass = robot_mass;
        state.com_world = com_robot_world;
        state.com_vel_world = vcom_robot_world;
        state.p_payload_world = p_body_world;
        state.payload_mass = 0.0;
        state.payload_offset_body.setZero();
        state.com_offset_body = R_wb.transpose() * (state.com_world - p_body_world);
        return state;
    }

    const Eigen::Vector3d payload_pos_world = p_body_world + R_wb * payload_cfg.com_body;
    const Eigen::Vector3d payload_vel_world =
        v_body_world + omega_body_world.cross(R_wb * payload_cfg.com_body);

    const double total_mass = robot_mass + payload_cfg.mass;
    state.mass = total_mass;
    state.com_world = (robot_mass * com_robot_world + payload_cfg.mass * payload_pos_world) / total_mass;
    state.com_vel_world = (robot_mass * vcom_robot_world + payload_cfg.mass * payload_vel_world) / total_mass;
    state.p_payload_world = payload_pos_world;
    state.payload_mass = payload_cfg.mass;
    state.payload_offset_body = payload_cfg.com_body;
    state.com_offset_body = R_wb.transpose() * (state.com_world - p_body_world);

    return state;
}

} // namespace wbic
