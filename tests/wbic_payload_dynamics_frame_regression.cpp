#include <array>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "WbicControllerInternal.hpp"

namespace {

Eigen::Matrix3d CrossMatrix(const Eigen::Vector3d& r)
{
    Eigen::Matrix3d result;
    result <<   0.0, -r.z(),  r.y(),
              r.z(),    0.0, -r.x(),
             -r.y(),  r.x(),    0.0;
    return result;
}

bool CheckOrientation(const char* name,
                      const Eigen::Matrix3d& R_body,
                      const wbic::PayloadConfig& payload,
                      const Eigen::Matrix<double, wbic::kVelocityDimension,
                                          wbic::kVelocityDimension>& expected_M)
{
    Eigen::Matrix<double, wbic::kVelocityDimension, wbic::kVelocityDimension> M =
        Eigen::Matrix<double, wbic::kVelocityDimension,
                      wbic::kVelocityDimension>::Zero();
    wbic::GeneralizedVelocity h = wbic::GeneralizedVelocity::Zero();

    wbic::detail::AugmentPayloadDynamicsBodyFrame(payload, R_body, &M, &h);

    const Eigen::Vector3d f_g_world(0.0, 0.0, payload.mass * 9.81);
    const Eigen::Vector3d expected_force = R_body.transpose() * f_g_world;
    const Eigen::Vector3d expected_torque = payload.com_body.cross(expected_force);

    constexpr double kTolerance = 1e-12;
    if (!h.segment<3>(0).isApprox(expected_force, kTolerance)) {
        std::cerr << name << ": payload gravity is not expressed in BODY frame\n";
        return false;
    }
    if (!h.segment<3>(3).isApprox(expected_torque, kTolerance)) {
        std::cerr << name << ": payload gravity torque is not expressed in BODY frame\n";
        return false;
    }
    if (!M.isApprox(expected_M, kTolerance)) {
        std::cerr << name << ": payload mass matrix depends on WORLD-frame lever arm\n";
        return false;
    }
    return true;
}

}  // namespace

int main()
{
    wbic::PayloadConfig payload;
    payload.enabled = true;
    payload.mass = 2.3;
    payload.com_body = Eigen::Vector3d(0.17, -0.06, 0.11);

    const Eigen::Matrix3d r_cross = CrossMatrix(payload.com_body);
    Eigen::Matrix<double, wbic::kVelocityDimension, wbic::kVelocityDimension> expected_M =
        Eigen::Matrix<double, wbic::kVelocityDimension,
                      wbic::kVelocityDimension>::Zero();
    expected_M.block<3, 3>(0, 0) = payload.mass * Eigen::Matrix3d::Identity();
    expected_M.block<3, 3>(3, 3) = payload.mass * r_cross.transpose() * r_cross;
    expected_M.block<3, 3>(0, 3) = payload.mass * r_cross.transpose();
    expected_M.block<3, 3>(3, 0) = payload.mass * r_cross;

    constexpr double kTenDegrees = 10.0 * 3.14159265358979323846 / 180.0;
    const std::array<std::pair<const char*, Eigen::Matrix3d>, 3> orientations{{
        {"R=I", Eigen::Matrix3d::Identity()},
        {"pitch=10deg", Eigen::AngleAxisd(kTenDegrees, Eigen::Vector3d::UnitY()).toRotationMatrix()},
        {"roll=10deg", Eigen::AngleAxisd(kTenDegrees, Eigen::Vector3d::UnitX()).toRotationMatrix()},
    }};

    for (const auto& orientation : orientations) {
        if (!CheckOrientation(
                orientation.first, orientation.second, payload, expected_M)) {
            return 1;
        }
    }

    std::cout << "[TEST] wbic_payload_dynamics_frame_regression PASSED\n";
    return 0;
}
