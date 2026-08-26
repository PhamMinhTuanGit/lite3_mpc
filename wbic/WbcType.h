#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Core>

#include <pinocchio/multibody/model.hpp>

/**
 * Common input types for the Lite3 WBIC pipeline.
 *
 * All public leg-indexed data uses the controller order [FR, FL, HR, HL].
 * Quantities consumed by Pinocchio/WBIC use double precision and fixed-size
 * storage so constructing these inputs does not allocate in the control loop.
 */
namespace wbic {

inline constexpr int kNumLegs = 4;
inline constexpr int kContactDimension = 3;
inline constexpr int kConfigurationDimension = 19;
inline constexpr int kVelocityDimension = 18;

enum class Leg : std::size_t {
    FR = 0,
    FL = 1,
    HR = 2,
    HL = 3,
};

using Configuration = Eigen::Matrix<double, kConfigurationDimension, 1>;
using GeneralizedVelocity = Eigen::Matrix<double, kVelocityDimension, 1>;
using FootVector = Eigen::Vector3d;
using ContactPhase = Eigen::Matrix<double, kNumLegs, 1>;
using FootJacobian =
    Eigen::Matrix<double, kContactDimension, kVelocityDimension>;
using ContactFlags = std::array<bool, kNumLegs>;
using FootFrameIds = std::array<pinocchio::FrameIndex, kNumLegs>;
using FootVectorArray = std::array<FootVector, kNumLegs>;
using FootJacobianArray = std::array<FootJacobian, kNumLegs>;

enum class InputStatus {
    Ok = 0,
    MissingModel,
    InvalidModelDimension,
    NonFiniteValue,
    InvalidBaseQuaternion,
    InvalidFootFrame,
    DuplicateFootFrame,
    InvalidContactPhase,
};

inline Configuration neutralConfiguration()
{
    Configuration q = Configuration::Zero();
    // Pinocchio free-flyer convention: [x, y, z, qx, qy, qz, qw].
    q[6] = 1.0;
    return q;
}

/** Inputs of the Dynamics module described in document/IPO.md. */
struct DynamicsInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Configuration q = neutralConfiguration();
    GeneralizedVelocity v = GeneralizedVelocity::Zero();

    // Non-owning: the model must outlive this input object.
    const pinocchio::Model* model = nullptr;
    FootFrameIds foot_ids{{
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
        std::numeric_limits<pinocchio::FrameIndex>::max(),
    }};

    DynamicsInput() = default;

    DynamicsInput(const pinocchio::Model& model_in,
                  const Configuration& q_in,
                  const GeneralizedVelocity& v_in,
                  const FootFrameIds& foot_ids_in) noexcept
        : q(q_in), v(v_in), model(&model_in), foot_ids(foot_ids_in)
    {
    }

    InputStatus validate(double quaternion_tolerance = 1e-6) const noexcept
    {
        if (model == nullptr) {
            return InputStatus::MissingModel;
        }
        if (model->nq != kConfigurationDimension
            || model->nv != kVelocityDimension) {
            return InputStatus::InvalidModelDimension;
        }
        if (!q.allFinite() || !v.allFinite()) {
            return InputStatus::NonFiniteValue;
        }

        const double quaternion_norm = q.template segment<4>(3).norm();
        if (!std::isfinite(quaternion_norm)
            || std::abs(quaternion_norm - 1.0) > quaternion_tolerance) {
            return InputStatus::InvalidBaseQuaternion;
        }

        for (std::size_t leg = 0; leg < foot_ids.size(); ++leg) {
            if (foot_ids[leg] >= model->nframes) {
                return InputStatus::InvalidFootFrame;
            }
            for (std::size_t previous = 0; previous < leg; ++previous) {
                if (foot_ids[leg] == foot_ids[previous]) {
                    return InputStatus::DuplicateFootFrame;
                }
            }
        }
        return InputStatus::Ok;
    }
};

/** Inputs of the Contact Assembly module described in document/IPO.md. */
struct ContactAssemblyInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactFlags contact{{false, false, false, false}};
    FootVectorArray f_mpc{};
    ContactPhase phase = ContactPhase::Zero();
    FootJacobianArray Jf{};
    FootVectorArray dJdq_f{};

    ContactAssemblyInput()
    {
        for (auto& force : f_mpc) {
            force.setZero();
        }
        for (auto& jacobian : Jf) {
            jacobian.setZero();
        }
        for (auto& bias_acceleration : dJdq_f) {
            bias_acceleration.setZero();
        }
    }

    InputStatus validate(double phase_tolerance = 1e-9) const noexcept
    {
        if (!phase.allFinite()) {
            return InputStatus::NonFiniteValue;
        }
        if ((phase.array() < -phase_tolerance).any()
            || (phase.array() > 1.0 + phase_tolerance).any()) {
            return InputStatus::InvalidContactPhase;
        }

        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (!f_mpc[leg].allFinite() || !Jf[leg].allFinite()
                || !dJdq_f[leg].allFinite()) {
                return InputStatus::NonFiniteValue;
            }
        }
        return InputStatus::Ok;
    }
};

}  // namespace wbic
