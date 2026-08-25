#pragma once

#include <Eigen/Core>

#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Lite3DynamicModel.hpp"

struct Lite3MappedState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Lite3Dynamics::Configuration q = Lite3Dynamics::Configuration::Zero();
    Lite3Dynamics::Velocity v = Lite3Dynamics::Velocity::Zero();
    Eigen::Matrix<double, Lite3Dynamics::kNumActuatedJoints, 1> motor_torque =
        Eigen::Matrix<double, Lite3Dynamics::kNumActuatedJoints, 1>::Zero();
};

enum class Lite3StateMapStatus {
    Ok = 0,
    NonFiniteInput,
    InvalidRotation
};

/** Maps controller feedback [FR,FL,HR,HL] into Pinocchio coordinates. */
class Lite3StateMapper {
public:
    Lite3StateMapStatus map(
        const StateEstimate<float>& state,
        const LegControllerData<float> (&legs)[Lite3Dynamics::kNumLegs],
        const Lite3Dynamics& dynamics,
        Lite3MappedState& output) const noexcept;
};
