#pragma once

#include <Eigen/Dense>

#include "CentroidalModel.hpp"
#include "WbcType.h"

namespace wbic::detail {

void AugmentPayloadDynamicsBodyFrame(
    const PayloadConfig& payload_config,
    const Eigen::Matrix3d& R_body,
    Eigen::Matrix<double, kVelocityDimension, kVelocityDimension>* M,
    GeneralizedVelocity* h) noexcept;

}  // namespace wbic::detail
