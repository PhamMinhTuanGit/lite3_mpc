#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

/**
 * Rigid-body model of the DeepRobotics Lite3.
 *
 * The model is constructed directly in C++ so the deployment executable does
 * not depend on Pinocchio's compiled URDF parser. Joint, link and frame names
 * intentionally match the Lite3 URDF/MJCF files in third_party.
 *
 * Public leg indexing follows the controller convention [FR, FL, HR, HL],
 * independently of the construction order used by the URDF [FL, FR, HL, HR].
 */
class Lite3Dynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int kNumLegs = 4;
    static constexpr int kJointsPerLeg = 3;
    static constexpr int kNumActuatedJoints = 12;
    static constexpr int kNq = 19;
    static constexpr int kNv = 18;

    enum class Leg : std::size_t {
        FR = 0,
        FL = 1,
        HR = 2,
        HL = 3
    };

    enum class LegJoint : std::size_t {
        HipX = 0,
        HipY = 1,
        Knee = 2
    };

    using Configuration = Eigen::Matrix<double, kNq, 1>;
    using Velocity = Eigen::Matrix<double, kNv, 1>;
    using MassMatrix = Eigen::Matrix<double, kNv, kNv>;
    using FootJacobian = Eigen::Matrix<double, 3, kNv>;
    using FootJacobianArray = std::array<FootJacobian, kNumLegs>;

    Lite3Dynamics();

    const pinocchio::Model& model() const noexcept { return model_; }
    const pinocchio::Data& data() const noexcept { return data_; }
    pinocchio::Data& data() noexcept { return data_; }

    pinocchio::FrameIndex footFrameId(Leg leg) const noexcept;
    pinocchio::JointIndex jointId(Leg leg, LegJoint joint) const noexcept;

    /** Compute and return the complete symmetric generalized mass matrix. */
    const Eigen::MatrixXd& computeMassMatrix(const Eigen::VectorXd& q);

    /** Compute h(q,v) = C(q,v)v + g(q). */
    const Eigen::VectorXd& computeNonlinearEffects(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v);

    /** Compute generalized gravity forces g(q). */
    const Eigen::VectorXd& computeGravity(const Eigen::VectorXd& q);

    /** Compute the Coriolis matrix satisfying h(q,v) = C(q,v)v + g(q). */
    const Eigen::MatrixXd& computeCoriolisMatrix(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v);

    /** Compute one world-aligned translational foot Jacobian. */
    const FootJacobian& computeFootJacobian(
        Leg leg,
        const Eigen::VectorXd& q);

    /** Compute all foot Jacobians with one joint-kinematics pass. */
    const FootJacobianArray& computeFootJacobians(const Eigen::VectorXd& q);

    /** Construct the fixed Lite3 model from the canonical URDF parameters. */
    static pinocchio::Model buildLite3Model();

private:
    void cacheAndValidateIds();
    void validateConfigurationSize(const Eigen::VectorXd& q) const;
    void validateVelocitySize(const Eigen::VectorXd& v) const;

    pinocchio::Model model_;
    pinocchio::Data data_;

    std::array<pinocchio::FrameIndex, kNumLegs> foot_ids_{};
    std::array<std::array<pinocchio::JointIndex, kJointsPerLeg>, kNumLegs>
        joint_ids_{};
    FootJacobianArray foot_jacobians_{};
};
