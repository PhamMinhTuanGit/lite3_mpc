#include "Lite3DynamicModel.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>

namespace {

using pinocchio::Frame;
using pinocchio::FrameIndex;
using pinocchio::Inertia;
using pinocchio::JointIndex;
using pinocchio::JointModelRevoluteUnaligned;
using pinocchio::Model;
using pinocchio::SE3;

constexpr double kExpectedMass = 11.9376;
constexpr double kMassTolerance = 1e-10;

struct LegParameters {
    const char* prefix;
    double body_x;
    double body_y;
    double hip_y;
    Eigen::Vector3d hip_com;
    Eigen::Vector3d hip_inertia_diagonal;
    Eigen::Vector3d thigh_com;
    Eigen::Vector3d shank_com;
};

Eigen::Matrix3d diagonalInertia(const Eigen::Vector3d& diagonal)
{
    return diagonal.asDiagonal();
}

Inertia makeInertia(
    double mass,
    const Eigen::Vector3d& com,
    const Eigen::Vector3d& inertia_diagonal)
{
    return Inertia(mass, com, diagonalInertia(inertia_diagonal));
}

JointIndex addRevoluteJoint(
    Model& model,
    JointIndex parent_joint,
    FrameIndex parent_frame,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& translation,
    const std::string& joint_name,
    const std::string& body_name,
    const Inertia& body_inertia,
    double lower_limit,
    double upper_limit,
    double effort_limit,
    double velocity_limit,
    FrameIndex& body_frame)
{
    Eigen::VectorXd max_effort(1);
    Eigen::VectorXd max_velocity(1);
    Eigen::VectorXd min_config(1);
    Eigen::VectorXd max_config(1);
    max_effort << effort_limit;
    max_velocity << velocity_limit;
    min_config << lower_limit;
    max_config << upper_limit;

    const JointIndex joint_id = model.addJoint(
        parent_joint,
        JointModelRevoluteUnaligned(axis),
        SE3(Eigen::Matrix3d::Identity(), translation),
        joint_name,
        max_effort,
        max_velocity,
        min_config,
        max_config);

    const FrameIndex joint_frame = model.addJointFrame(joint_id, parent_frame);
    model.appendBodyToJoint(joint_id, body_inertia, SE3::Identity());
    body_frame = model.addBodyFrame(
        body_name, joint_id, SE3::Identity(), static_cast<int>(joint_frame));
    return joint_id;
}

double totalMass(const Model& model)
{
    double mass = 0.0;
    for (const auto& inertia : model.inertias) {
        mass += inertia.mass();
    }
    return mass;
}

}  // namespace

Lite3Dynamics::Lite3Dynamics()
    : model_(buildLite3Model()), data_(model_)
{
    cacheAndValidateIds();
}

pinocchio::Model Lite3Dynamics::buildLite3Model()
{
    Model model;
    model.name = "Lite3";
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    model.gravity.angular().setZero();

    const JointIndex torso_id = model.addJoint(
        0,
        pinocchio::JointModelFreeFlyer(),
        SE3::Identity(),
        "floating_base");
    const FrameIndex torso_joint_frame = model.addJointFrame(torso_id);

    const Inertia torso_inertia = makeInertia(
        5.6056,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.02456, 0.05518, 0.07016));
    model.appendBodyToJoint(torso_id, torso_inertia, SE3::Identity());
    const FrameIndex torso_body_frame = model.addBodyFrame(
        "TORSO", torso_id, SE3::Identity(), static_cast<int>(torso_joint_frame));

    const Eigen::Vector3d thigh_inertia_diagonal(0.005736, 0.004960, 0.001436);
    const Eigen::Vector3d shank_inertia_diagonal(
        0.00089039, 0.00090672, 0.000031266);

    // Construction order deliberately matches the URDF/MJCF. Public lookup
    // uses named IDs and therefore remains [FR, FL, HR, HL].
    const std::array<LegParameters, kNumLegs> legs{{
        {"FL",  0.1745,  0.062,  0.09735,
         Eigen::Vector3d(-0.00601, -0.0066532, 0.00034295),
         Eigen::Vector3d(0.0003949, 0.0004028, 0.0004472),
         Eigen::Vector3d(-0.0039245, -0.014632, -0.025146),
         Eigen::Vector3d(0.0064794, -0.0000014535, -0.12157)},
        {"FR",  0.1745, -0.062, -0.09735,
         Eigen::Vector3d(-0.010579, 0.011358, 0.00048546),
         Eigen::Vector3d(0.0004472, 0.0004028, 0.0003949),
         Eigen::Vector3d(-0.0039245, 0.014632, -0.025146),
         Eigen::Vector3d(0.0064794, -0.0000014552, -0.12157)},
        {"HL", -0.1745,  0.062,  0.09735,
         Eigen::Vector3d(0.010905, -0.012636, 0.001051),
         Eigen::Vector3d(0.0003949, 0.0004028, 0.0004472),
         Eigen::Vector3d(-0.0039245, -0.014632, -0.025146),
         Eigen::Vector3d(0.0064794, -0.0000014558, -0.12157)},
        {"HR", -0.1745, -0.062, -0.09735,
         Eigen::Vector3d(0.010354, 0.011423, 0.00049498),
         // Canonical URDF value. The current MJCF swaps HR Ixx and Izz.
         Eigen::Vector3d(0.0004472, 0.0004028, 0.0003949),
         Eigen::Vector3d(-0.0039245, 0.014632, -0.025146),
         Eigen::Vector3d(0.0064794, -0.0000014529, -0.12157)}
    }};

    const Eigen::Vector3d hip_x_axis(-1.0, 0.0, 0.0);
    const Eigen::Vector3d pitch_axis(0.0, -1.0, 0.0);

    for (const LegParameters& leg : legs) {
        const std::string prefix(leg.prefix);

        FrameIndex hip_body_frame = 0;
        const JointIndex hip_id = addRevoluteJoint(
            model, torso_id, torso_body_frame, hip_x_axis,
            Eigen::Vector3d(leg.body_x, leg.body_y, 0.0),
            prefix + "_HipX_joint", prefix + "_HIP",
            makeInertia(0.550, leg.hip_com, leg.hip_inertia_diagonal),
            -0.523, 0.523, 24.0, 26.2, hip_body_frame);

        FrameIndex thigh_body_frame = 0;
        const JointIndex thigh_id = addRevoluteJoint(
            model, hip_id, hip_body_frame, pitch_axis,
            Eigen::Vector3d(0.0, leg.hip_y, 0.0),
            prefix + "_HipY_joint", prefix + "_THIGH",
            makeInertia(0.860, leg.thigh_com, thigh_inertia_diagonal),
            -2.67, 0.314, 24.0, 26.2, thigh_body_frame);

        FrameIndex shank_body_frame = 0;
        const JointIndex knee_id = addRevoluteJoint(
            model, thigh_id, thigh_body_frame, pitch_axis,
            Eigen::Vector3d(0.0, 0.0, -0.20),
            prefix + "_Knee_joint", prefix + "_SHANK",
            makeInertia(0.153, leg.shank_com, shank_inertia_diagonal),
            0.524, 2.792, 36.0, 17.3, shank_body_frame);

        const SE3 foot_placement(
            Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, -0.21012));

        // A frame has no inertia. Fold the fixed point-mass foot into the knee
        // joint at its fixed-link placement, then register the frame hierarchy.
        model.appendBodyToJoint(
            knee_id,
            Inertia(0.020, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()),
            foot_placement);

        const FrameIndex ankle_frame = model.addFrame(Frame(
            prefix + "_Ankle", knee_id, shank_body_frame, foot_placement,
            pinocchio::FIXED_JOINT));
        model.addBodyFrame(
            prefix + "_FOOT", knee_id, foot_placement,
            static_cast<int>(ankle_frame));
    }

    if (model.nq != kNq || model.nv != kNv || model.njoints != 14) {
        throw std::logic_error("Lite3 model has unexpected dimensions");
    }

    const double mass = totalMass(model);
    if (std::abs(mass - kExpectedMass) > kMassTolerance) {
        throw std::logic_error("Lite3 model has unexpected total mass");
    }

    return model;
}

void Lite3Dynamics::cacheAndValidateIds()
{
    const std::array<const char*, kNumLegs> prefixes{{"FR", "FL", "HR", "HL"}};

    for (std::size_t leg = 0; leg < prefixes.size(); ++leg) {
        const std::string prefix(prefixes[leg]);
        const std::array<std::string, kJointsPerLeg> joint_names{{
            prefix + "_HipX_joint",
            prefix + "_HipY_joint",
            prefix + "_Knee_joint"
        }};

        for (std::size_t joint = 0; joint < joint_names.size(); ++joint) {
            if (!model_.existJointName(joint_names[joint])) {
                throw std::logic_error("Missing Lite3 joint: " + joint_names[joint]);
            }
            joint_ids_[leg][joint] = model_.getJointId(joint_names[joint]);
        }

        const std::string foot_name = prefix + "_FOOT";
        if (!model_.existFrame(foot_name, pinocchio::BODY)) {
            throw std::logic_error("Missing Lite3 foot frame: " + foot_name);
        }
        foot_ids_[leg] = model_.getFrameId(foot_name, pinocchio::BODY);
    }
}

pinocchio::FrameIndex Lite3Dynamics::footFrameId(Leg leg) const noexcept
{
    return foot_ids_[static_cast<std::size_t>(leg)];
}

pinocchio::JointIndex Lite3Dynamics::jointId(Leg leg, LegJoint joint) const noexcept
{
    return joint_ids_[static_cast<std::size_t>(leg)][static_cast<std::size_t>(joint)];
}

const Eigen::MatrixXd& Lite3Dynamics::computeMassMatrix(const Eigen::VectorXd& q)
{
    validateConfigurationSize(q);
    pinocchio::crba(model_, data_, q);
    data_.M.template triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().template triangularView<Eigen::StrictlyLower>();
    return data_.M;
}

const Eigen::VectorXd& Lite3Dynamics::computeNonlinearEffects(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v)
{
    validateConfigurationSize(q);
    validateVelocitySize(v);
    return pinocchio::nonLinearEffects(model_, data_, q, v);
}

const Eigen::VectorXd& Lite3Dynamics::computeGravity(const Eigen::VectorXd& q)
{
    validateConfigurationSize(q);
    return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

void Lite3Dynamics::validateConfigurationSize(const Eigen::VectorXd& q) const
{
    if (q.size() != model_.nq) {
        throw std::invalid_argument("Lite3 q must have size 19");
    }
}

void Lite3Dynamics::validateVelocitySize(const Eigen::VectorXd& v) const
{
    if (v.size() != model_.nv) {
        throw std::invalid_argument("Lite3 v must have size 18");
    }
}
