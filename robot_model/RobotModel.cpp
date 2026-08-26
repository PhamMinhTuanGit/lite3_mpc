#include "RobotModel.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute-unaligned.hpp>

namespace {

using pinocchio::Frame;
using pinocchio::FrameIndex;
using pinocchio::Inertia;
using pinocchio::JointIndex;
using pinocchio::JointModelFreeFlyer;
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

// ═══════════════════════════════════════════════════════════════════════════
//  1. XÂY DỰNG MODEL LITE3 (Programmatic / Header-Only Pinocchio)
// ═══════════════════════════════════════════════════════════════════════════
pinocchio::Model RobotModel::buildLite3Model() {
    Model model;
    model.name = "Lite3";
    model.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);
    model.gravity.angular().setZero();

    const JointIndex torso_id = model.addJoint(
        0,
        JointModelFreeFlyer(),
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

    // Construction order matches canonical Lite3 URDF/MJCF [FL, FR, HL, HR].
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

        // Point-mass foot folded into knee joint
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

bool RobotModel::build(const RobotModelConfig& cfg, std::string* err) {
  auto fail = [&](const std::string& m) { if (err) *err = m; return false; };

  try {
    model_ = buildLite3Model();
  } catch (const std::exception& e) {
    return fail(std::string("buildLite3Model failed: ") + e.what());
  }

  if (model_.nq != kNq || model_.nv != kNv)
    return fail("nq/nv = " + std::to_string(model_.nq) + "/" +
                std::to_string(model_.nv) + ", can 19/18");

  // --- Frame điểm tiếp xúc: gốc link foot ở TÂM quả cầu, tiếp xúc thấp hơn r ---
  for (int i = 0; i < kNumLegs; ++i) {
    const std::string& src = cfg.foot_frames[i];
    if (!model_.existFrame(src)) return fail("thieu frame '" + src + "'");
    const FrameIndex sid = model_.getFrameId(src);

    if (cfg.foot_radius > 0.0) {
      const Frame& sf = model_.frames[sid];
      SE3 off = SE3::Identity();
      off.translation() << 0.0, 0.0, -cfg.foot_radius;
      model_.addFrame(Frame(src + "_contact", sf.parentJoint, sid,
                            sf.placement * off, pinocchio::OP_FRAME));
      foot_fid_[i] = static_cast<int>(model_.getFrameId(src + "_contact"));
    } else {
      foot_fid_[i] = static_cast<int>(sid);
    }
  }

  // Support "TORSO" or "base"
  std::string base_frame = cfg.base_frame;
  if (!model_.existFrame(base_frame)) {
    if (model_.existFrame("TORSO")) {
      base_frame = "TORSO";
    } else {
      return fail("thieu '" + cfg.base_frame + "'");
    }
  }
  base_fid_ = static_cast<int>(model_.getFrameId(base_frame));

  // --- Ánh xạ thứ tự khớp: Pinocchio đánh số theo cây động học của NÓ ---
  for (int k = 0; k < kNumJoints; ++k) {
    const std::string& jn = cfg.joint_names[k];
    if (!model_.existJointName(jn)) return fail("thieu khop '" + jn + "'");
    const JointIndex jid = model_.getJointId(jn);
    if (model_.joints[jid].nq() != 1) return fail("'" + jn + "' khong phai 1-DoF");
    idx_q_[k] = model_.joints[jid].idx_q();
    idx_v_[k] = model_.joints[jid].idx_v();
  }

  data_ = pinocchio::Data(model_);

  // --- Armature: giữ riêng, cộng vào chéo M sau crba ---
  armature_ = VecX::Zero(kNv);
  for (int k = 0; k < kNumJoints; ++k) armature_[idx_v_[k]] = cfg.armature;

  mass_   = totalMass(model_);
  I_body_ = model_.inertias[1].inertia().matrix();   // joint 1 = freeflyer = base

  // --- Pre-allocate toàn bộ, hot path không cấp phát ---
  q_ = VecX::Zero(kNq);  q_[6] = 1.0;          // quat xyzw -> w = 1
  v_ = VecX::Zero(kNv);  a0_ = VecX::Zero(kNv);
  M_.setZero(kNv, kNv);  h_.setZero(kNv);
  Jc_.setZero(3 * kNumLegs, kNv);  dJv_.setZero(3 * kNumLegs);
  Jb_.setZero(6, kNv);   Jtmp_.setZero(6, kNv);
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  3. UPDATE MỖI CHU KỲ
// ═══════════════════════════════════════════════════════════════════════════
void RobotModel::setState(const Vec3& p_world, const Eigen::Quaterniond& q_wb,
                          const Vec3& v_lin_world, const Vec3& omega_world,
                          const Eigen::Ref<const Vec12>& qj,
                          const Eigen::Ref<const Vec12>& dqj) {
  const Eigen::Quaterniond qn = q_wb.normalized();
  R_wb_ = qn.toRotationMatrix();

  q_.head<3>()     = p_world;
  q_.segment<4>(3) << qn.x(), qn.y(), qn.z(), qn.w();   // ★ xyzw, KHÔNG phải wxyz

  // ★ v của JointModelFreeFlyer nằm ở BODY frame. Sai chỗ này -> QP ra nghiệm
  //   đẹp, robot rung rồi ngã, không exception nào.
  v_.head<3>()     = R_wb_.transpose() * v_lin_world;
  v_.segment<3>(3) = R_wb_.transpose() * omega_world;

  for (int k = 0; k < kNumJoints; ++k) {
    q_[idx_q_[k]] = qj[k];
    v_[idx_v_[k]] = dqj[k];
  }
}

void RobotModel::updateKinematics() {
  // a = 0  ->  classical acceleration của frame CHÍNH LÀ J̇·v
  // (đừng dùng spatial acceleration: thiếu số hạng tích chéo)
  pinocchio::forwardKinematics(model_, data_, q_, v_, a0_);
  pinocchio::computeJointJacobians(model_, data_);      // dùng oMi đã có, khỏi FK lần 2
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < kNumLegs; ++i) {
    Jtmp_.setZero();                          // ★ getFrameJacobian CỘNG DỒN
    pinocchio::getFrameJacobian(model_, data_, foot_fid_[i], pinocchio::LOCAL_WORLD_ALIGNED, Jtmp_);
    Jc_.middleRows<3>(3 * i) = Jtmp_.topRows<3>();          // point contact
    dJv_.segment<3>(3 * i) =
        pinocchio::getFrameClassicalAcceleration(model_, data_, foot_fid_[i],
                                                 pinocchio::LOCAL_WORLD_ALIGNED).linear();
  }

  Jb_.setZero();
  pinocchio::getFrameJacobian(model_, data_, base_fid_, pinocchio::LOCAL_WORLD_ALIGNED, Jb_);
  const auto ab = pinocchio::getFrameClassicalAcceleration(model_, data_, base_fid_,
                                                           pinocchio::LOCAL_WORLD_ALIGNED);
  dJvb_.head<3>() = ab.linear();
  dJvb_.tail<3>() = ab.angular();
}

void RobotModel::updateDynamics() {
  pinocchio::crba(model_, data_, q_);
  // ★ crba chỉ điền tam giác TRÊN -> phải mirror, nếu không EoM sai âm thầm
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  M_ = data_.M;
  M_.diagonal() += armature_;

  pinocchio::nonLinearEffects(model_, data_, q_, v_);   // data_.nle = C·v + g
  h_ = data_.nle;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Tiện ích
// ═══════════════════════════════════════════════════════════════════════════
Vec12 RobotModel::jointTorque(const Eigen::Ref<const VecX>& qddot,
                              const Eigen::Ref<const VecX>& fc) const {
  const VecX tau_pin = M_.bottomRows<12>() * qddot + h_.tail<12>()
                     - Jc_.rightCols<12>().transpose() * fc;
  Vec12 tau_hw;
  for (int k = 0; k < kNumJoints; ++k) tau_hw[k] = tau_pin[idx_v_[k] - 6];
  return tau_hw;
}

Vec3 RobotModel::footPos(int leg) const {
  return data_.oMf[foot_fid_[leg]].translation();
}
Vec3 RobotModel::footVel(int leg) const {
  return pinocchio::getFrameVelocity(model_, const_cast<pinocchio::Data&>(data_), foot_fid_[leg],
                                     pinocchio::LOCAL_WORLD_ALIGNED).linear();
}

std::string RobotModel::dump() const {
  std::ostringstream os;
  os << model_.name << "  nq=" << model_.nq << " nv=" << model_.nv
     << "  mass=" << mass_ << " kg\n";
  for (int k = 0; k < kNumJoints; ++k)
    os << "  hw" << k << " -> q" << idx_q_[k] << " v" << idx_v_[k] << "\n";
  for (int i = 0; i < kNumLegs; ++i)
    os << "  foot" << i << " = " << model_.frames[foot_fid_[i]].name << "\n";
  return os.str();
}

