#pragma once
#include <array>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>



using Vec3  = Eigen::Vector3d;
using Vec6  = Eigen::Matrix<double, 6, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using VecX  = Eigen::VectorXd;
using MatX  = Eigen::MatrixXd;

inline constexpr int kNumLegs = 4;
inline constexpr int kNumJoints = 12;
inline constexpr int kNv = 18;   // 6 (freeflyer) + 12
inline constexpr int kNq = 19;   // 7 (pos+quat) + 12

struct RobotModelConfig {
  std::string urdf_path = "";
  std::string base_frame = "TORSO";
  // Thứ tự chân: FR, FL, HR, HL
  std::array<std::string, 4> foot_frames = {
      "FR_FOOT", "FL_FOOT", "HR_FOOT", "HL_FOOT"};
  // Thứ tự khớp: FR (HipX, HipY, Knee), FL (...), HR (...), HL (...)
  std::array<std::string, 12> joint_names = {
      "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
      "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
      "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
      "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint"};
  double foot_radius = 0.0;   // >0 -> tự sinh frame tiếp xúc lệch (0,0,-r)
  double armature    = 0.0;   // I_rotor*n^2, cộng vào chéo M -> QP ổn định số
};

class RobotModel {
 public:
  // ───── 1. XÂY DỰNG ─────────────────────────────────────────────────────
  /// Xây dựng Pinocchio model trực tiếp trong C++ (không phụ thuộc URDF runtime)
  static pinocchio::Model buildLite3Model();

  bool build(const RobotModelConfig& cfg = {}, std::string* err = nullptr);

  // ───── 3. UPDATE MỖI CHU KỲ ────────────────────────────────────────────
  /// Nạp state. Nhận vận tốc base ở WORLD frame, tự xoay sang BODY.
  void setState(const Vec3& p_world, const Eigen::Quaterniond& q_wb,
                const Vec3& v_lin_world, const Vec3& omega_world,
                const Eigen::Ref<const Vec12>& q_joint_hw,
                const Eigen::Ref<const Vec12>& dq_joint_hw);

  void updateKinematics();   // FK + frame placement + Jacobian + J̇·v   (~19 µs)
  void updateDynamics();     // M (mirror + armature) + h                (~20 µs)
  void update() { updateKinematics(); updateDynamics(); }

  // ───── 2. MA TRẬN ĐỘNG LỰC HỌC ─────────────────────────────────────────
  const MatX& M() const { return M_; }      // 18×18, đối xứng đầy đủ
  const VecX& h() const { return h_; }      // 18, = C·v + g

  const MatX& Jc()  const { return Jc_; }   // 12×18, 4 chân xếp chồng (3 hàng/chân)
  const VecX& dJv() const { return dJv_; }  // 12,    J̇·v tương ứng
  const MatX& Jb()  const { return Jb_; }   // 6×18,  base, LOCAL_WORLD_ALIGNED
  const Vec6& dJvb() const { return dJvb_; }

  auto Mu() const { return M_.topRows<6>();    }   // 6 hàng KHÔNG actuated
  auto Ma() const { return M_.bottomRows<12>(); }  // 12 hàng actuated
  auto hu() const { return h_.head<6>();  }
  auto ha() const { return h_.tail<12>(); }

  /// τ = Mₐ·q̈ + hₐ − Jc,ₐᵀ·F   (fc: 12×1, chân swing để 0). Trả về thứ tự HARDWARE.
  Vec12 jointTorque(const Eigen::Ref<const VecX>& qddot,
                    const Eigen::Ref<const VecX>& fc) const;

  // ───── Truy vấn động học ───────────────────────────────────────────────
  Vec3 footPos(int leg) const;      // world
  Vec3 footVel(int leg) const;      // world
  Vec3 basePos() const { return q_.head<3>(); }
  Eigen::Matrix3d baseRot() const { return R_wb_; }

  // ───── Hằng số cho SRBD MPC ────────────────────────────────────────────
  double mass() const { return mass_; }
  const Eigen::Matrix3d& baseInertia() const { return I_body_; }

  const pinocchio::Model& model() const { return model_; }
  pinocchio::Data&        data()        { return data_; }
  const VecX& q() const { return q_; }
  const VecX& v() const { return v_; }
  int idxV(int hw) const { return idx_v_[hw]; }   // hardware -> pinocchio
  int idxQ(int hw) const { return idx_q_[hw]; }
  std::string dump() const;

 private:
  pinocchio::Model model_;
  pinocchio::Data  data_;

  std::array<int, 4>  foot_fid_{};
  std::array<int, 12> idx_q_{}, idx_v_{};
  int base_fid_ = -1;

  VecX q_, v_, a0_, armature_;
  Eigen::Matrix3d R_wb_ = Eigen::Matrix3d::Identity();

  MatX M_, Jc_, Jb_, Jtmp_;
  VecX h_, dJv_;
  Vec6 dJvb_ = Vec6::Zero();

  double mass_ = 0.0;
  Eigen::Matrix3d I_body_ = Eigen::Matrix3d::Identity();
};

