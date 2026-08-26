#include <array>
#include <cmath>
#include <iostream>
#include <string>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "RobotModel.hpp"

namespace {

constexpr double kExpectedMass = 11.9376;
constexpr double kMassTol = 1e-10;
constexpr double kFootPosTol = 1e-12;

const std::array<const char*, ::kNumLegs> kFootNames{
    {"FR_FOOT", "FL_FOOT", "HR_FOOT", "HL_FOOT"}};

const std::array<const char*, ::kNumJoints> kJointNames{{
    "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
    "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
    "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
    "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint"}};

const std::array<Eigen::Vector3d, 4> kExpectedFootPositions{{
    Eigen::Vector3d( 0.1745, -0.15935, -0.41012),  // FR
    Eigen::Vector3d( 0.1745,  0.15935, -0.41012),  // FL
    Eigen::Vector3d(-0.1745, -0.15935, -0.41012),  // HR
    Eigen::Vector3d(-0.1745,  0.15935, -0.41012),  // HL
}};

RobotModelConfig makeConfig(const std::string& urdf_path)
{
    RobotModelConfig cfg;
    cfg.urdf_path = urdf_path;
    cfg.base_frame = "TORSO";
    for (int i = 0; i < kNumLegs; ++i) cfg.foot_frames[i] = kFootNames[i];
    for (int i = 0; i < kNumJoints; ++i) cfg.joint_names[i] = kJointNames[i];
    cfg.foot_radius = 0.0;
    cfg.armature = 0.0;
    return cfg;
}

} // unnamed namespace

int main(int argc, char* argv[])
{
    const std::string urdf_path =
        (argc > 1) ? argv[1]
                   : "third_party/deep_robotics_model/Lite3/Lite3_urdf/urdf/Lite3.urdf";

    // ════════════════════════════════════════════════════════════════
    // 1. Build model from URDF
    // ════════════════════════════════════════════════════════════════
    RobotModelConfig cfg = makeConfig(urdf_path);
    RobotModel robot;
    std::string err;
    if (!robot.build(cfg, &err)) {
        std::cerr << "FAIL[1] build: " << err << "\n";
        return 1;
    }
    const auto& model = robot.model();
    if (model.nq != 19 || model.nv != 18) {
        std::cerr << "FAIL[2] nq=" << model.nq << " nv=" << model.nv
                  << " (expect 19/18)\n";
        return 2;
    }
    if (std::abs(robot.mass() - kExpectedMass) > kMassTol) {
        std::cerr << "FAIL[3] mass=" << robot.mass()
                  << " (expect " << kExpectedMass << ")\n";
        return 3;
    }

    // ════════════════════════════════════════════════════════════════
    // 2. Forward kinematics — foot positions in neutral config
    // ════════════════════════════════════════════════════════════════
    Eigen::Matrix<double, 19, 1> q = Eigen::Matrix<double, 19, 1>::Zero();
    q[6] = 1.0;                                    // quaternion w = 1

    Eigen::Matrix<double, 12, 1> qj = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> dqj = Eigen::Matrix<double, 12, 1>::Zero();

    robot.setState(q.head<3>(),
                   Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                   qj, dqj);
    robot.updateKinematics();

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Eigen::Vector3d pos = robot.footPos(leg);
        if ((pos - kExpectedFootPositions[leg]).norm() > kFootPosTol) {
            std::cerr << "FAIL[4] foot" << leg << ": " << pos.transpose()
                      << " (expect " << kExpectedFootPositions[leg].transpose()
                      << ")\n";
            return 4;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // 3. Mass matrix — symmetric and finite
    // ════════════════════════════════════════════════════════════════
    robot.updateDynamics();
    const auto& M = robot.M();
    if (!M.allFinite()) {
        std::cerr << "FAIL[5] mass matrix has non-finite entries\n";
        return 5;
    }
    if ((M - M.transpose()).norm() > 1e-12) {
        std::cerr << "FAIL[6] mass matrix not symmetric, diff="
                  << (M - M.transpose()).norm() << "\n";
        return 6;
    }

    // ════════════════════════════════════════════════════════════════
    // 4. Nonlinear effects at v=0: h() should equal gravity
    // ════════════════════════════════════════════════════════════════
    {
        auto& data = robot.data();
        const Eigen::VectorXd qcur = robot.q();
        const auto& grav = pinocchio::computeGeneralizedGravity(model, data, qcur);
        if ((robot.h() - grav).norm() > 1e-12) {
            std::cerr << "FAIL[7] gravity at v=0 mismatch, diff="
                      << (robot.h() - grav).norm() << "\n";
            return 7;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // 5. Dynamics at non-zero velocity
    // ════════════════════════════════════════════════════════════════
    for (int sample = 0; sample < 8; ++sample) {
        q.setZero();
        q[6] = 1.0;
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

        for (int j = 0; j < kNumJoints; ++j) {
            const int iq = robot.idxQ(j);
            const int iv = robot.idxV(j);
            q[iq]  = -0.9 + 0.13 * sample + 0.04 * j;       // config
            v[iv]  = -0.7 + 0.09 * sample + 0.03 * j;       // velocity
            qj[j]  = q[iq];
            dqj[j] = v[iv];
        }

        const Eigen::Vector3d v_lin(0.2, -0.1, 0.05);
        const Eigen::Vector3d omega(-0.3, 0.15, 0.1);
        v.head<3>() = v_lin;
        v.segment<3>(3) = omega;

        robot.setState(q.head<3>(),
                       Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                       v_lin, omega, qj, dqj);
        robot.update();

        auto& data = robot.data();
        const auto& C = pinocchio::computeCoriolisMatrix(
            model, data, robot.q(), robot.v());
        const auto& gv = pinocchio::computeGeneralizedGravity(
            model, data, robot.q());
        const Eigen::VectorXd expected_h = C * robot.v() + gv;

        if (!robot.h().allFinite() || !C.allFinite() || !gv.allFinite()) {
            std::cerr << "FAIL[8] non-finite at sample " << sample << "\n";
            return 8;
        }
        if ((robot.h() - expected_h).norm() > 1e-9) {
            std::cerr << "FAIL[9] h() mismatch at sample "
                      << sample << ": diff="
                      << (robot.h() - expected_h).norm() << "\n";
            return 9;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // 6. Base inertia — symmetric and positive‑definite
    // ════════════════════════════════════════════════════════════════
    {
        const Eigen::Matrix3d I_body = robot.baseInertia();
        if (!I_body.allFinite()
            || (I_body - I_body.transpose()).norm() > 1e-12) {
            std::cerr << "FAIL[10] base inertia invalid\n";
            return 10;
        }
        if (I_body.determinant() <= 0.0) {
            std::cerr << "FAIL[11] base inertia not positive-definite\n";
            return 11;
        }
    }

    // ════════════════════════════════════════════════════════════════
    // 7. jointTorque sanity — zero qddot & zero fc ≈ 0
    // ════════════════════════════════════════════════════════════════
    {
        q.setZero();  q[6] = 1.0;
        qj.setZero(); dqj.setZero();
        robot.setState(q.head<3>(),
                       Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       qj, dqj);
        robot.update();
        const Eigen::Matrix<double, 12, 1> tau_zero =
            robot.jointTorque(Eigen::VectorXd::Zero(model.nv),
                              Eigen::VectorXd::Zero(12));
        if (!tau_zero.allFinite()) {
            std::cerr << "FAIL[12] jointTorque non-finite\n";
            return 12;
        }
        for (int k = 0; k < kNumJoints; ++k) {
            if (std::abs(tau_zero[k] - robot.h()[robot.idxV(k)]) > 1e-12) {
                std::cerr << "FAIL[13] jointTorque mismatch with h(): k=" << k
                          << " tau=" << tau_zero[k] << " h=" << robot.h()[robot.idxV(k)] << "\n";
                return 13;
            }
        }
    }

    // ════════════════════════════════════════════════════════════════
    // 8. Foot Jacobian — finite‑difference validation
    // ════════════════════════════════════════════════════════════════
    {
        q.setZero();  q[6] = 1.0;
        qj.setZero();
        robot.setState(q.head<3>(),
                       Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       qj, dqj);
        robot.updateKinematics();

        constexpr double eps = 1e-7;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int j = 0; j < 3; ++j) {
                const int j_idx = leg * 3 + j;

                // forward perturbation
                qj[j_idx] = eps;
                robot.setState(q.head<3>(),
                               Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               qj, dqj);
                robot.updateKinematics();
                const Eigen::Vector3d p_plus = robot.footPos(leg);

                // backward perturbation
                qj[j_idx] = -eps;
                robot.setState(q.head<3>(),
                               Eigen::Quaterniond(q[6], q[3], q[4], q[5]),
                               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               qj, dqj);
                robot.updateKinematics();
                const Eigen::Vector3d p_minus = robot.footPos(leg);

                qj[j_idx] = 0.0;  // restore

                const Eigen::Vector3d numeric =
                    (p_plus - p_minus) / (2.0 * eps);

                const int iv = robot.idxV(j_idx);
                const Eigen::Vector3d analytic =
                    robot.Jc().block<3, 1>(3 * leg, iv);

                if ((numeric - analytic).norm() > 3e-6) {
                    std::cerr << "FAIL[14] J foot leg=" << leg
                              << " joint=" << j
                              << " num=" << numeric.transpose()
                              << " an=" << analytic.transpose()
                              << " d=" << (numeric - analytic).norm() << "\n";
                    return 14;
                }
            }
        }
    }

    std::cout << "RobotModel smoke OK: nq=" << model.nq
              << " nv=" << model.nv
              << " mass=" << robot.mass() << "\n";
    return 0;
}