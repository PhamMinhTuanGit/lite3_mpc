/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"

#include <math.h>
#include <sys/time.h>

#include <fstream>
#include <vector>
/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero()
{
    tauFeedForward = Vec3<T>::Zero();
    forceFeedForward = Vec3<T>::Zero();
    qDes = Vec3<T>::Zero();
    qdDes = Vec3<T>::Zero();
    pDes = Vec3<T>::Zero();
    vDes = Vec3<T>::Zero();
    kpCartesian = Mat3<T>::Zero();
    kdCartesian = Mat3<T>::Zero();
    kpJoint = Mat3<T>::Zero();
    kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero()
{
    q = Vec3<T>::Zero();
    qd = Vec3<T>::Zero();
    p = Vec3<T>::Zero();
    v = Vec3<T>::Zero();
    J = Mat3<T>::Zero();
    tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand()
{
    for (auto &cmd : commands)
    {
        cmd.zero();
    }
    _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain)
{
    zeroCommand();
    if (robot == RobotType::CHEETAH_3)
    {
        for (int leg = 0; leg < 4; leg++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                commands[leg].kdCartesian(axis, axis) = gain;
            }
        }
    }
    else
    { // mini-cheetah
        for (int leg = 0; leg < 4; leg++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                commands[leg].kdJoint(axis, axis) = gain;
            }
        }
    }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(LegData *legData)
{
    for (size_t leg = 0; leg < 4; leg++)
    {
        // q: 关节角
        datas[leg].q(0) = legData->q_abad[leg];
        datas[leg].q(1) = legData->q_hip[leg];
        datas[leg].q(2) = legData->q_knee[leg];

        // qd 关节角速度？
        datas[leg].qd(0) = legData->qd_abad[leg];
        datas[leg].qd(1) = legData->qd_hip[leg];
        datas[leg].qd(2) = legData->qd_knee[leg];

        // J and p 雅可比和足端位置
        computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J), &(datas[leg].p), leg);

        // v 足端速度
        datas[leg].v = datas[leg].J * datas[leg].qd;
    }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegController<T>::updateCommand(LegCommand *legCommand, Vec4<T> &crtlParam)
{
    for (int leg = 0; leg < 4; leg++)
    {
        // tauFF obtains torque from the controller.
        Vec3<T> legTorque = commands[leg].tauFeedForward;

        // forceFF obtains the torque from the controller.
        Vec3<T> footForce = commands[leg].forceFeedForward;
        // Vec3<T> footForce(0, 0, 10);

        // Cartesian PD in rectangular coordinates pd
        footForce += commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
        footForce += commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

        // Torque (conversion of force into torque)
        legTorque += datas[leg].J.transpose() * footForce;

        // Calculate the desired joint angle
        computeLegIK(_quadruped, commands[leg].pDes, &(commands[leg].qDes), leg);
        if (leg == 1 || leg == 3)
        {
            legCommand->tau_abad_ff[leg] =
                1 * crtlParam(2) * (0.0 - datas[leg].q(0)) - 1 * crtlParam(3) * datas[leg].qd(0) + legTorque(0);
            legCommand->tau_hip_ff[leg] =
                1 * crtlParam(2) * (0.0 - datas[leg].q(1)) - 1 * crtlParam(3) * datas[leg].qd(1) + legTorque(1);
            legCommand->tau_knee_ff[leg] =
                1 * crtlParam(2) * (0.0 - datas[leg].q(2)) - 1 * crtlParam(3) * datas[leg].qd(2) + legTorque(2);
        }
        else
        {
            legCommand->tau_abad_ff[leg] =
                crtlParam(2) * (0.0 - datas[leg].q(0)) - crtlParam(3) * datas[leg].qd(0) + legTorque(0);
            legCommand->tau_hip_ff[leg] =
                crtlParam(2) * (0.0 - datas[leg].q(1)) - crtlParam(3) * datas[leg].qd(1) + legTorque(1);
            legCommand->tau_knee_ff[leg] =
                crtlParam(2) * (0.0 - datas[leg].q(2)) - crtlParam(3) * datas[leg].qd(2) + legTorque(2);
        }

        // std::ofstream fp;
        // fp.open("position.txt", std::ofstream::app);
        // if(!fp){
        //   std::ofstream fpout("position.txt");
        //   fpout << commands[0].pDes(0) << "," << commands[0].pDes(1) << "," <<
        //   commands[0].pDes(2) << "," << commands[1].pDes(0) << "," <<
        //   commands[1].pDes(1) << "," << commands[1].pDes(2) << ","; fp.close();
        //   fpout.close();
        // }else{
        //   fp << commands[0].pDes(0) << "," << commands[0].pDes(1) << "," <<
        //   commands[0].pDes(2) << "," << commands[1].pDes(0) << "," <<
        //   commands[1].pDes(1) << "," << commands[1].pDes(2) << std::endl;
        //   fp.close();
        // }

        // legCommand->flags[leg] = _legsEnabled ? 1 : 0;
    }

    flags = flags + 1;

    // std::cout << "mpc force = " << std::endl;
    // for(int leg = 0; leg<4; leg++) {
    //   std::cout << commands[leg].forceFeedForward.transpose() << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "pDes = " << std::endl;
    // for(int leg = 0; leg<4; leg++) {
    //   std::cout << commands[leg].pDes.transpose() << " ";
    // }
    // std::cout << std::endl;
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/**
 * @brief Tính vị trí bàn chân và Jacobian của chân trong hệ tọa độ local leg.
 *
 * Hệ tọa độ local leg (gốc tại abad joint):
 * @code
 *        x (forward)
 *        │
 *        │   abad joint (q1, quay quanh +X)
 *        │      │
 *        │      └── hip joint (q2, quay quanh +Y)
 *        │              │
 *        │              └── l2 (thigh)
 *        │                      │
 *        │                  knee joint (q3, quay quanh +Y)
 *        │                      │
 *        └──────────────────────└── l3 (shank) ──→ foot
 * @endcode
 *
 * Kinematic chain (3-DOF per leg):
 * @code
 *  Frame 0 (abad)  →[Rx(q1)]→  Frame 1 (hip)  →[Ry(q2)]→  Frame 2 (knee)  →[Ry(q3)]→  foot
 * @endcode
 *
 * Vị trí bàn chân trong local leg frame:
 * @code
 *  px =  l3·sin(q2+q3) + l2·sin(q2)
 *  py =  (l1+l4)·sideSign·cos(q1) + l3·sin(q1)·cos(q2+q3) + l2·cos(q2)·sin(q1)
 *  pz =  (l1+l4)·sideSign·sin(q1) - l3·cos(q1)·cos(q2+q3) - l2·cos(q1)·cos(q2)
 * @endcode
 *
 * Jacobian J = ∂p/∂q (3×3, geometric Jacobian):
 * @code
 *       ┌  ∂px/∂q1    ∂px/∂q2           ∂px/∂q3      ┐
 *  J =  │  ∂py/∂q1    ∂py/∂q2           ∂py/∂q3      │
 *       └  ∂pz/∂q1    ∂pz/∂q2           ∂pz/∂q3      ┘
 *
 *     ┌  0            l3·c23 + l2·c2     l3·c23          ┐
 *  =  │  l3·c1·c23 + l2·c1·c2           -l3·s1·s23      │
 *     │  - (l1+l4)·side·s1  -l2·s1·s2  -l3·s1·s23       │
 *     └  l3·s1·c23 + l2·c2·s1           l3·c1·s23       ┘
 *        + (l1+l4)·side·c1   +l2·c1·s2  +l3·c1·s23
 * @endcode
 *
 * Ký hiệu rút gọn:
 * @code
 *   s1=sin(q1), c1=cos(q1)
 *   s2=sin(q2), c2=cos(q2)
 *   s3=sin(q3), c3=cos(q3)
 *   s23=sin(q2+q3)=s2·c3+c2·s3
 *   c23=cos(q2+q3)=c2·c3-s2·s3
 * @endcode
 *
 * @note Nếu truyền nullptr cho J hoặc p, phần tính tương ứng sẽ bị bỏ qua.
 *       Cho phép gọi hàm chỉ để lấy position (không cần Jacobian) hoặc ngược lại.
 *
 * @note sideSign:
 *   - Left legs  (FL=0, HL=2): sideSign = +1 → abad link lệch ra phía +Y
 *   - Right legs (FR=1, HR=3): sideSign = -1 → abad link lệch ra phía -Y
 *
 * @note Với Lite3: _kneeLinkY_offset = 0.0 → l4 = 0, (l1+l4) = l1 = 0.09735 m.
 *
 * @param[in]  quad   Mô hình robot, cung cấp các link length:
 *                    - _abadLinkLength   (l1): khoảng cách abad→hip theo Y  [m]
 *                    - _hipLinkLength    (l2): chiều dài thigh               [m]
 *                    - _kneeLinkLength   (l3): chiều dài shank               [m]
 *                    - _kneeLinkY_offset (l4): offset Y thêm vào abad link  [m]
 * @param[in]  q      Joint angles [q1, q2, q3] (rad):
 *                    - q(0) = q1: abad  (HipX), quay quanh trục +X
 *                    - q(1) = q2: hip   (HipY), quay quanh trục +Y
 *                    - q(2) = q3: knee  (Knee), quay quanh trục +Y
 * @param[out] J      Con trỏ đến Jacobian matrix 3×3.
 *                    Quan hệ: v_foot = J · q̇
 *                    Transpose: τ = Jᵀ · F_foot (force mapping)
 *                    Truyền nullptr để bỏ qua tính Jacobian.
 * @param[out] p      Con trỏ đến vector vị trí bàn chân 3×1 (m),
 *                    biểu diễn trong local leg frame.
 *                    Truyền nullptr để bỏ qua tính position.
 * @param[in]  leg    Chỉ số chân: 0=FL, 1=FR, 2=HL, 3=HR.
 *                    Dùng để lấy sideSign (+1/-1).
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T> &quad,
                                   Vec3<T>      &q,
                                   Mat3<T>      *J,
                                   Vec3<T>      *p,
                                   int           leg)
{
    // ── Link lengths ────────────────────────────────────────────────────
    T l1 = quad._abadLinkLength;    // abad  link: abad joint → hip joint (Y)
    T l2 = quad._hipLinkLength;     // thigh link: hip  joint → knee joint
    T l3 = quad._kneeLinkLength;    // shank link: knee joint → foot
    T l4 = quad._kneeLinkY_offset;  // extra Y offset trên abad link (Lite3=0)

    // sideSign: +1 cho left legs, -1 cho right legs
    T sideSign = quad.getSideSign(leg);

    // ── Trig values ─────────────────────────────────────────────────────
    T s1 = std::sin(q(0));   T c1 = std::cos(q(0));  // abad
    T s2 = std::sin(q(1));   T c2 = std::cos(q(1));  // hip
    T s3 = std::sin(q(2));   T c3 = std::cos(q(2));  // knee

    // Góc tổng hợp knee+hip (q2+q3), dùng angle-addition formula:
    //   cos(q2+q3) = c2*c3 - s2*s3
    //   sin(q2+q3) = s2*c3 + c2*s3
    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    /**
     * @brief Chỉnh sửa FK/Jacobian
     * - Code gốc theo Cheetah 3, có phần ngược theo  hip_X - joint (torso_to_abduct_fr_j) (<axis xyz="-1 0 0"/>)
     *
     */
    // ── Jacobian J = ∂p/∂q ──────────────────────────────────────────────
    if (J)
    {
        // Hàng 0: ∂px/∂qi  (px không phụ thuộc q1 → J(0,0)=0)
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = l3 * c23 + l2 * c2;   // ∂px/∂q2
        J->operator()(0, 2) = l3 * c23;              // ∂px/∂q3

        // Hàng 1: ∂py/∂qi
        J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2
                             - (l1 + l4) * sideSign * s1;  // ∂py/∂q1
        J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2; // ∂py/∂q2
        J->operator()(1, 2) = -l3 * s1 * s23;                 // ∂py/∂q3

        // Hàng 2: ∂pz/∂qi
        J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1
                             + (l1 + l4) * sideSign * c1;  // ∂pz/∂q1
        J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;  // ∂pz/∂q2
        J->operator()(2, 2) = l3 * c1 * s23;                  // ∂pz/∂q3
    }

    // ── Foot position p ─────────────────────────────────────────────────
    // Sau khi apply xoay abad q1 quanh +X (R_X(q1)):
    if (p)
    {
        // px: chỉ phụ thuộc q2, q3 (trục sagittal, không bị ảnh hưởng bởi abad)
        p->operator()(0) = l3 * s23 + l2 * s2;

        // py: abad link chiếu lên Y (×cos q1) + thigh/shank chiếu (×sin q1)
        p->operator()(1) = (l1 + l4) * sideSign * c1
                         + l3 * (s1 * c23)
                         + l2 * c2 * s1;

        // pz: abad link chiếu lên Z (×sin q1) - thigh/shank chiếu (×cos q1)
        p->operator()(2) = (l1 + l4) * sideSign * s1
                         - l3 * (c1 * c23)
                         - l2 * c1 * c2;
    }
}

template void computeLegJacobianAndPosition<double>(Quadruped<double> &quad,
                                                    Vec3<double> &q,
                                                    Mat3<double> *J,
                                                    Vec3<double> *p,
                                                    int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float> &quad,
                                                   Vec3<float> &q,
                                                   Mat3<float> *J,
                                                   Vec3<float> *p,
                                                   int leg);

template <typename T>
void computeLegIK(Quadruped<T> &quad, Vec3<T> &pDes, Vec3<T> *qDes, int leg)
{
    T l1 = quad._abadLinkLength + quad._kneeLinkY_offset;
    T l2 = quad._hipLinkLength;
    T l3 = quad._kneeLinkLength;
    T sideSign = quad.getSideSign(leg);

    T D = (pDes[0] * pDes[0] + pDes[1] * pDes[1] + pDes[2] * pDes[2] - l1 * l1 - l2 * l2 - l3 * l3) / (2 * l2 * l3);

    if (D > 1.00001 || D < -1.00001)
    {
        // printf("_______OUT OF DOMAIN_______!!!\n");
        if (D > 1.00001)
        {
            D = 0.99999;
        }
        if (D < -1.00001)
        {
            D = -0.99999;
        }
    }

    T gamma = atan2(-sqrt(1 - D * D), D);
    T tetta = -atan2(pDes[2], pDes[1]) - atan2(sqrt(pDes[1] * pDes[1] + pDes[2] * pDes[2] - l1 * l1), sideSign * l1);
    T alpha = atan2(-pDes[0], sqrt(pDes[1] * pDes[1] + pDes[2] * pDes[2] - l1 * l1)) -
              atan2(l3 * sin(gamma), l2 + l3 * cos(gamma));

    qDes->operator()(0) = -tetta;
    qDes->operator()(1) = alpha;
    qDes->operator()(2) = gamma;
}

// template <typename T>
// void computeLegIK(Quadruped<T>& quad, Vec3<T>& pDes, Vec3<T>* qDes, int leg)
// {
//   T l1 = quad._abadLinkLength + quad._kneeLinkY_offset;
//   T l2 = quad._hipLinkLength;
//   T l3 = quad._kneeLinkLength;
//   T sideSign = quad.getSideSign(leg);

//   T temp1 = (pDes(0) * pDes(0) + pDes(1) * pDes(1) + pDes(2) * pDes(2) - l1 *
//   l1 - l2 * l2 - l3 * l3) / (2 * l2 * l3); qDes->operator()(2) = acos(temp1);

//   T k1 = temp1;
//   T k2 = sqrt(1 - k1 * k1);
//   T temp2 = (l3 * k1 + l2) * (l3 * k1 + l2) + l3 * l3 * k2 * k2;
//   T temp3 = 2 * pDes(0) * (l3 * k1 + l2);
//   T temp4 = pDes(0) * pDes(0) - l3 * l3 * k2 * k2;
//   T temp5 = (temp3 - sqrt(temp3 * temp3 - 4 * temp2 * temp4)) / (2 * temp2);
//   qDes->operator()(1) = asin(temp5);

//   T temp6 = sideSign * l1 + sqrt(pDes(1) * pDes(1) + pDes(2) * pDes(2) - l1 *
//   l1); T temp7 = sideSign * l1 - sqrt(pDes(1) * pDes(1) + pDes(2) * pDes(2) -
//   l1 * l1); T temp8 = temp6 * temp6 + temp7 * temp7; T temp9 = 2 * temp6 *
//   (pDes(1) + pDes(2)); T temp10 = (pDes(1) + pDes(2)) * (pDes(1) + pDes(2)) -
//   temp7 * temp7; T temp11 = (temp9 + sqrt(temp9 * temp9 - 4 * temp8 *
//   temp10)) / (2 * temp8); qDes->operator()(0) = asin(temp11);
// }

// template <typename T>
// void computeLegIK(Quadruped<T>& quad, Vec3<T> pDes, std::vector<double>
// &qDes, int leg) {
//   T l1 = quad._abadLinkLength + quad._kneeLinkY_offset;
//   T l2 = quad._hipLinkLength;
//   T l3 = quad._kneeLinkLength;
//   T sideSign = quad.getSideSign(leg);

//   T tempL = sqrt(pDes[0] * pDes[0] + pDes[1] * pDes[1] + pDes[2] * pDes[2]);
//   T tempL23 = sqrt(tempL * tempL - l1 * l1);
//   T angle1 = fabs(
//       acos((l1 * l1 + tempL * tempL - tempL23 * tempL23) / (2 * l1 *
//       tempL)));
//   T tempAngle1 = acos(fabs(pDes[1]) / tempL);
//   qDes[0] = sideSign * (angle1 - tempAngle1);
//   T angle2 = fabs(acos((l2 * l2 + tempL * tempL - l3 * l3) / (2 * l2 *
//   tempL))); T tempAngle2 = asin(pDes[0] / tempL); qDes[1] = -(angle2 -
//   tempAngle2); T tempAngle3 =
//       fabs(acos((l2 * l2 + l3 * l3 - tempL * tempL) / (2 * l2 * l3)));
//   qDes[2] = 3.1415926535898 - tempAngle3;
// }
