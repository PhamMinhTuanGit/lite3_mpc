/*! @file MiniCheetahWithPayload.h
 *  @brief Utility function to build a Lite3 Quadruped object with payload
 *
 *  This file is a variant of MiniCheetah.h where the body (TORSO) inertial
 *  parameters have been updated to account for a 2 kg payload mounted 10 cm
 *  forward of the TORSO geometric center (along +x in the body frame).
 *
 *  The combined mass, CoM, and inertia are computed from Lite3_payload.xml:
 *    TORSO  : mass=5.6056, CoM=(0,0,0), diaginertia=(0.02456,0.05518,0.07016)
 *    Payload: mass=2.0,    pos=(0.10,0,0), diaginertia=(0.001,0.001,0.001)
 *    Combined: mass=7.6056, CoM=(0.026296,0,0)
 *              Ixx=0.025560, Iyy=0.070921, Izz=0.085901  (about combined CoM)
 *
 *  Leg link inertias are unchanged from MiniCheetah.h / the URDF.
 *
 *  Derived from: MiniCheetah.h
 *  Keep in sync when updating the original.
 */

#ifndef PROJECT_MINICHEETAH_WITH_PAYLOAD_H
#define PROJECT_MINICHEETAH_WITH_PAYLOAD_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Lite3 WITH a 2 kg payload mounted 10 cm
 * forward of TORSO center.
 *
 * Inertial parameters sourced directly from URDF (SolidWorks export) and
 * combined with the payload using the parallel-axis theorem.
 */
template <typename T>
Quadruped<T> buildMiniCheetahWithPayload() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  // ── Body-level scalars ─────────────────────────────────────────────────
  // Combined mass: TORSO (5.6056) + Payload (2.0)
  cheetah._bodyMass   = 7.6056;
  cheetah._bodyLength = 0.1745 * 2;   // 0.349 m
  cheetah._bodyWidth  = 0.062  * 2;   // 0.124 m
  cheetah._bodyHeight = 0.06   * 2;   // 0.120 m

  // ── Gear ratios (PLACEHOLDER – verify with DeepRobotics specs) ─────────
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio  = 6;
  cheetah._kneeGearRatio = 9.33;

  // ── Link lengths (from URDF joint <origin>) ─────────────────────────────
  cheetah._abadLinkLength   = 0.09735;
  cheetah._hipLinkLength    = 0.20;
  cheetah._kneeLinkY_offset = 0.0;
  cheetah._kneeLinkLength   = 0.21012;
  cheetah._maxLegLength     = 0.41012;

  // ── Motor parameters (PLACEHOLDER – verify with DeepRobotics specs) ────
  cheetah._motorTauMax      = 3.f;
  cheetah._batteryV         = 24;
  cheetah._motorKT          = 0.05;
  cheetah._motorR           = 0.173;
  cheetah._jointDamping     = 0.01;
  cheetah._jointDryFriction = 0.2;

  // ── Rotor inertia (PLACEHOLDER – same as Mini Cheetah) ─────────────────
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0,
                               0, 33, 0,
                               0,  0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

  // ── Abad (HIP link) ─────────────────────────────────────────────────────
  // URDF FL_HIP: mass=0.550, CoM=(-0.00601, -0.0066532, 0.00034295) [LEFT]
  // ixx=0.0003949, iyy=0.0004028, izz=0.0004472  (about CoM, all off-diag = 0)
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 394.9,   0.0,   0.0,
                             0.0, 402.8,   0.0,
                             0.0,   0.0, 447.2;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.00601, -0.0066532, 0.00034295);  // LEFT
  SpatialInertia<T> abadInertia(0.550, abadCOM, abadRotationalInertia);

  // ── Hip (THIGH link) ────────────────────────────────────────────────────
  // URDF FL_THIGH: mass=0.86, CoM=(-0.0039245, -0.014632, -0.025146)
  // ixx=0.005736, iyy=0.004960, izz=0.001436  (about CoM)
  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5736,    0,    0,
                            0,  4960,    0,
                            0,     0, 1436;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.0039245, -0.014632, -0.025146);
  SpatialInertia<T> hipInertia(0.86, hipCOM, hipRotationalInertia);

  // ── Knee (SHANK link) ───────────────────────────────────────────────────
  // URDF FL_SHANK: mass=0.153, CoM=(0.0064794, ~0, -0.12157)
  // ixx=0.00089039, iyy=0.00090672, izz=3.1266e-5  (about CoM)
  Mat3<T> kneeRotationalInertia;
  kneeRotationalInertia << 890.39,   0.0,    0.0,
                             0.0,  906.72,   0.0,
                             0.0,    0.0,  31.266;
  kneeRotationalInertia = kneeRotationalInertia * 1e-6;
  Vec3<T> kneeCOM(0.0064794, -1.4535e-6, -0.12157);
  SpatialInertia<T> kneeInertia(0.153, kneeCOM, kneeRotationalInertia);

  // ── Rotor spatial inertias (PLACEHOLDER) ────────────────────────────────
  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  // ── Body (TORSO + Payload) ──────────────────────────────────────────────
  // Combined inertia (about combined CoM, parallel-axis theorem):
  //   TORSO   : mass=5.6056, CoM=(0,0,0),        diaginertia=(0.02456,0.05518,0.07016)
  //   Payload : mass=2.0,    CoM=(0.10,0,0)rel,   diaginertia=(0.001,0.001,0.001)
  //   Combined: mass=7.6056, CoM=(0.026296,0,0)
  //   Ixx=0.025560, Iyy=0.070921, Izz=0.085901
  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 25560,     0,     0,
                              0,   70921,     0,
                              0,       0, 85901;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0.026296, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM, bodyRotationalInertia);

  // ── Assign inertias ──────────────────────────────────────────────────────
  cheetah._abadInertia      = abadInertia;
  cheetah._hipInertia       = hipInertia;
  cheetah._kneeInertia      = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia  = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia      = bodyInertia;

  // ── Joint / link locations ───────────────────────────────────────────────
  cheetah._abadRotorLocation = Vec3<T>(0.1745, 0.049, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation       = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation  = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation      = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_WITH_PAYLOAD_H