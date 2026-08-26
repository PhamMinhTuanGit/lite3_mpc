/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Lite3
 *
 * Inertial parameters sourced directly from URDF (SolidWorks export).
 * All URDF <inertia> values are expressed about the CoM frame (<origin>),
 * which is the convention expected by SpatialInertia<T>.
 *
 * Gear ratios, motor KT/R, and rotor inertias are NOT present in the URDF
 * and are carried over from Mini Cheetah as placeholders — verify against
 * DeepRobotics actuator specifications before use.
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  // ── Body-level scalars ─────────────────────────────────────────────────
  // Mass: URDF TORSO
  // Length/Width: 2× abad-joint x/y offset from TORSO (FL_HipX_joint origin)
  cheetah._bodyMass   = 5.6056;
  cheetah._bodyLength = 0.1745 * 2;   // 0.349 m
  cheetah._bodyWidth  = 0.062  * 2;   // 0.124 m
  cheetah._bodyHeight = 0.06   * 2;   // 0.120 m  (from TORSO collision box z)

  // ── Gear ratios (PLACEHOLDER – verify with DeepRobotics specs) ─────────
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio  = 6;
  cheetah._kneeGearRatio = 9.33;

  // ── Link lengths (from URDF joint <origin>) ─────────────────────────────
  //   FL_HipY_joint  origin xyz="0 0.09735 0"   → abad-link length
  //   FL_Knee_joint  origin xyz="0 0 -0.20"     → hip-link length
  //   FL_Ankle       origin xyz="0 0 -0.21012"  → knee-link length
  cheetah._abadLinkLength   = 0.09735;
  cheetah._hipLinkLength    = 0.20;
  cheetah._kneeLinkY_offset = 0.0;
  cheetah._kneeLinkLength   = 0.21012;
  cheetah._maxLegLength     = 0.41012;  // hipLinkLength + kneeLinkLength

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

  // ── Body (TORSO) ─────────────────────────────────────────────────────────
  // URDF TORSO: mass=5.6056, CoM=(0,0,0)
  // ixx=0.02456, iyy=0.05518, izz=0.07016  (about CoM = origin)
  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 24560,     0,     0,
                             0,   55180,     0,
                             0,       0, 70160;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
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

#endif  // PROJECT_MINICHEETAH_H
