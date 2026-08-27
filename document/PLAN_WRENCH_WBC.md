# PLAN — Centroidal Wrench Controller + WBIC

## Goal

Xây một controller mới cho **standing** để kiểm chứng giả thuyết:

> Standing pitch bias hiện tại chủ yếu đến từ model mismatch, đặc biệt là payload 2 kg, sai total mass, sai CoM và việc Convex MPC dùng TORSO origin thay vì whole-body CoM.

Controller mới phải bypass Convex MPC khi standing nhưng giữ nguyên pipeline walking/trot hiện tại.

Target chính:

```text
payload standing:
pitch mean hiện tại ≈ 0.0439 rad

target:
|pitch mean| < 0.01 rad

stretch target:
|pitch mean| < 0.005 rad
```

---

# 0. Current verified state

Repo:

```text
/home/tuanpm/Lite3_rl_deploy_cmpc/Lite3_rl_deploy
```

Standing official hiện tại đã xác minh:

```text
standing_mode && full_stance
    => lock_base_acceleration = true
    => delta_qddot_u = 0
```

Orientation task trong KinWBC hoạt động đúng:

```text
pitch_error mean            ≈ -0.043868 rad
orientation_error_y mean    ≈ -0.043868 rad

kp_body_ori                 = 100

x_ddot_ori_y mean           ≈ -4.3881 rad/s²

x_ddot_ori_y / error_y      ≈ 100
```

QP/WBIC:

```text
WBIC status                 = 100% OK
fallback                    = 0
contact residual max        ≈ 7.34e-13
EOM residual max            ≈ 4.72e-12
torque margin min           ≈ 14.5 Nm
```

Do đó không tiếp tục debug:

```text
RotationErrorSO3
orientation reference
task frame
KinWBC projection
WBIC feasibility
torque saturation
```

trừ khi có bằng chứng mới.

---

# 1. Root-cause hypothesis

MuJoCo đang chạy:

```text
Lite3_payload.xml
```

Payload:

```text
mass     = 2.0 kg
position = (+0.10, 0, 0) m in TORSO frame
```

Total simulator mass:

```text
13.9376 kg
```

Controller nominal:

```text
11.9376 kg
```

Payload tạo khoảng:

```text
gravity pitch moment ≈ 1.962 Nm
```

và dịch whole-body CoM về phía trước khoảng:

```text
ΔCoM_x ≈ +0.01596 m
```

Convex MPC hiện tại sử dụng:

```cpp
r_i = pFoot_i - seResult.position;
```

trong đó:

```text
seResult.position = TORSO/base origin
```

không phải actual CoM.

Giả thuyết chính:

```text
wrong mass
    +
wrong CoM
    +
wrong wrench reference
    +
unmodeled payload gravity moment
            ↓
standing pitch equilibrium bias
```

---

# 2. New architecture

## Standing

```text
State Estimator
      │
      ▼
Whole-body / payload-aware model
      │
      ├── total mass
      └── CoM
      │
      ▼
Centroidal Wrench Controller
      │
      ├── F_des
      └── M_des
      │
      ▼
GRF Distribution QP
      │
      └── f1*, f2*, f3*, f4*
      │
      ▼
Existing WBIC
      │
      ▼
Hybrid joint command
```

## Walking / trot / swing

Keep unchanged:

```text
Convex MPC
    ↓
WBIC
    ↓
command
```

Routing:

```cpp
if (standing_mode && full_stance) {
    use_centroidal_wrench_controller();
} else {
    use_existing_convex_mpc();
}
```

---

# 3. Phase 1 — Inspect existing interfaces

Before implementation:

1. Locate exact `Fr_des` flow from Convex MPC to WBIC.
2. Identify force convention:
   - force on robot
   - or force on ground.
3. Identify all coordinate frames used for:
   - foot position,
   - body position,
   - CoM,
   - GRF.
4. Locate existing:
   - SO(3) orientation error utility,
   - QP solver,
   - Pinocchio model/data access.
5. Identify standing desired:
   - position,
   - orientation,
   - velocity.
6. Verify existing leg ordering.

Do not modify behavior in this phase.

---

# 4. Phase 2 — Implement payload-aware centroidal model data

Create a clean source for:

```text
mass_used
com_world
```

Preferred:

```text
actual whole-body Pinocchio CoM
```

If Pinocchio nominal model does not contain payload, use experimental payload augmentation:

\[
p_{CoM,total}
=
\frac{
m_r p_{CoM,r}
+
m_p p_p
}{
m_r+m_p
}
\]

Payload position must be transformed:

```text
TORSO frame
    ↓ R_world_body + translation
world frame
```

Required modes:

```text
nominal
payload-aware
```

Avoid scattering hard-coded payload constants through multiple files.

Centralize configuration.

---

# 5. Phase 3 — Implement CentroidalWrenchController

Suggested files:

```text
wbic/CentroidalWrenchController.hpp
wbic/CentroidalWrenchController.cpp
```

Suggested input:

```cpp
struct CentroidalWrenchInput {
    double mass;

    Eigen::Vector3d com_world;
    Eigen::Vector3d com_des_world;

    Eigen::Vector3d com_vel_world;
    Eigen::Vector3d com_vel_des_world;

    Eigen::Matrix3d R_world_body;
    Eigen::Matrix3d R_des_world_body;

    Eigen::Vector3d omega_world;
    Eigen::Vector3d omega_des_world;

    std::array<Eigen::Vector3d, 4> foot_pos_world;
    std::array<bool, 4> contact;
};
```

Suggested output:

```cpp
struct CentroidalWrenchOutput {
    Eigen::Vector3d force_des_world;
    Eigen::Vector3d moment_des_world;

    std::array<Eigen::Vector3d, 4> grf_world;

    bool valid;
};
```

Naming may be adapted to repo conventions.

---

# 6. Phase 4 — Translational wrench

Implement:

\[
F_{des}
=
m
\left[
a_{des}
+
K_p(p_{des}-p)
+
K_d(v_{des}-v)
-
g
\right]
\]

Verify gravity convention first.

Expected stationary result for:

```text
g_world = [0, 0, -9.81]
```

is approximately:

\[
F_{des,z}=mg.
\]

Do not guess gravity sign.

---

# 7. Phase 5 — Rotational wrench

Reuse existing validated orientation error convention from KinWBC.

Do not introduce a new SO(3) convention unnecessarily.

Compute:

\[
\alpha_{des}
=
K_{p,R}e_R
+
K_{d,R}(\omega_{des}-\omega)
\]

Then either:

\[
M_{des}
=
I_{world}\alpha_{des}
\]

if reliable centroidal inertia is available,

or use a direct PD moment controller:

\[
M_{des}
=
K_{p,M}e_R
+
K_{d,M}e_\omega.
\]

Prefer the simplest implementation that is physically clear and easy to validate.

Do not tune aggressively in this task.

---

# 8. Phase 6 — GRF distribution QP

Decision variable:

\[
f=
[f_1^T,f_2^T,f_3^T,f_4^T]^T
\in\mathbb{R}^{12}
\]

Critical moment arm:

\[
r_i
=
p_{foot,i}
-
p_{CoM}.
\]

Never use:

```text
foot - TORSO
```

for this controller.

Build centroidal wrench:

\[
F=
\sum_i f_i
\]

\[
M=
\sum_i r_i\times f_i.
\]

Solve:

\[
\min_f
w_F
\left\|
\sum f_i-F_{des}
\right\|^2
+
w_M
\left\|
\sum r_i\times f_i-M_{des}
\right\|^2
+
w_{reg}\|f\|^2.
\]

Constraints for stance leg:

\[
f_z\ge0
\]

\[
|f_x|\le\mu f_z
\]

\[
|f_y|\le\mu f_z.
\]

Optionally:

\[
f_{z,min}\le f_z\le f_{z,max}.
\]

For swing leg:

\[
f_i=0.
\]

Reuse the existing QP infrastructure if possible.

Do not add unnecessary external dependencies.

---

# 9. Phase 7 — Unit tests

Add tests before full MuJoCo validation.

## Test 1 — Symmetric standing

Input:

```text
CoM centered
orientation = identity
F_des = [0, 0, mg]
M_des = 0
4 contacts
```

Expected:

```text
ΣFx ≈ 0
ΣFy ≈ 0
ΣFz ≈ mg
Σmoment ≈ 0
```

For symmetric geometry, Fz distribution should also be near symmetric.

---

## Test 2 — Pitch moment

Input:

```text
Mdes_y != 0
```

Verify:

```text
front/rear Fz redistribution
```

has the correct sign.

This test must catch pitch sign errors.

---

## Test 3 — CoM offset

Shift:

```text
CoM_x > 0
```

with:

```text
M_des = 0.
```

Expected:

GRF distribution changes so that:

\[
\sum_i
(p_i-p_{CoM})\times f_i
\approx0.
\]

This is one of the most important tests.

---

## Test 4 — Swing force

For:

```cpp
contact[i] = false;
```

verify:

```text
f_i == 0
```

within tolerance.

---

# 10. Phase 8 — Standing routing

At the controller integration layer:

```cpp
const bool use_new_controller =
    standing_mode && full_stance;
```

When true:

```text
Centroidal Wrench Controller
    ↓
GRF QP
    ↓
WBIC desired force
```

When false:

```text
existing Convex MPC Fr_des
    ↓
existing WBIC
```

Walking/trot behavior must remain unchanged.

---

# 11. Phase 9 — Preserve current WBIC

Do not change:

```cpp
lock_base_acceleration =
    input.standing_mode && full_stance;
```

Do not change:

```text
delta_qddot_u lock
orientation gains
task hierarchy
contact constraints
torque limits
gait timing
MPC Q
```

unless required solely for compilation/interface wiring.

---

# 12. Phase 10 — Logging

Extend CSV with:

```text
controller_mode

mass_used

com_x
com_y
com_z

payload_mass_used
payload_com_offset_x

Fdes_x
Fdes_y
Fdes_z

Mdes_x
Mdes_y
Mdes_z

My_des
My_grf

GRF_QP_status
GRF_QP_cost

GRF_QP_solve_time_us
```

Define:

\[
M_{y,GRF}
=
\left[
\sum_i
(p_{foot,i}-p_{CoM})\times f_i
\right]_y.
\]

Keep all existing WBIC diagnostics.

---

# 13. Phase 11 — Regression build

Run:

```bash
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
git diff --check
```

Required:

```text
existing tests pass
new tests pass
no regression
```

---

# 14. Phase 12 — MuJoCo experiment matrix

Run every case for approximately 30 s.

Compute metrics only for:

```text
t >= 5 s
```

## Case A

```text
MuJoCo       : nominal
controller   : nominal
```

Purpose:

baseline without model mismatch.

---

## Case B

```text
MuJoCo       : payload ON
controller   : nominal
```

Purpose:

reproduce current mismatch.

Expected pitch bias order:

```text
~0.044 rad
```

---

## Case C

```text
MuJoCo       : payload ON
controller   : payload-aware
controller   : Centroidal Wrench + WBIC
```

This is the main experiment.

Target:

```text
|pitch mean| < 0.01 rad
```

Stretch:

```text
|pitch mean| < 0.005 rad
```

---

# 15. Optional ablation

If time permits, isolate model effects.

## C1 — mass only

```text
mass corrected
CoM nominal
```

## C2 — mass + CoM

```text
mass corrected
CoM corrected
```

## C3 — mass + CoM + inertia

```text
all corrected
```

Compare reduction in pitch bias.

Expected hypothesis:

```text
CoM correction:
largest steady-state pitch improvement

mass:
large vertical force improvement

inertia:
mainly transient improvement
```

Do not assume this result; verify experimentally.

---

# 16. Required metrics

For every MuJoCo run report:

```text
pitch mean
pitch RMS
pitch peak-to-peak

roll mean
roll RMS

omega_y RMS

orientation_error_y mean
x_ddot_ori_y mean
x_ddot_ori_y RMS

Fz front mean/std
Fz rear mean/std
Fz total mean/std

My_des mean/std
My_grf mean/std

torque margin min

contact residual max
EOM residual max

GRF QP solve time mean/max

QP failures
WBIC failures
fallback count
```

---

# 17. Failure investigation order

If Case C still has significant pitch bias, inspect in this exact order:

1. Force convention.
2. Gravity sign.
3. CoM frame.
4. Payload transform.
5. `foot_world - com_world`.
6. Wrench reference point.
7. GRF pitch sign.
8. WBIC expected force convention.
9. Joint/leg ordering.
10. Pinocchio vs MuJoCo frame alignment.

Do not immediately tune gains.

---

# 18. Acceptance criteria

Implementation is accepted when:

- [ ] New standing controller exists as an isolated module.
- [ ] Convex MPC is preserved.
- [ ] Walking/trot path remains unchanged.
- [ ] Moment arms use actual CoM.
- [ ] Nominal/payload-aware modes exist.
- [ ] GRF QP symmetric-standing test passes.
- [ ] Pitch-moment sign test passes.
- [ ] CoM-offset test passes.
- [ ] Swing-contact test passes.
- [ ] Full build passes.
- [ ] Regression tests pass.
- [ ] WBIC remains feasible.
- [ ] No torque saturation.
- [ ] No fallback.
- [ ] Case A/B/C experiment completed.
- [ ] Payload-aware case significantly reduces pitch bias.
- [ ] Main target `|pitch mean| < 0.01 rad` is evaluated.
- [ ] Root cause conclusion is supported by quantitative data.

---

# 19. Worktree safety

Current worktree contains modifications from previous tasks.

Do not run destructive commands such as:

```bash
git reset --hard
git clean -fd
git checkout .
git restore .
```

unless explicitly authorized.

Preserve all existing changes.

Do not commit.

Do not push.

---

# 20. Final report

At completion report:

## Architecture

Describe the actual implemented flow.

## Files changed

List each changed/new file and purpose.

## Mathematical formulation

Document:

```text
F_des
M_des
GRF QP
moment arms
friction constraints
```

## Model handling

State precisely:

```text
mass source
CoM source
payload treatment
coordinate frames
```

## Tests

Report all build/unit/regression results.

## Experiment results

Produce A/B/C comparison table.

## Root-cause conclusion

Answer clearly:

> Does payload / mass / CoM mismatch explain the standing pitch bias?

## Remaining issues

Use logged evidence instead of speculation.

## Git status

List modified and untracked files.

Do not commit or push.

---

# Final objective

The purpose of this plan is not yet to create the final locomotion controller.

It is an experimental bridge from:

```text
Convex MPC + WBIC
```

toward:

```text
Centroidal NMPC + WBIC
```

by first proving that:

```text
correct mass
+
correct whole-body CoM
+
correct centroidal wrench
+
existing WBIC
```

can eliminate or substantially reduce the standing pitch bias caused by payload/model mismatch.