# Codex Implementation Plan — GMO Contact Estimator for Lite3

## Mục tiêu

Triển khai **Generalized Momentum Observer (GMO)** trên branch:

`feat/contact-estimator`

của repo:

`PhamMinhTuanGit/lite3_mpc`

Mục tiêu của phase này là tạo pipeline:

```text
Robot state
    │
    ▼
Lite3StateMapper
    │
    ├── q ∈ R19
    ├── v ∈ R18
    └── τ ∈ R12
    │
    ▼
Lite3Dynamics / Pinocchio
    │
    ├── M(q)
    ├── C(q,v)
    ├── g(q)
    └── Jfoot(q)
    │
    ▼
Generalized Momentum Observer
    │
    └── residual r ∈ R18
    │
    ▼
GRF estimator
    │
    ├── F_FR
    ├── F_FL
    ├── F_HR
    └── F_HL
    │
    ▼
Telemetry / logging only
```

**Không sử dụng output GMO để điều khiển robot hoặc thay đổi `contactEstimate` ở phase này.**

Mục tiêu trước mắt là validate rằng generalized residual và estimated GRF đúng trong simulation.

---

# 1. Nguyên tắc triển khai

Không sửa lớn architecture hiện tại.

Không thay MPC.

Không thay Kalman Filter.

Không thay `ContactEstimator` hiện tại ở bước đầu.

Không dùng numerical differentiation của velocity hoặc acceleration nếu có thể tránh.

Không dùng:

```cpp
M * qdd
```

để estimate external force.

Observer phải dựa trên generalized momentum:

[
p=M(q)v
]

và không yêu cầu đo (\dot v).

Toàn bộ module phải chạy được trên ARM64 deployment của Lite3.

Tránh thêm dependency mới ngoài:

* Eigen
* Pinocchio hiện có.

---

# 2. Kiểm tra model hiện tại

File:

```text
cmpc_ctrl/src/Pinocchio_Dynamics/Lite3DynamicModel.cpp
cmpc_ctrl/src/Pinocchio_Dynamics/include/Lite3DynamicModel.hpp
```

Hiện model đã có:

```cpp
computeMassMatrix(q)
computeNonlinearEffects(q, v)
computeGravity(q)
```

với:

```text
nq = 19
nv = 18
```

và floating base.

Giữ nguyên API hiện tại nếu không cần thiết phải thay đổi.

---

# 3. Thêm Coriolis matrix

## File cần sửa

```text
Lite3DynamicModel.hpp
Lite3DynamicModel.cpp
```

Thêm API:

```cpp
const Eigen::MatrixXd& computeCoriolisMatrix(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v);
```

Implementation dùng:

```cpp
pinocchio::computeCoriolisMatrix(model_, data_, q, v);
```

Return:

```cpp
data_.C
```

---

# 4. Thêm foot Jacobian bằng Pinocchio

Thêm:

```cpp
Eigen::Matrix<double, 3, Lite3Dynamics::kNv>
computeFootJacobian(
    Leg leg,
    const Eigen::VectorXd& q);
```

Jacobians phải dùng:

```cpp
pinocchio::LOCAL_WORLD_ALIGNED
```

Pipeline:

```cpp
pinocchio::forwardKinematics(...);
pinocchio::computeJointJacobians(...);
pinocchio::updateFramePlacements(...);

pinocchio::getFrameJacobian(...);
```

Chỉ trả về translational Jacobian:

```cpp
J6.topRows<3>()
```

Output:

[
J_i\in\mathbb R^{3\times18}
]

cho:

```text
FR
FL
HR
HL
```

theo public convention của controller.

---

# 5. Thêm unit test cho dynamics

Tạo:

```text
tests/test_lite3_dynamics.cpp
```

hoặc đưa vào test infrastructure hiện có nếu repo đã có.

## Test 1 — dimensions

Kiểm tra:

```text
nq = 19
nv = 18
mass matrix = 18x18
C = 18x18
g = 18
Jfoot = 3x18
```

## Test 2 — symmetry của M

[
M=M^T
]

Test:

```cpp
(M - M.transpose()).norm() < tolerance
```

## Test 3 — nonlinear effects

Kiểm tra:

[
h(q,v)=C(q,v)v+g(q)
]

với:

```cpp
h = computeNonlinearEffects(q, v);
C = computeCoriolisMatrix(q, v);
g = computeGravity(q);

error = h - (C * v + g);
```

Yêu cầu error nhỏ.

Không hard-code tolerance quá chặt nếu khác biệt numerical precision.

---

# 6. Tạo `Lite3StateMapper`

Tạo:

```text
cmpc_ctrl/src/Pinocchio_Dynamics/Lite3StateMapper.cpp

cmpc_ctrl/src/Pinocchio_Dynamics/include/Lite3StateMapper.hpp
```

Mục tiêu duy nhất của class này:

```text
StateEstimate
+
LegControllerData[4]
        ↓
Pinocchio q, v, tau
```

API đề xuất:

```cpp
class Lite3StateMapper {
public:
    static void buildConfiguration(
        const StateEstimate<float>& state,
        const LegControllerData<float>* legs,
        Eigen::VectorXd& q);

    static void buildVelocity(
        const StateEstimate<float>& state,
        const LegControllerData<float>* legs,
        Eigen::VectorXd& v);

    static void buildMotorTorque(
        const LegControllerData<float>* legs,
        Eigen::VectorXd& tau);
};
```

Có thể dùng `double` internal nếu thuận tiện.

---

# 7. Configuration mapping

Pinocchio configuration:

```text
q[0:3]   base position
q[3:7]   quaternion x y z w
q[7:19]  12 joints
```

Base:

```cpp
q.segment<3>(0) = state.position;
```

## Quaternion

Controller `StateEstimate::orientation` đang dùng:

```text
w x y z
```

Pinocchio cần:

```text
x y z w
```

Map explicit:

```cpp
q[3] = state.orientation[1];
q[4] = state.orientation[2];
q[5] = state.orientation[3];
q[6] = state.orientation[0];
```

Normalize quaternion trước khi đưa vào Pinocchio.

---

# 8. Joint order mapping

Đây là phần quan trọng.

Controller convention:

```text
0 = FR
1 = FL
2 = HR
3 = HL
```

Nhưng Pinocchio model được construct theo:

```text
FL
FR
HL
HR
```

Không được assume joint vector order giống controller.

Mapping explicit:

```text
Pinocchio FL ← controller leg 1
Pinocchio FR ← controller leg 0
Pinocchio HL ← controller leg 3
Pinocchio HR ← controller leg 2
```

Tương ứng:

```cpp
q.segment<3>(7)  = legs[1].q; // FL
q.segment<3>(10) = legs[0].q; // FR
q.segment<3>(13) = legs[3].q; // HL
q.segment<3>(16) = legs[2].q; // HR
```

Không duplicate magic mapping nhiều nơi.

Tạo helper table:

```cpp
constexpr std::array<int, 4> kPinocchioToControllerLeg = {
    1, // FL
    0, // FR
    3, // HL
    2  // HR
};
```

hoặc giải pháp rõ ràng tương đương.

---

# 9. Velocity mapping

Pinocchio generalized velocity:

[
v=
\begin{bmatrix}
v_b\
\omega_b\
\dot q_j
\end{bmatrix}
]

Cần xác nhận convention free-flyer của Pinocchio và dùng đúng frame.

Ưu tiên xây:

```cpp
v.head<3>()
v.segment<3>(3)
```

từ:

```text
state.vBody
state.omegaBody
```

nếu Pinocchio free-flyer velocity đang biểu diễn trong LOCAL/body frame.

Không tự ý dùng `vWorld` nếu convention của Pinocchio yêu cầu body frame.

Viết comment rõ convention.

Joint velocities map cùng order với joint positions.

---

# 10. Torque mapping

Dùng:

```cpp
legs[i].tauEstimate
```

là measured motor torque.

Không dùng MPC torque command làm input chính của GMO.

Output:

```cpp
Eigen::VectorXd tau_motor(12);
```

theo Pinocchio actuated joint order:

```text
FL
FR
HL
HR
```

Mỗi chân:

```text
HipX
HipY
Knee
```

Cần verify HipX sign.

---

# 11. Test mapping bằng kinematics

Trước khi viết GMO, thêm một executable/test để kiểm tra state mapping.

Ví dụ:

```text
tests/test_lite3_state_mapper.cpp
```

Với nhiều random joint configurations:

1. Set joint states vào controller representation.
2. Build Pinocchio `q`.
3. Compute foot positions bằng Pinocchio.
4. Compute foot positions bằng existing:

```cpp
computeLegJacobianAndPosition()
```

5. So sánh.

Làm tương tự với Jacobian.

Mục tiêu:

[
p_{MIT}\approx p_{Pinocchio}
]

[
J_{MIT}\approx J_{Pinocchio,joint}
]

Nếu sign không khớp, xử lý ở `Lite3StateMapper`.

Không sửa dynamics model để compensate sensor convention.

---

# 12. Tạo Generalized Momentum Observer

Tạo:

```text
cmpc_ctrl/src/Controllers/GeneralizedMomentumObserver.cpp

cmpc_ctrl/src/Controllers/GeneralizedMomentumObserver.h
```

Không kế thừa `GenericEstimator` ở version đầu.

Đây là một standalone dynamics observer.

API đề xuất:

```cpp
struct GMOResult {
    Eigen::Matrix<double, 18, 1> momentum;
    Eigen::Matrix<double, 18, 1> momentum_hat;
    Eigen::Matrix<double, 18, 1> residual;

    std::array<Eigen::Vector3d, 4> foot_force_world;
};

class GeneralizedMomentumObserver {
public:
    GeneralizedMomentumObserver(
        double dt,
        double gain);

    GMOResult update(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& v,
        const Eigen::VectorXd& motor_torque);

    void reset();

private:
    Lite3Dynamics dynamics_;

    Eigen::Matrix<double, 18, 1> p_hat_;
    Eigen::Matrix<double, 18, 1> residual_;

    double dt_;
    double gain_;
    bool initialized_;
};
```

Có thể chuyển gain thành vector sau.

---

# 13. Công thức GMO

Robot dynamics:

[
M(q)\dot v+C(q,v)v+g(q)
=======================

S^T\tau+\tau_{ext}
]

Generalized momentum:

[
p=M(q)v
]

Define:

[
\beta(q,v)
==========

g(q)-C(q,v)^Tv
]

Momentum dynamics:

[
\dot p
======

S^T\tau-\beta+\tau_{ext}
]

Observer:

[
\dot{\hat p}
============

S^T\tau-\beta+r
]

Residual:

[
r=K(p-\hat p)
]

Implementation mỗi tick:

```cpp
M = dynamics_.computeMassMatrix(q);
C = dynamics_.computeCoriolisMatrix(q, v);
g = dynamics_.computeGravity(q);

p = M * v;

beta = g - C.transpose() * v;
```

Build generalized torque:

```cpp
Eigen::Matrix<double, 18, 1> tau_gen;
tau_gen.setZero();

tau_gen.tail<12>() = motor_torque;
```

Initialization:

```cpp
if (!initialized_) {
    p_hat_ = p;
    residual_.setZero();
    initialized_ = true;
}
```

Update:

```cpp
residual_ = gain_ * (p - p_hat_);

p_hat_ += dt_ *
          (tau_gen - beta + residual_);
```

Không dùng acceleration.

---

# 14. Gain ban đầu

Control loop hiện tại khoảng:

```text
dt = 0.001 s
```

Bắt đầu conservative:

```text
K = 20–50 rad/s
```

Ví dụ:

```cpp
gain = 30.0;
```

Không tune aggressively ở version đầu.

Mục tiêu đầu tiên là xem signal.

Sau này có thể dùng diagonal gain:

```text
base translation
base rotation
joint residual
```

với gains khác nhau.

---

# 15. Chuyển residual thành GRF

Không sử dụng base residual để solve GRF ở version đầu.

Dùng actuated joint residual:

```text
residual.tail<12>()
```

và tách từng chân.

Với chân (i):

[
r_i\approx J_{i,a}^Tf_i
]

Trong đó:

[
J_{i,a}\in\mathbb R^{3\times3}
]

là joint block của foot Jacobian tương ứng với chân đó.

Estimate:

[
\hat f_i=
(J_iJ_i^T+\lambda I)^{-1}J_i r_i
]

Dùng damped least squares.

Implementation:

```cpp
Eigen::Matrix3d A =
    J_leg * J_leg.transpose()
    + lambda * Eigen::Matrix3d::Identity();

Eigen::Vector3d force =
    A.ldlt().solve(J_leg * residual_leg);
```

Khởi đầu:

```text
lambda = 1e-4
```

Expose thành parameter/constants để tune.

---

# 16. Không dùng trực tiếp `J.inverse()`

Không implement:

```cpp
J.transpose().inverse() * residual
```

vì dễ singular gần extended-leg configurations.

Luôn dùng:

```text
LDLT
+
damping
```

---

# 17. Force frame

GRF output phase đầu phải thống nhất:

```text
world frame
```

Tên biến rõ:

```cpp
foot_force_world
```

Nếu Jacobian dùng:

```cpp
LOCAL_WORLD_ALIGNED
```

thì force output tương ứng world-aligned translational axes.

Không mix với body-frame force từ existing MIT leg Jacobian.

---

# 18. Leg output convention

Public output GMO phải giữ controller convention:

```text
0 FR
1 FL
2 HR
3 HL
```

Dù internal Pinocchio order khác.

Không expose Pinocchio construction order ra bên ngoài module.

---

# 19. Tích hợp vào control loop

Version đầu không tích hợp vào `StateEstimatorContainer`.

Tạo GMO instance trong `GaitCtrller`.

Ví dụ member:

```cpp
std::unique_ptr<GeneralizedMomentumObserver> gmo_;
Lite3StateMapper state_mapper_;
```

Khởi tạo:

```cpp
gmo_ = std::make_unique<GeneralizedMomentumObserver>(
    0.001,
    30.0);
```

---

# 20. Vị trí chạy GMO

Current flow:

```text
PreWork()
    ↓
leg data update
    ↓
state estimator
    ↓
MPC
    ↓
leg control
```

Version đầu chạy GMO sau:

```cpp
PreWork()
```

và trước MPC.

Flow:

```text
IMU + motor feedback
        ↓
LegController::updateData
        ↓
OrientationEstimator
        ↓
LinearKF
        ↓
Lite3StateMapper
        ↓
GMO
        ↓
LOG ONLY
        ↓
MPC
```

GMO phải sử dụng measured motor torque từ tick hiện tại.

---

# 21. Không thay ContactEstimator ở phase này

Giữ nguyên:

```cpp
ContactEstimator<T>::run()
```

và behavior hiện tại.

Không set:

```cpp
state.contactEstimate = GMO...
```

Không thay covariance Kalman Filter.

Lý do:

`LinearKFPositionVelocityEstimator` hiện đang hiểu `contactEstimate` gần giống gait phase/trust profile, không phải probability thật.

Nếu đưa probability trực tiếp vào sẽ sai semantics.

---

# 22. Thêm telemetry GMO

Mở rộng:

```cpp
CmpcTelemetryData
```

thêm:

```cpp
float gmo_residual[18];

float gmo_force_world[4][3];

float gmo_force_norm[4];

float gmo_fz[4];
```

Optional:

```cpp
float gmo_momentum[18];
float gmo_momentum_hat[18];
```

Không cần log full matrices `M` hoặc `C`.

---

# 23. Ground-truth trong MuJoCo

Nếu simulation interface cho phép lấy contact force thực từ MuJoCo, thêm telemetry:

```cpp
float gt_contact_force_world[4][3];
float gt_contact[4];
```

Mục tiêu chính:

plot:

```text
GMO Fz FR vs MuJoCo Fz FR
GMO Fz FL vs MuJoCo Fz FL
GMO Fz HR vs MuJoCo Fz HR
GMO Fz HL vs MuJoCo Fz HL
```

---

# 24. Validation scenario 1 — robot đứng yên

Robot đứng 4 chân trên mặt phẳng.

Expected:

[
\sum_i F_{z,i}
\approx mg
]

Với model:

```text
mass ≈ 11.9376 kg
```

Expected total vertical force:

[
mg\approx117.1,N
]

Mỗi chân khi đứng cân bằng khoảng:

[
29,N
]

Không require từng chân chính xác 29 N vì CoM và posture làm phân bố khác nhau.

---

# 25. Validation scenario 2 — nhấc một chân

Robot đứng 3 chân.

Expected:

```text
swing leg Fz ≈ 0
stance legs Fz > 0
```

GMO phải phân biệt rõ swing/stance.

---

# 26. Validation scenario 3 — trot chậm

Run trot ở tốc độ thấp.

Plot:

```text
scheduled stance
actual MuJoCo contact
GMO Fz
```

Expected:

* GMO Fz tăng nhanh khi touchdown.
* GMO Fz về gần 0 khi liftoff.
* Signal không oscillate mạnh trong swing.
* Không xuất force rất lớn ở singularities.

---

# 27. Validation scenario 4 — early contact

Tạo terrain/chướng ngại vật khiến swing foot chạm đất sớm.

Expected:

```text
scheduled contact = false
MuJoCo contact = true
GMO Fz rises
```

Đây là use case quan trọng nhất cho contact estimator sau này.

---

# 28. Validation scenario 5 — model mismatch

Nếu thuận tiện, thay:

```text
body mass
payload
```

trong simulator nhưng giữ Pinocchio model cũ.

Quan sát residual bias.

Không cố giải quyết mismatch trong phase hiện tại.

Chỉ ghi lại mức bias để chuẩn bị threshold/fusion.

---

# 29. Logging cần có

Mỗi tick hoặc downsample hợp lý:

```text
timestamp

q[19]
v[18]
tau_motor[12]

momentum[18]
momentum_hat[18]
residual[18]

F_FR_xyz
F_FL_xyz
F_HR_xyz
F_HL_xyz

scheduled_contact[4]

MuJoCo_contact[4]
MuJoCo_GRF[4][3]
```

Nếu log bandwidth quá lớn, ưu tiên:

```text
residual_joint[12]
estimated_GRF[12]
scheduled_contact[4]
ground_truth_contact[4]
ground_truth_GRF[12]
```

---

# 30. Safety requirements

GMO phase đầu phải là read-only.

Không được:

* sửa motor torque command;
* sửa MPC force;
* trigger safety;
* thay gait;
* thay state estimator;
* thay contact schedule.

Nếu GMO throw exception hoặc sinh NaN:

```text
controller vẫn phải hoạt động bình thường
```

Có guard:

```cpp
if (!result.residual.allFinite()) {
    gmo_->reset();
}
```

Output invalid nên zero hoặc mark invalid.

---

# 31. Performance requirements

Target loop:

```text
1 kHz
```

Không allocate Eigen matrices lớn liên tục nếu có thể tránh.

Sau version correctness, optimize:

* fixed-size matrices;
* reuse Pinocchio `Data`;
* avoid unnecessary repeated forward kinematics;
* avoid repeated heap allocations.

Nhưng ưu tiên correctness trước premature optimization.

Thêm timing:

```cpp
Timer t_gmo;
...
telem->t_gmo_ms = t_gmo.getMs();
```

Target ban đầu:

```text
GMO << 1 ms
```

ưu tiên khoảng dưới vài trăm microseconds nếu khả thi.

---

# 32. CMake

Thêm source:

```text
cmpc_ctrl/src/Pinocchio_Dynamics/Lite3StateMapper.cpp

cmpc_ctrl/src/Controllers/GeneralizedMomentumObserver.cpp
```

vào:

```cmake
CMPC_SRCS
```

Không thêm external library.

---

# 33. Definition of Done — Phase 1

Phase GMO được coi là hoàn thành khi:

* `Lite3Dynamics` trả được `M`, `C`, `g`, `Jfoot`.
* Test xác nhận:

[
h\approx Cv+g.
]

* Có `Lite3StateMapper`.
* Joint/leg order đã được validate bằng foot position/Jacobian.
* GMO chạy ổn định ở 1 kHz.
* Không dùng acceleration.
* Không xuất NaN/Inf.
* Có residual 18-DOF.
* Có estimated GRF 4 chân.
* Output giữ convention `[FR, FL, HR, HL]`.
* Có telemetry.
* Robot simulation hoạt động giống trước khi bật GMO.
* Có plot estimated Fz so với ground-truth.
* GMO detect được touchdown/liftoff qualitatively.

---

# 34. Chưa làm trong Phase 1

Không triển khai những phần sau:

```text
contact probability
Bayesian fusion
phase-conditioned probability
terrain-dependent sigma
event-based gait switching
KF covariance adaptation bằng GMO
MPC adaptation
payload identification
model adaptation
online inertia estimation
```

Những phần này thuộc phase tiếp theo.

---

# 35. Phase 2 sau khi GMO validated

Sau khi estimated GRF đã đáng tin cậy, mới tạo:

```text
GMO force
   +
gait prior
   +
foot kinematics
   ↓
Probabilistic Contact Estimator
   ↓
P(contact_i)
```

Sau đó tách rõ:

```cpp
scheduledContactPhase
contactProbability
contactTrust
```

Không tiếp tục dùng một biến `contactEstimate` cho nhiều semantics.

Cuối cùng mới dùng:

```text
contactProbability / contactTrust
```

để điều chỉnh noise covariance của Linear KF.

---

# 36. Ưu tiên thực hiện

Codex thực hiện theo đúng thứ tự:

```text
1. C(q,v)
2. Jfoot
3. Dynamics tests
4. Lite3StateMapper
5. State-mapping tests
6. GeneralizedMomentumObserver
7. residual → GRF
8. Integrate read-only into GaitCtrller
9. telemetry
10. simulation validation
```

Không bắt đầu bước 6 nếu mapping/sign/order vẫn chưa được validate.

Không sửa `ContactEstimator` cho đến khi Phase 1 hoàn tất.

---

# 37. Deliverables

Sau khi hoàn thành, báo cáo:

1. Danh sách file được thêm/sửa.
2. Convention chính xác của `q`, `v`, torque.
3. Leg-order mapping.
4. Quaternion convention.
5. HipX sign convention.
6. GMO equations implemented.
7. GMO gain và damping hiện tại.
8. Timing trung bình/max của GMO.
9. Test results.
10. Kết quả đứng yên:

* tổng estimated (F_z);
* expected (mg).

11. Kết quả trot:

* correlation qualitative giữa estimated GRF và MuJoCo GRF.

12. Các mismatch hoặc vấn đề còn tồn tại.

Ưu tiên correctness, numerical stability và convention consistency hơn việc tối ưu code hoặc tích hợp contact logic sớm.
