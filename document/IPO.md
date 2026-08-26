# IPO các module trong WBIC

## Dynamics 
| | Nội dung |
|---|---|
| **I** | `q`, `v`, `model`, `foot_id[4]` |
| **P** | 1. `forwardKinematics(model, data, q, v, 0₁₈)` ← **a = 0 bắt buộc**<br>2. `computeJointJacobians`, `updateFramePlacements`<br>3. `crba` → **mirror tam giác dưới** → `A`<br>4. `Ainv = A.llt().solve(I)`<br>5. `nonLinearEffects` → `bg`<br>6. ∀i: `getFrameJacobian(LOCAL_WORLD_ALIGNED)` → `Jf[i]`; `getFrameClassicalAcceleration(LOCAL_WORLD_ALIGNED).linear()` → `J̇q̇f[i]`; `oMf.translation()` → `pf[i]`<br>7. `Jbase = blkdiag(R,R)` trên 6 cột đầu; `J̇q̇base = [R(ω_B×v_B); 0₃]` |
| **O** | `A, Ainv, bg, Jf[4], J̇q̇f[4], pf[4], Jbase, J̇q̇base` |

## Contact Assemly
| | Nội dung |
|---|---|
| **I** | `contact[4]`, `f_mpc[4]`, `phase[4]`, `Jf[4]`, `J̇q̇f[4]` |
| **P** | Duyệt `i = 0..3`, nếu `contact[i]`: push `i` vào `cid`; xếp chồng `Jc`, `J̇cq̇`, `f_mpc_vec` theo **đúng thứ tự `cid`** |
| **O** | `cid`, `nc`, `Jc(3nc×18)`, `J̇cq̇(3nc)`, `f_mpc_vec(3nc)` |

## Task Stack
| | Nội dung |
|---|---|
| **I** | `TaskCommand`, `Jbase`, `J̇q̇base`, `Jf`, `pf`, `v`, `R`, gains |
| **P** | **Task 1 – Orientation:** `J = Jbase.bottomRows<3>()`, `e = log3(R_des·Rᵀ)`, `ė = ω_des_W − R·ω_B`, `ẍ = ω̇_des + Kp∘e + Kd∘ė`<br>**Task 2 – Position:** `J = Jbase.topRows<3>()`, `e = p_des − p_W`, `ė = ṗ_des − v_W`<br>**Task 3..n – Foot (chỉ swing):** `J = Jf[i]`, `e = pf_des − pf[i]`, `ė = ṗf_des − Jf[i]·v` |
| **O** | `std::vector<KinTask> tasks` — **thứ tự cố định** Ori → Pos → Foot |