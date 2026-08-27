#!/usr/bin/env python3
"""
Run Case C experiment matrix evaluating Force-Rate Regularization Penalty in Centroidal GRF QP:
  - CASE C0: lambda_df = 0.0
  - CASE C1: lambda_df = 0.01 * scale = 0.10
  - CASE C2: lambda_df = 0.05 * scale = 0.50
  - CASE C3: lambda_df = 0.10 * scale = 1.00
  - CASE C4: lambda_df = 0.20 * scale = 2.00

Scale calculation:
  H_0 = w_F * A_F^T * A_F + w_M * A_M^T * A_M
  scale = mean(diag(H_0)) ≈ 10.0 (derived analytically and verified empirically).
"""

import os
import subprocess
import time
import csv
import math
import numpy as np

WORKSPACE = "/home/tuanpm/Lite3_rl_deploy_cmpc/Lite3_rl_deploy"
PYTHON_BIN = os.path.join(WORKSPACE, ".venv/bin/python3")
SIM_SCRIPT = os.path.join(WORKSPACE, "interface/robot/simulation/mujoco_simulation_with_payload.py")
DEPLOY_BIN = os.path.join(WORKSPACE, "build/cmpc_deploy")

def run_case(case_name, sim_env_vars, deploy_env_vars, csv_path, duration=25.0):
    print(f"\n=======================================================")
    print(f" Starting Experiment: {case_name}")
    print(f" CSV Target: {csv_path}")
    print(f"=======================================================")

    if os.path.exists(csv_path):
        os.remove(csv_path)

    # 1. Start simulation
    sim_env = os.environ.copy()
    sim_env["VIEWER"] = "0"
    sim_env["SIM_DURATION"] = str(duration)
    sim_env.update(sim_env_vars)

    sim_proc = subprocess.Popen(
        [PYTHON_BIN, SIM_SCRIPT],
        cwd=os.path.dirname(SIM_SCRIPT),
        env=sim_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(1.0) # Wait for UDP socket bind

    # 2. Start controller
    deploy_env = os.environ.copy()
    deploy_env["LITE3_STANDING_DIAGNOSTIC_CSV"] = csv_path
    deploy_env["LITE3_AUTO_START"] = "1"
    deploy_env.update(deploy_env_vars)

    deploy_proc = subprocess.Popen(
        [DEPLOY_BIN],
        cwd=WORKSPACE,
        env=deploy_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    # 3. Wait for simulation to finish (or timeout)
    t_start = time.time()
    try:
        sim_proc.wait(timeout=duration + 15.0)
    except subprocess.TimeoutExpired:
        print("[WARN] Sim process timeout, killing...")
        sim_proc.kill()

    # 4. Terminate controller
    deploy_proc.terminate()
    try:
        deploy_proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        deploy_proc.kill()

    print(f" Finished {case_name}. Elapsed: {time.time() - t_start:.2f}s")
    time.sleep(1.0)


def analyze_csv(csv_path):
    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV not found: {csv_path}")
        return None

    rows = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                t = float(r["time"])
                if t >= 5.0:
                    rows.append(r)
            except (ValueError, KeyError):
                continue

    if len(rows) == 0:
        print(f"[ERROR] No data for t >= 5.0s in {csv_path}")
        return None

    def get_arr(key, default=0.0):
        res = []
        for r in rows:
            val = r.get(key, None)
            if val is None or val == "" or val == "nan":
                res.append(default)
            else:
                try:
                    res.append(float(val))
                except ValueError:
                    res.append(default)
        return np.array(res, dtype=np.float64)

    pitch = get_arr("pitch")
    roll = get_arr("roll")
    omega_y = get_arr("omega_y")
    ori_err_y = get_arr("orientation_error_y")
    x_ddot_ori_y = get_arr("x_ddot_ori_y")

    fz_front = get_arr("Fz_front")
    fz_rear = get_arr("Fz_rear")
    fz_total = get_arr("Fz_total")

    my_des = get_arr("My_des")
    my_grf = get_arr("My_grf")
    my_tracking_err = my_grf - my_des

    df_norm = get_arr("df_norm")

    Fz_gravity = get_arr("Fz_gravity")
    Fz_pos_P = get_arr("Fz_pos_P")
    Fz_vel_D = get_arr("Fz_vel_D")
    My_P = get_arr("My_P")
    My_D = get_arr("My_D")

    grf_qp_time = get_arr("GRF_QP_solve_time_us")
    grf_qp_status = get_arr("GRF_QP_status")
    fallback = get_arr("fallback_count")

    status_col = [r.get("wbic_status", "") for r in rows]
    wbic_fails = sum(1 for s in status_col if s not in ("OK", "DISABLED", ""))

    metrics = {
        "samples": len(rows),
        "pitch_mean": float(np.mean(pitch)),
        "pitch_rms": float(np.sqrt(np.mean(pitch**2))),
        "pitch_p2p": float(np.ptp(pitch)),
        "pitch_mean_deg": float(np.rad2deg(np.mean(pitch))),
        
        "roll_mean": float(np.mean(roll)),
        "roll_rms": float(np.sqrt(np.mean(roll**2))),
        "omega_y_rms": float(np.sqrt(np.mean(omega_y**2))),

        "fz_front_mean": float(np.mean(fz_front)),
        "fz_front_std": float(np.std(fz_front)),
        "fz_rear_mean": float(np.mean(fz_rear)),
        "fz_rear_std": float(np.std(fz_rear)),
        "fz_total_mean": float(np.mean(fz_total)),
        "fz_total_std": float(np.std(fz_total)),

        "my_des_mean": float(np.mean(my_des)),
        "my_des_std": float(np.std(my_des)),
        "my_grf_mean": float(np.mean(my_grf)),
        "my_grf_std": float(np.std(my_grf)),
        "my_tracking_rms": float(np.sqrt(np.mean(my_tracking_err**2))),

        "df_norm_mean": float(np.mean(df_norm)),
        "df_norm_max": float(np.max(df_norm)),

        "Fz_gravity_mean": float(np.mean(Fz_gravity)),
        "Fz_pos_P_mean": float(np.mean(Fz_pos_P)),
        "Fz_pos_P_std": float(np.std(Fz_pos_P)),
        "Fz_vel_D_mean": float(np.mean(Fz_vel_D)),
        "Fz_vel_D_std": float(np.std(Fz_vel_D)),
        "My_P_mean": float(np.mean(My_P)),
        "My_P_std": float(np.std(My_P)),
        "My_D_mean": float(np.mean(My_D)),
        "My_D_std": float(np.std(My_D)),

        "grf_qp_time_mean": float(np.mean(grf_qp_time)),
        "grf_qp_time_max": float(np.max(grf_qp_time)),
        "grf_qp_failures": int(np.sum(grf_qp_status > 1)),
        "wbic_failures": int(wbic_fails),
        "fallback_count": float(fallback[-1] if len(fallback) > 0 else 0.0)
    }
    return metrics


def main():
    SCALE = 10.0 # mean(diag(H_0)) ≈ 10.0
    cases = [
        ("Case C0 (lambda=0.0)", 0.0 * SCALE, "/tmp/lite3_exp_case_c0.csv"),
        ("Case C1 (lambda=0.10)", 0.01 * SCALE, "/tmp/lite3_exp_case_c1.csv"),
        ("Case C2 (lambda=0.50)", 0.05 * SCALE, "/tmp/lite3_exp_case_c2.csv"),
        ("Case C3 (lambda=1.00)", 0.10 * SCALE, "/tmp/lite3_exp_case_c3.csv"),
        ("Case C4 (lambda=2.00)", 0.20 * SCALE, "/tmp/lite3_exp_case_c4.csv"),
    ]

    results = []
    for case_name, lambda_val, csv_path in cases:
        sim_env = {
            "GMO_ENABLE_PAYLOAD": "1",
            "ENABLE_PAYLOAD": "1"
        }
        deploy_env = {
            "LITE3_USE_CENTROIDAL_WRENCH": "1",
            "LITE3_PAYLOAD_AWARE": "1",
            "LITE3_PAYLOAD_MASS": "2.0",
            "LITE3_PAYLOAD_COM_X": "0.10",
            "LITE3_GRF_FORCE_RATE_WEIGHT": str(lambda_val)
        }
        run_case(case_name, sim_env, deploy_env, csv_path, duration=25.0)
        res = analyze_csv(csv_path)
        results.append((case_name, lambda_val, res))

    print("\n" + "=" * 125)
    print("                    CASE C FORCE-RATE REGULARIZATION EXPERIMENT MATRIX (t >= 5.0 s)")
    print("=" * 125)
    header = f"{'Metric':<32} | {'C0 (λ=0.0)':<16} | {'C1 (λ=0.10)':<16} | {'C2 (λ=0.50)':<16} | {'C3 (λ=1.00)':<16} | {'C4 (λ=2.00)':<16}"
    print(header)
    print("-" * 125)

    def row(label, key, fmt="{:+.6f}", key2=None):
        vals = []
        for _, _, res in results:
            if res is None:
                vals.append("N/A")
            elif key2:
                vals.append(f"{res[key]:.2f} ± {res[key2]:.2f}")
            else:
                val = res[key]
                if isinstance(val, int):
                    vals.append(f"{val}")
                elif isinstance(val, float):
                    vals.append(fmt.format(val))
                else:
                    vals.append(str(val))
        print(f"{label:<32} | {vals[0]:<16} | {vals[1]:<16} | {vals[2]:<16} | {vals[3]:<16} | {vals[4]:<16}")

    row("Pitch Mean (rad)", "pitch_mean", "{:+.6f}")
    row("Pitch Mean (deg)", "pitch_mean_deg", "{:+.4f}°")
    row("Pitch RMS (rad)", "pitch_rms", "{:.6f}")
    row("Pitch Peak-to-Peak (rad)", "pitch_p2p", "{:.6f}")
    row("Omega_y RMS (rad/s)", "omega_y_rms", "{:.6f}")
    row("Fz Total Mean/Std (N)", "fz_total_mean", key2="fz_total_std")
    row("Fz Front Mean/Std (N)", "fz_front_mean", key2="fz_front_std")
    row("Fz Rear Mean/Std (N)", "fz_rear_mean", key2="fz_rear_std")
    row("My_des Mean/Std (Nm)", "my_des_mean", key2="my_des_std")
    row("My_grf Mean/Std (Nm)", "my_grf_mean", key2="my_grf_std")
    row("My Tracking RMS (Nm)", "my_tracking_rms", "{:.6f}")
    row("Mean ||f_k - f_{k-1}|| (N)", "df_norm_mean", "{:.4f}")
    row("Max ||f_k - f_{k-1}|| (N)", "df_norm_max", "{:.4f}")
    row("Fz_pos_P Std (N)", "Fz_pos_P_std", "{:.4f}")
    row("Fz_vel_D Std (N)", "Fz_vel_D_std", "{:.4f}")
    row("My_P Std (Nm)", "My_P_std", "{:.4f}")
    row("My_D Std (Nm)", "My_D_std", "{:.4f}")
    row("GRF QP Solve Time (µs)", "grf_qp_time_mean", "{:.1f}")
    row("GRF QP Failures", "grf_qp_failures")
    row("WBIC Failures", "wbic_failures")
    row("Fallback Count", "fallback_count", "{:.0f}")
    print("=" * 125)


if __name__ == "__main__":
    main()
