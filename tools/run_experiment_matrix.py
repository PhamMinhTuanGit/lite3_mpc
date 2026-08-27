#!/usr/bin/env python3
"""
Run the experiment matrix for Centroidal Wrench Controller + WBIC validation:
  - Case A: Nominal robot (no payload in sim), nominal Convex MPC + WBIC
  - Case B: Payload in sim (2 kg at +0.10 m), nominal Convex MPC + WBIC (baseline mismatch)
  - Case C: Payload in sim (2 kg at +0.10 m), Payload-Aware Centroidal Wrench + WBIC

Computes and prints comprehensive metrics for t >= 5.0 s using standard csv & numpy.
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

def run_case(case_name, sim_env_vars, deploy_env_vars, csv_path, duration=30.0):
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
    torque_margin = get_arr("torque_margin", default=100.0)

    kin_residual = get_arr("kin_contact_residual")
    eom_residual = get_arr("eom_residual")

    grf_qp_time = get_arr("GRF_QP_solve_time_us")
    grf_qp_status = get_arr("GRF_QP_status")

    wbic_status_list = [r.get("wbic_status", "") for r in rows]
    wbic_failures = sum(1 for s in wbic_status_list if s not in ("OK", "QP_MAX_ITER"))
    
    fallback_arr = get_arr("fallback_count")
    fallback_count = fallback_arr[-1] - fallback_arr[0] if len(fallback_arr) > 0 else 0

    metrics = {
        "samples": len(rows),
        "pitch_mean": np.mean(pitch),
        "pitch_rms": np.sqrt(np.mean(pitch**2)),
        "pitch_p2p": np.ptp(pitch),
        "pitch_mean_deg": np.degrees(np.mean(pitch)),
        "roll_mean": np.mean(roll),
        "roll_rms": np.sqrt(np.mean(roll**2)),
        "omega_y_rms": np.sqrt(np.mean(omega_y**2)),
        "ori_err_y_mean": np.mean(ori_err_y),
        "x_ddot_ori_y_mean": np.mean(x_ddot_ori_y),
        "x_ddot_ori_y_rms": np.sqrt(np.mean(x_ddot_ori_y**2)),
        "fz_front_mean": np.mean(fz_front),
        "fz_front_std": np.std(fz_front),
        "fz_rear_mean": np.mean(fz_rear),
        "fz_rear_std": np.std(fz_rear),
        "fz_total_mean": np.mean(fz_total),
        "fz_total_std": np.std(fz_total),
        "my_des_mean": np.mean(my_des),
        "my_des_std": np.std(my_des),
        "my_grf_mean": np.mean(my_grf),
        "my_grf_std": np.std(my_grf),
        "torque_margin_min": np.nanmin(torque_margin),
        "kin_residual_max": np.nanmax(kin_residual),
        "eom_residual_max": np.nanmax(eom_residual),
        "grf_qp_time_mean": np.mean(grf_qp_time),
        "grf_qp_time_max": np.max(grf_qp_time),
        "grf_qp_failures": np.sum(grf_qp_status == 2),
        "wbic_failures": wbic_failures,
        "fallback_count": fallback_count,
    }
    return metrics


def main():
    csv_a = "/tmp/lite3_exp_case_a.csv"
    csv_b = "/tmp/lite3_exp_case_b.csv"
    csv_c = "/tmp/lite3_exp_case_c.csv"

    # Case A: Nominal MuJoCo (no payload), Nominal Controller
    run_case(
        case_name="Case A: Nominal Baseline",
        sim_env_vars={"GMO_ENABLE_PAYLOAD": "0"},
        deploy_env_vars={"LITE3_AUTO_START": "1", "LITE3_USE_CENTROIDAL_WRENCH": "0", "LITE3_PAYLOAD_AWARE": "0"},
        csv_path=csv_a,
        duration=30.0
    )

    # Case B: Payload MuJoCo (2kg @ +0.10m), Nominal Controller (Convex MPC + WBIC)
    run_case(
        case_name="Case B: Payload Baseline (Convex MPC + WBIC)",
        sim_env_vars={"GMO_ENABLE_PAYLOAD": "1"},
        deploy_env_vars={"LITE3_AUTO_START": "1", "LITE3_USE_CENTROIDAL_WRENCH": "0", "LITE3_PAYLOAD_AWARE": "0"},
        csv_path=csv_b,
        duration=30.0
    )

    # Case C: Payload MuJoCo (2kg @ +0.10m), Payload-Aware Centroidal Wrench + WBIC
    run_case(
        case_name="Case C: Payload-Aware Centroidal Wrench + WBIC",
        sim_env_vars={"GMO_ENABLE_PAYLOAD": "1"},
        deploy_env_vars={
            "LITE3_AUTO_START": "1",
            "LITE3_USE_CENTROIDAL_WRENCH": "1",
            "LITE3_PAYLOAD_AWARE": "1",
            "LITE3_PAYLOAD_MASS": "2.0",
            "LITE3_PAYLOAD_COM_X": "0.10"
        },
        csv_path=csv_c,
        duration=30.0
    )

    # Analysis
    res_a = analyze_csv(csv_a)
    res_b = analyze_csv(csv_b)
    res_c = analyze_csv(csv_c)

    print("\n" + "="*85)
    print("                    EXPERIMENT MATRIX RESULTS (t >= 5.0 s)")
    print("="*85)
    fmt = "{:<32} | {:<16} | {:<25} | {:<20}"
    print(fmt.format("Metric", "Case A (Nominal)", "Case B (Payload Mismatch)", "Case C (Payload Wrench)"))
    print("-" * 85)

    rows = [
        ("Pitch Mean (rad)", f"{res_a['pitch_mean']:+.6f}", f"{res_b['pitch_mean']:+.6f}", f"{res_c['pitch_mean']:+.6f}"),
        ("Pitch Mean (deg)", f"{res_a['pitch_mean_deg']:+.4f}°", f"{res_b['pitch_mean_deg']:+.4f}°", f"{res_c['pitch_mean_deg']:+.4f}°"),
        ("Pitch RMS (rad)", f"{res_a['pitch_rms']:.6f}", f"{res_b['pitch_rms']:.6f}", f"{res_c['pitch_rms']:.6f}"),
        ("Pitch Peak-to-Peak (rad)", f"{res_a['pitch_p2p']:.6f}", f"{res_b['pitch_p2p']:.6f}", f"{res_c['pitch_p2p']:.6f}"),
        ("Roll Mean (rad)", f"{res_a['roll_mean']:+.6f}", f"{res_b['roll_mean']:+.6f}", f"{res_c['roll_mean']:+.6f}"),
        ("Roll RMS (rad)", f"{res_a['roll_rms']:.6f}", f"{res_b['roll_rms']:.6f}", f"{res_c['roll_rms']:.6f}"),
        ("Omega_y RMS (rad/s)", f"{res_a['omega_y_rms']:.6f}", f"{res_b['omega_y_rms']:.6f}", f"{res_c['omega_y_rms']:.6f}"),
        ("Orientation Err_y Mean", f"{res_a['ori_err_y_mean']:+.6f}", f"{res_b['ori_err_y_mean']:+.6f}", f"{res_c['ori_err_y_mean']:+.6f}"),
        ("x_ddot_ori_y Mean (rad/s²)", f"{res_a['x_ddot_ori_y_mean']:+.4f}", f"{res_b['x_ddot_ori_y_mean']:+.4f}", f"{res_c['x_ddot_ori_y_mean']:+.4f}"),
        ("Fz Front Mean (N)", f"{res_a['fz_front_mean']:.2f} ± {res_a['fz_front_std']:.2f}", f"{res_b['fz_front_mean']:.2f} ± {res_b['fz_front_std']:.2f}", f"{res_c['fz_front_mean']:.2f} ± {res_c['fz_front_std']:.2f}"),
        ("Fz Rear Mean (N)", f"{res_a['fz_rear_mean']:.2f} ± {res_a['fz_rear_std']:.2f}", f"{res_b['fz_rear_mean']:.2f} ± {res_b['fz_rear_std']:.2f}", f"{res_c['fz_rear_mean']:.2f} ± {res_c['fz_rear_std']:.2f}"),
        ("Fz Total Mean (N)", f"{res_a['fz_total_mean']:.2f} ± {res_a['fz_total_std']:.2f}", f"{res_b['fz_total_mean']:.2f} ± {res_b['fz_total_std']:.2f}", f"{res_c['fz_total_mean']:.2f} ± {res_c['fz_total_std']:.2f}"),
        ("My_des Mean (Nm)", f"{res_a['my_des_mean']:+.3f}", f"{res_b['my_des_mean']:+.3f}", f"{res_c['my_des_mean']:+.3f}"),
        ("My_grf Mean (Nm)", f"{res_a['my_grf_mean']:+.3f}", f"{res_b['my_grf_mean']:+.3f}", f"{res_c['my_grf_mean']:+.3f}"),
        ("Torque Margin Min (Nm)", f"{res_a['torque_margin_min']:.2f}", f"{res_b['torque_margin_min']:.2f}", f"{res_c['torque_margin_min']:.2f}"),
        ("Contact Residual Max", f"{res_a['kin_residual_max']:.2e}", f"{res_b['kin_residual_max']:.2e}", f"{res_c['kin_residual_max']:.2e}"),
        ("EoM Residual Max", f"{res_a['eom_residual_max']:.2e}", f"{res_b['eom_residual_max']:.2e}", f"{res_c['eom_residual_max']:.2e}"),
        ("GRF QP Solve Time (µs)", f"{res_a['grf_qp_time_mean']:.1f} (max {res_a['grf_qp_time_max']:.1f})", f"{res_b['grf_qp_time_mean']:.1f} (max {res_b['grf_qp_time_max']:.1f})", f"{res_c['grf_qp_time_mean']:.1f} (max {res_c['grf_qp_time_max']:.1f})"),
        ("GRF QP Failures", f"{res_a['grf_qp_failures']}", f"{res_b['grf_qp_failures']}", f"{res_c['grf_qp_failures']}"),
        ("WBIC Failures", f"{res_a['wbic_failures']}", f"{res_b['wbic_failures']}", f"{res_c['wbic_failures']}"),
        ("Fallback Count", f"{res_a['fallback_count']}", f"{res_b['fallback_count']}", f"{res_c['fallback_count']}"),
    ]

    for r in rows:
        print(fmt.format(*r))
    print("="*85)

    # Acceptance check
    target_met = abs(res_c['pitch_mean']) < 0.01
    stretch_met = abs(res_c['pitch_mean']) < 0.005
    print(f"\n[EVALUATION]")
    print(f"  Case B Baseline Pitch Mean: {res_b['pitch_mean']:+.6f} rad ({res_b['pitch_mean_deg']:+.4f}°)")
    print(f"  Case C New Controller Pitch Mean: {res_c['pitch_mean']:+.6f} rad ({res_c['pitch_mean_deg']:+.4f}°)")
    print(f"  Target  |pitch mean| < 0.0100 rad: {'MET [PASS]' if target_met else 'FAILED'}")
    print(f"  Stretch |pitch mean| < 0.0050 rad: {'MET [PASS]' if stretch_met else 'NOT MET'}")


if __name__ == "__main__":
    main()
