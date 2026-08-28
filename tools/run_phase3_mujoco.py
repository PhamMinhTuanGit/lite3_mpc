#!/usr/bin/env python3
"""Run and analyze the Phase-3 closed-loop MuJoCo standing diagnostic."""

import csv
import math
import os
from pathlib import Path
import statistics
import subprocess
import time


WORKSPACE = Path(__file__).resolve().parents[1]
PYTHON_BIN = WORKSPACE / ".venv/bin/python3"
SIM_SCRIPT = WORKSPACE / "interface/robot/simulation/mujoco_simulation_with_payload.py"
DEPLOY_BIN = WORKSPACE / "build/cmpc_deploy"
CSV_PATH = WORKSPACE / "build/phase3_mujoco_closed_loop.csv"
SIM_LOG_PATH = WORKSPACE / "build/phase3_mujoco_sim.log"
DEPLOY_LOG_PATH = WORKSPACE / "build/phase3_mujoco_deploy.log"
SIM_DURATION_SECONDS = 26.0
ANALYSIS_START_SECONDS = 5.0


def finite_float(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def correlation(x_values, y_values):
    x_mean = statistics.fmean(x_values)
    y_mean = statistics.fmean(y_values)
    x_centered = [value - x_mean for value in x_values]
    y_centered = [value - y_mean for value in y_values]
    denominator = math.sqrt(
        sum(value * value for value in x_centered)
        * sum(value * value for value in y_centered)
    )
    if denominator == 0.0:
        return math.nan
    return sum(x * y for x, y in zip(x_centered, y_centered)) / denominator


def run_closed_loop():
    sim_env = os.environ.copy()
    sim_env.update({
        "VIEWER": "0",
        "SIM_DURATION": str(SIM_DURATION_SECONDS),
        "GMO_SIM_SCENARIO": "payload",
        "GMO_ENABLE_PAYLOAD": "1",
    })

    deploy_env = os.environ.copy()
    deploy_env.pop("LITE3_GRF_FORCE_RATE_WEIGHT", None)
    deploy_env.update({
        "LITE3_AUTO_START": "1",
        "LITE3_STANDING_DIAGNOSTIC_CSV": str(CSV_PATH),
        "LITE3_STANDING_TEST_MODE": "TEST_C_WBIC_LOCK_BASE",
        "LITE3_USE_CENTROIDAL_WRENCH": "1",
        "LITE3_PAYLOAD_AWARE": "1",
        "LITE3_PAYLOAD_MASS": "2.0",
        "LITE3_PAYLOAD_COM_X": "0.10",
    })

    with SIM_LOG_PATH.open("w") as sim_log, DEPLOY_LOG_PATH.open("w") as deploy_log:
        sim_process = subprocess.Popen(
            [str(PYTHON_BIN), str(SIM_SCRIPT)],
            cwd=SIM_SCRIPT.parent,
            env=sim_env,
            stdout=sim_log,
            stderr=subprocess.STDOUT,
        )
        deploy_process = None
        try:
            time.sleep(1.0)
            deploy_process = subprocess.Popen(
                [str(DEPLOY_BIN)],
                cwd=WORKSPACE,
                env=deploy_env,
                stdout=deploy_log,
                stderr=subprocess.STDOUT,
            )
            sim_return_code = sim_process.wait(timeout=SIM_DURATION_SECONDS + 15.0)
            if sim_return_code != 0:
                raise RuntimeError(f"MuJoCo exited with status {sim_return_code}")
        finally:
            if sim_process.poll() is None:
                sim_process.terminate()
                sim_process.wait(timeout=5.0)
            if deploy_process is not None and deploy_process.poll() is None:
                deploy_process.terminate()
                try:
                    deploy_process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    deploy_process.kill()
                    deploy_process.wait(timeout=3.0)


def analyze():
    keys = (
        "qddot_cmd_z",
        "qddot_cmd_pitch",
        "Fz_des_total",
        "Fz_f_opt",
        "My_des",
        "My_f_opt",
        "pitch",
        "omega_y",
    )
    samples = []
    all_times = []
    status_counts = {}
    with CSV_PATH.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            sample_time = finite_float(row, "time")
            if sample_time is None:
                continue
            all_times.append(sample_time)
            status = row.get("wbic_status", "")
            status_counts[status] = status_counts.get(status, 0) + 1
            values = {key: finite_float(row, key) for key in keys}
            if sample_time >= ANALYSIS_START_SECONDS and all(
                value is not None for value in values.values()
            ):
                values["time"] = sample_time
                samples.append(values)

    if not samples:
        raise RuntimeError("No complete telemetry samples for t >= 5 s")
    if max(all_times) < 20.0:
        raise RuntimeError(f"Closed-loop log is shorter than 20 s: {max(all_times):.3f} s")

    columns = {key: [sample[key] for sample in samples] for key in keys}
    metrics = {
        "logged_duration_s": max(all_times),
        "analysis_samples": len(samples),
        "qddot_cmd_z_rms": rms(columns["qddot_cmd_z"]),
        "qddot_cmd_z_std": statistics.pstdev(columns["qddot_cmd_z"]),
        "qddot_cmd_pitch_rms": rms(columns["qddot_cmd_pitch"]),
        "qddot_cmd_pitch_std": statistics.pstdev(columns["qddot_cmd_pitch"]),
        "Fz_f_opt_std": statistics.pstdev(columns["Fz_f_opt"]),
        "My_f_opt_std": statistics.pstdev(columns["My_f_opt"]),
        "corr_qddot_z_Fz": correlation(columns["qddot_cmd_z"], columns["Fz_f_opt"]),
        "corr_qddot_pitch_My": correlation(
            columns["qddot_cmd_pitch"], columns["My_f_opt"]
        ),
        "pitch_peak_to_peak": max(columns["pitch"]) - min(columns["pitch"]),
        "omega_y_rms": rms(columns["omega_y"]),
        "Fz_des_mean": statistics.fmean(columns["Fz_des_total"]),
        "Fz_f_opt_mean": statistics.fmean(columns["Fz_f_opt"]),
        "My_des_mean": statistics.fmean(columns["My_des"]),
        "My_f_opt_mean": statistics.fmean(columns["My_f_opt"]),
    }
    return metrics, status_counts


def main():
    run_closed_loop()
    metrics, status_counts = analyze()
    print("configuration: MuJoCo closed-loop, payload=2kg@+0.10m, "
          "centroidal=ON, payload-aware=ON, lock-base=ON")
    print(f"logged_duration_s={metrics['logged_duration_s']:.6f}")
    print(f"analysis: t >= {ANALYSIS_START_SECONDS:.1f}s, samples={metrics['analysis_samples']}")
    print(f"qddot_cmd_z: rms={metrics['qddot_cmd_z_rms']:.12g}, "
          f"std={metrics['qddot_cmd_z_std']:.12g} m/s^2")
    print(f"qddot_cmd_pitch: rms={metrics['qddot_cmd_pitch_rms']:.12g}, "
          f"std={metrics['qddot_cmd_pitch_std']:.12g} rad/s^2")
    print(f"Fz_f_opt: mean={metrics['Fz_f_opt_mean']:.12g}, "
          f"std={metrics['Fz_f_opt_std']:.12g} N")
    print(f"My_f_opt: mean={metrics['My_f_opt_mean']:.12g}, "
          f"std={metrics['My_f_opt_std']:.12g} Nm")
    print(f"Fz_des_mean={metrics['Fz_des_mean']:.12g} N")
    print(f"My_des_mean={metrics['My_des_mean']:.12g} Nm")
    print(f"corr(qddot_cmd_z,Fz_f_opt)={metrics['corr_qddot_z_Fz']:.12g}")
    print(f"corr(qddot_cmd_pitch,My_f_opt)={metrics['corr_qddot_pitch_My']:.12g}")
    print(f"pitch_peak_to_peak={metrics['pitch_peak_to_peak']:.12g} rad")
    print(f"omega_y_rms={metrics['omega_y_rms']:.12g} rad/s")
    print(f"wbic_status_counts={status_counts}")
    print(f"telemetry_csv={CSV_PATH}")
    print(f"sim_log={SIM_LOG_PATH}")
    print(f"deploy_log={DEPLOY_LOG_PATH}")


if __name__ == "__main__":
    main()
