#!/usr/bin/env python3
"""Run closed-loop MuJoCo and attribute KinWBC qddot_cmd oscillation."""

import csv
import math
import os
from pathlib import Path
import statistics

import run_phase3_mujoco as harness


WORKSPACE = Path(__file__).resolve().parents[1]
harness.CSV_PATH = WORKSPACE / "build/phase4_kinwbc_components.csv"
harness.SIM_LOG_PATH = WORKSPACE / "build/phase4_mujoco_sim.log"
harness.DEPLOY_LOG_PATH = WORKSPACE / "build/phase4_mujoco_deploy.log"
GROUND_TRUTH_PATH = WORKSPACE / "build/phase4_mujoco_ground_truth.csv"


def finite_float(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def rms(values):
    return math.sqrt(statistics.fmean(value * value for value in values))


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


def component_metrics(columns, component, command):
    values = columns[component]
    return {
        "rms": rms(values),
        "std": statistics.pstdev(values),
        "correlation": correlation(values, columns[command]),
    }


def analyze():
    keys = (
        "z_error",
        "vz_error",
        "z_acc_P",
        "z_acc_D",
        "z_acc_ff",
        "x_ddot_pos_z",
        "orientation_error_y",
        "omega_y_error",
        "pitch_acc_P",
        "pitch_acc_D",
        "pitch_acc_ff",
        "x_ddot_ori_y",
        "qddot_cmd_z",
        "qddot_cmd_pitch",
    )
    samples = []
    all_times = []
    statuses = {}
    with harness.CSV_PATH.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            sample_time = finite_float(row, "time")
            if sample_time is None:
                continue
            all_times.append(sample_time)
            status = row.get("wbic_status", "")
            statuses[status] = statuses.get(status, 0) + 1
            values = {key: finite_float(row, key) for key in keys}
            if sample_time >= harness.ANALYSIS_START_SECONDS and all(
                value is not None for value in values.values()
            ):
                samples.append(values)

    if not samples:
        raise RuntimeError("No complete Phase-4 samples for t >= 5 s")
    columns = {key: [sample[key] for sample in samples] for key in keys}

    results = {
        "logged_duration_s": max(all_times),
        "samples": len(samples),
        "statuses": statuses,
        "z_error_rms": rms(columns["z_error"]),
        "z_error_std": statistics.pstdev(columns["z_error"]),
        "vz_error_rms": rms(columns["vz_error"]),
        "vz_error_std": statistics.pstdev(columns["vz_error"]),
        "pitch_error_rms": rms(columns["orientation_error_y"]),
        "pitch_error_std": statistics.pstdev(columns["orientation_error_y"]),
        "omega_y_error_rms": rms(columns["omega_y_error"]),
        "omega_y_error_std": statistics.pstdev(columns["omega_y_error"]),
        "corr_z_P_D": correlation(columns["z_acc_P"], columns["z_acc_D"]),
        "corr_pitch_P_D": correlation(columns["pitch_acc_P"], columns["pitch_acc_D"]),
    }
    for component in ("z_acc_P", "z_acc_D", "z_acc_ff", "x_ddot_pos_z"):
        results[component] = component_metrics(columns, component, "qddot_cmd_z")
    for component in ("pitch_acc_P", "pitch_acc_D", "pitch_acc_ff", "x_ddot_ori_y"):
        results[component] = component_metrics(columns, component, "qddot_cmd_pitch")

    z_projection_error = [
        task - command
        for task, command in zip(columns["x_ddot_pos_z"], columns["qddot_cmd_z"])
    ]
    pitch_projection_error = [
        task - command
        for task, command in zip(columns["x_ddot_ori_y"], columns["qddot_cmd_pitch"])
    ]
    results["z_projection"] = {
        "task_rms": rms(columns["x_ddot_pos_z"]),
        "command_rms": rms(columns["qddot_cmd_z"]),
        "difference_rms": rms(z_projection_error),
        "difference_std": statistics.pstdev(z_projection_error),
        "correlation": correlation(columns["x_ddot_pos_z"], columns["qddot_cmd_z"]),
    }
    results["pitch_projection"] = {
        "task_rms": rms(columns["x_ddot_ori_y"]),
        "command_rms": rms(columns["qddot_cmd_pitch"]),
        "difference_rms": rms(pitch_projection_error),
        "difference_std": statistics.pstdev(pitch_projection_error),
        "correlation": correlation(columns["x_ddot_ori_y"], columns["qddot_cmd_pitch"]),
    }

    ground_truth_z = []
    ground_truth_vz = []
    with GROUND_TRUTH_PATH.open(newline="") as ground_truth_file:
        for row in csv.DictReader(ground_truth_file):
            sample_time = finite_float(row, "sim_time")
            z = finite_float(row, "world_z")
            vz = finite_float(row, "vz")
            if sample_time is not None and sample_time >= 5.0 and z is not None and vz is not None:
                ground_truth_z.append(z)
                ground_truth_vz.append(vz)
    results["ground_truth_z_std"] = statistics.pstdev(ground_truth_z)
    results["ground_truth_vz_rms"] = rms(ground_truth_vz)
    results["ground_truth_vz_std"] = statistics.pstdev(ground_truth_vz)
    return results


def print_component(name, metrics, unit):
    print(
        f"{name}: rms={metrics['rms']:.12g}, std={metrics['std']:.12g} {unit}, "
        f"corr_with_qddot={metrics['correlation']:.12g}"
    )


def main():
    os.environ["BASELOG_PATH"] = str(GROUND_TRUTH_PATH)
    os.environ["BASELOG_INTERVAL"] = "1"
    harness.run_closed_loop()
    results = analyze()
    print("configuration: MuJoCo closed-loop, payload=2kg@+0.10m, "
          "centroidal=ON, payload-aware=ON, lock-base=ON")
    print(f"logged_duration_s={results['logged_duration_s']:.6f}")
    print(f"analysis: t >= 5.0s, samples={results['samples']}")
    print(
        f"z_error: rms={results['z_error_rms']:.12g}, "
        f"std={results['z_error_std']:.12g} m"
    )
    print(
        f"vz_error: rms={results['vz_error_rms']:.12g}, "
        f"std={results['vz_error_std']:.12g} m/s"
    )
    for component in ("z_acc_P", "z_acc_D", "z_acc_ff", "x_ddot_pos_z"):
        print_component(component, results[component], "m/s^2")
    print(f"corr(z_acc_P,z_acc_D)={results['corr_z_P_D']:.12g}")
    print(
        f"MuJoCo ground-truth: z_std={results['ground_truth_z_std']:.12g} m, "
        f"vz_rms={results['ground_truth_vz_rms']:.12g}, "
        f"vz_std={results['ground_truth_vz_std']:.12g} m/s"
    )
    print(
        f"pitch_error: rms={results['pitch_error_rms']:.12g}, "
        f"std={results['pitch_error_std']:.12g} rad"
    )
    print(
        f"omega_y_error: rms={results['omega_y_error_rms']:.12g}, "
        f"std={results['omega_y_error_std']:.12g} rad/s"
    )
    for component in ("pitch_acc_P", "pitch_acc_D", "pitch_acc_ff", "x_ddot_ori_y"):
        print_component(component, results[component], "rad/s^2")
    print(f"corr(pitch_acc_P,pitch_acc_D)={results['corr_pitch_P_D']:.12g}")
    for name in ("z_projection", "pitch_projection"):
        metrics = results[name]
        print(
            f"{name}: task_rms={metrics['task_rms']:.12g}, "
            f"qddot_cmd_rms={metrics['command_rms']:.12g}, "
            f"difference_rms={metrics['difference_rms']:.12g}, "
            f"difference_std={metrics['difference_std']:.12g}, "
            f"corr={metrics['correlation']:.12g}"
        )
    print(f"wbic_status_counts={results['statuses']}")
    print(f"telemetry_csv={harness.CSV_PATH}")
    print(f"sim_log={harness.SIM_LOG_PATH}")
    print(f"deploy_log={harness.DEPLOY_LOG_PATH}")
    print(f"ground_truth_csv={GROUND_TRUTH_PATH}")


if __name__ == "__main__":
    main()
