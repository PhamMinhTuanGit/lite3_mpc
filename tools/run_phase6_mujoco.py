#!/usr/bin/env python3
"""Compare synchronized estimator signals with MuJoCo ground truth."""

import bisect
import csv
import math
import os
from pathlib import Path

import numpy as np

import run_phase3_mujoco as harness

WORKSPACE = Path(__file__).resolve().parents[1]
harness.CSV_PATH = WORKSPACE / "build/phase6_estimator.csv"
harness.SIM_LOG_PATH = WORKSPACE / "build/phase6_mujoco_sim.log"
harness.DEPLOY_LOG_PATH = WORKSPACE / "build/phase6_mujoco_deploy.log"
TRUTH_PATH = WORKSPACE / "build/phase6_mujoco_ground_truth.csv"
TIMING_PATH = WORKSPACE / "build/phase6_timing.csv"
FREQUENCIES = (0.824, 1.902)


def finite_float(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def load_sensor_events():
    events = []
    with TIMING_PATH.open(newline="") as timing_file:
        for row in csv.DictReader(timing_file):
            if row["event"] == "sensor_send":
                events.append((int(row["wall_ns"]), float(row["sim_time"])))
    return sorted(events)


def load_truth():
    rows = []
    with TRUTH_PATH.open(newline="") as truth_file:
        for row in csv.DictReader(truth_file):
            values = [finite_float(row, key) for key in
                      ("sim_time", "world_z", "vz", "pitch", "omega_y")]
            if all(value is not None for value in values):
                rows.append(values)
    return np.asarray(rows, dtype=np.float64)


def load_estimator(sensor_events):
    sensor_walls = [event[0] for event in sensor_events]
    by_sensor_time = {}
    first_done = None
    with harness.CSV_PATH.open(newline="") as estimator_file:
        for row in csv.DictReader(estimator_file):
            start = finite_float(row, "wbic_start_wall_ns")
            done = finite_float(row, "wbic_done_wall_ns")
            values = [finite_float(row, key) for key in ("body_z", "vz", "pitch", "omega_y")]
            if start is None or done is None or not all(value is not None for value in values):
                continue
            if first_done is None:
                first_done = done
            if (done - first_done) * 1e-9 < 5.0:
                continue
            index = bisect.bisect_right(sensor_walls, int(start)) - 1
            if index >= 0:
                by_sensor_time[sensor_events[index][1]] = values
    return np.asarray([[t, *values] for t, values in sorted(by_sensor_time.items())],
                      dtype=np.float64)


def spectral_metrics(reference, estimate, dt, frequency):
    segment_length = int(4.0 / dt)
    step = segment_length // 2
    window = np.hanning(segment_length)
    kernel = window * np.exp(-2j * np.pi * frequency * np.arange(segment_length) * dt)
    cross, ref_power, est_power = [], [], []
    for start in range(0, len(reference) - segment_length + 1, step):
        ref = reference[start:start + segment_length]
        est = estimate[start:start + segment_length]
        ref_coefficient = np.dot(ref - np.mean(ref), kernel)
        est_coefficient = np.dot(est - np.mean(est), kernel)
        cross.append(est_coefficient * np.conj(ref_coefficient))
        ref_power.append(abs(ref_coefficient) ** 2)
        est_power.append(abs(est_coefficient) ** 2)
    mean_cross = np.mean(cross)
    mean_ref_power = np.mean(ref_power)
    mean_est_power = np.mean(est_power)
    phase_radians = np.angle(mean_cross)
    coherence = abs(mean_cross) ** 2 / (mean_ref_power * mean_est_power)
    spectral_correlation = math.sqrt(coherence) * math.cos(phase_radians)
    return (math.sqrt(mean_est_power / mean_ref_power),
            math.degrees(phase_radians), coherence, spectral_correlation)


def main():
    os.environ["BASELOG_PATH"] = str(TRUTH_PATH)
    os.environ["BASELOG_INTERVAL"] = "1"
    os.environ["PHASE5_TIMING_LOG"] = str(TIMING_PATH)
    if os.environ.get("PHASE6_ANALYZE_ONLY", "0") != "1":
        harness.run_closed_loop()

    truth = load_truth()
    estimator = load_estimator(load_sensor_events())
    start_time = max(5.0, truth[0, 0], estimator[0, 0])
    end_time = min(truth[-1, 0], estimator[-1, 0])
    dt = 0.001
    time = np.arange(start_time, end_time, dt)
    truth_signals = {
        "z": np.interp(time, truth[:, 0], truth[:, 1]),
        "vz": np.interp(time, truth[:, 0], truth[:, 2]),
        "pitch": np.interp(time, truth[:, 0], truth[:, 3]),
        "omega_y": np.interp(time, truth[:, 0], truth[:, 4]),
    }
    estimate_signals = {
        "z": np.interp(time, estimator[:, 0], estimator[:, 1]),
        "vz": np.interp(time, estimator[:, 0], estimator[:, 2]),
        "pitch": np.interp(time, estimator[:, 0], estimator[:, 3]),
        "omega_y": np.interp(time, estimator[:, 0], estimator[:, 4]),
    }
    estimate_signals["dz_dt"] = np.gradient(estimate_signals["z"], dt)
    estimate_signals["dpitch_dt"] = np.gradient(estimate_signals["pitch"], dt)
    comparisons = {
        "z_est/truth": (truth_signals["z"], estimate_signals["z"]),
        "vz_est/truth": (truth_signals["vz"], estimate_signals["vz"]),
        "pitch_est/truth": (truth_signals["pitch"], estimate_signals["pitch"]),
        "omega_y_est/truth": (truth_signals["omega_y"], estimate_signals["omega_y"]),
        "vz_est/dz_est_dt": (estimate_signals["dz_dt"], estimate_signals["vz"]),
        "omega_y_est/dpitch_est_dt": (estimate_signals["dpitch_dt"], estimate_signals["omega_y"]),
    }
    print("configuration: synchronized estimator vs MuJoCo truth; no signal filtering")
    print(f"analysis_sim_time=[{start_time:.6f},{end_time:.6f}]s, samples={len(time)}")
    for frequency in FREQUENCIES:
        print(f"frequency={frequency:.3f} Hz")
        for name, pair in comparisons.items():
            gain, phase, coherence, correlation = spectral_metrics(*pair, dt, frequency)
            print(f"  {name}: gain={gain:.12g}, phase={phase:.12g} deg, "
                  f"coherence={coherence:.12g}, correlation={correlation:.12g}")
    print(f"estimator_csv={harness.CSV_PATH}")
    print(f"ground_truth_csv={TRUTH_PATH}")
    print(f"timing_csv={TIMING_PATH}")
    print(f"sim_log={harness.SIM_LOG_PATH}")
    print(f"deploy_log={harness.DEPLOY_LOG_PATH}")


if __name__ == "__main__":
    main()
