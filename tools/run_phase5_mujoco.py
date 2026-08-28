#!/usr/bin/env python3
"""Phase-5 closed-loop frequency, phase, D-work, and delay analysis."""

import bisect
import csv
import math
import os
from pathlib import Path
import statistics

import numpy as np

import run_phase3_mujoco as harness


WORKSPACE = Path(__file__).resolve().parents[1]
harness.CSV_PATH = WORKSPACE / "build/phase5_damping_phase.csv"
harness.SIM_LOG_PATH = WORKSPACE / "build/phase5_mujoco_sim.log"
harness.DEPLOY_LOG_PATH = WORKSPACE / "build/phase5_mujoco_deploy.log"
TIMING_PATH = WORKSPACE / "build/phase5_pipeline_timing.csv"


def finite_float(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def load_controller_samples():
    keys = (
        "body_z", "vz", "pitch", "omega_y", "z_error", "z_acc_P", "z_acc_D",
        "orientation_error_y", "pitch_acc_P", "pitch_acc_D", "qddot_cmd_z",
        "qddot_cmd_pitch", "Fz_f_opt", "My_f_opt", "wbic_start_wall_ns",
        "wbic_done_wall_ns",
    )
    samples = []
    with harness.CSV_PATH.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            values = {key: finite_float(row, key) for key in keys}
            if all(value is not None for value in values.values()):
                samples.append(values)
    samples.sort(key=lambda sample: sample["wbic_done_wall_ns"])
    origin = samples[0]["wbic_done_wall_ns"]
    samples = [sample for sample in samples
               if (sample["wbic_done_wall_ns"] - origin) * 1e-9 >= 5.0]
    return samples


def uniform_signals(samples):
    wall_time = np.array([sample["wbic_done_wall_ns"] for sample in samples]) * 1e-9
    wall_time -= wall_time[0]
    dt = float(np.median(np.diff(wall_time)))
    uniform_time = np.arange(0.0, wall_time[-1], dt)
    keys = tuple(key for key in samples[0]
                 if key not in ("wbic_start_wall_ns", "wbic_done_wall_ns"))
    signals = {
        key: np.interp(uniform_time, wall_time,
                       np.array([sample[key] for sample in samples], dtype=np.float64))
        for key in keys
    }
    return uniform_time, dt, signals


def dominant_frequency(signal, dt):
    centered = signal - np.mean(signal)
    frequencies = np.fft.rfftfreq(len(centered), dt)
    spectrum = np.abs(np.fft.rfft(centered))
    valid = (frequencies >= 0.1) & (frequencies <= 30.0)
    index = np.flatnonzero(valid)[np.argmax(spectrum[valid])]
    return float(frequencies[index]), int(index)


def phase_at_frequency(source, response, dt, frequency):
    source_fft = np.fft.rfft(source - np.mean(source))
    response_fft = np.fft.rfft(response - np.mean(response))
    frequencies = np.fft.rfftfreq(len(source), dt)
    index = int(np.argmin(np.abs(frequencies - frequency)))
    phase = math.degrees(np.angle(response_fft[index] * np.conj(source_fft[index])))
    return phase


def cross_correlation_lag(source, response, dt, max_lag_seconds=0.5):
    max_lag = min(int(max_lag_seconds / dt), len(source) // 4)
    best_lag = 0
    best_corr = 0.0
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            x, y = source[-lag:], response[:lag]
        elif lag > 0:
            x, y = source[:-lag], response[lag:]
        else:
            x, y = source, response
        x = x - np.mean(x)
        y = y - np.mean(y)
        denominator = math.sqrt(float(np.dot(x, x) * np.dot(y, y)))
        corr = float(np.dot(x, y) / denominator) if denominator > 0.0 else 0.0
        if abs(corr) > abs(best_corr):
            best_corr = corr
            best_lag = lag
    return best_lag * dt, best_corr


def pair_metrics(signals, dt, source, response, frequency):
    lag, corr = cross_correlation_lag(signals[source], signals[response], dt)
    return {
        "phase_deg": phase_at_frequency(signals[source], signals[response], dt, frequency),
        "lag_s": lag,
        "correlation": corr,
    }


def damping_work(acceleration, velocity):
    work = acceleration * velocity
    tolerance = 1e-12
    active = np.abs(work) > tolerance
    return {
        "mean": float(np.mean(work)),
        "opposing_fraction": float(np.mean(work[active] < 0.0)),
        "exciting_fraction": float(np.mean(work[active] > 0.0)),
    }


def percentile_ms(values, percentile):
    return float(np.percentile(np.array(values) * 1e-6, percentile))


def delay_metrics(samples):
    sensor_walls = []
    receives = []
    applies = {}
    with TIMING_PATH.open(newline="") as timing_file:
        for row in csv.DictReader(timing_file):
            wall_ns = int(row["wall_ns"])
            sequence = int(row["sequence"])
            if row["event"] == "sensor_send":
                sensor_walls.append(wall_ns)
            elif row["event"] == "command_receive":
                receives.append((wall_ns, sequence))
            elif row["event"] == "command_apply":
                applies[sequence] = wall_ns
    sensor_walls.sort()
    receives.sort()
    receive_walls = [item[0] for item in receives]
    sensor_to_wbic = []
    wbic_to_receive = []
    receive_to_apply = []
    sensor_to_apply = []
    for sample in samples:
        start = int(sample["wbic_start_wall_ns"])
        done = int(sample["wbic_done_wall_ns"])
        sensor_index = bisect.bisect_right(sensor_walls, start) - 1
        receive_index = bisect.bisect_left(receive_walls, done)
        if sensor_index < 0 or receive_index >= len(receives):
            continue
        receive_wall, sequence = receives[receive_index]
        apply_wall = applies.get(sequence)
        if apply_wall is None:
            continue
        sensor_wall = sensor_walls[sensor_index]
        sensor_to_wbic.append(done - sensor_wall)
        wbic_to_receive.append(receive_wall - done)
        receive_to_apply.append(apply_wall - receive_wall)
        sensor_to_apply.append(apply_wall - sensor_wall)
    result = {}
    for name, values in (
        ("sensor_to_wbic", sensor_to_wbic),
        ("wbic_to_receive", wbic_to_receive),
        ("receive_to_apply", receive_to_apply),
        ("sensor_to_apply", sensor_to_apply),
    ):
        result[name] = {
            "median_ms": percentile_ms(values, 50),
            "p95_ms": percentile_ms(values, 95),
            "mean_ms": statistics.fmean(values) * 1e-6,
        }
    return result


def main():
    os.environ["PHASE5_TIMING_LOG"] = str(TIMING_PATH)
    if os.environ.get("PHASE5_ANALYZE_ONLY", "0") != "1":
        harness.run_closed_loop()
    samples = load_controller_samples()
    _, dt, signals = uniform_signals(samples)

    frequency_signals = (
        "body_z", "vz", "pitch", "omega_y", "qddot_cmd_z",
        "qddot_cmd_pitch", "Fz_f_opt", "My_f_opt",
    )
    frequencies = {name: dominant_frequency(signals[name], dt)[0]
                   for name in frequency_signals}
    z_frequency = frequencies["qddot_cmd_z"]
    pitch_frequency = frequencies["qddot_cmd_pitch"]

    pairs = {
        "z_error_to_P": pair_metrics(signals, dt, "z_error", "z_acc_P", z_frequency),
        "vz_to_D": pair_metrics(signals, dt, "vz", "z_acc_D", z_frequency),
        "pitch_error_to_P": pair_metrics(
            signals, dt, "orientation_error_y", "pitch_acc_P", pitch_frequency),
        "omega_y_to_D": pair_metrics(
            signals, dt, "omega_y", "pitch_acc_D", pitch_frequency),
        "qddot_z_to_z": pair_metrics(
            signals, dt, "qddot_cmd_z", "body_z", z_frequency),
        "qddot_pitch_to_pitch": pair_metrics(
            signals, dt, "qddot_cmd_pitch", "pitch", pitch_frequency),
    }
    z_work = damping_work(signals["z_acc_D"], signals["vz"])
    pitch_work = damping_work(signals["pitch_acc_D"], signals["omega_y"])
    delays = delay_metrics(samples)

    print("configuration: MuJoCo closed-loop, payload=2kg@+0.10m, "
          "centroidal=ON, payload-aware=ON, lock-base=ON")
    print(f"analysis_samples={len(samples)}, resampled_dt={dt:.12g}s")
    for name in frequency_signals:
        print(f"dominant_frequency[{name}]={frequencies[name]:.12g} Hz")
    for name, metrics in pairs.items():
        print(f"phase[{name}]={metrics['phase_deg']:.12g} deg, "
              f"xcorr_lag={metrics['lag_s'] * 1e3:.12g} ms, "
              f"xcorr={metrics['correlation']:.12g}")
    print(f"z_D_work: mean={z_work['mean']:.12g}, "
          f"opposing={100*z_work['opposing_fraction']:.8f}%, "
          f"exciting={100*z_work['exciting_fraction']:.8f}%")
    print(f"pitch_D_work: mean={pitch_work['mean']:.12g}, "
          f"opposing={100*pitch_work['opposing_fraction']:.8f}%, "
          f"exciting={100*pitch_work['exciting_fraction']:.8f}%")
    for name, metrics in delays.items():
        print(f"delay[{name}]: median={metrics['median_ms']:.12g} ms, "
              f"mean={metrics['mean_ms']:.12g} ms, p95={metrics['p95_ms']:.12g} ms")
    print(f"telemetry_csv={harness.CSV_PATH}")
    print(f"timing_csv={TIMING_PATH}")
    print(f"sim_log={harness.SIM_LOG_PATH}")
    print(f"deploy_log={harness.DEPLOY_LOG_PATH}")


if __name__ == "__main__":
    main()
