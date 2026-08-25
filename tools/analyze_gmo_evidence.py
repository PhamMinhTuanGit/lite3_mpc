#!/usr/bin/env python3
"""Analyze read-only Lite3 GMO/GRF evidence against MuJoCo ground truth.

The script supports both the original telemetry schema and the hardened
evidence schema. It never changes controller configuration; suggested force
biases must be validated on a separate run before being copied into GMOConfig.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


LEGS = ("fr", "fl", "hr", "hl")


def percentile(values, q):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, q)) if values.size else math.nan


def roc_auc(labels, scores):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    finite = np.isfinite(scores)
    labels, scores = labels[finite], scores[finite]
    positives = int(labels.sum())
    negatives = int((~labels).sum())
    if positives == 0 or negatives == 0:
        return math.nan
    ranks = pd.Series(scores).rank(method="average").to_numpy()
    rank_sum = float(ranks[labels].sum())
    return (rank_sum - positives * (positives + 1) / 2) / (positives * negatives)


def pr_auc(labels, scores):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    finite = np.isfinite(scores)
    labels, scores = labels[finite], scores[finite]
    positives = int(labels.sum())
    if positives == 0 or positives == labels.size:
        return math.nan
    order = np.argsort(-scores, kind="stable")
    sorted_labels = labels[order]
    tp = np.cumsum(sorted_labels)
    fp = np.cumsum(~sorted_labels)
    recall = tp / positives
    precision = tp / np.maximum(tp + fp, 1)
    recall = np.concatenate(([0.0], recall))
    precision = np.concatenate(([1.0], precision))
    return float(np.trapz(precision, recall))


def operating_point(labels, scores, minimum_recall=0.95):
    labels = np.asarray(labels, dtype=bool)
    scores = np.asarray(scores, dtype=float)
    finite = np.isfinite(scores)
    labels, scores = labels[finite], scores[finite]
    positives = int(labels.sum())
    negatives = int((~labels).sum())
    if positives == 0 or negatives == 0:
        return {"threshold_n": math.nan, "recall": math.nan, "fpr": math.nan}
    order = np.argsort(-scores, kind="stable")
    sorted_labels = labels[order]
    sorted_scores = scores[order]
    tp = np.cumsum(sorted_labels)
    fp = np.cumsum(~sorted_labels)
    recall = tp / positives
    fpr = fp / negatives
    candidates = np.flatnonzero(recall >= minimum_recall)
    if not candidates.size:
        index = len(scores) - 1
    else:
        candidate_fpr = fpr[candidates]
        index = int(candidates[np.argmin(candidate_fpr)])
    return {
        "threshold_n": float(sorted_scores[index]),
        "recall": float(recall[index]),
        "fpr": float(fpr[index]),
    }


def transition_delays_ms(timestamps_ms, labels, scores, threshold, max_delay_ms=100.0):
    labels = np.asarray(labels, dtype=bool)
    predicted = np.asarray(scores, dtype=float) >= threshold
    timestamps_ms = np.asarray(timestamps_ms, dtype=float)
    delays = []
    for direction in (True, False):
        transitions = np.flatnonzero((labels[1:] == direction) & (labels[:-1] != direction)) + 1
        for index in transitions:
            deadline = timestamps_ms[index] + max_delay_ms
            cursor = index
            while cursor < labels.size and timestamps_ms[cursor] <= deadline:
                if predicted[cursor] == direction:
                    delays.append(abs(timestamps_ms[cursor] - timestamps_ms[index]))
                    break
                cursor += 1
    return delays


def require_columns(frame, columns):
    missing = [name for name in columns if name not in frame.columns]
    if missing:
        raise ValueError("missing telemetry columns: " + ", ".join(missing))


def analyze(csv_path):
    frame = pd.read_csv(csv_path)
    require_columns(frame, ["timestamp_ms", "gmo_valid", "gt_valid"])
    for leg in LEGS:
        require_columns(frame, [f"gmo_force_{leg}_z", f"gt_contact_{leg}", f"gt_force_{leg}_z"])

    numeric_valid = frame["gmo_valid"].fillna(0).to_numpy() > 0.5
    if "grf_valid" in frame:
        numeric_valid &= frame["grf_valid"].fillna(0).to_numpy() > 0.5
    gt_valid = frame["gt_valid"].fillna(0).to_numpy() > 0.5
    ready_mask = np.ones(len(frame), dtype=bool)
    if "gmo_ready" in frame:
        ready_mask &= frame["gmo_ready"].fillna(0).to_numpy() > 0.5
    if "grf_ready" in frame:
        ready_mask &= frame["grf_ready"].fillna(0).to_numpy() > 0.5
    evaluation_mask = numeric_valid & ready_mask & gt_valid

    if "gmo_samples_since_reset" in frame:
        post_warmup = frame["gmo_samples_since_reset"].fillna(0).to_numpy() >= 100
        eligible = post_warmup & gt_valid
    else:
        eligible = gt_valid
    invalid_rate = float((~numeric_valid[eligible]).mean()) if eligible.any() else math.nan
    reset_count = 0
    if "gmo_reset_count" in frame and len(frame):
        reset_values = frame["gmo_reset_count"].fillna(0).to_numpy(dtype=np.int64)
        reset_count = int(reset_values.max())

    evidence_time = frame["t_gmo_ms"].fillna(0).to_numpy(dtype=float, copy=True)
    if "t_grf_ms" in frame:
        evidence_time += frame["t_grf_ms"].fillna(0).to_numpy(dtype=float)

    result = {
        "file": str(csv_path),
        "rows": int(len(frame)),
        "evaluated_rows": int(evaluation_mask.sum()),
        "invalid_rate": invalid_rate,
        "unexpected_reset_count": reset_count,
        "timing_ms": {
            "p50": percentile(evidence_time, 50),
            "p95": percentile(evidence_time, 95),
            "p99": percentile(evidence_time, 99),
            "max": percentile(evidence_time, 100),
        },
        "legs": {},
    }

    timestamps = frame.loc[evaluation_mask, "timestamp_ms"].to_numpy(dtype=float)
    for leg in LEGS:
        labels = frame.loc[evaluation_mask, f"gt_contact_{leg}"].to_numpy(dtype=float) > 0.5
        scores = frame.loc[evaluation_mask, f"gmo_force_{leg}_z"].to_numpy(dtype=float)
        true_force = frame.loc[evaluation_mask, f"gt_force_{leg}_z"].to_numpy(dtype=float)
        point = operating_point(labels, scores)
        delays = transition_delays_ms(
            timestamps, labels, scores, point["threshold_n"])

        raw_bias = []
        for axis in ("x", "y", "z"):
            raw_name = f"gmo_force_raw_{leg}_{axis}"
            fallback_name = f"gmo_force_{leg}_{axis}"
            column = raw_name if raw_name in frame else fallback_name
            if column in frame:
                swing_values = frame.loc[
                    evaluation_mask & (frame[f"gt_contact_{leg}"].to_numpy() <= 0.5), column
                ].to_numpy(dtype=float)
                raw_bias.append(percentile(swing_values, 50))
            else:
                raw_bias.append(math.nan)

        leg_result = {
            "roc_auc": roc_auc(labels, scores),
            "pr_auc": pr_auc(labels, scores),
            "stance_samples": int(labels.sum()),
            "swing_samples": int((~labels).sum()),
            "stance_fz_median_n": percentile(scores[labels], 50),
            "swing_fz_median_n": percentile(scores[~labels], 50),
            "swing_fz_p95_n": percentile(scores[~labels], 95),
            "stance_fz_mae_n": float(np.mean(np.abs(scores[labels] - true_force[labels])))
            if labels.any() else math.nan,
            "operating_point": point,
            "event_delay_p95_ms": percentile(delays, 95),
            "matched_events": len(delays),
            "suggested_raw_swing_bias_world_n": raw_bias,
        }
        result["legs"][leg] = leg_result

    all_contact = evaluation_mask.copy()
    for leg in LEGS:
        all_contact &= frame[f"gt_contact_{leg}"].to_numpy(dtype=float) > 0.5
    for command in ("cmd_vx", "cmd_vy", "cmd_yaw_rate"):
        if command in frame:
            all_contact &= np.abs(frame[command].to_numpy(dtype=float)) < 0.03
    if all_contact.any():
        estimated_total = sum(
            frame.loc[all_contact, f"gmo_force_{leg}_z"].to_numpy(dtype=float)
            for leg in LEGS
        )
        true_total = sum(
            frame.loc[all_contact, f"gt_force_{leg}_z"].to_numpy(dtype=float)
            for leg in LEGS
        )
        nonzero = np.abs(true_total) > 1e-6
        relative_error = np.abs(estimated_total[nonzero] - true_total[nonzero]) / np.abs(true_total[nonzero])
        result["standing_total_fz_relative_error_median"] = percentile(relative_error, 50)
    else:
        result["standing_total_fz_relative_error_median"] = math.nan

    gates = {
        "invalid_rate_lt_0_1_percent": invalid_rate < 0.001,
        "no_unexpected_resets": reset_count == 0,
        "timing_p99_lt_0_5_ms": result["timing_ms"]["p99"] < 0.5,
        "all_legs_roc_auc_ge_0_90": all(
            metrics["roc_auc"] >= 0.90 for metrics in result["legs"].values()
        ),
        "all_legs_swing_fz_p95_lt_15_n": all(
            metrics["swing_fz_p95_n"] < 15.0 for metrics in result["legs"].values()
        ),
        "standing_total_fz_error_le_15_percent":
            result["standing_total_fz_relative_error_median"] <= 0.15,
        "all_legs_event_delay_p95_lt_20_ms": all(
            metrics["matched_events"] > 0 and metrics["event_delay_p95_ms"] < 20.0
            for metrics in result["legs"].values()
        ),
    }
    result["gates"] = gates
    result["passed"] = all(gates.values())
    return result


def print_report(result):
    print(f"GMO/GRF evidence: {result['file']}")
    print(f"rows={result['rows']} evaluated={result['evaluated_rows']} "
          f"invalid={100.0 * result['invalid_rate']:.4f}% "
          f"resets={result['unexpected_reset_count']}")
    timing = result["timing_ms"]
    print("timing [ms]: "
          f"p50={timing['p50']:.4f} p95={timing['p95']:.4f} "
          f"p99={timing['p99']:.4f} max={timing['max']:.4f}")
    for leg, metrics in result["legs"].items():
        point = metrics["operating_point"]
        print(
            f"{leg.upper()}: ROC={metrics['roc_auc']:.4f} PR={metrics['pr_auc']:.4f} "
            f"swing-p95={metrics['swing_fz_p95_n']:.2f}N "
            f"stance-MAE={metrics['stance_fz_mae_n']:.2f}N "
            f"threshold@95%recall={point['threshold_n']:.2f}N "
            f"FPR={100.0 * point['fpr']:.2f}% delay-p95={metrics['event_delay_p95_ms']:.2f}ms"
        )
        bias = metrics["suggested_raw_swing_bias_world_n"]
        print(f"    suggested raw swing bias [x,y,z] = [{bias[0]:.3f}, {bias[1]:.3f}, {bias[2]:.3f}] N")
    print("gates:")
    for name, passed in result["gates"].items():
        print(f"  {'PASS' if passed else 'FAIL'} {name}")
    print("OVERALL:", "PASS" if result["passed"] else "FAIL")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="cmpc_telemetry_*.csv")
    parser.add_argument("--json", dest="json_path", type=Path, help="write machine-readable report")
    parser.add_argument("--strict", action="store_true", help="exit non-zero when any gate fails")
    args = parser.parse_args()

    result = analyze(args.csv)
    print_report(result)
    if args.json_path:
        args.json_path.write_text(json.dumps(result, indent=2, allow_nan=True) + "\n")
    if args.strict and not result["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
