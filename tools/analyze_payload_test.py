#!/usr/bin/env python3
"""
Phân tích kết quả test cmpc_deploy + mujoco payload.

Đầu vào:
  --run-dir <dir>         thư mục chạy (chứa basestate.csv)
                            HOẶC
  --base <basestate.csv>  file ground-truth base-state của sim
  --ctrl-log <file>       (tuỳ chọn) stdout/stderr của cmpc_deploy
  --csv <data csv>        (tuỳ chọn) log CSV của DataStreaming

Kết xuất:
  - Bảng thống kê base vx theo cửa sổ thời gian (pha 0.1 m/s, pha 0.5 m/s).
  - Phát hiện rơi / mất kiểm soát: z hạ thấp, RPY lớn, trạng thái thoát CMPC.
  - Bão hoà mô-men (so với limit hip 40 / thigh 40 / knee 65 N·m).
  - Deadline miss / cảnh báo safety từ ctrl-log.
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

# Giới hạn mô-men Lite3 (lite3_control_parameters.cpp)
TORQUE_LIMIT = [40.0, 40.0, 65.0]      # hip, thigh, knee (lặp cho 4 chân)
STAND_HEIGHT = 0.33                     # stand_height_ ước lượng chiều cao thân trên đất

# Các trạng thái (types/custom_types.h): CMPC=3, JointDamping=2
STATE_NAMES = {0: "Idle", 1: "StandUp", 2: "JointDamping", 3: "CMPC", 4: "SitDown"}


def load_base(path):
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        header = next(r, None)
        for ln in r:
            if not ln or len(ln) < 7:
                continue
            rows.append([float(x) for x in ln[:7]])
    return rows


def stats_window(rows, t0, t1, col):
    vals = [r[col] for r in rows if t0 <= r[0] <= t1]
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    std = math.sqrt(var)
    return dict(n=n, mean=mean, std=std, mn=min(vals), mx=max(vals))


def analyze_base(rows, win1, win2):
    print("=" * 76)
    print("GIA TỐC CƠ SỞ (ground truth từ MuJoCo, world frame)")
    print("=" * 76)
    if rows:
        print(f"  khoảng thời gian         : {rows[0][0]:6.2f}s .. {rows[-1][0]:6.2f}s ({len(rows)} mẫu)")
    for name, (t0, t1) in (("PHA 0.1 m/s", win1), ("PHA 0.5 m/s", win2)):
        vx = stats_window(rows, t0, t1, 4)
        vy = stats_window(rows, t0, t1, 5)
        zc = stats_window(rows, t0, t1, 3)
        xf = stats_window(rows, t0, t1, 1)  # world_x: dịch chuyển tiến
        if not vx:
            print(f"\n  [{name}] cửa sổ {t0:.1f}-{t1:.1f}s: KHÔNG CÓ DỮ LIỆU")
            continue
        dx = xf["mx"] - xf["mn"] if xf else float("nan")
        tel = (t1 - t0)
        vel_from_disp = dx / tel if tel > 0 else float("nan")
        print(f"\n  [{name}] cửa sổ {t0:.1f}-{t1:.1f}s  ({vx['n']} mẫu)")
        print(f"     vx trung bình         = {vx['mean']:+.3f} m/s  (cmd = {'0.1' if '0.1' in name else '0.5'})")
        print(f"     vx khoảng [min,max]   = [{vx['mn']:+.3f}, {vx['mx']:+.3f}] m/s  (std {vx['std']:.3f})")
        print(f"     vy trung bình         = {vy['mean']:+.3f} m/s")
        print(f"     z torso trung bình    = {zc['mean']:.3f} m  (đứng mong đợi ~0.33 +لات)")
        print(f"     dịch chuyển tiến (dx) = {dx:.3f} m trong {tel:.1f}s -> velocity ~{vel_from_disp:.3f} m/s")

        # Nhận diện rơi
        if zc and zc["mean"] < 0.18:
            print(f"     ⚠ z torso quá thấp ({zc['mean']:.3f} m) -> nghi ngờ RƠI / nằm xuống.")
        if abs(vx["mean"]) < 1e-4:
            print(f"     ⚠ vx ~0 -> robot KHÔNG di chuyển tiến (khớp/gait có vấn đề).")


def analyze_ctrl_csv(path, win1, win2):
    print("\n" + "=" * 76)
    print("LOG PYTHON-CSV CỦA CONTROLLER (DataStreaming)")
    print("=" * 76)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        emap = reader.fieldnames
        state_idx = emap.index("state")
        rpy_idx = [emap.index(f"rpy{i}") for i in range(3)]
        tau_idx = [emap.index(f"tau{i}") for i in range(12)]
        dq_idx = [emap.index(f"dq{i}") for i in range(12)]

        # phát hiện bão hoà mô-men + trạng thái + vi phạm RPY, theo cửa sổ
        window_stats = {}
        transitions = []
        # Đọc toàn bộ để đếm chuyển trạng thái (file có thể lớn; hạn chế nếu quá)
        lines = list(reader)
        if not lines:
            print("  (CSV rỗng)")
            return
        # Trạng thái: liệt kê các chuyển đổi kèm thời gian
        prev = lines[0]["state"]
        # ước thời gian theo t_r (wall ms/1000) hoặc t_i (sim s). Ưu tiên t_i.
        def t_of(row):
            try:
                return float(row["t_i"])
            except Exception:
                return float(row.get("t_r", 0)) / 1000.0

        segs = []
        start_t = t_of(lines[0]); cur = prev
        for row in lines:
            s = row["state"]
            if s != cur:
                segs.append((cur, start_t, t_of(row)))
                cur = s; start_t = t_of(row)
        segs.append((cur, start_t, t_of(lines[-1])))
        print("  Các đoạn trạng thái:")
        for s, a, b in segs:
            print(f"     {STATE_NAMES.get(int(float(s)), s):14s} : {a:7.3f}s .. {b:7.3f}s")

        fell_to_damping = any(int(float(s)) == 2 for s, _, _ in segs)

        # Bão hoà mô-men & RPY trong 2 cửa sổ (theo t_i)
        def win_agg(t0, t1):
            sat = 0; tot = 0; rpy_max = 0.0; dq_max = 0.0; rows_in = 0
            for row in lines:
                tt = t_of(row)
                if not (t0 <= tt <= t1):
                    continue
                rows_in += 1
                rpyvals = [abs(float(row[emap[i]])) for i in rpy_idx]
                rpy_max = max(rpy_max, *rpyvals)
                dqvals = [abs(float(row[emap[i]])) for i in dq_idx]
                dq_max = max(dq_max, *dqvals)
                for j in range(12):
                    t = abs(float(row[tau_idx[j]]))
                    lim = TORQUE_LIMIT[j % 3]
                    tot += 1
                    if t > 0.95 * lim:
                        sat += 1
            return sat, tot, rows_in, rpy_max, dq_max

        for name, (t0, t1) in (("PHA 0.1 m/s", win1), ("PHA 0.5 m/s", win2)):
            sat, tot, n, rpy_max, dq_max = win_agg(t0, t1)
            if n == 0:
                print(f"\n  [{name}] không có mẫu CSV trong cửa sổ.")
                continue
            pct = 100.0 * sat / max(1, tot)
            rpy_deg = math.degrees(rpy_max)
            print(f"\n  [{name}] cửa sổ {t0:.1f}-{t1:.1f}s ({n} mẫu)")
            print(f"     RPY max (roll|pitch|yaw) = {rpy_deg:6.2f}°  (giới hạn an toàn roll 25°, pitch 30°)")
            print(f"     joint vel (dq) max        = {dq_max:6.2f} rad/s  (limit 30/30/20)")
            print(f"     mô-men bão hoà            = {sat}/{tot} mẫu ({pct:5.1f}% vượt 95% limit)")
            if rpy_deg > 30:
                print("     ⚠ RPY lớn -> nghi ngờ mất thăng bằng.")
            if pct > 5:
                print("     ⚠ mô-men thường xuyên sát giới hạn -> có thể thiếu công suất (payload nặng).")

        if fell_to_damping:
            print("\n  ⚠ Đã có đoạn JointDamping -> controller TỰ thoát CMPC (rơi/mất kiểm soát/deadline).")


def analyze_ctrl_log(path):
    print("\n" + "=" * 76)
    print("LOG STDOUT/STDERR CỦA cmpc_deploy")
    print("=" * 76)
    txt = Path(path).read_text(errors="replace") if path and Path(path).exists() else ""
    patterns = {
        "chuyển trạng thái (->)":    r"\S+\s+------------>\s+\S+",
        "Posture limit exceeded":    r"Posture limit exceeded.*",
        "missed deadline":           r"missed deadline.*",
        "WATCHDOG (mất gói)":        r"\[CRITICAL WATCHDOG\].*|forced transition|Forced transition.*",
        "[CMPC Safety]":             r"\[CMPC Safety\].*",
    }
    for label, pat in patterns.items():
        hits = re.findall(pat, txt)
        if hits:
            print(f"  [{label}] x{len(hits)}:")
            for h in hits[:6]:
                print("     •", h.strip()[:160])
        else:
            print(f"  [{label}] (không có)")
    # Lỗi runtime
    errs = re.findall(r"(Segmentation|Floating|what\(\):.*|Aborted|terminate called.*|Error .*socket.*)", txt)
    if errs:
        print("  ⚠ Lỗi runtime:")
        for e in errs[:6]:
            print("     •", e[:160])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?", help="thư mục chạy payload test (chứa basestate.csv)")
    ap.add_argument("--base", help="đường dẫn basestate.csv")
    ap.add_argument("--ctrl-log", help="log stdout của cmpc_deploy")
    ap.add_argument("--csv", help="file CSV của DataStreaming")
    # Cửa sổ phân tích (sim_time, giây). Phải khớp lịch bơm phím của orchestrator.
    ap.add_argument("--win1", default="9.8,16.0", help="cửa sổ pha 0.1 m/s (t0,t1 sim-sec)")
    ap.add_argument("--win2", default="18.5,26.0", help="cửa sổ pha 0.5 m/s (t0,t1 sim-sec)")
    args = ap.parse_args()

    base = args.base
    ctrl_log = args.ctrl_log
    csvp = args.csv
    if args.run_dir:
        rd = Path(args.run_dir)
        if not base: base = rd / "basestate.csv"
        if not ctrl_log: ctrl_log = rd / "cmpc_deploy.log"

    win1 = tuple(float(x) for x in args.win1.split(","))
    win2 = tuple(float(x) for x in args.win2.split(","))

    if base and Path(base).exists():
        analyze_base(load_base(str(base)), win1, win2)
    else:
        print(f"(không tìm thấy base log: {base})")

    if csvp and Path(csvp).exists():
        analyze_ctrl_csv(str(csvp), win1, win2)
    elif args.run_dir:
        # tìm CSV mới nhất trong data/
        cands = sorted(Path("data").glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        if cands:
            print(f"\n(sử dụng CSV controller mới nhất: {cands[0]})")
            analyze_ctrl_csv(str(cands[0]), win1, win2)

    if ctrl_log and Path(ctrl_log).exists():
        analyze_ctrl_log(str(ctrl_log))


if __name__ == "__main__":
    main()
