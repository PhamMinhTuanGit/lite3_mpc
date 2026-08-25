#!/usr/bin/env python3
"""Chạy tự động cmpc_deploy cho các kịch bản validation GMO/GRF.

MuJoCo được khởi động trước controller để tránh watchdog sensor 50 ms đẩy FSM
sang JointDamping trong lúc model còn đang load. Mọi mốc CLI sau đó tính theo
wall-clock từ khi cả hai process đã sẵn sàng.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CTRL_BIN = REPO / "build" / "cmpc_deploy"
SIM_DIR = REPO / "interface" / "robot" / "simulation"
SIM_SCRIPT = SIM_DIR / "mujoco_simulation_with_payload.py"
RUN_DIR = REPO / "data" / "payload_test_runs"
VENV_PY = REPO / ".venv" / "bin" / "python"
ANALYZER = REPO / "tools" / "analyze_gmo_evidence.py"


def send_key(proc, ch):
    """Bơm một ký tự vào KeyboardInterface của controller."""
    try:
        proc.stdin.write(ch.encode())
        proc.stdin.flush()
    except (BrokenPipeError, OSError, ValueError) as exc:
        raise RuntimeError(f"không thể gửi phím {ch!r}: {exc}") from exc


def wait_until(t0, offset, processes=()):
    """Đợi tới ``t0 + offset`` và báo lỗi ngay nếu process chết sớm."""
    target = t0 + offset
    while True:
        for name, proc in processes:
            rc = proc.poll()
            if rc is not None:
                raise RuntimeError(f"{name} đã dừng sớm (exit code {rc})")
        remaining = target - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.1))


def wait_for_sim(sim, base_log, timeout_s):
    """Đợi MuJoCo load model và ghi ít nhất một mẫu ground-truth."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rc = sim.poll()
        if rc is not None:
            raise RuntimeError(f"MuJoCo dừng khi khởi động (exit code {rc})")
        try:
            if base_log.exists() and len(base_log.read_text().splitlines()) >= 2:
                return
        except OSError:
            pass
        time.sleep(0.1)
    raise TimeoutError(f"MuJoCo chưa sẵn sàng sau {timeout_s:.1f}s")


def last_sim_time(base_log):
    """Lấy sim_time gần nhất để đồng bộ cửa sổ phân tích."""
    try:
        return float(base_log.read_text().splitlines()[-1].split(",", 1)[0])
    except (OSError, IndexError, ValueError):
        return None


def stop_process(proc, sig=signal.SIGINT, timeout_s=5.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.send_signal(sig)
        proc.wait(timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired):
        proc.kill()
        proc.wait(timeout=timeout_s)


def validate_args(parser, args, phase2_start):
    times = (args.t_stand, args.t_cmpc, args.t_vel1, args.vel1_hold,
             args.vel2_gap, args.vel2_hold, args.startup_timeout)
    if any(value < 0 for value in times):
        parser.error("các tham số thời gian phải >= 0")
    if args.vel1_press < 0 or args.vel2_press < 0:
        parser.error("vel1-press và vel2-press phải >= 0")
    if not (args.t_stand < args.t_cmpc < args.t_vel1):
        parser.error("cần t-stand < t-cmpc < t-vel1")
    phase2_end = phase2_start + args.vel2_press * args.vel2_gap + args.vel2_hold
    if args.t_sitdown < phase2_end:
        parser.error(f"t-sitdown phải >= cuối pha 2 ({phase2_end:.2f}s)")
    if args.total < args.t_sitdown:
        parser.error("total phải >= t-sitdown")


def parse_args():
    ap = argparse.ArgumentParser(description="Chạy validation GMO/GRF tự động với MuJoCo")
    ap.add_argument("--scenario", choices=("nominal", "payload", "early-contact"),
                    default="payload", help="mô hình/terrain dùng cho validation")
    ap.add_argument("--t-stand", type=float, default=3.0, help="mốc gửi z đứng dậy")
    ap.add_argument("--t-cmpc", type=float, default=8.0, help="mốc gửi x vào CMPC")
    ap.add_argument("--t-vel1", type=float, default=10.5, help="mốc bắt đầu pha 0.1 m/s")
    ap.add_argument("--vel1-press", type=int, default=2, help="số lần w cho pha 1")
    ap.add_argument("--vel1-hold", type=float, default=6.5, help="thời gian giữ pha 1")
    ap.add_argument("--vel2-press", type=int, default=8, help="số lần w thêm cho pha 2")
    ap.add_argument("--vel2-gap", type=float, default=0.25, help="khoảng cách w pha 2")
    ap.add_argument("--vel2-hold", type=float, default=8.0, help="thời gian giữ pha 2")
    ap.add_argument("--t-sitdown", type=float, default=None, help="mốc gửi z ngồi xuống")
    ap.add_argument("--total", type=float, default=None, help="tổng thời gian chạy")
    ap.add_argument("--startup-timeout", type=float, default=30.0,
                    help="thời gian tối đa đợi MuJoCo sẵn sàng")
    ap.add_argument("--viewer", action="store_true", help="bật MuJoCo viewer")
    ap.add_argument("--dry", action="store_true", help="chỉ in lịch")
    args = ap.parse_args()

    phase2_start = args.t_vel1 + args.vel1_press * 0.12 + args.vel1_hold
    if args.t_sitdown is None:
        args.t_sitdown = phase2_start + args.vel2_press * args.vel2_gap + args.vel2_hold
    if args.total is None:
        args.total = args.t_sitdown + 3.0
    validate_args(ap, args, phase2_start)
    return args, phase2_start


def print_schedule(args, phase2_start):
    print("=" * 72)
    print("LỊCH BƠM PHÍM (wall-clock, vx = forward_vel_scale * 0.5 m/s)")
    print("=" * 72)
    print(f"  T+{args.t_stand:5.2f}s : 'z' -> StandUp")
    print(f"  T+{args.t_cmpc:5.2f}s : 'x' -> CMPC")
    print(f"  T+{args.t_vel1:5.2f}s : 'w'x{args.vel1_press} -> vx=0.1 m/s; giữ {args.vel1_hold}s")
    print(f"  T+{phase2_start:5.2f}s : 'w'x{args.vel2_press} -> vx=0.5 m/s; giữ {args.vel2_hold}s")
    print(f"  T+{args.t_sitdown:5.2f}s : 'z' -> SitDown")
    print(f"  T+{args.total:5.2f}s : kết thúc")
    print("=" * 72)


def main():
    args, phase2_start = parse_args()
    print_schedule(args, phase2_start)
    if args.dry:
        return

    if not CTRL_BIN.is_file():
        sys.exit(f"Không tìm thấy {CTRL_BIN} — hãy build với BUILD_SIM=ON.")
    if not VENV_PY.is_file():
        sys.exit(f"Không tìm thấy venv python {VENV_PY}")
    if not SIM_SCRIPT.is_file():
        sys.exit(f"Không tìm thấy simulator {SIM_SCRIPT}")

    RUN_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = RUN_DIR / stamp
    suffix = 1
    while run_dir.exists():
        run_dir = RUN_DIR / f"{stamp}_{suffix}"
        suffix += 1
    run_dir.mkdir()

    ctrl_log = run_dir / "cmpc_deploy.log"
    sim_log = run_dir / "mujoco_sim.log"
    base_log = run_dir / "basestate.csv"
    timeline_log = run_dir / "timeline.log"
    metadata_path = run_dir / "metadata.json"
    ctrl_logf = ctrl_log.open("w")
    sim_logf = sim_log.open("w")
    timeline_logf = timeline_log.open("w", buffering=1)

    env_sim = dict(os.environ)
    env_sim.update(VIEWER="1" if args.viewer else "0",
                   BASELOG_PATH=str(base_log), BASELOG_INTERVAL="50",
                   GMO_SIM_SCENARIO=args.scenario,
                   GMO_ENABLE_PAYLOAD="1" if args.scenario == "payload" else "0")

    telemetry_before = set((REPO / "data").glob("cmpc_telemetry_*.csv"))

    print(f"\n[run] controller log : {ctrl_log}")
    print(f"[run] sim log        : {sim_log}")
    print(f"[run] base-state log : {base_log}")
    print(f"[run] timeline       : {timeline_log}")

    ctrl = sim = None
    events = []
    sim_time_origin = None
    completed = False
    try:
        print("[run] khởi động MuJoCo và đợi sensor sẵn sàng...")
        sim = subprocess.Popen([str(VENV_PY), str(SIM_SCRIPT)], cwd=str(SIM_DIR),
                               env=env_sim, stdout=sim_logf, stderr=subprocess.STDOUT)
        wait_for_sim(sim, base_log, args.startup_timeout)

        print("[run] MuJoCo đã sẵn sàng; khởi động controller...")
        ctrl = subprocess.Popen([str(CTRL_BIN)], cwd=str(CTRL_BIN.parent),
                                stdin=subprocess.PIPE, stdout=ctrl_logf,
                                stderr=subprocess.STDOUT)
        time.sleep(0.5)
        if ctrl.poll() is not None:
            raise RuntimeError(f"controller dừng khi khởi động (exit code {ctrl.returncode})")

        sim_time_origin = last_sim_time(base_log)
        t0 = time.monotonic()
        watched = (("controller", ctrl), ("MuJoCo", sim))

        def event(message, offset=None):
            elapsed = offset if offset is not None else time.monotonic() - t0
            line = f"[T+{elapsed:6.2f}s] {message}"
            print(line)
            timeline_logf.write(line + "\n")
            events.append({"wall_offset_s": round(elapsed, 6), "event": message})

        wait_until(t0, args.t_stand, watched)
        send_key(ctrl, "z")
        event("send 'z' (stand up)")
        wait_until(t0, args.t_cmpc, watched)
        send_key(ctrl, "x")
        event("send 'x' (CMPC)")

        cur = args.t_vel1
        event("PHA 1: ramp -> 0.1 m/s", cur)
        for _ in range(args.vel1_press):
            wait_until(t0, cur, watched)
            send_key(ctrl, "w")
            cur += 0.12
        wait_until(t0, cur + args.vel1_hold, watched)
        cur += args.vel1_hold
        event("hết pha 0.1 m/s")

        event("PHA 2: ramp -> 0.5 m/s", cur)
        for _ in range(args.vel2_press):
            wait_until(t0, cur, watched)
            send_key(ctrl, "w")
            cur += args.vel2_gap
        wait_until(t0, cur + args.vel2_hold, watched)
        event("hết pha 0.5 m/s")

        wait_until(t0, args.t_sitdown, watched)
        send_key(ctrl, "z")
        event("send 'z' (sit down)")
        wait_until(t0, args.total, watched)
        completed = True
    finally:
        print("\n[run] dừng các process...")
        if ctrl is not None and ctrl.poll() is None:
            try:
                ctrl.send_signal(signal.SIGINT)
                if ctrl.stdin:
                    ctrl.stdin.close()
                ctrl.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                stop_process(ctrl, signal.SIGKILL, 2)
        stop_process(sim)

        metadata = {
            "run_dir": str(run_dir),
            "completed": completed,
            "sim_time_origin_s": sim_time_origin,
            "schedule": vars(args),
            "phase2_start": phase2_start,
            "events": events,
            "exit_codes": {
                "controller": None if ctrl is None else ctrl.poll(),
                "mujoco": None if sim is None else sim.poll(),
            },
        }
        telemetry_after = set((REPO / "data").glob("cmpc_telemetry_*.csv"))
        new_telemetry = sorted(telemetry_after - telemetry_before,
                               key=lambda path: path.stat().st_mtime)
        telemetry_path = new_telemetry[-1] if new_telemetry else None
        metadata["telemetry_path"] = None if telemetry_path is None else str(telemetry_path)
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
        ctrl_logf.close()
        sim_logf.close()
        timeline_logf.close()

    print(f"\n[run] xong. Log tại: {run_dir}")
    if telemetry_path is not None and ANALYZER.is_file():
        report_path = run_dir / "gmo_evidence_report.json"
        print(f"[run] phân tích GMO/GRF: {telemetry_path}")
        subprocess.run([str(VENV_PY), str(ANALYZER), str(telemetry_path),
                        "--json", str(report_path)], check=False)
    if sim_time_origin is not None:
        win1 = (sim_time_origin + args.t_vel1 + 0.5,
                sim_time_origin + phase2_start - 0.3)
        win2 = (sim_time_origin + phase2_start + args.vel2_press * args.vel2_gap + 0.5,
                sim_time_origin + args.t_sitdown - 0.3)
        if win1[1] > win1[0] and win2[1] > win2[0]:
            print(f"Cửa sổ sim-time đề xuất: --win1 {win1[0]:.2f},{win1[1]:.2f} "
                  f"--win2 {win2[0]:.2f},{win2[1]:.2f}")
    print("Phân tích bằng: python3 tools/analyze_payload_test.py", run_dir)


if __name__ == "__main__":
    main()
