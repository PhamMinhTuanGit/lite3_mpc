#!/usr/bin/env python3
"""
Công cụ trực quan hóa Telemetry & Debug CMPC + Linear Kalman Filter (Lite3).

Sử dụng:
    python3 tools/plot_cmpc_telemetry.py                    # Tự động tìm file CSV mới nhất trong data/
    python3 tools/plot_cmpc_telemetry.py --file data/cmpc_telemetry_20260821_120000.csv
    python3 tools/plot_cmpc_telemetry.py --save plot_result.png
"""

import argparse
import glob
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def find_latest_csv():
    files = glob.glob("../data/cmpc_telemetry_*.csv") + glob.glob("data/cmpc_telemetry_*.csv")
    if not files:
        return None
    return max(files, key=os.path.getctime)

def plot_telemetry(csv_path, save_path=None):
    if not os.path.exists(csv_path):
        print(f"Lỗi: Không tìm thấy file {csv_path}")
        return

    print(f"Đang đọc dữ liệu telemetry từ: {csv_path}")
    df = pd.read_csv(csv_path)
    
    if df.empty:
        print("File CSV rỗng!")
        return

    t = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0  # seconds

    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=True)
    fig.suptitle(f"CMPC & Linear KF Telemetry Analysis - {os.path.basename(csv_path)}", fontsize=14, fontweight='bold')

    # 1. Forward Velocity Tracking: cmd vs des vs KF
    ax = axes[0, 0]
    ax.plot(t, df['cmd_vx'], 'k--', label='cmd_vx (Gamepad)', alpha=0.7)
    ax.plot(t, df['des_vx'], 'b-', label='des_vx (S-Curve Filtered)', linewidth=1.5)
    ax.plot(t, df['kf_vel_body_x'], 'r-', label='kf_vel_body_x (Linear KF)', linewidth=1.5)
    ax.set_ylabel('Velocity Vx (m/s)')
    ax.set_title('1. Forward Velocity Tracking (Vx)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 2. Lateral & Yaw Rate Tracking
    ax = axes[0, 1]
    ax.plot(t, df['des_vy'], 'b--', label='des_vy', alpha=0.7)
    ax.plot(t, df['kf_vel_body_y'], 'b-', label='kf_vel_body_y', linewidth=1.2)
    ax.plot(t, df['des_yaw_rate'], 'g--', label='des_yaw_rate (rad/s)', alpha=0.7)
    ax.plot(t, df['kf_omega_body_z'], 'g-', label='kf_omega_z (rad/s)', linewidth=1.2)
    ax.set_ylabel('Lateral & Yaw Rate')
    ax.set_title('2. Lateral (Vy) & Yaw Rate Tracking')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 3. Orientation (Euler Angles: Roll, Pitch, Yaw)
    ax = axes[1, 0]
    ax.plot(t, np.degrees(df['kf_roll']), 'r-', label='Roll (deg)', linewidth=1.2)
    ax.plot(t, np.degrees(df['kf_pitch']), 'g-', label='Pitch (deg)', linewidth=1.2)
    ax.plot(t, np.degrees(df['kf_yaw']), 'b-', label='Yaw (deg)', linewidth=1.0, alpha=0.5)
    ax.axhline(0, color='gray', linestyle=':')
    ax.set_ylabel('Angle (degrees)')
    ax.set_title('3. Robot Posture (Roll / Pitch / Yaw)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 4. Torso Height Z
    ax = axes[1, 1]
    ax.plot(t, df['kf_pos_z'], 'm-', label='Torso Height kf_pos_z (m)', linewidth=1.5)
    ax.axhline(0.29, color='k', linestyle='--', label='Nominal Height (0.29m)')
    ax.set_ylabel('Height Z (m)')
    ax.set_title('4. Body Height Stability (Z)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 5. Adaptive Mass Estimation
    ax = axes[2, 0]
    ax.plot(t, df['est_mass_raw'], 'c.', label='Mass Raw (Torques)', alpha=0.3, markersize=3)
    ax.plot(t, df['est_mass_filtered'], 'b-', label='Mass Filtered (MPC)', linewidth=2.0)
    ax.axhline(11.94, color='r', linestyle='--', label='Nominal Mass (11.94 kg)')
    ax.set_ylabel('Mass (kg)')
    ax.set_title('5. Adaptive Mass Estimator Convergence')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 6. Adaptive CoM Offset
    ax = axes[2, 1]
    ax.plot(t, df['est_com_x'] * 1000.0, 'r-', label='CoM Shift dX (mm)', linewidth=1.5)
    ax.plot(t, df['est_com_y'] * 1000.0, 'g-', label='CoM Shift dY (mm)', linewidth=1.5)
    ax.axhline(0, color='gray', linestyle=':')
    ax.set_ylabel('CoM Offset (mm)')
    ax.set_title('6. Estimated Center of Mass (CoM) Offset')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 7. Total Vertical Support Force
    ax = axes[3, 0]
    ax.plot(t, df['total_support_fz'], 'k-', label='Total Support Fz (N)', linewidth=1.2)
    ax.axhline(11.94 * 9.81, color='r', linestyle='--', label='Nominal Gravity Force (117 N)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Force (N)')
    ax.set_title('7. Total Ground Reaction Force (Fz)')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    # 8. Real-time Computation Latency
    ax = axes[3, 1]
    ax.plot(t, df['t_est_ms'], 'g-', label='KF Est Time (ms)', alpha=0.7)
    ax.plot(t, df['t_mpc_ms'], 'b-', label='MPC Solve Time (ms)', alpha=0.7)
    ax.plot(t, df['t_total_ms'], 'r-', label='Total Step Time (ms)', linewidth=1.2)
    ax.axhline(2.0, color='k', linestyle='--', label='2ms (500Hz Deadline)')
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Time (ms)')
    ax.set_title('8. Controller Computation Time & Latency')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=200)
        print(f"Đã lưu đồ thị phân tích vào: {save_path}")
    else:
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot CMPC & Linear KF Telemetry")
    parser.add_argument("--file", type=str, help="Path to telemetry CSV file")
    parser.add_argument("--save", type=str, help="Save figure to file (e.g. plot.png)")
    args = parser.parse_args()

    csv_file = args.file if args.file else find_latest_csv()
    if not csv_file:
        print("Không tìm thấy file telemetry CSV trong thư mục data/!")
        sys.exit(1)

    plot_telemetry(csv_file, args.save)
