import os
import time
import socket
import struct
import threading
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from colorama import init, Fore, Style

# Initialize colorama for colored terminal output
init(autoreset=True)

MODEL_NAME = "lite3"
XML_PATH = "../../../Lite3_description/lite3_mjcf/mjcf/Lite3_stair.xml"
LOCAL_PORT = 20001
CTRL_IP = "127.0.0.1"
CTRL_PORT = 30010
# Viewer có thể tắt bằng biến môi trường VIEWER=0 (cho chạy test headless tự động).
USE_VIEWER = os.environ.get("VIEWER", "1") == "1"
SIM_DURATION = float(os.environ.get("SIM_DURATION", "0"))
DT = 0.001
RENDER_INTERVAL = 10

# ── Ground-truth base state logging (chỉ bật khi có BASELOG_PATH) ───────────
# Ghi vận tốc / vị trí CoM theo world frame (chỉ có ở phía simulation) để xác
# minh rằng robot thực sự đạt vận tốc mong muốn.  Định dạng CSV:
#   sim_time, world_x, world_y, world_z, vx, vy, vz
BASELOG_PATH = os.environ.get("BASELOG_PATH")        # None = không ghi
BASELOG_INTERVAL = int(os.environ.get("BASELOG_INTERVAL", "50"))  # số step giữa 2 sample (50 -> 50 ms)
TIMING_LOG_PATH = os.environ.get("PHASE5_TIMING_LOG")


# ── GMO validation scenario (simulation only, controller model unchanged) ───
# When ENABLE_PAYLOAD is True, use Lite3_payload.xml which adds a separate 2 kg
# body 10 cm forward of the TORSO geometric center.  The controller model
# (MiniCheetah.h) retains the original parameters, creating a mass/CoM mismatch
# that tests disturbance-rejection and uncertainty handling.
GMO_SCENARIO = os.environ.get("GMO_SIM_SCENARIO", "payload")
ENABLE_PAYLOAD = os.environ.get("GMO_ENABLE_PAYLOAD", "1") == "1"
# ─────────────────────────────────────────────────────────────────────────────

URDF_INIT = {
    "lite3": np.array([0, -1.35453, 2.54948] * 4, dtype=np.float32)
}

class MuJoCoSimulation:
    def __init__(self, model_key: str = MODEL_NAME,
                 xml_relpath: str = XML_PATH,
                 local_port: int = LOCAL_PORT,
                 ctrl_ip: str = CTRL_IP,
                 ctrl_port: int = CTRL_PORT):

        # UDP communication
        self.local_port = local_port
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", local_port))
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ── Select model file ─────────────────────────────────────────────
        # When payload is enabled, use the XML that includes a separate 2 kg
        # payload body.  The controller model (MiniCheetah.h) is unchanged.
        if GMO_SCENARIO == "early-contact":
            # Stair terrain creates unscheduled/early touchdown opportunities.
            xml_relpath = "../../../Lite3_description/lite3_mjcf/mjcf/Lite3_stair.xml"
        else:
            xml_relpath = "Lite3_payload.xml"
        # ───────────────────────────────────────────────────────────────────

        # Load MJCF
        xml_full = str(Path(__file__).resolve().parent / xml_relpath)
        print("xml_full", xml_full)
        if not os.path.isfile(xml_full):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_full}")

        self.model = mujoco.MjModel.from_xml_path(xml_full)
        if not ENABLE_PAYLOAD:
            payload_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "payload")
            if payload_body_id >= 0:
                self.model.body_mass[payload_body_id] = 1e-6
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # Robot DOF list
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)
        self.foot_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("FR_FOOT_collision", "FL_FOOT_collision",
                         "HR_FOOT_collision", "HL_FOOT_collision")
        ]

        # Initialize standing pose
        self._set_initial_pose(model_key)

        # Buffers
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0
        self.last_print_time = 0  # Track last print time
        self.sensor_timing_events = []
        self.command_receive_events = []
        self.command_apply_events = []
        self.command_sequence = 0
        self.last_applied_command_sequence = 0
        self.last_command_packet = None

        print(f"[INFO] MuJoCo model loaded, scenario={GMO_SCENARIO}, "
              f"payload={ENABLE_PAYLOAD}, dof={self.dof_num}")

        # Visualization
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Ground-truth base state log file (mở nếu được yêu cầu qua env)
        self.baselog_fp = None
        if BASELOG_PATH:
            self.baselog_fp = open(BASELOG_PATH, "w")
            self.baselog_fp.write(
                "sim_time,world_x,world_y,world_z,vx,vy,vz,pitch,omega_y\n"
            )
            self.baselog_fp.flush()
            print(f"[INFO] Base-state log enabled -> {BASELOG_PATH} (every {BASELOG_INTERVAL} steps)")

    def _set_initial_pose(self, key: str):
        """Set joint positions to match PyBullet initial angles."""
        qpos0 = self.data.qpos.copy()
        qpos0[0:3] = [0.0, 0.0, 0.35]
        qpos0[3:7] = [1.0, 0.0, 0.0, 0.0]
        qpos0[7:7+self.dof_num] = URDF_INIT[key]
        self.data.qpos[:] = qpos0
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def print_debug_info(self):
        """Consolidated function to print debug information with colors and aligned formatting."""
        # Format arrays with 2 decimal places and fixed width
        def format_array(arr):
            return "[" + ", ".join(f"{x:6.2f}" for x in arr) + "]"

        # Get current joint states for printing
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)
        tau = self.input_tq.flatten()
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(mat, q_world.astype(np.float64))
        R = mat.reshape(3, 3)
        body_acc = self.data.sensordata[16:19]
        # Ground-truth base pose/vel (world frame) — chỉ print để chẩn đoán
        base_pos = self.data.qpos[0:3]
        base_vel = self.data.qvel[0:3]

        print(f"{Fore.CYAN}=== [Debug Info] ==={Style.RESET_ALL}")
        print(f"{Fore.CYAN}[Base] Pos (xyz) :{Style.RESET_ALL} {format_array(base_pos)}")
        print(f"{Fore.CYAN}[Base] Vel (xyz):{Style.RESET_ALL} {format_array(base_vel)}")
        print(f"{Fore.GREEN}[IMU] RPY        :{Style.RESET_ALL} {format_array(rpy.flatten())}")
        print(f"{Fore.GREEN}[IMU] Omega      :{Style.RESET_ALL} {format_array(angvel_b.flatten())}")
        print(f"{Fore.GREEN}[IMU] Acc_body   :{Style.RESET_ALL} {format_array(body_acc.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Position  :{Style.RESET_ALL} {format_array(q.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Velocity  :{Style.RESET_ALL} {format_array(dq.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Torque    :{Style.RESET_ALL} {format_array(tau.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Pos:{Style.RESET_ALL} {format_array(self.pos_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Pos:{Style.RESET_ALL} {format_array(q.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Vel:{Style.RESET_ALL} {format_array(self.vel_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Vel:{Style.RESET_ALL} {format_array(dq.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kp Term   :{Style.RESET_ALL} {format_array(self.kp_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kd Term   :{Style.RESET_ALL} {format_array(self.kd_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] FF Tau    :{Style.RESET_ALL} {format_array(self.tau_ff.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Final Torq:{Style.RESET_ALL} {format_array(self.input_tq.T.flatten())}")
        print(f"{Fore.CYAN}==================={Style.RESET_ALL}")

    def start(self):
        # Start UDP receiver thread
        threading.Thread(target=self._udp_receiver, daemon=True).start()
        print(f"[INFO] UDP receiver on 0.0.0.0:{self.local_port}")

        # Main simulation loop
        step = 0
        last_time = time.time()
        while True:
            if time.time() - last_time >= DT:
                last_time = time.time()

                step += 1
                # 控制律

                self._apply_joint_torque()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT
                if SIM_DURATION > 0 and self.timestamp >= SIM_DURATION:
                    print(f"[INFO] SIM_DURATION={SIM_DURATION}s reached, stopping simulation.")
                    self._write_timing_log()
                    break

                # 采样 & 发送观测
                self._send_robot_state(step)
                # Ground-truth base state log (nếu được bật)
                if self.baselog_fp and step % BASELOG_INTERVAL == 0:
                    bp = self.data.qpos[0:3]
                    bv = self.data.qvel[0:3]
                    q_world = self.data.qpos[3:7]
                    pitch = self.quaternion_to_euler(q_world)[1]
                    rotation_flat = np.zeros(9, dtype=np.float64)
                    mujoco.mju_quat2Mat(rotation_flat, q_world.astype(np.float64))
                    omega_world = rotation_flat.reshape(3, 3) @ self.data.qvel[3:6]
                    self.baselog_fp.write(
                        f"{self.timestamp},{bp[0]},{bp[1]},{bp[2]},"
                        f"{bv[0]},{bv[1]},{bv[2]},{pitch},{omega_world[1]}\n")
                    self.baselog_fp.flush()
                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

                # Print at 0.5 Hz (every 2 seconds)
                current_time = time.perf_counter()
                if current_time - self.last_print_time >= 2.0:
                    self.print_debug_info()
                    self.last_print_time = current_time


    def _udp_receiver(self):
        """
        12f kp | 12f pos | 12f kd | 12f vel | 12f tau = 240 bytes
        """
        fmt = f'{self.dof_num}f' * 5
        expected = struct.calcsize(fmt)
        while True:
            data, addr = self.recv_sock.recvfrom(expected)
            if len(data) < expected:
                print(f"[WARN] UDP packet size {len(data)} != {expected}")
                continue
            if data != self.last_command_packet:
                self.last_command_packet = data
                self.command_sequence += 1
                self.command_receive_events.append(
                    (self.command_sequence, self.timestamp, time.monotonic_ns())
                )
            unpacked = struct.unpack(fmt, data)
            self.kp_cmd = np.asarray(unpacked[0:self.dof_num], dtype=np.float32).reshape(self.dof_num, 1)
            self.pos_cmd = np.asarray(unpacked[self.dof_num:self.dof_num * 2], dtype=np.float32).reshape(self.dof_num,
                                                                                                         1)
            self.kd_cmd = np.asarray(unpacked[self.dof_num * 2:self.dof_num * 3], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.vel_cmd = np.asarray(unpacked[self.dof_num * 3:self.dof_num * 4], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.tau_ff = np.asarray(unpacked[self.dof_num * 4:], dtype=np.float32).reshape(self.dof_num, 1)

    def _apply_joint_torque(self):
        if self.command_sequence != self.last_applied_command_sequence:
            self.last_applied_command_sequence = self.command_sequence
            self.command_apply_events.append(
                (self.command_sequence, self.timestamp, time.monotonic_ns())
            )
        # Current joint states
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)

        # τ = kp*(q_d - q) + kd*(dq_d - dq) + τ_ff
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq) +
            self.tau_ff
        )
        # Write to control buffer
        self.data.ctrl[:] = self.input_tq.flatten()

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _get_ground_truth_contacts(self):
        contact_flags = np.zeros(4, dtype=np.float32)
        force_world = np.zeros((4, 3), dtype=np.float32)
        geom_to_leg = {geom_id: leg for leg, geom_id in enumerate(self.foot_geom_ids)
                       if geom_id >= 0}
        force_torque = np.zeros(6, dtype=np.float64)
        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            if contact.efc_address < 0:
                continue
            if contact.geom1 not in geom_to_leg and contact.geom2 not in geom_to_leg:
                continue
            mujoco.mj_contactForce(self.model, self.data, contact_id, force_torque)
            world_force = contact.frame.reshape(3, 3).T @ force_torque[:3]
            if contact.geom1 in geom_to_leg:
                leg = geom_to_leg[contact.geom1]
                force_world[leg] -= world_force.astype(np.float32)
                if np.linalg.norm(world_force) > 1e-6:
                    contact_flags[leg] = 1.0
            if contact.geom2 in geom_to_leg:
                leg = geom_to_leg[contact.geom2]
                force_world[leg] += world_force.astype(np.float32)
                if np.linalg.norm(world_force) > 1e-6:
                    contact_flags[leg] = 1.0
        return contact_flags, force_world

    def _send_robot_state(self, step: int):
        # IMU
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        body_acc = self.data.sensordata[16:19]

        # Joints
        q = self.data.qpos[7:7+self.dof_num]
        dq = self.data.qvel[6:6+self.dof_num]
        tau = self.input_tq.flatten()
        contact_flags, force_world = self._get_ground_truth_contacts()

        # Pack and send
        payload = np.concatenate((
            np.array([self.timestamp], dtype=np.float64),
            np.asarray(rpy, dtype=np.float32),
            np.asarray(body_acc, dtype=np.float32),
            np.asarray(angvel_b, dtype=np.float32),
            q.astype(np.float32),
            dq.astype(np.float32),
            tau.astype(np.float32),
            contact_flags,
            force_world.reshape(-1)
        ))
        fmt = "1d" + f"{len(payload)-1}f"
        try:
            self.sensor_timing_events.append((step, self.timestamp, time.monotonic_ns()))
            self.send_sock.sendto(struct.pack(fmt, *payload),
                                  self.ctrl_addr)
        except socket.error as ex:
            print(f"[UDP send] {ex}")

    def _write_timing_log(self):
        if not TIMING_LOG_PATH:
            return
        events = []
        events.extend((wall_ns, "sensor_send", seq, sim_time)
                      for seq, sim_time, wall_ns in self.sensor_timing_events)
        events.extend((wall_ns, "command_receive", seq, sim_time)
                      for seq, sim_time, wall_ns in self.command_receive_events)
        events.extend((wall_ns, "command_apply", seq, sim_time)
                      for seq, sim_time, wall_ns in self.command_apply_events)
        events.sort(key=lambda event: event[0])
        with open(TIMING_LOG_PATH, "w") as timing_file:
            timing_file.write("event,sequence,sim_time,wall_ns\n")
            for wall_ns, event, sequence, sim_time in events:
                timing_file.write(f"{event},{sequence},{sim_time},{wall_ns}\n")


if __name__ == "__main__":
    sim = MuJoCoSimulation()
    sim.start()
