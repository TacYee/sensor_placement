#!/usr/bin/env python3
import argparse
import csv
import os
import struct
import time
from pathlib import Path
from datetime import datetime

import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.utils import uri_helper

from FileLogger import FileLogger


TRAJ_PORT = 0x0F  # must match CRTP_PORT_TRAJECTORY in firmware


def safe_set_param(cf, name, value, delay=0.05):
    try:
        cf.param.set_value(name, str(value))
        time.sleep(delay)
        print(f"[PARAM] {name} = {value}")
    except Exception as e:
        print(f"[WARN] Failed to set {name}: {e}")


def arm_drone(cf):
    print("[ARM] Sending arming request...")
    cf.platform.send_arming_request(True)
    time.sleep(1.0)
    print("[ARM] Armed.")


def disarm_drone(cf):
    print("[DISARM] Sending disarming request...")
    try:
        cf.platform.send_arming_request(False)
        time.sleep(0.5)
        print("[DISARM] Disarmed.")
    except Exception as e:
        print(f"[WARN] Disarm failed: {e}")


def setup_console(cf):
    def console_callback(text):
        print(text, end="")

    try:
        cf.console.receivedChar.add_callback(console_callback)
        print("[CONSOLE] Crazyflie console callback registered.")
    except Exception as e:
        print(f"[WARN] Could not register console callback: {e}")


def reset_traj_flags(cf):
    safe_set_param(cf, "traj.abort", "0")
    safe_set_param(cf, "traj.forceCollision", "0")
    safe_set_param(cf, "traj.start", "0")

    # Contact detection parameters:
    # real speed < 0.02 m/s for 8 consecutive checks.
    # Firmware checks roughly every 50 ms, so detection delay ~= 8 * 50 ms = 0.4 s.
    safe_set_param(cf, "traj.colMinSpeed", "0.08")
    safe_set_param(cf, "traj.colCount", "5")

    safe_set_param(cf, "traj.clear", "1", delay=0.10)


def init_servo_ccw(cf):
    """
    Initialize servo to CCW before flight.

    Firmware mapping:
        servotest.cmd = 1 -> CCW
        servotest.cmd = 2 -> CW
    """
    print("[SERVO INIT] Waiting for firmware default servo action...")
    time.sleep(1.0)

    print("[SERVO INIT] Moving servo to CCW position...")
    safe_set_param(cf, "servotest.cmd", "1", delay=0.8)
    print("[SERVO INIT] Servo CCW command sent.")


def start_trajectory(cf):
    safe_set_param(cf, "traj.start", "1", delay=0.1)


def abort_and_land(cf):
    safe_set_param(cf, "traj.abort", "1", delay=0.1)


def find_column(fieldnames, candidates):
    name_map = {name.lower(): name for name in fieldnames}
    for c in candidates:
        if c.lower() in name_map:
            return name_map[c.lower()]
    return None

def reset_kalman(cf):
    print("[EST] Reset Kalman estimator")
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(1.0)

def get_log_filename(args):
    fileroot = args.fileroot
    Path(fileroot).mkdir(parents=True, exist_ok=True)

    if args.filename is not None:
        name = args.filename + ".csv"
        fname = os.path.normpath(os.path.join(os.getcwd(), fileroot, name))

        i = 0
        while os.path.isfile(fname):
            i += 1
            name = args.filename + "_" + str(i) + ".csv"
            fname = os.path.normpath(os.path.join(os.getcwd(), fileroot, name))
    else:
        date = datetime.today().strftime("%Y-%m-%d+%H:%M:%S")
        name = f"{date}+second_drone.csv"
        fname = os.path.normpath(os.path.join(os.getcwd(), fileroot, name))

    return fname


def setup_file_logger(cf, args):
    """
    Reuse your existing FileLogger + logconfig.json.

    Expected configs in logconfig.json:
        state        -> usually stateEstimate.x/y/z
        orientation  -> usually stabilizer.yaw or stateEstimate.yaw

    Optional:
        velocity config, default name: velocity
        mission config, default name: mission
    """
    if args.no_log:
        print("[LOG] Disabled by --no_log.")
        return None

    Path(args.fileroot).mkdir(parents=True, exist_ok=True)

    log_file = get_log_filename(args)
    print(f"[LOG] Log location: {log_file}")

    flogger = FileLogger(cf, args.logconfig, log_file)

    # Required logs
    try:
        flogger.enableConfig("state")
        print("[LOG] Enabled config: state")
    except Exception as e:
        print(f"[WARN] Could not enable config 'state': {e}")

    try:
        flogger.enableConfig("orientation")
        print("[LOG] Enabled config: orientation")
    except Exception as e:
        print(f"[WARN] Could not enable config 'orientation': {e}")

    # Optional velocity log
    if args.enable_velocity_log:
        try:
            flogger.enableConfig(args.velocity_config_name)
            print(f"[LOG] Enabled velocity config: {args.velocity_config_name}")
        except Exception as e:
            print(f"[WARN] Could not enable velocity config '{args.velocity_config_name}': {e}")

    # Optional mission debug log
    if args.enable_mission_log:
        try:
            flogger.enableConfig(args.mission_config_name)
            print(f"[LOG] Enabled mission config: {args.mission_config_name}")
        except Exception as e:
            print(f"[WARN] Could not enable mission config '{args.mission_config_name}': {e}")

    # Optional trajectory debug log
    if args.enable_traj_log:
        try:
            flogger.enableConfig(args.traj_config_name)
            print(f"[LOG] Enabled trajectory config: {args.traj_config_name}")
        except Exception as e:
            print(f"[WARN] Could not enable trajectory config '{args.traj_config_name}': {e}")

    flogger.start()
    print("[LOG] FileLogger started.")

    return flogger


def stop_file_logger(flogger):
    if flogger is None:
        return

    try:
        if hasattr(flogger, "stop"):
            flogger.stop()
            print("[LOG] FileLogger stopped.")
        else:
            print("[LOG] FileLogger has no stop() method; relying on process exit/flush.")
    except Exception as e:
        print(f"[WARN] Failed to stop FileLogger: {e}")


def load_landing_frame_xy_csv(csv_path):
    """
    Load rotated/translated landing-frame trajectory.

    Expected columns from your postprocess:
        x_local, y_local, yaw_local

    Only x_local/y_local are sent, because firmware receives <ff>.
    yaw_local is currently not used.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")

    points = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            raise RuntimeError(f"CSV has no header: {csv_path}")

        x_col = find_column(reader.fieldnames, ["x_local", "x_local_m", "x", "x_m"])
        y_col = find_column(reader.fieldnames, ["y_local", "y_local_m", "y", "y_m"])

        if x_col is None or y_col is None:
            raise RuntimeError(
                f"Cannot find x/y columns. Found columns: {reader.fieldnames}"
            )

        for row in reader:
            if row[x_col] in ["", None] or row[y_col] in ["", None]:
                continue

            x = float(row[x_col])
            y = float(row[y_col])

            if np.isfinite(x) and np.isfinite(y):
                points.append((x, y))

    traj = np.asarray(points, dtype=np.float32)

    if traj.ndim != 2 or traj.shape[1] != 2 or len(traj) < 2:
        raise RuntimeError(f"Invalid trajectory loaded: shape={traj.shape}")

    return traj


def send_xy_packet(cf, x, y, delay=0.01):
    pk = CRTPPacket()
    pk.port = TRAJ_PORT
    pk.channel = 0
    pk.data = struct.pack("<ff", float(x), float(y))
    cf.send_packet(pk)
    time.sleep(delay)


def send_trajectory(cf, traj_points, packet_delay=0.01, print_all_points=False):
    traj_points = np.asarray(traj_points, dtype=np.float32)

    print(f"[TRAJ] Sending {len(traj_points)} xy points on CRTP port 0x{TRAJ_PORT:02X}")

    for i, pt in enumerate(traj_points):
        x = float(pt[0])
        y = float(pt[1])

        send_xy_packet(cf, x, y, delay=packet_delay)

        if print_all_points or i % 20 == 0 or i == len(traj_points) - 1:
            print(f"  point {i:03d}: x={x:+.3f}, y={y:+.3f}")

    send_xy_packet(cf, float("nan"), float("nan"), delay=0.05)
    print("[TRAJ] End marker NaN/NaN sent.")


def print_preview(traj, max_rows=8):
    print("========== Landing-frame trajectory preview ==========")
    print(f"points: {len(traj)}")
    print(f"first:  x={traj[0, 0]:+.3f}, y={traj[0, 1]:+.3f}")
    print(f"last:   x={traj[-1, 0]:+.3f}, y={traj[-1, 1]:+.3f}")

    print("First points:")
    for i in range(min(max_rows, len(traj))):
        print(f"  {i:03d}: x={traj[i, 0]:+.3f}, y={traj[i, 1]:+.3f}")

    if len(traj) > max_rows:
        print("...")
        print("Last points:")
        start = max(0, len(traj) - max_rows)
        for i in range(start, len(traj)):
            print(f"  {i:03d}: x={traj[i, 0]:+.3f}, y={traj[i, 1]:+.3f}")


def main():
    parser = argparse.ArgumentParser(
        description="Send rotated/translated landing-frame trajectory to second drone."
    )

    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"),
        help="Crazyflie URI",
    )

    parser.add_argument(
        "--traj_csv",
        required=True,
        help="Path to next_drone_full_trajectory_landing_frame_XXXX.csv",
    )

    parser.add_argument(
        "--packet_delay",
        type=float,
        default=0.01,
        help="Delay between CRTP packets [s].",
    )

    parser.add_argument(
        "--force-collision-after",
        type=float,
        default=None,
        help="Seconds after start to trigger traj.forceCollision=1 for testing.",
    )

    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Only preview trajectory, do not connect/send.",
    )

    parser.add_argument(
        "--no-disarm-on-exit",
        action="store_true",
        help="Do not send disarm request on exit.",
    )

    parser.add_argument(
        "--print_all_points",
        action="store_true",
        help="Print every sent trajectory point.",
    )

    # FileLogger arguments
    parser.add_argument(
        "--fileroot",
        type=str,
        default="second_drone_logs",
        help="Folder to save FileLogger CSV.",
    )

    parser.add_argument(
        "--logconfig",
        type=str,
        default="logconfig.json",
        help="Path to FileLogger logconfig json.",
    )

    parser.add_argument(
        "--filename",
        type=str,
        default=None,
        help="Optional log filename without .csv.",
    )

    parser.add_argument(
        "--no_log",
        action="store_true",
        help="Disable FileLogger logging.",
    )

    parser.add_argument(
        "--enable_velocity_log",
        action="store_true",
        help="Enable velocity log config from logconfig.json.",
    )

    parser.add_argument(
        "--velocity_config_name",
        type=str,
        default="velocity",
        help="Velocity config name in logconfig.json.",
    )

    parser.add_argument(
        "--enable_mission_log",
        action="store_true",
        help="Enable mission log config from logconfig.json.",
    )

    parser.add_argument(
        "--mission_config_name",
        type=str,
        default="mission",
        help="Mission config name in logconfig.json.",
    )

    parser.add_argument(
        "--enable_traj_log",
        action="store_true",
        help="Enable trajectory log config from logconfig.json.",
    )

    parser.add_argument(
        "--traj_config_name",
        type=str,
        default="traj",
        help="Trajectory config name in logconfig.json.",
    )

    parser.add_argument(
        "--init_servo_ccw",
        action="store_true",
        help="Move servo to CCW before arming/takeoff.",
    )

    args = parser.parse_args()

    traj = load_landing_frame_xy_csv(args.traj_csv)
    print_preview(traj)

    if args.dry_run:
        print("[DRY RUN] Not connecting to Crazyflie.")
        return

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(args.uri, cf=cf) as scf:
        cf = scf.cf

        setup_console(cf)
        time.sleep(0.5)

        flogger = None

        try:
            print(f"[CONNECT] Connected to {args.uri}")

            # Start logging as early as possible after connection.
            flogger = setup_file_logger(cf, args)

            reset_traj_flags(cf)

            if args.init_servo_ccw:
                init_servo_ccw(cf)
            reset_kalman(cf)
            arm_drone(cf)

            send_trajectory(
                cf,
                traj,
                packet_delay=args.packet_delay,
                print_all_points=args.print_all_points,
            )

            time.sleep(0.3)

            print("[START] traj.start = 1")
            start_trajectory(cf)

            t0 = time.time()
            force_collision_sent = False

            print("[RUN] Mission started.")
            print("[RUN] Press Ctrl+C to request onboard landing.")

            while True:
                elapsed = time.time() - t0

                if (
                    args.force_collision_after is not None
                    and not force_collision_sent
                    and elapsed >= args.force_collision_after
                ):
                    safe_set_param(cf, "traj.forceCollision", "1", delay=0.1)
                    force_collision_sent = True

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[KEYBOARD] Ctrl+C detected. Requesting onboard landing.")
            abort_and_land(cf)
            try:
                time.sleep(6.0)
            except KeyboardInterrupt:
                print("\n[KEYBOARD] Second Ctrl+C: exiting immediately.")

        finally:
            stop_file_logger(flogger)

            if not args.no_disarm_on_exit:
                disarm_drone(cf)


if __name__ == "__main__":
    main()