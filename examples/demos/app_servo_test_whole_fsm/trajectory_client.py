#!/usr/bin/env python3
import argparse
import math
import struct
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.utils import uri_helper

TRAJ_PORT = 0x0F  # must match CRTP_PORT_TRAJECTORY / TRAJECTORY_CRTP_PORT in firmware


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
    """
    Receive firmware DEBUG_PRINT / console output in this Python terminal.
    """
    def console_callback(text):
        print(text, end="")

    try:
        cf.console.receivedChar.add_callback(console_callback)
        print("[CONSOLE] Crazyflie console callback registered.")
    except Exception as e:
        print(f"[WARN] Could not register console callback: {e}")
        try:
            print("[DEBUG] cf.console dir:", dir(cf.console))
        except Exception:
            pass

def reset_traj_flags(cf):
    safe_set_param(cf, "traj.abort", "0")
    safe_set_param(cf, "traj.forceCollision", "0")
    safe_set_param(cf, "traj.start", "0")

    # Contact detection parameters
    # real speed < 0.02 m/s for 8 consecutive checks
    # check period in firmware is about 50 ms, so detection delay ≈ 8 * 50 ms = 0.4 s
    safe_set_param(cf, "traj.colMinSpeed", "0.02")
    safe_set_param(cf, "traj.colCount", "8")

    safe_set_param(cf, "traj.clear", "1", delay=0.10)


def send_xy_packet(cf, x, y, delay=0.01):
    pk = CRTPPacket()
    pk.port = TRAJ_PORT
    pk.channel = 0
    pk.data = struct.pack("<ff", float(x), float(y))
    cf.send_packet(pk)
    time.sleep(delay)


def send_trajectory(cf, traj_points):
    """
    Old verified protocol:
      one CRTP packet per point, payload <ff> = x,y
      final NaN,NaN packet marks the trajectory as complete
    """
    print(f"[TRAJ] Sending {len(traj_points)} xy points on CRTP port 0x{TRAJ_PORT:02X}")
    for i, pt in enumerate(traj_points):
        x, y = pt[0], pt[1]
        send_xy_packet(cf, x, y)
        print(f"  point {i}: x={x:.3f}, y={y:.3f}, packet=8 bytes")

    send_xy_packet(cf, float("nan"), float("nan"), delay=0.05)
    print("[TRAJ] End marker NaN/NaN sent.")


def start_trajectory(cf):
    print("[START] traj.start = 1")
    safe_set_param(cf, "traj.start", "1", delay=0.1)


def trigger_collision(cf):
    print("[TEST] traj.forceCollision = 1")
    safe_set_param(cf, "traj.forceCollision", "1", delay=0.1)


def abort_and_land(cf):
    print("[STOP] Requesting onboard landing through traj.abort=1")
    safe_set_param(cf, "traj.abort", "1", delay=0.1)


def example_trajectory(example_id):
    if example_id == 1:
        return [
            (0.0, 0.0),
            (0.2, 0.0),
            (0.4, 0.0),
        ]

    if example_id == 2:
        return [
            (0.0, 0.0),
            (0.2, 0.0),
            (0.4, 0.0),
            (0.6, 0.0),
        ]

    if example_id == 3:
        points = []
        print("Enter trajectory points: x y")
        print("Example: 0.2 0.0")
        print("Finish with Ctrl+D.")
        try:
            while True:
                line = input("Point: ").strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 2:
                    print("Need 2 values: x y")
                    continue
                x, y = map(float, parts)
                points.append((x, y))
                print(f"Added: ({x}, {y})")
        except EOFError:
            pass
        return points

    raise ValueError("example_id must be 1, 2, or 3")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7"),
        help="Crazyflie URI",
    )
    parser.add_argument(
        "--example",
        type=int,
        default=2,
        choices=[1, 2, 3],
        help="1=short line, 2=wall approach, 3=manual xy input",
    )
    parser.add_argument(
        "--force-collision-after",
        type=float,
        default=None,
        help="Seconds after start to trigger traj.forceCollision=1.",
    )
    parser.add_argument(
        "--no-disarm-on-exit",
        action="store_true",
        help="Do not send disarm request when exiting.",
    )
    args = parser.parse_args()

    points = example_trajectory(args.example)
    if len(points) < 2:
        print("[ERROR] Need at least 2 xy points.")
        return

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(args.uri, cf=cf) as scf:
        cf = scf.cf

        setup_console(cf)
        time.sleep(0.5)

        try:
            print(f"[CONNECT] Connected to {args.uri}")
            reset_traj_flags(cf)
            arm_drone(cf)
            send_trajectory(cf, points)
            time.sleep(0.3)
            start_trajectory(cf)

            t0 = time.time()
            force_collision_sent = False
            print("[RUN] Onboard state sequence:")
            print("      hovering 5s -> trajectory following [contact detection active] -> fly forward -> contact -> servo CW -> fly backward -> landing")
            print("[RUN] Press Ctrl+C to request landing.")

            while True:
                elapsed = time.time() - t0
                if (
                    args.force_collision_after is not None
                    and not force_collision_sent
                    and elapsed >= args.force_collision_after
                ):
                    trigger_collision(cf)
                    force_collision_sent = True
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[KEYBOARD] Ctrl+C detected.")
            abort_and_land(cf)
            try:
                time.sleep(6.0)
            except KeyboardInterrupt:
                print("\n[KEYBOARD] Second Ctrl+C: exiting immediately.")
        finally:
            if not args.no_disarm_on_exit:
                disarm_drone(cf)


if __name__ == "__main__":
    main()
