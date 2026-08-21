#!/usr/bin/env python3
import argparse
import struct
import time

import cflib.crtp
import cflib.crazyflie.platformservice as platformservice
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils import uri_helper

TRAJECTORY_CMD_CLEAR = 0xA0
TRAJECTORY_CMD_WAYPOINT = 0xA1


def safe_set_param(cf, name, value, delay=0.05):
    try:
        cf.param.set_value(name, str(value))
        time.sleep(delay)
        print(f"[PARAM] {name} = {value}")
    except Exception as e:
        print(f"[WARN] Failed to set {name}: {e}")


def send_app_channel_packet(cf, data: bytes, delay=0.03):
    """Replacement for cf.appch.send_packet(data), compatible with older cflib."""
    pk = CRTPPacket()
    pk.port = CRTPPort.PLATFORM
    pk.channel = platformservice.APP_CHANNEL
    pk.data = data
    cf.send_packet(pk)
    time.sleep(delay)


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


def reset_traj_flags(cf):
    safe_set_param(cf, "traj.abort", "0")
    safe_set_param(cf, "traj.forceCollision", "0")
    safe_set_param(cf, "traj.start", "0")


def abort_and_land(cf):
    print("[STOP] Requesting onboard abort/landing...")
    safe_set_param(cf, "traj.abort", "1", delay=0.1)


def clear_waypoint_buffer(cf):
    print("[APP_CHANNEL] Clearing onboard waypoint buffer")
    send_app_channel_packet(cf, bytes([TRAJECTORY_CMD_CLEAR]))
    time.sleep(0.1)


def send_waypoints(cf, waypoints):
    """
    Send one waypoint per app-channel packet.

    This avoids the CRTP payload limit. One packet is:
      [0xA1] + struct '<ffffI' = 1 + 20 = 21 bytes
    """
    clear_waypoint_buffer(cf)

    print(f"[APP_CHANNEL] Sending {len(waypoints)} waypoints as separate packets")
    for i, wp in enumerate(waypoints):
        x, y, z, yaw, duration_ms = wp
        payload = bytearray()
        payload.append(TRAJECTORY_CMD_WAYPOINT)
        payload.extend(struct.pack("<ffffI", float(x), float(y), float(z), float(yaw), int(duration_ms)))
        print(f"  waypoint {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}, duration={int(duration_ms)} ms, packet={len(payload)} bytes")
        send_app_channel_packet(cf, bytes(payload), delay=0.05)

    time.sleep(0.3)
    print("[APP_CHANNEL] Waypoints sent.")


def start_trajectory(cf):
    print("[START] traj.start = 1")
    safe_set_param(cf, "traj.start", "1", delay=0.1)


def trigger_collision(cf):
    print("[TEST] traj.forceCollision = 1")
    safe_set_param(cf, "traj.forceCollision", "1", delay=0.1)


def example_waypoints(example_id):
    if example_id == 1:
        return [
            (0.0, 0.0, 0.30, 0.0, 1500),
            (0.2, 0.0, 0.30, 0.0, 1500),
            (0.4, 0.0, 0.30, 0.0, 1500),
        ]

    if example_id == 2:
        return [
            (0.0, 0.0, 0.30, 0.0, 1000),
            (0.2, 0.0, 0.30, 0.0, 1000),
            (0.4, 0.0, 0.30, 0.0, 1000),
            (0.6, 0.0, 0.30, 0.0, 1000),
        ]

    if example_id == 3:
        waypoints = []
        print("Enter waypoints: x y z yaw_rad duration_ms")
        print("Example: 0.2 0.0 0.3 0.0 1000")
        print("Finish with Ctrl+D.")
        try:
            while True:
                line = input("Waypoint: ").strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != 5:
                    print("Need 5 values: x y z yaw_rad duration_ms")
                    continue
                x, y, z, yaw, duration = parts
                waypoints.append((float(x), float(y), float(z), float(yaw), int(float(duration))))
                print(f"Added: {waypoints[-1]}")
        except EOFError:
            pass
        return waypoints

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
        help="1=simple line, 2=wall approach, 3=manual input",
    )
    parser.add_argument(
        "--force-collision-after",
        type=float,
        default=None,
        help="Seconds after trajectory start to trigger traj.forceCollision=1.",
    )
    parser.add_argument(
        "--no-disarm-on-exit",
        action="store_true",
        help="Do not send disarm request when exiting.",
    )
    args = parser.parse_args()

    waypoints = example_waypoints(args.example)
    if len(waypoints) == 0:
        print("[ERROR] No waypoints to send.")
        return

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(args.uri, cf=cf) as scf:
        cf = scf.cf
        try:
            print(f"[CONNECT] Connected to {args.uri}")
            reset_traj_flags(cf)
            arm_drone(cf)
            send_waypoints(cf, waypoints)
            start_trajectory(cf)

            t0 = time.time()
            force_collision_sent = False
            print("[RUN] Drone is executing onboard trajectory.")
            print("[RUN] Press Ctrl+C to request abort/landing.")

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
            time.sleep(2.0)
        finally:
            if not args.no_disarm_on_exit:
                disarm_drone(cf)


if __name__ == "__main__":
    main()
