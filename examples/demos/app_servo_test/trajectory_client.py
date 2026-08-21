#!/usr/bin/env python3
"""
Trajectory Tracking Client - Python Ground Station Interface

This script demonstrates how to send waypoint trajectory data to the Crazyflie
running the trajectory tracking firmware.

The Crazyflie will:
1. Receive waypoints from this script via app_channel
2. Execute waypoint-following
3. After final waypoint, fly straight forward
4. Detect collision via nominal dynamics
5. Deploy end-effector (activate IO_3/PB4)
6. Back up and land
"""

import struct
import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


class TrajectoryClient:
    """Client for sending trajectory data to Crazyflie"""
    
    # Protocol constants
    TRAJECTORY_CMD_WAYPOINT = 0xA1
    
    def __init__(self, uri):
        self.uri = uri
        self.cf = Crazyflie(rw_cache='./cache')
        self.is_connected = False
        
        # Setup callback
        self.cf.connected.add_callback(self._connected_callback)
        self.cf.disconnected.add_callback(self._disconnected_callback)
    
    def _connected_callback(self, uri):
        print(f"Connected to {uri}")
        self.is_connected = True
    
    def _disconnected_callback(self, uri):
        print(f"Disconnected from {uri}")
        self.is_connected = False
    
    def connect(self):
        """Connect to the Crazyflie"""
        print(f"Connecting to {self.uri}...")
        self.cf.open_link(self.uri)
        
        # Wait for connection
        timeout = 5  # seconds
        start = time.time()
        while not self.is_connected and time.time() - start < timeout:
            time.sleep(0.1)
        
        if not self.is_connected:
            raise RuntimeError("Failed to connect to Crazyflie")
        
        print("Connected successfully!")
    
    def disconnect(self):
        """Disconnect from the Crazyflie"""
        if self.is_connected:
            self.cf.close_link()
    
    def send_waypoints(self, waypoints):
        """
        Send waypoints to the Crazyflie
        
        Args:
            waypoints: List of tuples (x, y, z, yaw, duration_ms)
                      Each waypoint is (float, float, float, float, uint32)
                      x, y, z: Position in meters
                      yaw: Angle in radians
                      duration_ms: Time to reach this waypoint in milliseconds
        """
        # Prepare data packet
        # Format: [0xA1] + waypoint_data
        # Each waypoint: 5 floats (x, y, z, yaw, duration) = 20 bytes
        
        data = bytearray()
        data.append(self.TRAJECTORY_CMD_WAYPOINT)
        
        for wp in waypoints:
            x, y, z, yaw, duration = wp
            # Pack as: float, float, float, float, uint32
            packed = struct.pack('<ffffi', x, y, z, yaw, duration)
            data.extend(packed)
        
        # Send data via app_channel
        print(f"Sending {len(waypoints)} waypoints ({len(data)} bytes)...")
        self.cf.appch.send_packet(bytes(data))
        
        # Wait a bit for transmission
        time.sleep(0.5)
        
        print("Waypoints sent!")
    
    def send_trajectory_example_1(self):
        """
        Example 1: Simple square trajectory
        The drone will:
        1. Takeoff to 1m height
        2. Follow waypoints in a square pattern
        3. At final waypoint, fly straight forward (will hit wall)
        4. Deploy end-effector and back up
        5. Land
        """
        waypoints = [
            # (x, y, z, yaw, duration_ms)
            (0.0, 0.0, 0.3, 0.0, 2000),      # Takeoff to 0.3m
            (0.5, 0.0, 0.3, 0.0, 2000),      # Move forward
            (1.0, 0.0, 0.3, 0.0, 2000)
        ]
        self.send_waypoints(waypoints)
    
    def send_trajectory_example_2(self):
        """
        Example 2: Linear trajectory approaching wall
        The drone will:
        1. Takeoff
        2. Move forward in a straight line toward wall
        3. Detect collision
        4. Deploy end-effector
        5. Back up and land
        """
        waypoints = [
            # (x, y, z, yaw, duration_ms)
            (0.0, 0.0, 0.5, 0.0, 1000),   # Takeoff to 0.5m
            (0.2, 0.0, 0.5, 0.0, 1000),   # Small step forward
            (0.4, 0.0, 0.5, 0.0, 1000),   # Continue forward
            (0.6, 0.0, 0.5, 0.0, 1000),   # Almost at wall
        ]
        self.send_waypoints(waypoints)
    
    def send_trajectory_example_3(self):
        """
        Example 3: Custom trajectory (user input)
        """
        print("\nEnter waypoints (x y z yaw duration_ms), one per line.")
        print("Press Ctrl+D (or Ctrl+Z on Windows) when done.")
        print("Example: 0.5 0.0 1.0 0.0 2000")
        
        waypoints = []
        try:
            while True:
                try:
                    line = input("Waypoint: ").strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) != 5:
                        print("Error: Please provide 5 values (x y z yaw duration_ms)")
                        continue
                    
                    x, y, z, yaw, duration = map(float, parts)
                    duration = int(duration)
                    waypoints.append((x, y, z, yaw, duration))
                    print(f"Added waypoint: ({x}, {y}, {z}, {yaw}, {duration})")
                except ValueError:
                    print("Error: Invalid input. Please provide numbers.")
        except EOFError:
            pass
        
        if waypoints:
            self.send_waypoints(waypoints)
        else:
            print("No waypoints entered.")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Trajectory Tracking Client')
    parser.add_argument('--uri', default='radio://0/80/2M', 
                       help='Crazyflie URI (default: radio://0/80/2M)')
    parser.add_argument('--example', type=int, default=1, choices=[1, 2, 3],
                       help='Example trajectory (1=square, 2=wall, 3=custom)')
    
    args = parser.parse_args()
    
    # Create and connect client
    client = TrajectoryClient(args.uri)
    
    try:
        client.connect()
        time.sleep(1)
        
        # Send trajectory
        if args.example == 1:
            print("\n=== Example 1: Square Trajectory ===")
            client.send_trajectory_example_1()
        elif args.example == 2:
            print("\n=== Example 2: Wall Approach Trajectory ===")
            client.send_trajectory_example_2()
        else:
            print("\n=== Example 3: Custom Trajectory ===")
            client.send_trajectory_example_3()
        
        # Keep running for a while to let drone execute
        print("\nDrone executing trajectory. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()
