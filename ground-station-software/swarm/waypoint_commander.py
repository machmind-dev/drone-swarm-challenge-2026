#!/usr/bin/env python3
"""
waypoint_commander.py — Interactive arena waypoint sender for Mach Mind drones.

Usage:
    python3 waypoint_commander.py

At the prompt enter:  <drone_id> <x> <y> [z] [yaw_deg]
  drone_id : 1-5
  x, y     : arena coordinates in metres  (x: 0-20, y: 0-10)
  z        : altitude in metres, default 1.5
  yaw_deg  : heading in degrees, default 0  (0=+X, 90=+Y, 180=-X, -90=-Y)

Examples:
    3 10 5          → drone 3 to (10, 5) at 1.5 m, yaw 0°
    3 10 5 2.0      → drone 3 to (10, 5) at 2.0 m, yaw 0°
    3 10 5 1.5 90   → drone 3 to (10, 5) at 1.5 m, facing +Y
    all 10 5        → all drones to (10, 5)

Distances > 1 m Manhattan from current position trigger the Manhattan sequencer
(X-leg first, then Y-leg, 1 m steps). Shorter distances stream directly.

Publishes: /gcs/drone_{N}/control  geometry_msgs/PoseStamped  (arena frame)
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

NUM_DRONES   = 5
DEFAULT_ALT  = 1.5
ARENA_X_MAX  = 20.0
ARENA_Y_MAX  = 10.0
ARENA_Z_MAX  = 5.0
ARENA_Z_MIN  = 0.1


class WaypointCommander(Node):

    def __init__(self):
        super().__init__('waypoint_commander')
        self._pubs = {
            i: self.create_publisher(PoseStamped, f'/gcs/drone_{i}/control', 10)
            for i in range(1, NUM_DRONES + 1)
        }

    def send(self, drone_id: int, x: float, y: float, z: float, yaw_deg: float):
        x = max(0.0, min(ARENA_X_MAX, x))
        y = max(0.0, min(ARENA_Y_MAX, y))
        z = max(ARENA_Z_MIN, min(ARENA_Z_MAX, z))

        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._pubs[drone_id].publish(msg)
        print(f'  D{drone_id} → x={x:.2f}  y={y:.2f}  z={z:.2f}  yaw={yaw_deg:.0f}°')


def parse_line(line: str):
    """Return list of (drone_id, x, y, z, yaw_deg) tuples or None on error."""
    parts = line.split()
    if len(parts) < 3:
        return None

    raw_id = parts[0].lower()
    try:
        x       = float(parts[1])
        y       = float(parts[2])
        z       = float(parts[3]) if len(parts) > 3 else DEFAULT_ALT
        yaw_deg = float(parts[4]) if len(parts) > 4 else 0.0
    except ValueError:
        return None

    if raw_id == 'all':
        return [(i, x, y, z, yaw_deg) for i in range(1, NUM_DRONES + 1)]

    try:
        drone_id = int(raw_id)
    except ValueError:
        return None

    if drone_id < 1 or drone_id > NUM_DRONES:
        print(f'  drone_id must be 1-{NUM_DRONES} or "all"')
        return None

    return [(drone_id, x, y, z, yaw_deg)]


def main():
    rclpy.init()
    node = WaypointCommander()

    print()
    print('  Mach Mind — waypoint commander')
    print(f'  Arena: x=0-{ARENA_X_MAX:.0f} m  y=0-{ARENA_Y_MAX:.0f} m  z=0-{ARENA_Z_MAX:.0f} m')
    print('  Format: <drone_id|all> <x> <y> [z] [yaw_deg]')
    print('  Type "q" to quit.')
    print()

    try:
        while True:
            try:
                line = input('Waypoint: ').strip()
            except EOFError:
                break

            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                break

            cmds = parse_line(line)
            if cmds is None:
                print('  Usage: <drone_id|all> <x> <y> [z] [yaw_deg]')
                continue

            for drone_id, x, y, z, yaw_deg in cmds:
                node.send(drone_id, x, y, z, yaw_deg)
                rclpy.spin_once(node, timeout_sec=0)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
