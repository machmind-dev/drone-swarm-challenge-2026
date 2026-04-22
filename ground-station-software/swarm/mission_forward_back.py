#!/usr/bin/env python3
"""
mission_forward_back.py — Forward / rotate / back flight path.

Usage:
    python3 mission_forward_back.py [DRONE_ID]   (default: 1)

Sequence (executes once drone enters MISSION state):
    1. Hold at home   — waits for Phase 3 climb to complete
    2. Fly 1 m forward (+X in PX4 local frame, facing North)
    3. Rotate 180° in place
    4. Fly 1 m back to start (facing South)
    5. Send COMMAND_RETURN_HOME

Publishes:
    /gcs/drone_{ID}/control   geometry_msgs/PoseStamped
        position.x/y  — NED East/North (m) from PX4 local origin
        position.z    — altitude, positive up (m)
        orientation   — quaternion encoding yaw (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))
                        yaw=0 → North, yaw=π → South (NED, clockwise positive)

    /gcs/drone_{ID}/command   std_msgs/String
        COMMAND_RETURN_HOME

Subscribes:
    /drone_{ID}/state         std_msgs/String — waits for "mission"
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ── Parameters ────────────────────────────────────────────────────────────────
DRONE_ID       = int(sys.argv[1]) if len(sys.argv) > 1 else 1
CRUISE_ALT_M   = 1.5   # must match MISSION_TAKEOFF_ALT_M in firmware
FORWARD_DIST_M = 1.0   # metres along +X (PX4 local North)
CLIMB_WAIT_S   = 6.0   # seconds to wait after MISSION detected (Phase 3 = 5 s)
DWELL_S        = 5.0   # seconds to hold each waypoint before proceeding
STATE_TIMEOUT_S = 120  # abort if drone doesn't enter mission within this time


# ── Node ─────────────────────────────────────────────────────────────────────
class MissionNode(Node):

    def __init__(self):
        super().__init__('swarm_mission_forward_back')
        self.state = ''

        self._control_pub = self.create_publisher(
            PoseStamped, f'/gcs/drone_{DRONE_ID}/control', 10)
        self._command_pub = self.create_publisher(
            String, f'/gcs/drone_{DRONE_ID}/command', 10)
        self.create_subscription(
            String, f'/drone_{DRONE_ID}/state', self._state_cb, 10)

    def _state_cb(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f'D{DRONE_ID} state → {msg.data}')
            self.state = msg.data

    # ── Publishers ────────────────────────────────────────────────────────────
    def send_setpoint(self, x: float, y: float, z_up: float, yaw_deg: float = 0.0):
        """Publish a position + yaw setpoint.

        x, y   : NED frame (m), origin = PX4 home at arming
        z_up   : altitude (m), positive up
        yaw_deg: heading in degrees, 0 = North (+X), 90 = East (+Y), clockwise
        """
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z_up)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._control_pub.publish(msg)

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._command_pub.publish(msg)
        self.get_logger().info(f'CMD → {cmd}')


# ── Helpers ───────────────────────────────────────────────────────────────────
def spin_for(node: MissionNode, seconds: float):
    """Spin the ROS executor for the given duration (no setpoint published)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)


def dwell(node: MissionNode, x: float, y: float, z_up: float,
          yaw_deg: float, seconds: float, label: str):
    """Publish setpoint at 20 Hz for `seconds`, aborting if state leaves mission."""
    node.get_logger().info(
        f'{label}  →  ({x:.2f} m, {y:.2f} m, {z_up:.2f} m up, yaw={yaw_deg:.0f}°)'
        f'  for {seconds:.1f} s'
    )
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if node.state.lower() != 'mission':
            node.get_logger().warn('State left mission — aborting flight path')
            return False
        node.send_setpoint(x, y, z_up, yaw_deg)
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def wait_for_state(node: MissionNode, target: str, timeout_s: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.state.lower() == target.lower():
            return True
    return False


# ── Mission ───────────────────────────────────────────────────────────────────
def run_mission(node: MissionNode):
    log = node.get_logger()

    # Step 0 — wait for climb to complete
    log.info(f'Climb wait: holding home for {CLIMB_WAIT_S:.0f} s ...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < CLIMB_WAIT_S:
        if node.state.lower() != 'mission':
            log.warn('State left mission during climb wait — aborting')
            return
        node.send_setpoint(0.0, 0.0, CRUISE_ALT_M, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)

    # Step 1 — fly 1 m forward (face North, move +X)
    ok = dwell(node, FORWARD_DIST_M, 0.0, CRUISE_ALT_M, 0.0,  DWELL_S, 'Step 1 — Forward 1 m')
    if not ok:
        return

    # Step 2 — rotate 180° in place
    ok = dwell(node, FORWARD_DIST_M, 0.0, CRUISE_ALT_M, 180.0, DWELL_S, 'Step 2 — Rotate 180°')
    if not ok:
        return

    # Step 3 — fly back 1 m (facing South, moving -X toward home)
    ok = dwell(node, 0.0, 0.0, CRUISE_ALT_M, 180.0, DWELL_S, 'Step 3 — Back 1 m')
    if not ok:
        return

    # Done — return home
    log.info('Flight path complete — sending COMMAND_RETURN_HOME')
    node.send_command('COMMAND_RETURN_HOME')


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MissionNode()
    log = node.get_logger()

    log.info(f'Mach Mind — forward/back mission  (drone {DRONE_ID})')
    log.info(f'Waiting up to {STATE_TIMEOUT_S} s for D{DRONE_ID} to enter MISSION state...')
    log.info('(ARM the drone and press MISSION in rqt to start)')

    if not wait_for_state(node, 'mission', STATE_TIMEOUT_S):
        log.error(f'Timed out — D{DRONE_ID} never entered mission state. Exiting.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    run_mission(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
