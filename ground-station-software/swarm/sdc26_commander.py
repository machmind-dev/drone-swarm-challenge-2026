#!/usr/bin/env python3
"""
sdc26_commander.py — Mach Mind swarm orchestrator for the Swarm Drone Challenge 2026.

BACKBONE / SCAFFOLD — ROS plumbing is wired and runnable; the mission decision
logic (assignment, fallback, leader check) is stubbed with TODOs. Fill the
stubs in over the next steps.

Role model (must mirror the firmware DRONE_ROLE table in
drone-comms-esp32s3/main/main.c — keyed on DRONE_ID):

    Drone 1, 3 → Seeker    only role allowed to publish box locations
    Drone 2, 4 → Executor  only role sent to capture a discovered opponent box
    Drone 5    → Leader    checks the home base while no boxes are captured

What this node will do (see stubbed methods):
  1. Track box locations published by Seekers on /visualization_marker (CUBEs).
  2. After DISCOVERY_TIMEOUT_S, fill still-missing boxes from a predefined
     candidate list (TODO #2).
  3. Assign opponent boxes to Executors nearest-first, one box at a time, and
     send them there via /gcs/drone_<ID>/control (TODO #3).
  4. Send the Leader to the home base while no boxes have been captured.
  5. Publish the consolidated role roster on /gcs/system/roles, and own the
     GCS→drone CONFIG_ROLE_* provision (firmware no-op for now).

Subscribes:
    /visualization_marker          visualization_msgs/Marker   (box CUBEs, ns red/blue)
    /drone_<N>/role                std_msgs/String             (firmware-published role)
    /drone_<N>/state               std_msgs/String             (flight state)
    /drone_<N>/vision_pose         geometry_msgs/PoseStamped   (arena pose)
    /gcs/system/team_color         std_msgs/String             (red/blue, transient_local)

Publishes:
    /gcs/drone_<N>/control         geometry_msgs/PoseStamped   (setpoint, arena frame)
    /gcs/drone_<N>/config          std_msgs/String             (CONFIG_ROLE_* provision)
    /gcs/system/roles              std_msgs/String             (roster JSON, transient_local)

Usage:
    python3 sdc26_commander.py [--boxes-timeout SECONDS] [--start-altitude METRES]
                               [--dry-run] [--team {red,blue}]
"""

import argparse
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

# ── Swarm config (mirror of firmware constants) ─────────────────────────────
NUM_DRONES = 5

ROLES = {1: 'seeker', 2: 'executor', 3: 'seeker', 4: 'executor', 5: 'leader'}

# Box ArUco IDs by team (from BOX_IDS in main.c).
BOX_IDS_BLUE = (31, 32, 33, 34, 35, 36)
BOX_IDS_RED  = (41, 42, 43, 44, 45, 46)

# Team areas in arena metres (from BOX_*_X_MIN/MAX, BOX_AREA_Y_* in main.c).
RED_X   = (0.0, 7.0)
BLUE_X  = (13.0, 20.0)
AREA_Y  = (0.0, 10.0)

EXPECTED_BOX_COUNT  = 3       # physical boxes in the arena
DISCOVERY_TIMEOUT_S = 120.0   # after this, fill missing boxes from fallback list
START_ALT_M         = 1.0     # starting altitude all drones climb to; the per-drone
                              # mission altitude differs and is set later by assignment
ARRIVAL_RADIUS_M    = 0.7     # executor "reached" a box within this distance
ARRIVAL_HOLD_TICKS  = 4       # consecutive in-radius ticks to count as captured
CONTROL_PERIOD_S    = 0.5     # control loop rate (2 Hz)

# Predefined fallback candidate positions (opponent area) for undiscovered boxes.
# TODO: tune to the actual arena layout. (x, y) in arena metres.
FALLBACK_CANDIDATES = [
    (16.0, 2.5),
    (16.0, 5.0),
    (16.0, 7.5),
]


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class SDC26Commander(Node):

    def __init__(self, team=None, discovery_timeout=DISCOVERY_TIMEOUT_S,
                 start_alt=START_ALT_M, dry_run=False):
        super().__init__('sdc26_commander')

        # Our team — opponent boxes are the target. Source of truth is the RQT
        # panel's latched /gcs/system/team_color; `team` here is only an optional
        # offline override (e.g. for --dry-run without RQT). None = wait for RQT.
        self.team = team
        self.discovery_timeout = discovery_timeout
        self.start_alt = start_alt
        self.dry_run = dry_run

        # ── Runtime state ───────────────────────────────────────────────────
        # box_id -> {'x','y','z','team','last_seen','source'('seen'|'fallback')}
        self.boxes = {}
        self.drone_states = {}               # id -> str
        self.drone_roles = {}                # id -> str (firmware-reported)
        self.drone_poses = {}                # id -> (x, y, z)
        self.assignments = {}                # executor id -> box_id (current target)
        self.captured = set()                # box_ids confirmed captured
        self._arrival_ticks = {}             # executor id -> consecutive in-radius count
        self._fallback_applied = False
        self._start_t = self.get_clock().now()

        # ── QoS ─────────────────────────────────────────────────────────────
        latched = QoSProfile(depth=1)
        latched.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE

        # ── Publishers ──────────────────────────────────────────────────────
        self.control_pubs = {
            i: self.create_publisher(PoseStamped, f'/gcs/drone_{i}/control', 10)
            for i in range(1, NUM_DRONES + 1)
        }
        self.config_pubs = {
            i: self.create_publisher(String, f'/gcs/drone_{i}/config', 10)
            for i in range(1, NUM_DRONES + 1)
        }
        self.roster_pub = self.create_publisher(String, '/gcs/system/roles', latched)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Marker, '/visualization_marker', self._marker_cb, 50)
        self.create_subscription(String, '/gcs/system/team_color', self._team_color_cb, latched)
        for i in range(1, NUM_DRONES + 1):
            self.create_subscription(String, f'/drone_{i}/role',
                                     lambda m, d=i: self._role_cb(m, d), 10)
            self.create_subscription(String, f'/drone_{i}/state',
                                     lambda m, d=i: self._state_cb(m, d), 10)
            self.create_subscription(PoseStamped, f'/drone_{i}/vision_pose',
                                     lambda m, d=i: self._pose_cb(m, d), 10)

        self.create_timer(CONTROL_PERIOD_S, self._control_loop)

        self._publish_roster()
        if self.team:
            self.get_logger().info(
                f'SDC26 Commander up — team={self.team}, opponent boxes='
                f'{self._opponent_box_ids()}, dry_run={self.dry_run}')
        else:
            self.get_logger().info(
                f'SDC26 Commander up — waiting for team from RQT '
                f'(/gcs/system/team_color); dry_run={self.dry_run}')

    # ════════════════════════ Helpers ════════════════════════
    def _opponent_box_ids(self):
        """ArUco IDs of the opponent team's boxes (the capture targets)."""
        if self.team == 'red':
            return BOX_IDS_BLUE
        if self.team == 'blue':
            return BOX_IDS_RED
        return ()   # team not yet known — waiting for /gcs/system/team_color

    def _home_base(self):
        """Own-team-area centroid the Leader checks. TODO: refine to real base."""
        xr = RED_X if self.team == 'red' else BLUE_X
        return ((xr[0] + xr[1]) / 2.0, (AREA_Y[0] + AREA_Y[1]) / 2.0)

    def _elapsed_s(self):
        return (self.get_clock().now() - self._start_t).nanoseconds / 1e9

    def _executor_ids(self):
        return [i for i, r in ROLES.items() if r == 'executor']

    def _leader_ids(self):
        return [i for i, r in ROLES.items() if r == 'leader']

    # ════════════════════════ Subscriptions ════════════════════════
    def _marker_cb(self, msg: Marker):
        # Same filter as the RQT panel: team-coloured CUBEs are boxes.
        if msg.type != Marker.CUBE or msg.ns not in ('red', 'blue'):
            return
        self.boxes[msg.id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'team': msg.ns,
            'last_seen': self._elapsed_s(),
            'source': 'seen',
        }

    def _role_cb(self, msg: String, drone_id: int):
        role = msg.data.strip().lower()
        self.drone_roles[drone_id] = role
        expected = ROLES.get(drone_id)
        if role and expected and role != expected and role != 'idle':
            self.get_logger().warn(
                f'D{drone_id} reports role "{role}" but table expects "{expected}"')

    def _state_cb(self, msg: String, drone_id: int):
        self.drone_states[drone_id] = msg.data.strip().lower()

    def _pose_cb(self, msg: PoseStamped, drone_id: int):
        self.drone_poses[drone_id] = (msg.pose.position.x,
                                      msg.pose.position.y,
                                      msg.pose.position.z)

    def _team_color_cb(self, msg: String):
        t = msg.data.strip().lower()
        if t in ('red', 'blue') and t != self.team:
            self.team = t
            self.get_logger().info(f'team set from GCS → {self.team}')
            self._publish_roster()

    # ════════════════════════ Control loop (stubbed) ════════════════════════
    def _control_loop(self):
        """Periodic orchestration tick. Each step is a stub for now."""
        if not self.team:
            return   # hold until RQT publishes our team on /gcs/system/team_color
        self._update_box_registry()
        self._apply_fallback_if_due()
        self._assign_executors()
        self._update_leader()

    def _update_box_registry(self):
        """TODO: expire stale boxes, reconcile detections. Currently a no-op —
        _marker_cb already caches the latest CUBE per id."""
        pass

    def _apply_fallback_if_due(self):
        """TODO #2: once elapsed >= discovery_timeout, assign predefined
        FALLBACK_CANDIDATES to any expected opponent box still undiscovered,
        marking source='fallback'. Run once."""
        if self._fallback_applied or self._elapsed_s() < self.discovery_timeout:
            return
        # TODO: pick missing opponent box ids, pop fallback candidates, populate
        #       self.boxes[...] with source='fallback', log each.
        self._fallback_applied = True
        self.get_logger().info('discovery timeout reached — fallback fill (TODO)')

    def _assign_executors(self):
        """TODO #3: nearest-first greedy. For each free executor (in mission,
        no live target or target reached), claim the nearest unclaimed opponent
        box, send it there via _send_control, and detect arrival (within
        ARRIVAL_RADIUS_M for ARRIVAL_HOLD_TICKS) to mark it captured and free
        the executor for the next box."""
        pass

    def _update_leader(self):
        """Leader: while no boxes captured, send it to the home base; stop once
        the first capture happens. TODO: implement once capture tracking lands."""
        pass

    # ════════════════════════ Outgoing ════════════════════════
    def _send_control(self, drone_id: int, x: float, y: float, z: float, yaw_deg=0.0):
        if self.dry_run:
            self.get_logger().info(f'[dry-run] D{drone_id} → ({x:.2f}, {y:.2f}, {z:.2f})')
            return
        yaw = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.control_pubs[drone_id].publish(msg)

    def _send_config_role(self, drone_id: int, role: str):
        """PROVISION ONLY — firmware treats CONFIG_ROLE_* as a no-op (role stays
        compile-time DRONE_ROLE). The Commander owns this GCS→drone path (not the
        RQT panel). Wire to a real runtime role when the firmware supports it."""
        msg = String()
        msg.data = f'CONFIG_ROLE_{role.upper()}'
        self.config_pubs[drone_id].publish(msg)
        self.get_logger().info(
            f'D{drone_id} → /gcs/drone_{drone_id}/config : {msg.data} (provision, no-op)')

    def _publish_roster(self):
        msg = String()
        msg.data = json.dumps({'team': self.team, 'roles': ROLES})
        self.roster_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='Mach Mind SDC26 swarm commander (backbone).')
    ap.add_argument('--team', choices=['red', 'blue'], default=None,
                    help='optional offline override; normally taken live from the '
                         'RQT panel (/gcs/system/team_color)')
    ap.add_argument('--boxes-timeout', type=float, default=DISCOVERY_TIMEOUT_S,
                    metavar='SECONDS',
                    help='seconds before still-undiscovered boxes use the fallback list')
    ap.add_argument('--start-altitude', type=float, default=START_ALT_M,
                    metavar='METRES',
                    help='starting altitude all drones climb to (default 1.0 m); the '
                         'per-drone mission altitude differs and is set by assignment')
    ap.add_argument('--dry-run', action='store_true',
                    help='log assignments without publishing setpoints (no drone moves)')
    args = ap.parse_args()

    rclpy.init()
    node = SDC26Commander(team=args.team,
                          discovery_timeout=args.boxes_timeout,
                          start_alt=args.start_altitude, dry_run=args.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
