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
from rclpy.executors import ExternalShutdownException
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
STARTUP_DELAY_S     = 5.0     # grace period after start before any drone command
COMMAND_COOLDOWN_S  = 5.0     # post-arrival dwell at the box before returning (COOLDOWN)

# Executor return path, by scene (LH=red, RH=blue). After dwelling at the box the
# executor flies back keeping the box's Y to the team-zone border X, then 3 m out
# of the zone, and hovers. (red: 5 → 8, blue: 15 → 12.)
ZONE_BORDER_X = {'red': 5.0,  'blue': 15.0}
ZONE_OUT_X    = {'red': 8.0,  'blue': 12.0}

# Random fallback positions for still-missing opponent boxes, by our scene/team.
# LH scene = team red  (opponent boxes assumed in the blue area, x~17-18);
# RH scene = team blue (opponent boxes assumed in the red  area, x~4).
# (x, y) in arena metres. The [RND] tag for these positions is TERMINAL-ONLY.
FALLBACK_LH = [(17.0, 2.0), (18.0, 5.0), (17.0, 8.0)]   # team red  / LH scene
FALLBACK_RH = [(4.0, 2.0),  (4.0, 5.0),  (4.0, 8.0)]    # team blue / RH scene

# ── Terminal dashboard ──────────────────────────────────────────────────────
TEAL  = '\033[38;2;51;117;110m'
WHITE = '\033[38;2;220;220;220m'
CYAN  = '\033[38;2;0;220;200m'
DIM   = '\033[38;2;130;130;130m'
RESET = '\033[0m'

LOGO = r"""   ███╗   ███╗ █████╗  ██████╗██╗  ██╗    ███╗   ███╗██╗███╗   ██╗██████╗
   ████╗ ████║██╔══██╗██╔════╝██║  ██║    ████╗ ████║██║████╗  ██║██╔══██╗
   ██╔████╔██║███████║██║     ███████║    ██╔████╔██║██║██╔██╗ ██║██║  ██║
   ██║╚██╔╝██║██╔══██║██║     ██╔══██║    ██║╚██╔╝██║██║██║╚██╗██║██║  ██║
   ██║ ╚═╝ ██║██║  ██║╚██████╗██║  ██║    ██║ ╚═╝ ██║██║██║ ╚████║██████╔╝
   ╚═╝     ╚═╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝    ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝
   http://machmind.dev                               Team Mach Mind (c) 2026"""


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
        self.drone_poses = {}                # id -> (x, y, z) — RViz-actual (disc marker)
        self.drone_hdg = {}                  # id -> heading degrees (disc orientation)
        # Per-executor mission state machine (see _advance_executor):
        #   idle → to_box → (dwell/cooldown) → to_border → to_out → hover
        self._exec = {ex: {'phase': 'idle', 'box': None, 'ref_y': None,
                           'target': None, 'arr_ticks': 0}
                      for ex in self._executor_ids()}
        self._cooldown_until = {}            # id -> ts when post-arrival cooldown ends
        self._last_cmd = None                # (drone_id, role, x, y) of last setpoint
        self._first_render = True            # full clear once, then overwrite in place
        self._fallback_applied = False
        self._fallback_ids = set()           # ids we publish as fallback (ignore echoes)
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
        self.box_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Marker, '/visualization_marker', self._marker_cb, 50)
        self.create_subscription(String, '/gcs/system/team_color', self._team_color_cb, latched)
        for i in range(1, NUM_DRONES + 1):
            self.create_subscription(String, f'/drone_{i}/role',
                                     lambda m, d=i: self._role_cb(m, d), 10)
            self.create_subscription(String, f'/drone_{i}/state',
                                     lambda m, d=i: self._state_cb(m, d), 10)
            # Drone position/heading come from the RViz disc marker on
            # /visualization_marker (see _marker_cb) — the actual displayed pose,
            # not the vision_pose (which can be held/faded after ArUco loss).

        self.create_timer(CONTROL_PERIOD_S, self._control_loop)
        self.create_timer(1.0, self._render)

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

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _fmt_dur(s):
        m, sec = divmod(int(s), 60)
        return f'{m:02d}:{sec:02d}'

    @staticmethod
    def _yaw_deg(qx, qy, qz, qw):
        """Heading in arena degrees [0,360) from a quaternion (0=+X, 90=+Y)."""
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        return math.degrees(yaw) % 360.0

    def _executor_ids(self):
        return [i for i, r in ROLES.items() if r == 'executor']

    def _leader_ids(self):
        return [i for i, r in ROLES.items() if r == 'leader']

    def _zone_x(self):
        """(border_x, out_x) for our team zone, by scene. LH/red: 5 → 8;
        RH/blue: 15 → 12."""
        return ZONE_BORDER_X.get(self.team, 5.0), ZONE_OUT_X.get(self.team, 8.0)

    def _arrived(self, drone_id, target):
        """True once the drone has held within ARRIVAL_RADIUS_M of target for
        ARRIVAL_HOLD_TICKS consecutive ticks."""
        pose = self.drone_poses.get(drone_id)
        rec = self._exec.get(drone_id)
        if pose is None or target is None or rec is None:
            return False
        if _dist(pose, target) <= ARRIVAL_RADIUS_M:
            rec['arr_ticks'] += 1
        else:
            rec['arr_ticks'] = 0
        return rec['arr_ticks'] >= ARRIVAL_HOLD_TICKS

    # ════════════════════════ Subscriptions ════════════════════════
    def _marker_cb(self, msg: Marker):
        # Drone disc (CYLINDER, ns 'drone_<id>') — the RViz-actual pose + heading
        # the firmware draws. Use it for LOC/ALT/HDG and executor arrival.
        if msg.type == Marker.CYLINDER and msg.ns.startswith('drone_'):
            try:
                n = int(msg.ns.split('_', 1)[1])
            except (IndexError, ValueError):
                n = msg.id // 100
            self.drone_poses[n] = (msg.pose.position.x, msg.pose.position.y,
                                   msg.pose.position.z)
            self.drone_hdg[n] = self._yaw_deg(msg.pose.orientation.x,
                                              msg.pose.orientation.y,
                                              msg.pose.orientation.z,
                                              msg.pose.orientation.w)
            return
        # Same filter as the RQT panel: team-coloured CUBEs are boxes.
        if msg.type != Marker.CUBE or msg.ns not in ('red', 'blue'):
            return
        if msg.id in self._fallback_ids:
            # Tell our own fallback echo from a genuine firmware detection of the
            # same id: same position = our echo (ignore, keep the fallback);
            # different position = the box was actually FOUND, so yield to the
            # firmware (it owns found boxes — we drop the fallback and stop
            # republishing it).
            fb = self.boxes.get(msg.id)
            if fb and (abs(msg.pose.position.x - fb['x']) < 0.5 and
                       abs(msg.pose.position.y - fb['y']) < 0.5):
                return
            self._fallback_ids.discard(msg.id)
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
        if self._elapsed_s() < STARTUP_DELAY_S:
            return   # startup grace period — track boxes but send no commands yet
        self._assign_executors()
        self._update_leader()

    def _update_box_registry(self):
        """TODO: expire stale boxes, reconcile detections. Currently a no-op —
        _marker_cb already caches the latest CUBE per id."""
        pass

    def _apply_fallback_if_due(self):
        """TODO #2: once elapsed >= discovery_timeout, fill every still-missing
        opponent box with a predefined random position (scene-dependent) under an
        opponent ArUco id that is not already in use. Runs once.

        source='fallback' drives the terminal-only [RND] marker — if these are
        ever published to RViz, publish plain coordinates, never the tag."""
        if self._fallback_applied or self._elapsed_s() < self.discovery_timeout:
            return
        self._fallback_applied = True

        coords = FALLBACK_LH if self.team == 'red' else FALLBACK_RH
        opp_ids = self._opponent_box_ids()
        opp_team = 'blue' if self.team == 'red' else 'red'

        discovered = [bid for bid in self.boxes if bid in opp_ids]
        n_missing = max(0, len(coords) - len(discovered))
        if n_missing == 0:
            self.get_logger().info('boxes-timeout: all opponent boxes found — no fallback')
            return

        # Take the lowest opponent ArUco ids not already in use, one per missing box.
        unused = [bid for bid in opp_ids if bid not in self.boxes]
        for i in range(min(n_missing, len(unused), len(coords))):
            bid = unused[i]
            x, y = coords[i]
            self.boxes[bid] = {'x': float(x), 'y': float(y), 'z': 0.0,
                               'team': opp_team, 'last_seen': self._elapsed_s(),
                               'source': 'fallback'}
            self._fallback_ids.add(bid)
            # Publish to RViz ONCE — markers persist; if the box is later found,
            # the firmware's marker (same id) overwrites ours.
            self.box_pub.publish(self._box_marker(bid, x, y))
            self.box_pub.publish(self._box_label_marker(bid, x, y))
            self.get_logger().info(
                f'boxes-timeout: fallback box id={bid} -> ({x:.0f}, {y:.0f}) [RND]')

    def _assign_executors(self):
        """Assign opponent boxes to idle executors (nearest-first, no two on the
        same box), then advance each executor's mission state machine."""
        self._claim_boxes_for_idle_executors()
        for ex in self._executor_ids():
            self._advance_executor(ex)

    def _claimed_boxes(self):
        return {r['box'] for r in self._exec.values() if r['box'] is not None}

    def _claim_boxes_for_idle_executors(self):
        """Each idle executor claims the nearest unclaimed opponent box and is
        sent there. Claims are exclusive so the two executors stay separate."""
        claimed = self._claimed_boxes()
        opp = self._opponent_box_ids()
        for ex in self._executor_ids():
            rec = self._exec[ex]
            if rec['phase'] != 'idle':
                continue
            cands = [bid for bid in self.boxes if bid in opp and bid not in claimed]
            if not cands:
                continue
            pose = self.drone_poses.get(ex)
            if pose is not None:
                bid = min(cands, key=lambda b: _dist(
                    pose, (self.boxes[b]['x'], self.boxes[b]['y'])))
            else:
                bid = min(cands)          # deterministic fallback (lowest id)
            bx, by = self.boxes[bid]['x'], self.boxes[bid]['y']
            rec.update(phase='to_box', box=bid, ref_y=by, target=(bx, by), arr_ticks=0)
            claimed.add(bid)
            self._send_control(ex, bx, by, self.start_alt)
            self.get_logger().info(
                f'executor D{ex} → box {bid} ({bx:.1f}, {by:.1f})')

    def _advance_executor(self, ex):
        """Step one executor through to_box → dwell → to_border → to_out → hover."""
        rec = self._exec[ex]
        phase = rec['phase']
        if phase in ('idle', 'hover'):
            return
        if not self._arrived(ex, rec['target']):
            return

        border_x, out_x = self._zone_x()
        ref_y = rec['ref_y']
        now = self._now_s()

        if phase == 'to_box':
            # Dwell at the box for the cooldown, then return to the zone border.
            cu = self._cooldown_until.get(ex)
            if cu is None:
                self._cooldown_until[ex] = now + COMMAND_COOLDOWN_S
                return
            if now < cu:
                return
            self._cooldown_until.pop(ex, None)
            rec.update(phase='to_border', target=(border_x, ref_y), arr_ticks=0)
            self._send_control(ex, border_x, ref_y, self.start_alt)
            self.get_logger().info(f'executor D{ex} → border ({border_x:.0f}, {ref_y:.1f})')

        elif phase == 'to_border':
            # Exit the zone by 3 m, same Y, then hover.
            rec.update(phase='to_out', target=(out_x, ref_y), arr_ticks=0)
            self._send_control(ex, out_x, ref_y, self.start_alt)
            self.get_logger().info(f'executor D{ex} → out ({out_x:.0f}, {ref_y:.1f})')

        elif phase == 'to_out':
            rec.update(phase='hover', arr_ticks=0)
            self.get_logger().info(f'executor D{ex} mission complete → hover')

    def _update_leader(self):
        """Leader: while no boxes captured, send it to the home base; stop once
        the first capture happens. TODO: implement once capture tracking lands."""
        pass

    # ════════════════════════ Outgoing ════════════════════════
    def _send_control(self, drone_id: int, x: float, y: float, z: float, yaw_deg=0.0):
        role = self.drone_roles.get(drone_id) or ROLES.get(drone_id, '—')
        self._last_cmd = (drone_id, role, x, y)
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

    def _box_marker(self, box_id, x, y):
        """CUBE marker matching the firmware box format (frame map, ns by team,
        0.5 m cube, z=0.25, blue/red colour by id). The [RND] tag is NOT included
        — that marker is terminal-only."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'blue' if box_id <= 36 else 'red'
        m.id = int(box_id)
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        if box_id <= 36:
            m.color.r, m.color.g, m.color.b = 0.1, 0.3, 0.9
        else:
            m.color.r, m.color.g, m.color.b = 0.9, 0.1, 0.1
        m.color.a = 0.75
        return m

    def _box_label_marker(self, box_id, x, y):
        """'(X,Y)' label matching the firmware: ns 'box_label', integer coords,
        1 m above the box. No [RND] — terminal-only."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'box_label'
        m.id = int(box_id)
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f'({round(x)},{round(y)})'
        return m

    # ════════════════════════ Terminal dashboard ════════════════════════
    def _render(self):
        """Redraw the operator dashboard once per second.

        Flicker-free: instead of clearing the whole screen each tick (which
        blanks then repaints), move the cursor home and overwrite in place —
        '\\033[K' erases each line's tail, '\\033[J' wipes any leftover lines.
        A single full clear runs only on the first frame."""
        lines = [TEAL + l + RESET for l in LOGO.split('\n')]
        lines.append('')

        team = self.team or '—'
        elapsed = self._elapsed_s()
        mode = '  [DRY-RUN]' if self.dry_run else ''
        if elapsed < STARTUP_DELAY_S:
            mode += f'  [STARTUP HOLD {STARTUP_DELAY_S - elapsed:.0f}s]'
        lines.append(f'{WHITE}   [SDC26 Commander]{RESET}   team={team}   '
                     f'uptime={self._fmt_dur(elapsed)}{mode}')
        lines.append('')

        # Box line — up to EXPECTED_BOX_COUNT, sorted by id. '[RND]' next to the
        # coordinate marks a random fallback position (not a real detection).
        # This marker is TERMINAL-ONLY: it is derived from the internal 'source'
        # field here and must never be added to any published marker/topic/label.
        ids = sorted(self.boxes.keys())[:EXPECTED_BOX_COUNT]
        cells = []
        for i in range(EXPECTED_BOX_COUNT):
            if i < len(ids):
                b = self.boxes[ids[i]]
                tag = ' [RND]' if b.get('source') == 'fallback' else ''
                cells.append(f'BOX{i + 1}: ({b["x"]:.1f}, {b["y"]:.1f}){tag}')
            else:
                cells.append(f'BOX{i + 1}: (—)')
        lines.append(CYAN + '   ' + '   '.join(cells) + RESET)

        # Countdown until random fallback positions are published for any
        # still-missing boxes (TODO #2).
        if self._fallback_applied:
            fb = 'random fallback for missing boxes: published — marked [RND]'
        else:
            remaining = max(0.0, self.discovery_timeout - elapsed)
            fb = f'random fallback for missing boxes in: {remaining:.0f}s'
        lines.append(DIM + '   ' + fb + RESET)
        lines.append('')

        # Per-drone table.
        lines.append(WHITE + f'   {"DRONE":<6}{"STATUS":<15}{"ROLE":<9}'
                     f'{"LOC":<13}{"ALT":<6}{"HDG":<6}{"WP":<13}{"COOLDOWN":<8}' + RESET)
        now = self._now_s()
        for n in range(1, NUM_DRONES + 1):
            status = self.drone_states.get(n, '—')
            role = self.drone_roles.get(n) or ROLES.get(n, '—')
            loc = self.drone_poses.get(n)
            loc_s = f'({loc[0]:.1f}, {loc[1]:.1f})' if loc else '—'
            alt_s = f'{loc[2]:.1f}m' if loc else '—'
            hdg = self.drone_hdg.get(n)
            hdg_s = f'{hdg:.0f}°' if hdg is not None else '—'
            rec = self._exec.get(n)
            if rec and rec['phase'] == 'hover':
                wp_s = 'hover'
            elif rec and rec['target'] is not None:
                tx, ty = rec['target']
                wp_s = f'({tx:.1f}, {ty:.1f})'
            else:
                wp_s = '—'
            cu = self._cooldown_until.get(n)
            if cu is None:
                cd_s = '—'
            else:
                rem = cu - now
                cd_s = f'{rem:.1f}s' if rem > 0 else 'ready'
            lines.append(f'   {n:<6}{status:<15}{role:<9}{loc_s:<13}'
                         f'{alt_s:<6}{hdg_s:<6}{wp_s:<13}{cd_s:<8}')

        # Last command sent — below the table.
        if self._last_cmd:
            d, role, x, y = self._last_cmd
            last_cmd_s = f'D{d} {role} ({x:.1f}, {y:.1f})'
        else:
            last_cmd_s = '—'
        lines.append('')
        lines.append(f'{WHITE}   last command sent:{RESET} {last_cmd_s}')
        lines.append('')
        lines.append(DIM + '   refresh 1 Hz · Ctrl-C to quit' + RESET)

        prefix = '\033[2J\033[H' if self._first_render else '\033[H'
        self._first_render = False
        buf = prefix + '\n'.join(line + '\033[K' for line in lines) + '\033[J'
        print(buf, end='', flush=True)


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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
