#!/usr/bin/env python3
"""
mission_forward_back.py — L-loop flight path with interactive showcase modes.

Usage:
    python3 mission_forward_back.py [DRONE_ID [MODE]]   (DRONE_ID default: 1)
    MODE: fly | auto | loop  — skips the interactive menu if given

After the drone enters MISSION state the script pauses for an Enter press,
then offers three execution modes:

  1 — Auto      each step runs for DWELL_S seconds, advances automatically
  2 — Fly       real-time keyboard control of the drone
  3 — Loop      repeats full auto sequence until Ctrl-C

Keyboard controls in Fly mode:
  ↑ / W        forward  (in facing direction)
  ↓ / S        backward
  ← / A        strafe left
  → / D        strafe right
  Q            rotate left  90°
  E            rotate right 90°
  R / Page Up  climb   0.5 m
  F / Page Dn  descend 0.5 m
  H / Enter    return home
  Esc / X      abort → return home

Auto sequence (10 steps):
    0. Hold at home   — waits for Phase 3 climb to complete
    1. Fly 1 m forward (North, +X)
    2. Rotate -90° left (West, yaw=270°)
    3. Fly 2 m forward (West, -Y)
    4. Climb to 3 m altitude
    5. Rotate -90° left (South, yaw=180°)
    6. Fly 1 m forward (South, -X)
    7. Rotate -90° left (East, yaw=90°)
    8. Fly 2 m forward (East, +Y) — returns over home XY
    9. Descend to 0.5 m altitude
   10. COMMAND_RETURN_HOME  (skipped in Loop mode — repeats instead)

Publishes:
    /gcs/drone_{ID}/control   geometry_msgs/PoseStamped
    /gcs/drone_{ID}/command   std_msgs/String

Subscribes:
    /drone_{ID}/state         std_msgs/String
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

# ── Parameters ────────────────────────────────────────────────────────────────
DRONE_ID        = int(sys.argv[1]) if len(sys.argv) > 1 else 1
_MODE_ARG       = sys.argv[2].lower() if len(sys.argv) > 2 else None  # fly|auto|loop
CRUISE_ALT_M    = 1.5   # must match MISSION_TAKEOFF_ALT_M in firmware
HIGH_ALT_M      = 3.0
LOW_ALT_M       = 0.5
CLIMB_WAIT_S    = 6.0
DWELL_S         = 5.0

STATE_TIMEOUT_S = 120

# Fly-mode increments
FLY_STEP_M      = 0.5   # metres per forward/back/strafe keypress
FLY_ALT_STEP_M  = 0.5   # metres per climb/descend keypress
FLY_ROT_DEG     = 90.0  # degrees per rotate keypress
FLY_ALT_MIN_M   = 0.3   # safety floor
FLY_ALT_MAX_M   = 3.0   # safety ceiling


# ── Node ──────────────────────────────────────────────────────────────────────
class MissionNode(Node):

    def __init__(self):
        super().__init__(f'swarm_mission_forward_back_d{DRONE_ID}')
        self.state = ''
        self._control_pub = self.create_publisher(
            PoseStamped, f'/gcs/drone_{DRONE_ID}/control', 10)
        self._command_pub = self.create_publisher(
            String, f'/gcs/drone_{DRONE_ID}/command', 10)
        self._waypoint_pub = self.create_publisher(
            Marker, f'/drone_{DRONE_ID}/waypoint_marker', 10)
        self.create_subscription(
            String, f'/drone_{DRONE_ID}/state', self._state_cb, 10)

    def _state_cb(self, msg: String):
        if msg.data != self.state:
            self.get_logger().info(f'D{DRONE_ID} state → {msg.data}')
            self.state = msg.data

    def send_setpoint(self, x: float, y: float, z_up: float, yaw_deg: float = 0.0):
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z_up)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._control_pub.publish(msg)

    def send_waypoint_marker(self, x: float, y: float, z: float):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.ns = 'waypoint'
        m.id = DRONE_ID
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.85
        m.lifetime.sec = 2
        self._waypoint_pub.publish(m)

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._command_pub.publish(msg)
        self.get_logger().info(f'CMD → {cmd}')


# ── Raw keyboard reader ───────────────────────────────────────────────────────
# Token constants
K_FORWARD  = 'forward'
K_BACKWARD = 'backward'
K_LEFT     = 'left'
K_RIGHT    = 'right'
K_ROT_L    = 'rot_left'
K_ROT_R    = 'rot_right'
K_CLIMB    = 'climb'
K_DESCEND  = 'descend'
K_HOME     = 'home'
K_ABORT    = 'abort'

def _read_keys(queue: list, lock: threading.Lock, stop: threading.Event):
    """Background thread: reads raw keypresses and appends tokens to queue."""
    import tty as _tty, termios as _termios, select as _select
    fd = sys.stdin.fileno()
    old = _termios.tcgetattr(fd)
    _tty.setraw(fd)
    try:
        while not stop.is_set():
            if not _select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            ch = sys.stdin.read(1)
            token = None
            if ch == '\x1b':
                if _select.select([sys.stdin], [], [], 0.05)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[' and _select.select([sys.stdin], [], [], 0.05)[0]:
                        ch3 = sys.stdin.read(1)
                        token = {
                            'A': K_FORWARD,
                            'B': K_BACKWARD,
                            'D': K_LEFT,
                            'C': K_RIGHT,
                            '5': K_CLIMB,
                            '6': K_DESCEND,
                        }.get(ch3, K_ABORT)
                if token is None:
                    token = K_ABORT
            else:
                token = {
                    'w': K_FORWARD,  'W': K_FORWARD,
                    's': K_BACKWARD, 'S': K_BACKWARD,
                    'a': K_LEFT,     'A': K_LEFT,
                    'd': K_RIGHT,    'D': K_RIGHT,
                    'q': K_ROT_L,    'Q': K_ROT_L,
                    'e': K_ROT_R,    'E': K_ROT_R,
                    'r': K_CLIMB,    'R': K_CLIMB,
                    'f': K_DESCEND,  'F': K_DESCEND,
                    'h': K_HOME,     'H': K_HOME,
                    '\r': K_HOME,
                    'x': K_ABORT,    'X': K_ABORT,
                }.get(ch)
            if token is not None:
                with lock:
                    queue.append(token)
    finally:
        _termios.tcsetattr(fd, _termios.TCSADRAIN, old)


# ── Helpers ───────────────────────────────────────────────────────────────────
def wait_for_state(node: MissionNode, target: str, timeout_s: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.state.lower() == target.lower():
            return True
    return False


def step_auto(node: MissionNode, x, y, z, yaw, label, dwell=None) -> bool:
    secs = dwell if dwell is not None else DWELL_S
    node.get_logger().info(
        f'{label}  →  ({x:.2f}, {y:.2f}, {z:.2f} m↑, {yaw:.0f}°)  [{secs:.0f} s]')
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        if node.state.lower() != 'mission':
            node.get_logger().warn('State left mission — aborting')
            return False
        node.send_setpoint(x, y, z, yaw)
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


# ── Auto sequence ─────────────────────────────────────────────────────────────
STEPS = [
    ( 0.0,  0.0, CRUISE_ALT_M,   0.0, 'Step 0 — Hold home (climb wait)'),
    ( 1.0,  0.0, CRUISE_ALT_M,   0.0, 'Step 1 — Forward 1 m (North)'),
    ( 1.0,  0.0, CRUISE_ALT_M, 270.0, 'Step 2 — Rotate -90° (West)'),
    ( 1.0, -2.0, CRUISE_ALT_M, 270.0, 'Step 3 — Forward 2 m (West)'),
    ( 1.0, -2.0, HIGH_ALT_M,   270.0, 'Step 4 — Climb to 3 m'),
    ( 1.0, -2.0, HIGH_ALT_M,   180.0, 'Step 5 — Rotate -90° (South)'),
    ( 0.0, -2.0, HIGH_ALT_M,   180.0, 'Step 6 — Forward 1 m (South)'),
    ( 0.0, -2.0, HIGH_ALT_M,    90.0, 'Step 7 — Rotate -90° (East)'),
    ( 0.0,  0.0, HIGH_ALT_M,    90.0, 'Step 8 — Forward 2 m (East)'),
    ( 0.0,  0.0, LOW_ALT_M,     90.0, 'Step 9 — Descend to 0.5 m'),
]


# ── Mode runners ──────────────────────────────────────────────────────────────
def run_auto(node: MissionNode):
    node.get_logger().info('Mode: AUTO')
    for i, (x, y, z, yaw, label) in enumerate(STEPS):
        dwell = CLIMB_WAIT_S if i == 0 else DWELL_S
        if not step_auto(node, x, y, z, yaw, label, dwell=dwell):
            return
    node.get_logger().info('Sequence complete — sending COMMAND_RETURN_HOME')
    node.send_command('COMMAND_RETURN_HOME')


def run_fly(node: MissionNode):
    """Real-time keyboard flight control."""
    log = node.get_logger()
    log.info('Mode: FLY — keyboard control')

    # Wait for climb
    print(f'\n  Waiting for climb ({CLIMB_WAIT_S:.0f} s) ...')
    if not step_auto(node, 0.0, 0.0, CRUISE_ALT_M, 0.0,
                     'Climb wait', dwell=CLIMB_WAIT_S):
        return

    # Position state (NED, yaw in degrees)
    pos = [0.0, 0.0, CRUISE_ALT_M]   # x, y, z
    yaw = 0.0

    key_queue: list = []
    lock = threading.Lock()
    stop_event = threading.Event()
    reader = threading.Thread(
        target=_read_keys, args=(key_queue, lock, stop_event), daemon=True)
    reader.start()

    def status():
        yaw_label = {0: 'N', 45: 'NE', 90: 'E', 135: 'SE',
                     180: 'S', 225: 'SW', 270: 'W', 315: 'NW'}.get(int(yaw) % 360, f'{yaw:.0f}°')
        print(f'\r  pos ({pos[0]:+.1f}, {pos[1]:+.1f}, {pos[2]:.1f} m)  '
              f'yaw {yaw_label:<3}  '
              f'[W/S/A/D=move  Q/E=rotate  R/F=alt  H=home  X=abort]   ',
              end='', flush=True)

    print('\n  Keyboard flight active.\n')
    print('    ↑/W  forward      ↓/S  backward     ←/A  strafe left   →/D  strafe right')
    print('    Q    rotate left  E    rotate right  R    climb         F    descend')
    print('    H / Enter  return home               X / Esc  abort\n')

    try:
        while True:
            if node.state.lower() != 'mission':
                log.warn('State left mission — exiting fly mode')
                break

            # Drain key queue
            with lock:
                tokens = list(key_queue)
                key_queue.clear()

            done = False
            moved = False
            for token in tokens:
                yaw_rad = math.radians(yaw)
                if token == K_FORWARD:
                    pos[0] += FLY_STEP_M * math.cos(yaw_rad)
                    pos[1] += FLY_STEP_M * math.sin(yaw_rad)
                    moved = True
                elif token == K_BACKWARD:
                    pos[0] -= FLY_STEP_M * math.cos(yaw_rad)
                    pos[1] -= FLY_STEP_M * math.sin(yaw_rad)
                    moved = True
                elif token == K_LEFT:
                    # strafe left = 90° CCW from heading
                    pos[0] += FLY_STEP_M * math.cos(yaw_rad - math.pi / 2)
                    pos[1] += FLY_STEP_M * math.sin(yaw_rad - math.pi / 2)
                    moved = True
                elif token == K_RIGHT:
                    pos[0] += FLY_STEP_M * math.cos(yaw_rad + math.pi / 2)
                    pos[1] += FLY_STEP_M * math.sin(yaw_rad + math.pi / 2)
                    moved = True
                elif token == K_ROT_L:
                    yaw = (yaw - FLY_ROT_DEG) % 360
                    moved = True
                elif token == K_ROT_R:
                    yaw = (yaw + FLY_ROT_DEG) % 360
                    moved = True
                elif token == K_CLIMB:
                    pos[2] = min(pos[2] + FLY_ALT_STEP_M, FLY_ALT_MAX_M)
                    moved = True
                elif token == K_DESCEND:
                    pos[2] = max(pos[2] - FLY_ALT_STEP_M, FLY_ALT_MIN_M)
                    moved = True
                elif token in (K_HOME, K_ABORT):
                    done = True
                    break

            if moved:
                node.send_waypoint_marker(pos[0], pos[1], pos[2])
            status()
            node.send_setpoint(pos[0], pos[1], pos[2], yaw)
            rclpy.spin_once(node, timeout_sec=0.05)

            if done:
                break

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print()

    log.info('Fly mode ended — sending COMMAND_RETURN_HOME')
    node.send_command('COMMAND_RETURN_HOME')


def run_loop(node: MissionNode):
    node.get_logger().info('Mode: LOOP — repeating until Ctrl-C')
    lap = 1
    try:
        while True:
            node.get_logger().info(f'--- Lap {lap} ---')
            for i, (x, y, z, yaw, label) in enumerate(STEPS):
                dwell = CLIMB_WAIT_S if i == 0 else DWELL_S
                if not step_auto(node, x, y, z, yaw, label, dwell=dwell):
                    return
            lap += 1
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Loop stopped — sending COMMAND_RETURN_HOME')
    node.send_command('COMMAND_RETURN_HOME')


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MissionNode()
    log = node.get_logger()

    print()
    print(f'  Mach Mind — mission script  (drone {DRONE_ID})')
    print(f'  Waiting up to {STATE_TIMEOUT_S} s for D{DRONE_ID} to enter MISSION state...')
    print('  (ARM the drone and press MISSION in rqt to start)')
    print()

    if not wait_for_state(node, 'mission', STATE_TIMEOUT_S):
        log.error(f'Timed out — D{DRONE_ID} never entered mission state. Exiting.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print()
    print(f'  D{DRONE_ID} is in MISSION state.')
    input('  Press ENTER to begin... ')
    print()

    _mode_map = {'auto': run_auto, 'fly': run_fly, 'loop': run_loop,
                 '1': run_auto, '2': run_fly, '3': run_loop}
    if _MODE_ARG and _MODE_ARG in _mode_map:
        print(f'  Mode: {_MODE_ARG} (from argument)')
        print()
        _mode_map[_MODE_ARG](node)
    else:
        print('  Select flight mode:')
        print('    1 — Auto      (L-loop, each step 5 s)')
        print('    2 — Fly       (real-time keyboard control)')
        print('    3 — Loop      (repeat auto sequence until Ctrl-C)')
        print()

        while True:
            choice = input('  Enter choice [1/2/3]: ').strip()
            if choice in ('1', '2', '3'):
                break
            print('  Please enter 1, 2, or 3.')

        print()
        _mode_map[choice](node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
