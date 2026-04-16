#!/usr/bin/env python3
"""
Physical button state visualiser for RViz.

Reads GPIO buttons and publishes a MarkerArray showing their states as
labelled coloured panels in the upper team zone in RViz.

Layout (left → right):
  [ARM]  [MISSION]  [EMERG]

ARM:            held button  — ARMED while held, DISARMED when released.
MISSION / EMERG: toggle buttons — each press flips ON ↔ OFF.

Position is configurable via the STATUS_* constants below.
"""

import time

import gpiod
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, Vector3

# ── GPIO ──────────────────────────────────────────────────────────────────────
GPIO_CHIP = "gpiochip4"
PINS = {
    5:  "ARM",
    6:  "MISSION",
    13: "EMERG",
}

# ── RViz panel position (left of arena, stacked vertically) ──────────────────
# Arena occupies x = 0–20 m, y = 0–10 m.  Panels sit just left of x = 0.
STATUS_X       = -2.0   # fixed x (outside left edge of arena)
STATUS_Y_BASE  =  8.0   # y centre of topmost button (ARM)
STATUS_Z       =  0.1   # height above ground plane
STATUS_SPACING =  2.5   # centre-to-centre gap in Y between buttons

BTN_W          =  1.8   # panel long side  (m) — becomes height after 90° rotation
BTN_H          =  0.55  # panel short side (m) — becomes width after 90° rotation
BTN_D          =  0.05  # panel depth (thin flat slab)
TEXT_SIZE      =  0.28  # label text height (m)
TEXT_Z_OFFSET  =  0.06  # label raised above panel surface

# 90° rotation around Z: quaternion (x=0, y=0, z=sin45°, w=cos45°)
_SIN45 = 0.7071067811865476
_COS45 = 0.7071067811865476

PUBLISH_HZ     = 10     # marker republish rate
LONG_PRESS_S   = 3.0   # seconds — threshold for KILL ALL (hold ≥ 3 s, matches rqt EMERGENCY_HOLD_SECONDS)
DRONE_COUNT    = 5      # number of drones to command

# ── Colours ───────────────────────────────────────────────────────────────────
RED          = ColorRGBA(r=0.85, g=0.12, b=0.12, a=0.95)
GREEN        = ColorRGBA(r=0.10, g=0.72, b=0.22, a=0.95)
AMBER        = ColorRGBA(r=0.90, g=0.60, b=0.00, a=0.95)
BLUE         = ColorRGBA(r=0.10, g=0.40, b=0.90, a=0.95)
DIMMED       = ColorRGBA(r=0.22, g=0.22, b=0.22, a=0.85)
DIMMED_WHITE = ColorRGBA(r=0.72, g=0.72, b=0.72, a=1.0)

# ── Mission states (cycle on each press) ─────────────────────────────────────
MISSION_STATES = [
    (BLUE,   "MISSION"),
]

# ── Emergency states ──────────────────────────────────────────────────────────
EMERG_STATES = [
    (GREEN,  "OK"),                # 0 — idle / safe
    (RED,    "EMERGENCY_LAND"),    # 1 — short press from OK
    (RED,    "KILL_ALL"),          # 2 — hold ≥ 5 s from OK
]


def make_panel(uid: int, y: float, color: ColorRGBA, frame: str = "map") -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = "button_status"
    m.id = uid
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position = Point(x=STATUS_X, y=y, z=STATUS_Z)
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = _SIN45
    m.pose.orientation.w = _COS45
    m.scale = Vector3(x=BTN_W, y=BTN_H, z=BTN_D)
    m.color = color
    m.frame_locked = True
    return m


def make_label(uid: int, y: float, text: str, frame: str = "map") -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = "button_status"
    m.id = uid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position = Point(x=STATUS_X, y=y, z=STATUS_Z + TEXT_Z_OFFSET)
    m.pose.orientation.w = 1.0
    m.scale.z = TEXT_SIZE
    m.color = DIMMED_WHITE
    m.text = text
    m.frame_locked = True
    return m


class ButtonStatusNode(Node):
    def __init__(self):
        super().__init__("button_status_node")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, "/button_status_markers", qos)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        # Per-drone command publishers — hardware has direct priority over rqt
        self._drone_command_pubs = [
            self.create_publisher(String, f"/gcs/drone_{i}/command", 10)
            for i in range(1, DRONE_COUNT + 1)
        ]

        # Latched state topics — rqt panel subscribes to mirror HW state and lock its buttons
        self._hw_arm_pub     = self.create_publisher(String, "/gcs/hw_arm_state",     qos)
        self._hw_mission_pub = self.create_publisher(String, "/gcs/hw_mission_state", qos)
        self._hw_emerg_pub   = self.create_publisher(String, "/gcs/hw_emerg_state",   qos)

        # Drone state tracking — required to enforce mission prerequisites (mirrors rqt logic)
        self._drone_states: dict[int, str] = {}
        for i in range(1, DRONE_COUNT + 1):
            self.create_subscription(
                String, f"/drone_{i}/state",
                lambda msg, drone_id=i: self._drone_state_callback(msg, drone_id), 10,
            )

        # GPIO setup
        self._chip = gpiod.Chip(GPIO_CHIP)
        self._lines: dict[int, gpiod.Line] = {}
        self._states: dict[int, int] = {}

        for pin, name in PINS.items():
            line = self._chip.get_line(pin)
            line.request(
                consumer=f"button_{name.lower()}",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
            )
            self._lines[pin] = line
            self._states[pin] = line.get_value()
            self.get_logger().info(f"GPIO {pin} ({name}) initial state: {self._states[pin]}")

        # Mission cycles through 3 states on each press
        self._mission_idx = 0

        # Emergency: 0=OK, 1=EMERG LAND, 2=KILL ALL
        # Resolved on button release; long press (≥ LONG_PRESS_S) → KILL ALL
        self._emerg_state      = 0
        self._emerg_press_time: float | None = None

        # Publish initial latched states so rqt reflects correct state on startup
        self._hw_arm_pub.publish(String(data="DISARMED"))
        self._hw_emerg_pub.publish(String(data="OK"))

        self.get_logger().info("Button status node started — publishing to /button_status_markers")

    # ── gpio read ──────────────────────────────────────────────────────────────
    def _read_gpio(self) -> None:
        for pin in PINS:
            new = self._lines[pin].get_value()
            if new != self._states[pin]:
                name = PINS[pin]
                self.get_logger().info(
                    f"{name} {'PRESSED' if new == 0 else 'RELEASED'} (GPIO{pin})"
                )

                if pin == 5:                         # ARM — active while held
                    if new == 0:                     # pressed → ARM all drones
                        self._publish_command_all("COMMAND_ARM")
                        self._hw_arm_pub.publish(String(data="ARMED"))
                    else:                            # released → DISARM all drones
                        self._publish_command_all("COMMAND_DISARM")
                        self._hw_arm_pub.publish(String(data="DISARMED"))

                elif pin == 6:                       # MISSION — cycle on press, armed drones only
                    if new == 0:                     # pressed
                        self._mission_idx = (self._mission_idx + 1) % len(MISSION_STATES)
                        _, label = MISSION_STATES[self._mission_idx]
                        self._publish_command_armed_only("COMMAND_MISSION_START")
                        self._hw_mission_pub.publish(String(data=label))
                    else:                            # released
                        self._hw_mission_pub.publish(String(data="RELEASED"))

                elif pin == 13:                      # EMERG — falling edge arms timer; rising edge resolves
                    if new == 0:                     # pressed: record time, auto-kill fires in polling loop
                        self._emerg_press_time = time.monotonic()
                    else:                            # released
                        if self._emerg_state != 0:   # any press from active state → reset to OK
                            self._emerg_state = 0
                            self._hw_emerg_pub.publish(String(data="OK"))
                            self.get_logger().info("EMERG reset to OK")
                        elif self._emerg_press_time is not None:
                            # Released before auto-kill threshold → emergency land
                            self._emerg_state = 1
                            self._publish_command_all("COMMAND_ELAND")
                            self._hw_emerg_pub.publish(String(data="EMERGENCY_LAND"))
                            self.get_logger().info("EMERG → EMERG LAND (short press)")
                        self._emerg_press_time = None

                self._states[pin] = new

        # Auto-fire KILL while button still held — mirrors rqt QTimer tick behaviour
        if (self._emerg_press_time is not None
                and self._emerg_state == 0
                and time.monotonic() - self._emerg_press_time >= LONG_PRESS_S):
            self._emerg_state = 2
            self._publish_command_all("COMMAND_KILL")
            self._hw_emerg_pub.publish(String(data="KILL_ALL"))
            self.get_logger().info("EMERG → KILL ALL (auto-fire after 3 s hold)")
            self._emerg_press_time = None  # prevent re-triggering on continued hold

    # ── drone state callback ───────────────────────────────────────────────────
    def _drone_state_callback(self, msg: String, drone_id: int) -> None:
        self._drone_states[drone_id] = msg.data.lower()

    # ── command helpers ────────────────────────────────────────────────────────
    def _publish_command_all(self, command: str) -> None:
        msg = String(data=command)
        for pub in self._drone_command_pubs:
            pub.publish(msg)
        self.get_logger().info(f"HW → all drones: {command}")

    def _publish_command_armed_only(self, command: str) -> None:
        """Send command only to drones in 'armed' state — mirrors rqt _mission_all() gate."""
        msg = String(data=command)
        sent = []
        for i, pub in enumerate(self._drone_command_pubs, start=1):
            if self._drone_states.get(i) == "armed":
                pub.publish(msg)
                sent.append(i)
        if sent:
            self.get_logger().info(f"HW → armed drones {sent}: {command}")
        else:
            self.get_logger().warn("HW MISSION blocked — no drones in armed state")

    # ── marker build ───────────────────────────────────────────────────────────
    def _build_markers(self) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        arm_active = (self._states[5] == 0)   # held — active while pressed

        m_color, m_label = MISSION_STATES[self._mission_idx]
        e_color, e_label = EMERG_STATES[self._emerg_state]

        slots = [
            # (y_pos, panel_color, label)
            (STATUS_Y_BASE,
             RED if arm_active else DIMMED,
             "ARMED" if arm_active else "DISARMED"),
            (STATUS_Y_BASE - STATUS_SPACING,
             m_color, m_label),
            (STATUS_Y_BASE - 2 * STATUS_SPACING,
             e_color, e_label),
        ]

        markers = []
        for idx, (y, color, label) in enumerate(slots):
            panel = make_panel(idx * 2,     y, color)
            text  = make_label(idx * 2 + 1, y, label)
            panel.header.stamp = now
            text.header.stamp  = now
            markers += [panel, text]

        ma = MarkerArray()
        ma.markers = markers
        return ma

    # ── timer callback ─────────────────────────────────────────────────────────
    def _publish(self) -> None:
        self._read_gpio()
        self._pub.publish(self._build_markers())

    def destroy_node(self) -> None:
        for line in self._lines.values():
            line.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ButtonStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
