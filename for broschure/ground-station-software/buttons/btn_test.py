#!/usr/bin/env python3
"""
Physical button node for Global Flight Controls.

Reads GPIO buttons and publishes commands directly to all drones plus latched
HW state topics consumed by the rqt GCS panel.

ARM:   hold button  — COMMAND_ARM while held, COMMAND_DISARM on release.
MISSION: press      — COMMAND_MISSION_START to armed drones only.
EMERG:   press/hold — short release → COMMAND_ELAND; hold ≥ 3 s → COMMAND_KILL.
"""

import math
import time

import gpiod
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

# ── GPIO ──────────────────────────────────────────────────────────────────────
GPIO_CHIP = "gpiochip4"
PINS = {
    5:  "ARM",
    6:  "MISSION",
    13: "EMERG",
}

POLL_HZ      = 10    # GPIO poll rate
LONG_PRESS_S = 3.0   # seconds — threshold for KILL ALL (matches rqt EMERGENCY_HOLD_SECONDS)
DRONE_COUNT  = 5     # number of drones to command

# ── Mission states (cycle on each press) ─────────────────────────────────────
MISSION_STATES = ["MISSION"]


class ButtonStatusNode(Node):
    def __init__(self):
        super().__init__("button_status_node")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

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

        # Mission cycles through states on each press
        self._mission_idx = 0

        # Emergency: 0=OK, 1=EMERG LAND, 2=KILL ALL
        self._emerg_state          = 0
        self._emerg_press_time:    float | None = None
        self._emerg_countdown_last = -1

        # Publish initial latched states so rqt reflects correct state on startup
        self._hw_arm_pub.publish(String(data="DISARMED"))
        self._hw_emerg_pub.publish(String(data="OK"))

        self._timer = self.create_timer(1.0 / POLL_HZ, self._tick)
        self.get_logger().info("Button node started")

    # ── timer callback ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        self._read_gpio()

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
                        label = MISSION_STATES[self._mission_idx]
                        self._publish_command_armed_only("COMMAND_MISSION_START")
                        self._hw_mission_pub.publish(String(data=label))
                    else:                            # released
                        self._hw_mission_pub.publish(String(data="RELEASED"))

                elif pin == 13:                      # EMERG — falling edge arms timer; rising edge resolves
                    if new == 0:                     # pressed: record time, auto-kill fires in polling loop
                        self._emerg_press_time = time.monotonic()
                        self._emerg_countdown_last = -1  # reset so first poll publishes KILL_IN_3
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

        # Countdown + auto-fire KILL while button held — mirrors rqt QTimer tick behaviour
        if self._emerg_press_time is not None and self._emerg_state == 0:
            elapsed = time.monotonic() - self._emerg_press_time
            if elapsed >= LONG_PRESS_S:
                self._emerg_state = 2
                self._publish_command_all("COMMAND_KILL")
                self._hw_emerg_pub.publish(String(data="KILL_ALL"))
                self.get_logger().info("EMERG → KILL ALL (auto-fire after 3 s hold)")
                self._emerg_press_time = None
                self._emerg_countdown_last = -1
            else:
                countdown = math.ceil(LONG_PRESS_S - elapsed)
                if countdown != self._emerg_countdown_last:
                    self._emerg_countdown_last = countdown
                    self._hw_emerg_pub.publish(String(data=f"KILL_IN_{countdown}"))

    # ── drone state callback ───────────────────────────────────────────────────
    def _drone_state_callback(self, msg: String, drone_id: int) -> None:
        self._drone_states[drone_id] = msg.data.lower()

    # ── command helpers ────────────────────────────────────────────────────────
    def _publish_command_all(self, command: str) -> None:
        for i, pub in enumerate(self._drone_command_pubs, start=1):
            msg = String()
            msg.data = command
            pub.publish(msg)
            self.get_logger().info(f"D{i} → /gcs/drone_{i}/command : {command}")

    def _publish_command_armed_only(self, command: str) -> None:
        """Send command only to drones in 'armed' state — mirrors rqt _mission_all() gate."""
        sent = []
        for i, pub in enumerate(self._drone_command_pubs, start=1):
            if self._drone_states.get(i) == "armed":
                msg = String()
                msg.data = command
                pub.publish(msg)
                self.get_logger().info(f"D{i} → /gcs/drone_{i}/command : {command}")
                sent.append(i)
        if not sent:
            self.get_logger().warn("HW MISSION blocked — no drones in armed state")

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
