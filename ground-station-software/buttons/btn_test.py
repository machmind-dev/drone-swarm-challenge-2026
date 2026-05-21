#!/usr/bin/env python3
"""
Physical button node for Global Flight Controls.

  ARM:     dead-man's switch — hold → COMMAND_ARM + CONFIG_SOURCE_GCS,
                               release → COMMAND_DISARM immediately.
           (Physical buttons are hold-to-arm by design; rqt panel uses toggle
            because software buttons cannot be held.)
  MISSION: press             — COMMAND_MISSION_START to armed drones only,
                               with 400 ms ARM→MISSION guard (same as panel).
  EMERG:   press/hold        — short release → COMMAND_ELAND;
                               hold ≥ 3 s    → COMMAND_KILL.

On startup publishes CONFIG_SOURCE_GCS to every drone (same as panel).
"""

import math
import time

import gpiod
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# ── GPIO ──────────────────────────────────────────────────────────────────────
GPIO_CHIP = "gpiochip4"
PINS = {
    5:  "ARM",
    6:  "MISSION",
    13: "EMERG",
}

POLL_HZ               = 20    # GPIO poll rate (raised for snappier debounce resolution)
LONG_PRESS_S          = 3.0   # seconds — threshold for KILL ALL
DRONE_COUNT           = 5
ARM_MISSION_GUARD_S   = 0.400 # 400 ms — same as panel's ARM_MISSION_GUARD_MS
MISSION_PENDING_TIMEOUT_S = 8.0  # same as panel's MAX_WAIT_S
DEBOUNCE_S            = 0.050 # 50 ms — suppress contact bounce on physical buttons


class ButtonStatusNode(Node):
    def __init__(self):
        super().__init__("button_status_node")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Per-drone command publishers
        self._drone_command_pubs = [
            self.create_publisher(String, f"/gcs/drone_{i}/command", 10)
            for i in range(1, DRONE_COUNT + 1)
        ]

        # Per-drone config publishers — used to send CONFIG_SOURCE_GCS
        self._drone_config_pubs = [
            self.create_publisher(String, f"/gcs/drone_{i}/config", 10)
            for i in range(1, DRONE_COUNT + 1)
        ]

        # Per-drone control publishers — setpoints before mission start
        self._drone_control_pubs = [
            self.create_publisher(PoseStamped, f"/gcs/drone_{i}/control", 10)
            for i in range(1, DRONE_COUNT + 1)
        ]

        # Per-drone vision pose cache
        self._drone_poses: dict[int, PoseStamped] = {}
        for i in range(1, DRONE_COUNT + 1):
            self.create_subscription(
                PoseStamped, f"/drone_{i}/vision_pose",
                lambda msg, drone_id=i: self._vision_pose_callback(msg, drone_id), 10,
            )

        # Latched state topics — rqt panel subscribes to mirror HW state
        self._hw_arm_pub     = self.create_publisher(String, "/gcs/hw_arm_state",     qos)
        self._hw_mission_pub = self.create_publisher(String, "/gcs/hw_mission_state", qos)
        self._hw_emerg_pub   = self.create_publisher(String, "/gcs/hw_emerg_state",   qos)

        # Drone state tracking
        self._drone_states: dict[int, str] = {}
        for i in range(1, DRONE_COUNT + 1):
            self.create_subscription(
                String, f"/drone_{i}/state",
                lambda msg, drone_id=i: self._drone_state_callback(msg, drone_id), 10,
            )

        # GPIO setup
        self._chip = gpiod.Chip(GPIO_CHIP)
        self._lines: dict[int, gpiod.Line] = {}
        self._gpio_states: dict[int, int] = {}
        self._last_transition: dict[int, float] = {}  # monotonic time of last accepted edge per pin

        for pin, name in PINS.items():
            line = self._chip.get_line(pin)
            line.request(
                consumer=f"button_{name.lower()}",
                type=gpiod.LINE_REQ_DIR_IN,
                flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
            )
            self._lines[pin] = line
            self._gpio_states[pin] = line.get_value()
            self._last_transition[pin] = 0.0
            self.get_logger().info(f"GPIO {pin} ({name}) initial state: {self._gpio_states[pin]}")

        self._arm_sent_times: dict[int, float] = {}   # monotonic time ARM was sent per drone

        # Pending mission retry — mirrors panel's _mission_all_attempt loop
        self._mission_pending: bool = False
        self._mission_pending_since: float = 0.0
        self._mission_sent_ids: set = set()

        # Emergency state
        self._emerg_state          = 0   # 0=OK, 1=ELAND, 2=KILL
        self._emerg_press_time:    float | None = None
        self._emerg_countdown_last = -1

        # Publish initial latched states
        self._hw_arm_pub.publish(String(data="DISARMED"))
        self._hw_emerg_pub.publish(String(data="OK"))

        # Send CONFIG_SOURCE_GCS to all drones on startup — same as panel's
        # gcs_radio.setChecked(True) after connect()
        self._publish_config_all("CONFIG_SOURCE_GCS")
        self.get_logger().info("Startup: CONFIG_SOURCE_GCS sent to all drones")

        self._timer = self.create_timer(1.0 / POLL_HZ, self._tick)
        self.get_logger().info("Button node started")

    # ── timer callback ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        self._read_gpio()
        self._check_pending_mission()

    # ── pending mission retry — mirrors panel's _mission_all_attempt ──────────
    def _check_pending_mission(self) -> None:
        if not self._mission_pending:
            return
        elapsed = time.monotonic() - self._mission_pending_since
        if elapsed > MISSION_PENDING_TIMEOUT_S:
            self._mission_pending = False
            self._mission_sent_ids = set()
            self.get_logger().warn("MISSION pending timed out — no drones confirmed armed")
            return
        if self._try_send_mission():
            self._mission_pending = False

    def _try_send_mission(self) -> bool:
        """
        Send COMMAND_MISSION_START to drones that are armed and past the ARM guard.
        Returns True if all arm_candidates received MISSION_START.
        Mirrors panel's _mission_all_attempt logic.
        """
        arm_candidates = set(self._arm_sent_times.keys())
        if not arm_candidates:
            return False

        now = time.monotonic()
        newly_sent, still_waiting = [], []

        for i in sorted(arm_candidates):
            if i in self._mission_sent_ids:
                continue
            state = self._drone_states.get(i)
            if state in {"mission", "landing", "returning_home"}:
                self._mission_sent_ids.add(i)
                continue
            if state != "armed":
                still_waiting.append(i)
                continue
            # ARM→MISSION guard — identical to panel's ARM_MISSION_GUARD_MS
            elapsed_ms = (now - self._arm_sent_times.get(i, 0)) * 1000
            if elapsed_ms < ARM_MISSION_GUARD_S * 1000:
                still_waiting.append(i)
                continue
            self._publish_mission_setpoint(i)
            self._publish_command(i, "COMMAND_MISSION_START")
            self._mission_sent_ids.add(i)
            newly_sent.append(i)

        if newly_sent:
            self.get_logger().info(f"MISSION sent to D{newly_sent}")

        pending = arm_candidates - self._mission_sent_ids
        return len(pending) == 0

    # ── gpio read ──────────────────────────────────────────────────────────────
    def _read_gpio(self) -> None:
        now_t = time.monotonic()
        for pin in PINS:
            new = self._lines[pin].get_value()
            if new != self._gpio_states[pin]:
                # Debounce — ignore edges that arrive within 50 ms of the last accepted one
                if (now_t - self._last_transition[pin]) < DEBOUNCE_S:
                    self._gpio_states[pin] = new  # track level silently, don't act
                    continue
                self._last_transition[pin] = now_t
                name = PINS[pin]
                self.get_logger().info(
                    f"{name} {'PRESSED' if new == 0 else 'RELEASED'} (GPIO{pin})"
                )

                if pin == 5:   # ARM — dead-man's switch (both edges)
                    if new == 0:   # pressed → ARM
                        self._handle_arm_press()
                    else:          # released → DISARM immediately
                        self._handle_arm_release()

                elif pin == 6:   # MISSION — press to start
                    if new == 0:
                        self._handle_mission_press()
                    else:
                        self._hw_mission_pub.publish(String(data="RELEASED"))

                elif pin == 13:   # EMERG
                    if new == 0:
                        self._emerg_press_time = time.monotonic()
                        self._emerg_countdown_last = -1
                    else:
                        if self._emerg_state != 0:
                            self._emerg_state = 0
                            self._hw_emerg_pub.publish(String(data="OK"))
                            self.get_logger().info("EMERG reset to OK")
                        elif self._emerg_press_time is not None:
                            self._emerg_state = 1
                            self._publish_command_all("COMMAND_ELAND")
                            self._hw_emerg_pub.publish(String(data="EMERGENCY_LAND"))
                            self.get_logger().info("EMERG → EMERG LAND (short press)")
                        self._emerg_press_time = None

                self._gpio_states[pin] = new

        # EMERG countdown + auto-kill while button held
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

    # ── ARM dead-man's switch ──────────────────────────────────────────────────
    def _handle_arm_press(self) -> None:
        """Button held — arm all drones and switch them to GCS control."""
        self._publish_config_all("CONFIG_SOURCE_GCS")
        self._publish_command_all("COMMAND_ARM")
        now = time.monotonic()
        for i in range(1, DRONE_COUNT + 1):
            self._arm_sent_times[i] = now
        self._hw_arm_pub.publish(String(data="ARMED"))
        self.get_logger().info("ARM held → ARMED (CONFIG_SOURCE_GCS + COMMAND_ARM sent)")

    def _handle_arm_release(self) -> None:
        """Button released — disarm all drones immediately, cancel any pending mission."""
        self._publish_command_all("COMMAND_DISARM")
        self._arm_sent_times.clear()
        self._mission_pending = False
        self._mission_sent_ids = set()
        self._hw_arm_pub.publish(String(data="DISARMED"))
        self.get_logger().info("ARM released → DISARMED")

    # ── MISSION press — mirrors panel's _mission_all ───────────────────────────
    def _handle_mission_press(self) -> None:
        if not self._arm_sent_times:
            self.get_logger().warn("MISSION blocked — ARM not active")
            self._hw_mission_pub.publish(String(data="RELEASED"))
            return
        self._mission_pending = True
        self._mission_pending_since = time.monotonic()
        self._mission_sent_ids = set()
        self._hw_mission_pub.publish(String(data="MISSION"))
        self.get_logger().info("MISSION pressed — dispatching with ARM guard…")
        # Attempt immediately; _check_pending_mission will retry each tick
        self._try_send_mission()

    # ── mission setpoint helper ────────────────────────────────────────────────
    def _publish_mission_setpoint(self, drone_id: int) -> None:
        pose = self._drone_poses.get(drone_id)
        if pose is None:
            self.get_logger().warn(f"D{drone_id} — no vision pose, skipping setpoint")
            return
        distance = float(drone_id)
        yaw = self._yaw_from_pose(pose)
        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.pose.position.x = pose.pose.position.x + math.cos(yaw) * distance
        sp.pose.position.y = pose.pose.position.y + math.sin(yaw) * distance
        sp.pose.position.z = pose.pose.position.z
        sp.pose.orientation.w = 1.0
        self._drone_control_pubs[drone_id - 1].publish(sp)
        self.get_logger().info(
            f"D{drone_id} → /gcs/drone_{drone_id}/control : "
            f"({sp.pose.position.x:.2f}, {sp.pose.position.y:.2f}) [{distance:.0f} m fwd]"
        )

    # ── drone callbacks ────────────────────────────────────────────────────────
    def _drone_state_callback(self, msg: String, drone_id: int) -> None:
        self._drone_states[drone_id] = msg.data.lower()

    def _vision_pose_callback(self, msg: PoseStamped, drone_id: int) -> None:
        self._drone_poses[drone_id] = msg

    # ── helpers ────────────────────────────────────────────────────────────────
    @staticmethod
    def _yaw_from_pose(pose: PoseStamped) -> float:
        q = pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _publish_command_all(self, command: str) -> None:
        for i, pub in enumerate(self._drone_command_pubs, start=1):
            msg = String(data=command)
            pub.publish(msg)
            self.get_logger().info(f"D{i} → /gcs/drone_{i}/command : {command}")

    def _publish_command(self, drone_id: int, command: str) -> None:
        msg = String(data=command)
        self._drone_command_pubs[drone_id - 1].publish(msg)
        self.get_logger().info(f"D{drone_id} → /gcs/drone_{drone_id}/command : {command}")

    def _publish_config_all(self, config: str) -> None:
        for i, pub in enumerate(self._drone_config_pubs, start=1):
            msg = String(data=config)
            pub.publish(msg)
            self.get_logger().info(f"D{i} → /gcs/drone_{i}/config : {config}")

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
