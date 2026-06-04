# gcs_button_panel.py
import math
import os
import time
from collections import deque
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Int8
from visualization_msgs.msg import Marker, MarkerArray

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QGroupBox,
    QRadioButton, QButtonGroup
)
from rqt_gui_py.plugin import Plugin


class GcsButtonPanel(Plugin):

    DRONE_COUNT = 5
    ARM_MISSION_GUARD_MS = 400   # minimum ms between ARM and MISSION_START
    DRONE_OFFLINE_TIMEOUT_S = 3  # seconds without a state message → OFFLINE
    VERSION = "1.3.9"

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("GcsButtonPanel")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node: Node = rclpy.create_node("machmind_rqt_buttons")

        self.drone_states = {}
        self.drone_roles = {}
        self.drone_battery = {}
        self.drone_last_seen: dict[int, float] = {}  # monotonic time of last state message per drone
        self.ui_refs = {}
        self._arm_sent_times: dict[int, float] = {}  # monotonic time when ARM was sent per drone
        self._mission_all_active: bool = False       # True while _mission_all_attempt loop is running
        self._mission_sent_ids: set = set()          # drones that received MISSION_START this cycle
        self._mission_start_time: float = 0.0        # monotonic time _mission_all was first called
        self.command_publishers = {}
        self.config_publishers = {}
        self.state_subscribers = {}
        self.role_subscribers = {}
        self.battery_subscribers = {}

        self._sw_emerg_active = False   # True after ELAND fired; shows button as active

        # Box label tracking — subscribe to /visualization_marker to detect firmware CUBEs
        self._box_positions: dict[int, tuple] = {}  # box_id -> (x, y, z, ns)

        # Scene publishers
        self.marker_pub = self.node.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_array_pub = self.node.create_publisher(MarkerArray, "/visualization_marker_array", 10)
        self.node.create_subscription(
            Marker, "/visualization_marker", self._marker_cb, 10)
        self.team_area_pub = self.node.create_publisher(String, "/gcs/system/team_area", 10)
        tc_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.team_color_pub = self.node.create_publisher(String, "/gcs/system/team_color", tc_qos)

        # Drone trails — nav_msgs/Path, max 300 poses per drone (~60-90 s at ~3-5 Hz)
        _TRAIL_MAX = 300
        self._trail_deques = {}
        self._trail_pubs   = {}
        self._trail_subs   = {}
        for i in range(1, self.DRONE_COUNT + 1):
            self._trail_deques[i] = deque(maxlen=_TRAIL_MAX)
            self._trail_pubs[i]   = self.node.create_publisher(
                Path, f"/drone_{i}/trail", 10)
            self._trail_subs[i]   = self.node.create_subscription(
                PoseStamped, f"/drone_{i}/vision_pose",
                lambda msg, did=i: self._trail_cb(msg, did), 10)

        # Per-drone flat arrow markers published on /drone_N/heading_marker
        # Add these to RViz as Marker display, topic /drone_N/heading_marker
        self._drone_marker_pubs = {
            i: self.node.create_publisher(Marker, f"/drone_{i}/heading_marker", 10)
            for i in range(1, self.DRONE_COUNT + 1)
        }

        for i in range(1, self.DRONE_COUNT + 1):
            self.command_publishers[i] = self.node.create_publisher(
                String, f"/gcs/drone_{i}/command", 10
            )
            self.config_publishers[i] = self.node.create_publisher(
                String, f"/gcs/drone_{i}/config", 10
            )
            self.state_subscribers[i] = self.node.create_subscription(
                String, f"/drone_{i}/state",
                lambda msg, drone_id=i: self._state_callback(msg, drone_id), 10
            )
            self.role_subscribers[i] = self.node.create_subscription(
                String, f"/drone_{i}/role",
                lambda msg, drone_id=i: self._role_callback(msg, drone_id), 10
            )
            self.battery_subscribers[i] = self.node.create_subscription(
                Int8, f"/drone_{i}/battery",
                lambda msg, drone_id=i: self._battery_callback(msg, drone_id), 10
            )

        self._widget = QWidget()
        self._widget.setWindowTitle("Mach Mind GCS")
        self._widget.setStyleSheet("""
            QWidget { background-color: #1e1e1e; color: #d0d0d0; font-family: Arial; font-size: 10px; }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 6px;
                padding: 4px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 6px;
                padding: 0 3px 0 3px;
            }
        """)

        main_layout = QVBoxLayout()
        main_layout.setSpacing(4)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.addWidget(self._build_global_controls())
        main_layout.addWidget(self._build_drones_group())
        main_layout.addWidget(self._build_scene_management())

        version_row = QHBoxLayout()
        version_row.setContentsMargins(0, 2, 2, 0)
        version_row.addStretch()
        version_label = QLabel(f"v{self.VERSION}")
        version_label.setStyleSheet("color: #ffffff; font-size: 8px;")
        version_row.addWidget(version_label)
        main_layout.addLayout(version_row)

        self._widget.setLayout(main_layout)

        # Hardware button override — subscribe after widget is built so button refs exist
        self._hw_arm_locked   = False
        self._hw_emerg_locked = False
        hw_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.node.create_subscription(String, "/gcs/hw_arm_state",     self._hw_arm_callback,     hw_qos)
        self.node.create_subscription(String, "/gcs/hw_mission_state", self._hw_mission_callback, hw_qos)
        self.node.create_subscription(String, "/gcs/hw_emerg_state",   self._hw_emerg_callback,   hw_qos)

        context.add_widget(self._widget)
        self._widget.raise_()
        self._widget.activateWindow()

        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.timer.start(50)

        # Heartbeat watchdog — marks drones OFFLINE when state messages stop
        self._online_timer = QTimer()
        self._online_timer.setInterval(1000)
        self._online_timer.timeout.connect(self._check_drone_staleness)
        self._online_timer.start()


    # ================= Global Controls =================
    def _build_global_controls(self):
        box = QGroupBox("Global Flight Controls")
        layout = QHBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(4, 4, 4, 4)

        base_style = """
            QPushButton {
                color: white;
                font-weight: bold;
                border-radius: 6px;
                font-size: 10px;
                min-height: 26px;
                padding: 4px 6px;
            }
            QPushButton:pressed { background-color: #555555; }
        """

        self.arm_all_btn = QPushButton("ARM ALL")
        self.arm_all_btn.setCheckable(True)
        self.arm_all_btn.setStyleSheet(base_style + """
            QPushButton { background-color: #3a3a3a; }
            QPushButton:checked { background-color: #b00020; }
        """)
        self.arm_all_btn.clicked.connect(self._arm_all_toggle)
        layout.addWidget(self.arm_all_btn)

        self.mission_all_btn = QPushButton("MISSION ALL")
        self.mission_all_btn.setStyleSheet(base_style + "QPushButton { background-color: #3a3a3a; }")
        self.mission_all_btn.clicked.connect(self._mission_all)
        layout.addWidget(self.mission_all_btn)
        self._mission_all_base_style = base_style

        self.emergency_all_btn = QPushButton("EMERGENCY ALL\nE-LAND")
        self.emergency_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff8c00;
                color: black;
                font-weight: bold;
                border-radius: 6px;
                font-size: 10px;
                min-height: 26px;
                padding: 4px 6px;
                border: 1px solid #d97a00;
            }
            QPushButton:pressed {
                background-color: #cc5500;
                color: white;
            }
        """)
        self.emergency_all_btn.clicked.connect(self._emergency_all)
        layout.addWidget(self.emergency_all_btn)

        box.setLayout(layout)
        return box

    # ================= Drones =================
    def _build_drones_group(self):
        box = QGroupBox("Drones")
        layout = QGridLayout()
        layout.setSpacing(4)
        layout.setContentsMargins(4, 4, 4, 4)

        for col in range(self.DRONE_COUNT):
            layout.addWidget(self._build_single_drone_panel(col + 1), 0, col)

        box.setLayout(layout)
        return box

    def _build_single_drone_panel(self, drone_id: int):
        container = QWidget()
        layout = QVBoxLayout()
        layout.setSpacing(3)
        layout.setContentsMargins(2, 2, 2, 2)

        strip = QLabel()
        strip.setFixedHeight(4)
        strip.setStyleSheet("background-color: gray; border-radius: 2px;")
        layout.addWidget(strip)

        title = QLabel(f"D{drone_id}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-weight:bold; font-size:10px;")
        layout.addWidget(title)

        role_label = QLabel("IDLE")
        role_label.setAlignment(Qt.AlignCenter)
        role_label.setStyleSheet("background-color:#444; border-radius:4px; padding:1px; font-size:9px;")
        layout.addWidget(role_label)

        state_label = QLabel("DISARMED")
        state_label.setAlignment(Qt.AlignCenter)
        state_label.setStyleSheet("font-size:9px;")
        layout.addWidget(state_label)

        btn_style = """
            QPushButton {
                background-color: #3a3a3a;
                color: white;
                border-radius: 5px;
                font-size: 9px;
                min-height: 22px;
                padding: 2px;
            }
            QPushButton:pressed { background-color: #555; }
        """

        arm_btn = QPushButton("ARM")
        arm_btn.setCheckable(True)
        arm_btn.setStyleSheet(btn_style + "QPushButton:checked { background-color: #b00020; }")
        arm_btn.clicked.connect(partial(self._send_arm_toggle, drone_id))
        layout.addWidget(arm_btn)

        mission_btn = QPushButton("MISSION")
        mission_btn.setStyleSheet(btn_style + """
            QPushButton:enabled  { background-color: #2d6a4f; }
            QPushButton:disabled { background-color: #3a3a3a; }
        """)
        mission_btn.clicked.connect(partial(self._send_command, drone_id, "COMMAND_MISSION_START"))
        mission_btn.setEnabled(False)
        layout.addWidget(mission_btn)

        home_btn = QPushButton("HOME")
        home_btn.setStyleSheet(btn_style + """
            QPushButton:enabled  { background-color: #2d6a4f; }
            QPushButton:disabled { background-color: #3a3a3a; }
        """)
        home_btn.clicked.connect(partial(self._send_command, drone_id, "COMMAND_RETURN_HOME"))
        home_btn.setEnabled(False)
        layout.addWidget(home_btn)

        emergency_btn = QPushButton("EMERG")
        emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff8c00;
                color: black;
                font-weight: bold;
                border-radius: 5px;
                font-size: 9px;
                min-height: 22px;
                padding: 2px;
                border: 1px solid #d97a00;
            }
            QPushButton:pressed {
                background-color: #ff3b30;
                color: white;
            }
        """)
        emergency_btn.clicked.connect(partial(self._publish, drone_id, "COMMAND_ELAND"))
        layout.addWidget(emergency_btn)

        small_toggle_style = """
            QPushButton {
                background-color: #3a3a3a;
                color: white;
                border-radius: 5px;
                font-size: 9px;
                min-height: 20px;
                max-height: 20px;
                padding: 1px;
            }
            QPushButton:checked {
                background-color: #2d6a4f;
                color: white;
            }
            QPushButton:pressed {
                background-color: #555;
            }
        """

        camera_btn = QPushButton("CAM")
        camera_btn.setCheckable(True)
        camera_btn.setStyleSheet(small_toggle_style)
        camera_btn.clicked.connect(partial(self._send_config_toggle, drone_id, camera_btn,
                                           "CONFIG_CAMERA_ENABLE", "CONFIG_CAMERA_DISABLE"))
        layout.addWidget(camera_btn)

        vision_btn = QPushButton("ARUCO EKF")
        vision_btn.setCheckable(True)
        vision_btn.setChecked(False)
        vision_btn.setStyleSheet(small_toggle_style)
        vision_btn.clicked.connect(partial(self._send_config_toggle, drone_id, vision_btn,
                                           "CONFIG_VISION_ENABLE", "CONFIG_VISION_DISABLE"))
        layout.addWidget(vision_btn)

        rc_radio = QRadioButton("RC")
        gcs_radio = QRadioButton("GCS")
        rc_radio.setStyleSheet("font-size:9px;")
        gcs_radio.setStyleSheet("font-size:9px;")
        group = QButtonGroup(container)
        group.addButton(rc_radio)
        group.addButton(gcs_radio)
        rc_radio.toggled.connect(partial(self._send_config_source, drone_id, "CONFIG_SOURCE_RC"))
        gcs_radio.toggled.connect(partial(self._send_config_source, drone_id, "CONFIG_SOURCE_GCS"))
        gcs_radio.setChecked(True)   # after connect so toggled fires and CONFIG_SOURCE_GCS is published on startup

        radio_layout = QHBoxLayout()
        radio_layout.setSpacing(2)
        radio_layout.setContentsMargins(0, 0, 0, 0)
        radio_layout.addWidget(rc_radio)
        radio_layout.addWidget(gcs_radio)
        layout.addLayout(radio_layout)

        self.ui_refs[drone_id] = {
            "arm": arm_btn,
            "mission": mission_btn,
            "home": home_btn,
            "state": state_label,
            "role": role_label,
            "strip": strip,
            "emergency": emergency_btn,
            "title": title,
            "vision": vision_btn,
        }

        container.setLayout(layout)
        return container

    # ================= Drone Trail =================
    def _trail_cb(self, msg: PoseStamped, drone_id: int):
        self._trail_deques[drone_id].append(msg)
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = msg.header.stamp
        path.poses = list(self._trail_deques[drone_id])
        self._trail_pubs[drone_id].publish(path)

        # Heading triangle — flat TRIANGLE_LIST, apex (widest angle) faces heading.
        # Camera +Z = optical axis = forward; extract yaw, compute world-frame vertices.
        q = msg.pose.orientation
        fwd_x = 2.0 * (q.x * q.z + q.y * q.w)
        fwd_y = 2.0 * (q.y * q.z - q.x * q.w)
        yaw_rad = math.atan2(fwd_y, fwd_x)
        fx = math.cos(yaw_rad);  fy =  math.sin(yaw_rad)   # forward unit vector
        lx = -math.sin(yaw_rad); ly =  math.cos(yaw_rad)   # left unit vector
        x = msg.pose.position.x;  y = msg.pose.position.y;  z = msg.pose.position.z
        front_d = 0.18;  back_d = 0.06;  side_d = 0.12
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "drone_heading"
        m.id = drone_id
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = 0.2;  m.color.g = 0.6;  m.color.b = 1.0;  m.color.a = 0.95
        m.lifetime.sec = 2
        m.points.append(Point(x=x + front_d*fx,            y=y + front_d*fy,            z=z))
        m.points.append(Point(x=x - back_d*fx + side_d*lx, y=y - back_d*fy + side_d*ly, z=z))
        m.points.append(Point(x=x - back_d*fx - side_d*lx, y=y - back_d*fy - side_d*ly, z=z))
        self._drone_marker_pubs[drone_id].publish(m)

    # ================= Scene Management =================
    def _build_scene_management(self):
        box = QGroupBox("Scene Management")
        layout = QHBoxLayout()
        layout.setSpacing(6)
        layout.setContentsMargins(4, 4, 4, 4)

        style = """
            QPushButton {
                background-color: #2f4f6f;
                color: white;
                font-weight: bold;
                border-radius: 6px;
                font-size: 10px;
                padding: 4px 8px;
                min-height: 24px;
            }
            QPushButton:pressed { background-color: #3f6f9f; }
        """

        lh_btn = QPushButton("LH Scene")
        lh_btn.setStyleSheet(style)
        lh_btn.clicked.connect(self._publish_lh_scene)
        layout.addWidget(lh_btn)

        rh_btn = QPushButton("RH Scene")
        rh_btn.setStyleSheet(style)
        rh_btn.clicked.connect(self._publish_rh_scene)
        layout.addWidget(rh_btn)

        axis_btn = QPushButton("Display Axis")
        axis_btn.setStyleSheet(style)
        axis_btn.clicked.connect(self._publish_axis_markers)
        layout.addWidget(axis_btn)

        box.setLayout(layout)
        return box

    # ================= Emergency Logic =================
    def _emergency_all(self):
        if self._sw_emerg_active:
            self._sw_emerg_active = False
            self._set_emerg_btn_style("idle")
            self.emergency_all_btn.setText("EMERGENCY ALL\nE-LAND")
            return
        self._sw_emerg_active = True
        self._set_emerg_btn_style("eland")
        self.emergency_all_btn.setText("EMERGENCY LAND (ACTIVE)")
        for drone_id in range(1, self.DRONE_COUNT + 1):
            self._publish(drone_id, "COMMAND_ELAND")

    def _set_emerg_btn_style(self, state: str):
        styles = {
            "idle":  ("#ff8c00", "#d97a00", "black"),
            "eland": ("#cc5500", "#993d00", "white"),
        }
        bg, border, fg = styles.get(state, styles["idle"])
        self.emergency_all_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: {fg};
                font-weight: bold;
                border-radius: 6px;
                font-size: 10px;
                min-height: 26px;
                padding: 4px 6px;
                border: 1px solid {border};
            }}
            QPushButton:pressed {{ background-color: #ff3b30; color: white; }}
        """)

    # ================= Axis Publishing =================
    def _publish_axis_markers(self):
        axes = [
            # (id, tip_x, tip_y, tip_z, r, g, b, label)
            (1, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, "X"),
            (2, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, "Y"),
            (3, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, "Z"),
        ]
        arr = MarkerArray()
        for mid, tx, ty, tz, r, g, b, label in axes:
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.ns = "axes"
            arrow.id = mid
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05
            arrow.scale.y = 0.10
            arrow.scale.z = 0.0
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 1.0
            arrow.points.append(Point(x=0.0, y=0.0, z=0.0))
            arrow.points.append(Point(x=tx,  y=ty,  z=tz))
            arr.markers.append(arrow)

            text = Marker()
            text.header.frame_id = "map"
            text.ns = "axes_labels"
            text.id = mid
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = tx * 1.15
            text.pose.position.y = ty * 1.15
            text.pose.position.z = tz * 1.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
            text.text = label
            arr.markers.append(text)

        # Arena corner coordinate labels
        corners = [
            (400, 0.0,  0.0,  "0,0"),
            (401, 20.0, 0.0,  "20,0"),
            (402, 0.0,  10.0, "0,10"),
            (403, 20.0, 10.0, "20,10"),
        ]
        for cid, cx, cy, label in corners:
            ct = Marker()
            ct.header.frame_id = "map"
            ct.ns = "corner_labels"
            ct.id = cid
            ct.type = Marker.TEXT_VIEW_FACING
            ct.action = Marker.ADD
            ct.pose.position.x = cx
            ct.pose.position.y = cy
            ct.pose.position.z = 0.3
            ct.pose.orientation.w = 1.0
            ct.scale.z = 0.4
            ct.color.r = ct.color.g = ct.color.b = 1.0
            ct.color.a = 0.85
            ct.text = label
            arr.markers.append(ct)

        self.marker_array_pub.publish(arr)
        self.node.get_logger().info("SCENE: Axis markers published → /visualization_marker_array")

    # ================= Scene Publishing =================
    def _publish_base_floor(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "arena"
        marker.id = 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 10.0
        marker.pose.position.y = 5.0
        marker.pose.position.z = -0.02
        marker.pose.orientation.w = 1.0
        marker.scale.x = 20.0
        marker.scale.y = 10.0
        marker.scale.z = 0.02
        marker.color.r = marker.color.g = marker.color.b = 0.22
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _zone_marker(self, x, color, mid):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "arena"
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = 5.0
        m.pose.position.z = -0.005
        m.pose.orientation.w = 1.0
        m.scale.x = 6.6667
        m.scale.y = 10.0
        m.scale.z = 0.01
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def _zone_label(self, x, text, mid):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "arena_labels"
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = 5.0
        m.pose.position.z = 0.3
        m.pose.orientation.w = 1.0
        m.scale.z = 0.6
        m.color.r = m.color.g = m.color.b = 0.68
        m.color.a = 1.0
        m.text = text
        return m

    def _launch_marker(self, mid, x, y):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "launch_points"
        m.id = mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05          # line width 5 cm
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.0, 1.0  # neutral yellow
        fx, fy = float(x), float(y)
        h = 0.35                   # half-arm length 35 cm
        # Two crossing diagonals at floor level
        m.points.append(Point(x=fx - h, y=fy - h, z=0.01))
        m.points.append(Point(x=fx + h, y=fy + h, z=0.01))
        m.points.append(Point(x=fx + h, y=fy - h, z=0.01))
        m.points.append(Point(x=fx - h, y=fy + h, z=0.01))
        return m

    def _publish_launch_points(self, coords, arr):
        for idx, (x, y) in enumerate(coords):
            arr.markers.append(self._launch_marker(300 + idx, x, y))

    def _monument_mesh(self, mid, x, y, qz, qw):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "monuments"
        m.id = mid
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.01
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        m.scale.x = m.scale.y = m.scale.z = 0.001
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.mesh_use_embedded_materials = True
        dae = os.path.join(
            os.path.expanduser("~"),
            "drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae"
        )
        m.mesh_resource = f"file://{dae}"
        return m

    def _monument_label(self, ns, mid, x, y, z, text):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = 0.4
        m.color.r = m.color.g = m.color.b = 0.95
        m.color.a = 1.0
        m.text = text
        return m

    def _publish_monuments(self):
        # SDC2026 arena pole layout (confirmed 2026-05-20):
        #   8 poles total; each pole carries TOP marker (z=4m) and BOTTOM marker (z=2m).
        #   y=10 wall: x=5 (6/14), x=10 (7/15), x=15 (8/16)  — face -Y (mesh rot=0°)
        #   y=0  wall: x=5 (4/12), x=10 (3/11), x=15 (2/10)  — face +Y (mesh rot=180°)
        #   x=0  end:  y=5 (5/13)                             — face +X (mesh rot=90°)
        #   x=20 end:  y=5 (1/9)                              — face -X (mesh rot=270°)
        arr = MarkerArray()

        # Meshes — y=10 wall, face -Y (rotation 0°: qz=0, qw=1)
        for mid, x in zip(range(1, 4), [5.0, 10.0, 15.0]):
            arr.markers.append(self._monument_mesh(mid, x, 10.0, 0.0, 1.0))
        # Meshes — y=0 wall, face +Y (rotation 180°: qz=1, qw=0)
        for mid, x in zip(range(4, 7), [5.0, 10.0, 15.0]):
            arr.markers.append(self._monument_mesh(mid, x, 0.0, 1.0, 0.0))
        # Mesh — x=0 end, face +X (rotation 90°)
        arr.markers.append(self._monument_mesh(7, 0.0, 5.0,  0.70710678, 0.70710678))
        # Mesh — x=20 end, face -X (rotation 270°)
        arr.markers.append(self._monument_mesh(8, 20.0, 5.0, -0.70710678, 0.70710678))
        # Delete stale mesh IDs from old 12-pole layout
        for stale_id in [9, 10, 11, 12]:
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "monuments"
            m.id = stale_id
            m.action = Marker.DELETE
            arr.markers.append(m)

        self.marker_array_pub.publish(arr)

        # Labels
        label_arr = MarkerArray()
        # y=10 wall (y=10.9): x=5/10/15, top/bottom marker IDs
        top_row = [
            (109, 110,  5.0, '6',  '14'),
            (111, 112, 10.0, '7',  '15'),
            (113, 114, 15.0, '8',  '16'),
        ]
        for top_id, bot_id, x, top_txt, bot_txt in top_row:
            label_arr.markers.append(self._monument_label("labels_top",    top_id, x, 10.9, 4.0, top_txt))
            label_arr.markers.append(self._monument_label("labels_bottom", bot_id, x, 10.9, 2.0, bot_txt))
        # y=0 wall (y=-0.9): x=5/10/15, top/bottom marker IDs
        bottom_row = [
            (101, 102,  5.0, '4',  '12'),
            (103, 104, 10.0, '3',  '11'),
            (105, 106, 15.0, '2',  '10'),
        ]
        for top_id, bot_id, x, top_txt, bot_txt in bottom_row:
            label_arr.markers.append(self._monument_label("labels_top",    top_id, x, -0.9, 4.0, top_txt))
            label_arr.markers.append(self._monument_label("labels_bottom", bot_id, x, -0.9, 2.0, bot_txt))
        # x=0 end pole (x=-0.9, y=5)
        label_arr.markers.append(self._monument_label("labels_top",    117, -0.9, 5.0, 4.0, '5'))
        label_arr.markers.append(self._monument_label("labels_bottom", 118, -0.9, 5.0, 2.0, '13'))
        # x=20 end pole (x=20.9, y=5)
        label_arr.markers.append(self._monument_label("labels_top",    119, 20.9, 5.0, 4.0, '1'))
        label_arr.markers.append(self._monument_label("labels_bottom", 120, 20.9, 5.0, 2.0, '9'))
        # Delete stale label IDs from old 12-pole layout
        for stale_ns, stale_id in [
            ("labels_top",    107), ("labels_bottom", 108),   # old y=0 x=16 pole
            ("labels_top",    115), ("labels_bottom", 116),   # old y=10 x=16 pole
            ("labels_top",    121), ("labels_bottom", 122),   # old x=20 y=6.66 pole
            ("labels_top",    123), ("labels_bottom", 124),   # old x=20 y=3.33 pole
        ]:
            m = Marker()
            m.header.frame_id = "map"
            m.ns = stale_ns
            m.id = stale_id
            m.action = Marker.DELETE
            label_arr.markers.append(m)

        self.marker_array_pub.publish(label_arr)

    def _publish_lh_scene(self):
        self._publish_base_floor()
        self._publish_monuments()
        msg = String(); msg.data = "LH"
        self.team_area_pub.publish(msg)
        color_msg = String(); color_msg.data = "red"
        self.team_color_pub.publish(color_msg)
        self.node.get_logger().info("SCENE: LH loaded → team_area=LH  team_color=red")
        arr = MarkerArray()
        arr.markers.append(self._zone_marker(3.3333, (0.8, 0.3, 0.3, 0.30), 101))
        arr.markers.append(self._zone_marker(10.0, (0.5, 0.5, 0.5, 0.25), 102))
        arr.markers.append(self._zone_marker(16.6667, (0.2, 0.4, 0.8, 0.30), 103))
        arr.markers.append(self._zone_label(3.3333, "TEAM-ZONE", 201))
        arr.markers.append(self._zone_label(10.0, "NO-MAN'S-LAND", 202))
        arr.markers.append(self._zone_label(16.6667, "OPPONENT-ZONE", 203))
        self._publish_launch_points(
            [(1,1),(1,2),(1,3),(1,4),(1,5)], arr)
        self.marker_array_pub.publish(arr)

    def _publish_rh_scene(self):
        self._publish_base_floor()
        self._publish_monuments()
        msg = String(); msg.data = "RH"
        self.team_area_pub.publish(msg)
        color_msg = String(); color_msg.data = "blue"
        self.team_color_pub.publish(color_msg)
        self.node.get_logger().info("SCENE: RH loaded → team_area=RH  team_color=blue")
        arr = MarkerArray()
        arr.markers.append(self._zone_marker(3.3333, (0.8, 0.3, 0.3, 0.30), 101))
        arr.markers.append(self._zone_marker(10.0, (0.5, 0.5, 0.5, 0.25), 102))
        arr.markers.append(self._zone_marker(16.6667, (0.2, 0.4, 0.8, 0.30), 103))
        arr.markers.append(self._zone_label(3.3333, "OPPONENT-ZONE", 201))
        arr.markers.append(self._zone_label(10.0, "NO-MAN'S-LAND", 202))
        arr.markers.append(self._zone_label(16.6667, "TEAM-ZONE", 203))
        self._publish_launch_points(
            [(19,9),(19,8),(19,7),(19,6),(19,5)], arr)
        self.marker_array_pub.publish(arr)

    # ================= Commands =================
    def _arm_all_toggle(self):
        command = "COMMAND_ARM" if self.arm_all_btn.isChecked() else "COMMAND_DISARM"
        self.arm_all_btn.setText("ARMING (SW)" if command == "COMMAND_ARM" else "ARM ALL")
        now = time.monotonic()
        for drone_id in range(1, self.DRONE_COUNT + 1):
            if command == "COMMAND_ARM":
                self._arm_sent_times[drone_id] = now
            else:
                self._arm_sent_times.pop(drone_id, None)
            self._publish(drone_id, command)

    def _mission_all(self):
        if self._mission_all_active:
            # Second click while dispatching in progress — cancel the retry loop
            self._mission_all_active = False
            self.mission_all_btn.setText("MISSION ALL")
            return

        arm_candidates = set(self._arm_sent_times.keys())
        if not arm_candidates:
            self.node.get_logger().warn("MISSION ALL: no ARM candidates — press ARM ALL first")
            return

        self._mission_all_active = True
        self._mission_sent_ids = set()
        self._mission_start_time = time.monotonic()
        self.mission_all_btn.setText("MISSION ALL (dispatching…)")
        self._mission_all_attempt()

    def _mission_all_attempt(self):
        """Retry loop: sends MISSION_START per-drone once its state confirms 'armed'."""
        MAX_WAIT_S = 8.0
        RETRY_MS   = 400

        if not self._mission_all_active:
            return

        arm_candidates = set(self._arm_sent_times.keys())
        if not arm_candidates:
            self._mission_all_active = False
            self.mission_all_btn.setText("MISSION ALL")
            return

        now = time.monotonic()
        elapsed_total = now - self._mission_start_time
        newly_ready, still_waiting = [], []

        for i in sorted(arm_candidates):
            if i in self._mission_sent_ids:
                continue
            state = self.drone_states.get(i)
            if state in {"armed", "returning_home"}:
                newly_ready.append(i)
            elif state in {"mission", "landing"}:
                self._mission_sent_ids.add(i)   # already flying — count as done
            elif elapsed_total < MAX_WAIT_S:
                still_waiting.append(i)
            else:
                self.node.get_logger().warn(
                    f"D{i}: gave up waiting for arm confirm after {elapsed_total:.1f}s "
                    f"(state={state})")

        self.node.get_logger().info(
            f"MISSION ALL: sending={newly_ready} waiting={still_waiting} "
            f"done={sorted(self._mission_sent_ids)} elapsed={elapsed_total:.1f}s"
        )

        for drone_id in newly_ready:
            self._publish(drone_id, "COMMAND_MISSION_START")
            self._mission_sent_ids.add(drone_id)

        if still_waiting:
            self.mission_all_btn.setText(
                f"MISSION ALL (waiting {len(still_waiting)}…)")
            QTimer.singleShot(RETRY_MS, self._mission_all_attempt)
        else:
            self._mission_all_active = False
            if self._mission_sent_ids:
                self.mission_all_btn.setText(
                    f"MISSION STARTING ({len(self._mission_sent_ids)})")
            else:
                self.mission_all_btn.setText("MISSION ALL")

    def _send_command(self, drone_id: int, command: str):
        if command == "COMMAND_MISSION_START":
            if self.drone_states.get(drone_id) not in {"armed", "returning_home"}:
                self.node.get_logger().warn(f"D{drone_id}: not armed → mission blocked")
                return
            elapsed_ms = (time.monotonic() - self._arm_sent_times.get(drone_id, 0)) * 1000
            if elapsed_ms < self.ARM_MISSION_GUARD_MS:
                remaining = int(self.ARM_MISSION_GUARD_MS - elapsed_ms) + 10
                self.node.get_logger().info(
                    f"D{drone_id}: ARM guard active ({elapsed_ms:.0f} ms elapsed), retrying in {remaining} ms")
                QTimer.singleShot(remaining, lambda: self._send_command(drone_id, command))
                return
        self._publish(drone_id, command)

    def _send_arm_toggle(self, drone_id: int):
        btn = self.ui_refs[drone_id]["arm"]
        command = "COMMAND_ARM" if btn.isChecked() else "COMMAND_DISARM"
        btn.setText("ARMED" if command == "COMMAND_ARM" else "ARM")
        if command == "COMMAND_ARM":
            self._arm_sent_times[drone_id] = time.monotonic()
        else:
            self._arm_sent_times.pop(drone_id, None)
        self._publish(drone_id, command)

    def _send_config_toggle(self, drone_id, button, cmd_on, cmd_off):
        msg = String()
        msg.data = cmd_on if button.isChecked() else cmd_off
        self.config_publishers[drone_id].publish(msg)
        self.node.get_logger().info(f"D{drone_id} → /gcs/drone_{drone_id}/config : {msg.data}")

    def _send_config_source(self, drone_id, command, checked):
        if not checked:
            return
        msg = String()
        msg.data = command
        self.config_publishers[drone_id].publish(msg)
        self.node.get_logger().info(f"D{drone_id} → /gcs/drone_{drone_id}/config : {msg.data}")

    # ================= Feedback =================
    def _state_callback(self, msg: String, drone_id: int):
        state = msg.data.lower()
        self.drone_states[drone_id] = state
        self.drone_last_seen[drone_id] = time.monotonic()
        ui = self.ui_refs.get(drone_id)
        if not ui:
            return

        # Re-enable controls that were locked while drone was OFFLINE
        if not ui["arm"].isEnabled():
            ui["arm"].setEnabled(True)
        ui["state"].setStyleSheet("font-size:9px;")
        ui["state"].setText(state.upper())
        color_map = {
            "disarmed": "#555",
            "armed": "#b00020",
            "mission": "#1f6aa5",
            "returning_home": "#2d6a4f",
            "landing": "#c77d2b",
            "killed": "#000000",
        }
        ui["strip"].setStyleSheet(f"background-color: {color_map.get(state, '#555')}; border-radius: 2px;")
        ARMED_STATES = {"armed", "mission", "returning_home", "landing"}
        ui["mission"].setEnabled(state in {"armed", "returning_home"})
        ui["home"].setEnabled(state in {"mission", "returning_home"})

        _btn_base = """
            QPushButton {
                color: white; border-radius: 5px; font-size: 9px;
                min-height: 22px; padding: 2px;
            }
            QPushButton:pressed { background-color: #555; }
        """
        mission_color = "#1f6aa5" if state == "mission" else "#2d6a4f" if state in {"armed", "returning_home"} else "#3a3a3a"
        home_color    = "#1f6aa5" if state == "returning_home" else "#2d6a4f" if state == "mission" else "#3a3a3a"
        ui["mission"].setStyleSheet(_btn_base + f"QPushButton {{ background-color: {mission_color}; }}")
        ui["home"].setStyleSheet(_btn_base + f"QPushButton {{ background-color: {home_color}; }}")
        ui["arm"].setChecked(state in ARMED_STATES)
        ui["arm"].setText("ARMED" if state in ARMED_STATES else "ARM")

        any_armed   = any(s in ARMED_STATES for s in self.drone_states.values())
        any_mission = any(s == "mission"    for s in self.drone_states.values())
        self.arm_all_btn.setChecked(any_armed)
        self.arm_all_btn.setText("ARMED (ACTIVE)" if any_armed else "ARM ALL")

        if any_mission:
            mission_all_color = "#1f6aa5"
            self.mission_all_btn.setText("MISSION (RUNNING)")
        elif any_armed:
            mission_all_color = "#2d6a4f"
            if not self._mission_all_active:
                self.mission_all_btn.setText("MISSION ALL")
        else:
            mission_all_color = "#3a3a3a"
            if not self._mission_all_active:
                self.mission_all_btn.setText("MISSION ALL")
        self.mission_all_btn.setStyleSheet(
            self._mission_all_base_style + f"QPushButton {{ background-color: {mission_all_color}; }}"
        )

    def _check_drone_staleness(self):
        """Called every second — marks drones as OFFLINE when no state message arrives."""
        now = time.monotonic()
        for drone_id in range(1, self.DRONE_COUNT + 1):
            ui = self.ui_refs.get(drone_id)
            if not ui:
                continue
            last = self.drone_last_seen.get(drone_id)
            stale = last is None or (now - last) > self.DRONE_OFFLINE_TIMEOUT_S
            if stale and self.drone_states.get(drone_id) != "__offline__":
                self.drone_states[drone_id] = "__offline__"
                ui["state"].setText("OFFLINE")
                ui["state"].setStyleSheet("font-size:9px; color:#888;")
                ui["strip"].setStyleSheet("background-color: #333; border-radius: 2px;")
                ui["arm"].setEnabled(False)
                ui["mission"].setEnabled(False)
                ui["home"].setEnabled(False)
            elif not stale and self.drone_states.get(drone_id) == "__offline__":
                # Back online — re-enable controls; next state callback will fill in real state
                ui["arm"].setEnabled(True)
                ui["state"].setStyleSheet("font-size:9px;")

    def _role_callback(self, msg: String, drone_id: int):
        role = msg.data.upper()
        self.drone_roles[drone_id] = role
        ui = self.ui_refs.get(drone_id)
        if ui:
            ui["role"].setText(role[:10])

    def _battery_callback(self, msg: Int8, drone_id: int):
        pct = msg.data
        self.drone_battery[drone_id] = pct
        ui = self.ui_refs.get(drone_id)
        if not ui:
            return
        if pct < 0:
            ui["title"].setText(f"D{drone_id}")
        else:
            color = "#cc3300" if pct < 20 else "#cc8800" if pct < 40 else "#d0d0d0"
            ui["title"].setStyleSheet(f"font-weight:bold; font-size:10px; color:{color};")
            ui["title"].setText(f"D{drone_id}  BAT:{pct}%")

    def _publish(self, drone_id, command):
        msg = String()
        msg.data = command
        self.command_publishers[drone_id].publish(msg)
        self.node.get_logger().info(f"D{drone_id} → /gcs/drone_{drone_id}/command : {command}")

    # ================= Hardware Button Callbacks =================
    def _hw_arm_callback(self, msg: String):
        hw_armed = (msg.data == "ARMED")
        self._hw_arm_locked = hw_armed
        # Disable software ARM ALL while hardware is holding arm — hardware has priority
        self.arm_all_btn.setEnabled(not hw_armed)
        self.arm_all_btn.setChecked(hw_armed)
        # Record ARM time so the mission guard works even when ARM came from hardware
        now = time.monotonic()
        for drone_id in range(1, self.DRONE_COUNT + 1):
            if hw_armed:
                self._arm_sent_times[drone_id] = now
            else:
                self._arm_sent_times.pop(drone_id, None)
            # Lock per-drone ARM buttons while hardware holds ARM to prevent accidental DISARM
            if drone_id in self.ui_refs:
                self.ui_refs[drone_id]["arm"].setEnabled(not hw_armed)
        ARMED_STATES = {"armed", "mission", "returning_home", "landing"}
        any_active = any(s in ARMED_STATES for s in self.drone_states.values())
        if any_active:
            self.arm_all_btn.setText("ARMED (ACTIVE)")
        elif hw_armed:
            self.arm_all_btn.setText("ARMING (HW)")
        else:
            self.arm_all_btn.setText("ARM ALL")

    def _hw_mission_callback(self, msg: String):
        any_mission = any(s == "mission" for s in self.drone_states.values())
        if any_mission:
            color, text = "#1f6aa5", "MISSION (RUNNING)"
        elif msg.data == "MISSION":
            color, text = "#1f6aa5", "MISSION STARTING (HW)"
        else:
            color, text = "#3a3a3a", "MISSION ALL"
        self.mission_all_btn.setStyleSheet(
            self._mission_all_base_style + f"QPushButton {{ background-color: {color}; }}"
        )
        self.mission_all_btn.setText(text)

    def _hw_emerg_callback(self, msg: String):
        data = msg.data
        active = (data != "OK")
        self._hw_emerg_locked = active
        self.emergency_all_btn.setEnabled(not active)
        if data == "OK":
            self._sw_emerg_active = False
            self._set_emerg_btn_style("idle")
            self.emergency_all_btn.setText("EMERGENCY ALL\nE-LAND")
        else:
            self._set_emerg_btn_style("eland")
            self.emergency_all_btn.setText("EMERGENCY LAND (HW)")

    # ================= Box Label Tracking =================
    def _marker_cb(self, msg: Marker):
        # Drone disc (CYLINDER, ns="drone_N", id=N*100) — published during inertial
        # fallback when vision_pose is not active. Feeds the trail so it stays
        # alive between ArUco detections without any S3 firmware change.
        if (msg.type == Marker.CYLINDER and msg.action == Marker.ADD
                and msg.ns.startswith("drone_")):
            try:
                drone_id = int(msg.ns.split("_")[1])
                if drone_id in self._trail_deques and msg.id == drone_id * 100:
                    ps = PoseStamped()
                    ps.header.frame_id = "map"
                    ps.header.stamp = msg.header.stamp
                    ps.pose = msg.pose
                    self._trail_cb(ps, drone_id)
            except (ValueError, IndexError):
                pass

        if msg.ns not in ("blue", "red") or msg.type != Marker.CUBE:
            return
        if msg.action == Marker.ADD:
            self._box_positions[msg.id] = (
                msg.pose.position.x, msg.pose.position.y,
                msg.pose.position.z, msg.ns)
            self._publish_box_label(msg.id)

    def _publish_box_label(self, box_id: int):
        entry = self._box_positions.get(box_id)
        if entry is None:
            return
        x, y, z, ns = entry
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = box_id + 1000
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + 0.35   # 0.1 m above box top (cube centre + 0.25 half-height)
        m.pose.orientation.w = 1.0
        m.scale.z = 0.35
        if ns == "blue":
            m.color.r, m.color.g, m.color.b = 0.5, 0.8, 1.0
        else:
            m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.5
        m.color.a = 1.0
        m.text = str(box_id)
        self.marker_pub.publish(m)

    def shutdown_plugin(self):
        self.timer.stop()
        self._online_timer.stop()
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
