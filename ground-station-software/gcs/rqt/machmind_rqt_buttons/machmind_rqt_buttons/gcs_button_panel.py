# gcs_button_panel.py
from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
    EMERGENCY_HOLD_SECONDS = 3

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("GcsButtonPanel")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node: Node = rclpy.create_node("machmind_rqt_buttons")

        self.drone_states = {}
        self.drone_roles = {}
        self.ui_refs = {}
        self.command_publishers = {}
        self.config_publishers = {}
        self.state_subscribers = {}
        self.role_subscribers = {}

        # Global emergency countdown
        self.global_emergency_counter = self.EMERGENCY_HOLD_SECONDS
        self.global_emergency_hold = False
        self.global_emergency_timer = QTimer()
        self.global_emergency_timer.setInterval(1000)
        self.global_emergency_timer.timeout.connect(self._global_emergency_tick)

        # Per-drone emergency countdown
        self.drone_emergency_timers = {}
        self.drone_emergency_counters = {}
        self.drone_emergency_holds = {}

        # Scene publishers
        self.marker_pub = self.node.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_array_pub = self.node.create_publisher(MarkerArray, "/visualization_marker_array", 10)

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

            timer = QTimer()
            timer.setInterval(1000)
            timer.timeout.connect(partial(self._drone_emergency_tick, i))
            self.drone_emergency_timers[i] = timer
            self.drone_emergency_counters[i] = self.EMERGENCY_HOLD_SECONDS
            self.drone_emergency_holds[i] = False

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

        self._widget.setLayout(main_layout)
        context.add_widget(self._widget)

        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.timer.start(50)

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
        self.mission_all_btn.setStyleSheet(base_style + "QPushButton { background-color: #2d6a4f; }")
        self.mission_all_btn.clicked.connect(self._mission_all)
        layout.addWidget(self.mission_all_btn)

        self.emergency_all_btn = QPushButton("EMERGENCY ALL\nE-LAND / HOLD 3s: KILL")
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
                background-color: #ff3b30;
                color: white;
            }
        """)
        self.emergency_all_btn.pressed.connect(self._start_global_emergency)
        self.emergency_all_btn.released.connect(self._release_global_emergency)
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
        mission_btn.setStyleSheet(btn_style + "QPushButton { background-color: #2d6a4f; }")
        mission_btn.clicked.connect(partial(self._send_command, drone_id, "COMMAND_MISSION_START"))
        mission_btn.setEnabled(False)
        layout.addWidget(mission_btn)

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
        emergency_btn.pressed.connect(partial(self._start_drone_emergency, drone_id))
        emergency_btn.released.connect(partial(self._release_drone_emergency, drone_id))
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

        vision_btn = QPushButton("VIS")
        vision_btn.setCheckable(True)
        vision_btn.setStyleSheet(small_toggle_style)
        vision_btn.clicked.connect(partial(self._send_config_toggle, drone_id, vision_btn,
                                           "CONFIG_VISION_ENABLE", "CONFIG_VISION_DISABLE"))
        layout.addWidget(vision_btn)

        rc_radio = QRadioButton("RC")
        gcs_radio = QRadioButton("GCS")
        rc_radio.setChecked(True)
        rc_radio.setStyleSheet("font-size:9px;")
        gcs_radio.setStyleSheet("font-size:9px;")
        group = QButtonGroup(container)
        group.addButton(rc_radio)
        group.addButton(gcs_radio)
        rc_radio.toggled.connect(partial(self._send_config_source, drone_id, "CONFIG_SOURCE_RC"))
        gcs_radio.toggled.connect(partial(self._send_config_source, drone_id, "CONFIG_SOURCE_GCS"))

        radio_layout = QHBoxLayout()
        radio_layout.setSpacing(2)
        radio_layout.setContentsMargins(0, 0, 0, 0)
        radio_layout.addWidget(rc_radio)
        radio_layout.addWidget(gcs_radio)
        layout.addLayout(radio_layout)

        self.ui_refs[drone_id] = {
            "arm": arm_btn,
            "mission": mission_btn,
            "state": state_label,
            "role": role_label,
            "strip": strip,
            "emergency": emergency_btn,
        }

        container.setLayout(layout)
        return container

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

        box.setLayout(layout)
        return box

    # ================= Emergency Logic =================
    def _start_global_emergency(self):
        self.global_emergency_hold = True
        self.global_emergency_counter = self.EMERGENCY_HOLD_SECONDS
        self.emergency_all_btn.setText(f"KILL IN {self.global_emergency_counter}")
        self.global_emergency_timer.start()

    def _global_emergency_tick(self):
        if not self.global_emergency_hold:
            return
        self.global_emergency_counter -= 1
        if self.global_emergency_counter > 0:
            self.emergency_all_btn.setText(f"KILL IN {self.global_emergency_counter}")
        else:
            self.global_emergency_timer.stop()
            self.global_emergency_hold = False
            self._reset_global_emergency_text()
            self._kill_all()

    def _release_global_emergency(self):
        if self.global_emergency_hold and self.global_emergency_timer.isActive():
            self.global_emergency_timer.stop()
            self.global_emergency_hold = False
            self._reset_global_emergency_text()
            for drone_id in range(1, self.DRONE_COUNT + 1):
                self._publish(drone_id, "COMMAND_ELAND")

    def _reset_global_emergency_text(self):
        self.emergency_all_btn.setText("EMERGENCY ALL\nE-LAND / HOLD 3s: KILL")

    def _start_drone_emergency(self, drone_id):
        self.drone_emergency_holds[drone_id] = True
        self.drone_emergency_counters[drone_id] = self.EMERGENCY_HOLD_SECONDS
        self.ui_refs[drone_id]["emergency"].setText(f"KILL {self.drone_emergency_counters[drone_id]}")
        self.drone_emergency_timers[drone_id].start()

    def _drone_emergency_tick(self, drone_id):
        if not self.drone_emergency_holds[drone_id]:
            return
        self.drone_emergency_counters[drone_id] -= 1
        btn = self.ui_refs[drone_id]["emergency"]
        if self.drone_emergency_counters[drone_id] > 0:
            btn.setText(f"KILL {self.drone_emergency_counters[drone_id]}")
        else:
            self.drone_emergency_timers[drone_id].stop()
            self.drone_emergency_holds[drone_id] = False
            btn.setText("EMERG")
            self._publish(drone_id, "COMMAND_KILL")

    def _release_drone_emergency(self, drone_id):
        timer = self.drone_emergency_timers[drone_id]
        if self.drone_emergency_holds[drone_id] and timer.isActive():
            timer.stop()
            self.drone_emergency_holds[drone_id] = False
            self.ui_refs[drone_id]["emergency"].setText("EMERG")
            self._publish(drone_id, "COMMAND_ELAND")

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

    def _publish_lh_scene(self):
        self._publish_base_floor()
        arr = MarkerArray()
        arr.markers.append(self._zone_marker(3.3333, (0.2, 0.4, 0.8, 0.30), 101))
        arr.markers.append(self._zone_marker(10.0, (0.5, 0.5, 0.5, 0.25), 102))
        arr.markers.append(self._zone_marker(16.6667, (0.8, 0.3, 0.3, 0.30), 103))
        arr.markers.append(self._zone_label(3.3333, "TEAM-ZONE", 201))
        arr.markers.append(self._zone_label(10.0, "NO-MAN'S-LAND", 202))
        arr.markers.append(self._zone_label(16.6667, "OPPONENT-ZONE", 203))
        self.marker_array_pub.publish(arr)

    def _publish_rh_scene(self):
        self._publish_base_floor()
        arr = MarkerArray()
        arr.markers.append(self._zone_marker(3.3333, (0.8, 0.3, 0.3, 0.30), 101))
        arr.markers.append(self._zone_marker(10.0, (0.5, 0.5, 0.5, 0.25), 102))
        arr.markers.append(self._zone_marker(16.6667, (0.2, 0.4, 0.8, 0.30), 103))
        arr.markers.append(self._zone_label(3.3333, "OPPONENT-ZONE", 201))
        arr.markers.append(self._zone_label(10.0, "NO-MAN'S-LAND", 202))
        arr.markers.append(self._zone_label(16.6667, "TEAM-ZONE", 203))
        self.marker_array_pub.publish(arr)

    # ================= Commands =================
    def _arm_all_toggle(self):
        command = "COMMAND_ARM" if self.arm_all_btn.isChecked() else "COMMAND_DISARM"
        self.arm_all_btn.setText("ARMED" if command == "COMMAND_ARM" else "ARM ALL")
        for drone_id in range(1, self.DRONE_COUNT + 1):
            self._publish(drone_id, command)

    def _mission_all(self):
        for drone_id in range(1, self.DRONE_COUNT + 1):
            if self.drone_states.get(drone_id) == "armed":
                self._publish(drone_id, "COMMAND_MISSION_START")

    def _kill_all(self):
        for drone_id in range(1, self.DRONE_COUNT + 1):
            self._publish(drone_id, "COMMAND_KILL")

    def _send_command(self, drone_id: int, command: str):
        if command == "COMMAND_MISSION_START" and self.drone_states.get(drone_id) != "armed":
            self.node.get_logger().warn(f"Drone {drone_id} not armed → mission blocked")
            return
        self._publish(drone_id, command)

    def _send_arm_toggle(self, drone_id: int):
        current_state = self.drone_states.get(drone_id, "disarmed")
        command = "COMMAND_ARM" if current_state == "disarmed" else "COMMAND_DISARM"
        self._publish(drone_id, command)

    def _send_config_toggle(self, drone_id, button, cmd_on, cmd_off):
        msg = String()
        msg.data = cmd_on if button.isChecked() else cmd_off
        self.config_publishers[drone_id].publish(msg)

    def _send_config_source(self, drone_id, command, checked):
        if not checked:
            return
        msg = String()
        msg.data = command
        self.config_publishers[drone_id].publish(msg)

    # ================= Feedback =================
    def _state_callback(self, msg: String, drone_id: int):
        state = msg.data.lower()
        self.drone_states[drone_id] = state
        ui = self.ui_refs.get(drone_id)
        if not ui:
            return

        ui["state"].setText(state.upper())
        color_map = {
            "disarmed": "#555",
            "armed": "#b00020",
            "mission": "#1f6aa5",
            "landing": "#c77d2b",
            "killed": "#000000",
        }
        ui["strip"].setStyleSheet(f"background-color: {color_map.get(state, '#555')}; border-radius: 2px;")
        ui["mission"].setEnabled(state == "armed")
        ui["arm"].setChecked(state == "armed")

    def _role_callback(self, msg: String, drone_id: int):
        role = msg.data.upper()
        self.drone_roles[drone_id] = role
        ui = self.ui_refs.get(drone_id)
        if ui:
            ui["role"].setText(role[:10])

    def _publish(self, drone_id, command):
        msg = String()
        msg.data = command
        self.command_publishers[drone_id].publish(msg)

    def shutdown_plugin(self):
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
