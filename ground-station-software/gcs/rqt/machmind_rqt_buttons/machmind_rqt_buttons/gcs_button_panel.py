from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QLabel,
    QGroupBox,
    QSizePolicy,
    QMessageBox,
)
from rqt_gui_py.plugin import Plugin


class GcsButtonPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("GcsButtonPanel")

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node: Node = rclpy.create_node("machmind_rqt_buttons")
        self.publisher = self.node.create_publisher(String, "/gcs_command", 10)

        self._widget = QWidget()
        self._widget.setWindowTitle("Mach Mind GCS Buttons")
        self._widget.setStyleSheet("""
            QWidget {
                background-color: #1e1e1e;
                color: #d0d0d0;
                font-family: Arial;
            }
        """)

        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        title = QLabel("Mach Mind GCS Control Panel")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 14px;
                font-weight: bold;
                padding: 2px;
                color: #d8d8d8;
            }
        """)
        main_layout.addWidget(title)

        self.status_label = QLabel("Status: READY")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #2b2b2b;
                color: #cfcfcf;
                border: 1px solid #444;
                padding: 3px;
                font-size: 11px;
                border-radius: 3px;
            }
        """)
        main_layout.addWidget(self.status_label)

        main_layout.addWidget(self._build_emergency_group())
        main_layout.addWidget(self._build_system_group())
        main_layout.addWidget(self._build_drones_group())
        main_layout.addWidget(self._build_scene_group())

        self._widget.setLayout(main_layout)
        context.add_widget(self._widget)

    def _make_group_box(self, title: str) -> QGroupBox:
        box = QGroupBox(title)
        box.setStyleSheet("""
            QGroupBox {
                font-size: 12px;
                font-weight: bold;
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 4px;
                padding-top: 4px;
                color: #d6d6d6;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 6px;
                padding: 0 3px;
            }
        """)
        return box

    def _make_button(
        self,
        text: str,
        command: str,
        min_height: int = 32,
        style: str = "normal",
        checkable: bool = False,
    ) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(min_height)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        button.setCheckable(checkable)

        if checkable:
            button.clicked.connect(partial(self.publish_toggle_command, button, command))
        elif command == "kill_all":
            button.clicked.connect(partial(self.confirm_and_publish, command))
        else:
            button.clicked.connect(partial(self.publish_command, command))

        if style == "normal":
            button.setStyleSheet("""
                QPushButton {
                    background-color: #2f3438;
                    color: #dcdcdc;
                    border: 1px solid #555;
                    border-radius: 3px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 3px;
                }
                QPushButton:hover {
                    background-color: #3a4046;
                }
                QPushButton:pressed {
                    background-color: #24282c;
                }
            """)
        elif style == "warning":
            button.setStyleSheet("""
                QPushButton {
                    background-color: #c77d2b;
                    color: black;
                    border: 1px solid #7a4a13;
                    border-radius: 3px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 3px;
                }
                QPushButton:hover {
                    background-color: #d48a36;
                }
                QPushButton:pressed {
                    background-color: #ab6620;
                }
            """)
        elif style == "hazard":
            button.setStyleSheet("""
                QPushButton {
                    background-color: #e0b000;
                    color: black;
                    border: 2px solid black;
                    border-radius: 3px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 4px;
                }
                QPushButton:hover {
                    background-color: #f0bf1a;
                }
                QPushButton:pressed {
                    background-color: #c79a00;
                }
            """)
        elif style == "scene":
            button.setStyleSheet("""
                QPushButton {
                    background-color: #4c5b6b;
                    color: #d8d8d8;
                    border: 1px solid #667788;
                    border-radius: 3px;
                    font-size: 12px;
                    font-weight: bold;
                    padding: 3px;
                }
                QPushButton:hover {
                    background-color: #5a6b7d;
                }
                QPushButton:pressed {
                    background-color: #3d4b59;
                }
            """)
        elif style == "video_toggle":
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3a3a3a;
                    color: #cccccc;
                    border: 1px solid #666;
                    border-radius: 3px;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 2px;
                }
                QPushButton:hover {
                    background-color: #4a4a4a;
                }
                QPushButton:checked {
                    background-color: #2d6a4f;
                    color: white;
                    border: 1px solid #52b788;
                }
            """)

        return button

    def _build_emergency_group(self) -> QGroupBox:
        box = self._make_group_box("Emergency")
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        land_all = self._make_button("LAND ALL", "land_all", min_height=38, style="warning")
        kill_all = self._make_button("⚠ KILL ALL ⚠", "kill_all", min_height=38, style="hazard")

        layout.addWidget(land_all)
        layout.addWidget(kill_all)
        box.setLayout(layout)
        return box

    def _build_system_group(self) -> QGroupBox:
        box = self._make_group_box("System")
        layout = QGridLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)

        buttons = [
            ("ARM", "arm"),
            ("TAKEOFF", "takeoff"),
            ("START MISSION", "start_mission"),
            ("RETURN BASE", "return_base"),
            ("LAND", "land"),
            ("DISARM", "disarm"),
        ]

        for idx, (label, cmd) in enumerate(buttons):
            btn = self._make_button(label, cmd, min_height=32, style="normal")
            layout.addWidget(btn, 0, idx)

        box.setLayout(layout)
        return box

    def _build_drones_group(self) -> QGroupBox:
        box = self._make_group_box("Drones")
        layout = QGridLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        for col in range(5):
            drone_id = col + 1

            title = QLabel(f"D{drone_id}")
            title.setAlignment(Qt.AlignCenter)
            title.setStyleSheet("""
                QLabel {
                    font-size: 12px;
                    font-weight: bold;
                    color: #cfcfcf;
                    padding: 1px;
                }
            """)
            layout.addWidget(title, 0, col)

            col_widget = QWidget()
            col_layout = QVBoxLayout()
            col_layout.setContentsMargins(0, 0, 0, 0)
            col_layout.setSpacing(4)

            video_btn = self._make_button(
                "VIDEO OFF",
                f"drone_{drone_id}_video",
                min_height=28,
                style="video_toggle",
                checkable=True,
            )
            emerg_btn = self._make_button(
                "E-LAND",
                f"drone_{drone_id}_emer_land",
                min_height=28,
                style="warning",
            )
            rtb_btn = self._make_button(
                "RTB",
                f"drone_{drone_id}_rtb",
                min_height=28,
                style="normal",
            )

            col_layout.addWidget(video_btn)
            col_layout.addWidget(emerg_btn)
            col_layout.addWidget(rtb_btn)

            col_widget.setLayout(col_layout)
            layout.addWidget(col_widget, 1, col)

        box.setLayout(layout)
        return box

    def _build_scene_group(self) -> QGroupBox:
        box = self._make_group_box("Scene")
        layout = QHBoxLayout()
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        load_lh = self._make_button("LOAD LH ZONE", "scene_load_LH_team_zone", min_height=32, style="scene")
        load_rh = self._make_button("LOAD RH ZONE", "scene_load_RH_team_zone", min_height=32, style="scene")

        layout.addWidget(load_lh)
        layout.addWidget(load_rh)
        box.setLayout(layout)
        return box

    def publish_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.status_label.setText(f"Status: SENT → {command}")
        self.node.get_logger().info(f"Published /gcs_command: {command}")

    def publish_toggle_command(self, button: QPushButton, base_command: str) -> None:
        state = "on" if button.isChecked() else "off"
        full_command = f"{base_command}_{state}"

        msg = String()
        msg.data = full_command
        self.publisher.publish(msg)

        button.setText("VIDEO ON" if button.isChecked() else "VIDEO OFF")
        self.status_label.setText(f"Status: SENT → {full_command}")
        self.node.get_logger().info(f"Published /gcs_command: {full_command}")

    def confirm_and_publish(self, command: str) -> None:
        reply = QMessageBox.question(
            self._widget,
            "CONFIRM ACTION",
            f"Are you sure you want to execute:\n{command} ?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self.publish_command(command)
        else:
            self.status_label.setText("Status: CANCELLED")

    def shutdown_plugin(self) -> None:
        if hasattr(self, "node"):
            self.node.destroy_node()
