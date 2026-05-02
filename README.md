# Swarm Drone Challenge 2026

Source code of the solution by **Team Mach Mind** for the Swarm Drone Challenge 2026, organised by [MBDA](https://www.mbda-systems.com) and [brigkAIR](https://www.brigkair.com).

---

## Team

**Mach Mind** — [machmind.dev](https://machmind.dev)

| Member | Role |
|--------|------|
| Mindaugas Jonauskis | |
| Hauke Renk | |

---

## Repository Structure

```
drone-swarm-challenge-2026/
├── drone-firmware/          # ESP32-S3 drone node firmware (ESP-IDF + micro-ROS)
├── drone-vision/            # Vision board firmware (ArUco detection + POSE)
│   ├── ESP32P4/             # Waveshare ESP32-P4 — OV5647 MIPI-CSI, world-frame POSE
│   └── ESP32S3/             # Seeed XIAO ESP32-S3 — OV3660/OV2640 DVP, ArUco detection
├── ground-station-software/ # ROS2 ground control station, swarm algorithms & vision
├── hardware/                # Mechanical and electrical design files
├── launchers/               # Platform-specific launch scripts
│   ├── ubuntu-gnome-pc/     # x86_64 Ubuntu GNOME (development GCS)
│   └── ubuntu-xfce-pi5/     # ARM64 Raspberry Pi 5 (field GCS)
└── docs/                    # Documentation and media
```

### drone-firmware

ESP-IDF firmware for the drone node running on a Seeed Studio XIAO ESP32-S3. Handles sensor fusion (VL53L1X ToF rangers), MAVLink telemetry to PX4, camera streaming over micro-ROS, and obstacle detection.

Key components:
- `components/esp32-camera` — ESP32 camera driver
- `components/micro_ros_espidf_component` — micro-ROS ESP-IDF integration
- `components/VL53L1-ULD-ESP` — VL53L1X time-of-flight sensor driver
- `docker/` — Containerised IDF development environment

### drone-vision

Vision board firmware in two variants:

- **ESP32P4/** — Waveshare ESP32-P4 WiFi6 + OV5647 MIPI-CSI. Real-time ArUco detection at 360 MHz; outputs world-frame POSE over UART. IDF 5.3+.
- **ESP32S3/** — Seeed XIAO ESP32-S3 + OV3660/OV2640 DVP. ArUco detection pipeline; used for benchmarking and earlier prototypes. IDF 5.0.

### ground-station-software

ROS2-based ground control station running on an x86\_64 Ubuntu PC (home/development) and an ARM64 Raspberry Pi 5 (field GCS).

Key components:
- `gcs/rqt/` — Custom rqt button panel for mission control
- `gcs/rviz/` — RViz scene configurations
- `swarm/algorithm/` — Core swarm logic
- `swarm/data/` — Parameters, simulation results, logs and telemetry
- `vision/` — OpenCV / ArUco marker detection scripts (`aruco_node.py`)
- `buttons/` — Physical GPIO button handler ROS2 publisher

### launchers

Platform-specific shell scripts that start the full GCS stack, vision nodes, and development environments. See the platform READMEs for details.

### hardware

Mechanical and electrical design files for the drone and ground station.

### docs

Documentation, field photos, CAD exports and software diagrams.

---

## Getting Started

See the platform-specific launcher README for setup and launch instructions:

- [Ubuntu GNOME PC (x86\_64)](launchers/ubuntu-gnome-pc/README.md)
- [Ubuntu XFCE Pi5 (ARM64)](launchers/ubuntu-xfce-pi5/README.md)
- [Drone Firmware](drone-firmware/README.md)
- [Vision Board — ESP32-P4](drone-vision/ESP32P4/README.md)
- [Vision Board — ESP32-S3](drone-vision/ESP32S3/README.md)

---

## License

Private — All rights reserved. Team Mach Mind, 2026.
