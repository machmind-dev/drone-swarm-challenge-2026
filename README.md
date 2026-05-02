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
├── drone-firmware/          # ESP32-S3 drone node firmware (ESP-IDF)
├── drone-vision/            # ESP32-P4 vision board firmware (ArUco + POSE)
│   └── ESP32P4/             # OV5647 MIPI-CSI, ArUco detection, world-frame POSE
├── ground-station-software/ # ROS2 ground control station, swarm algorithms & vision
├── hardware/                # Mechanical and electrical design files
└── docs/                    # Documentation and media
```

### drone-firmware

ESP-IDF firmware for the drone node running on an ESP32-S3. Handles sensor fusion (VL53L1X ToF rangers), MAVLink telemetry, camera streaming over micro-ROS, and obstacle detection.

Key components:
- `components/esp32-camera` — ESP32 camera driver
- `components/micro_ros_espidf_component` — micro-ROS ESP-IDF integration
- `components/VL53L1-ULD-ESP` — VL53L1X time-of-flight sensor driver
- `docker/` — Containerised IDF development environment

### ground-station-software

ROS2-based ground control station running on an x86\_64 Ubuntu PC (home/development) and an ARM64 Raspberry Pi 5 (field GCS).

Key components:
- `gcs/rqt/` — Custom rqt button panel for mission control
- `gcs/rviz/` — RViz scene configurations
- `swarm/algorithm/` — Core swarm logic
- `swarm/data/` — Parameters, simulation results, logs and telemetry
- `vision/` — OpenCV / ArUco marker detection scripts
- `buttons/` — Physical button handler ROS2 publisher
- `launchers/` — Platform-specific setup and launch scripts

### hardware

Mechanical and electrical design files for the drone and ground station.

### docs

Documentation, field photos, CAD exports and software diagrams.

---

## Getting Started

See the platform-specific launcher README for setup and launch instructions:

- [Ubuntu GNOME PC (x86\_64)](ground-station-software/launchers/ubuntu-gnome-pc/README.md)
- [Ubuntu XFCE Pi5 (ARM64)](ground-station-software/launchers/ubuntu-xfce-pi5/README.md)
- [Drone Firmware](drone-firmware/README.md)
- [Vision Board (ESP32-P4)](drone-vision/ESP32P4/README.md)

---

## License

Private — All rights reserved. Team Mach Mind, 2026.
