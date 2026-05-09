# Swarm Drone Challenge 2026

Source code of the solution by **Team Mach Mind** for the Swarm Drone Challenge 2026, organised by [MBDA](https://www.mbda-systems.com) and [brigkAIR](https://www.brigkair.com).

| Version | Event | MCU | CPU | Memory | ArUco Resolution | ArUco Max Detection |
|---------|-------|-----|-----|--------|-----------------|---------------------|
| [v1.0](drone-firmware/v1.0/) | SDC 2026 Qualifying | ESP32-S3 | 240 MHz | 8 MB PSRAM | 80×60 (QQVGA) | ~3–4 m |
| [v2.0](drone-firmware/v2.0/) | SDC 2026 Finals | ESP32-P4 + ESP32-S3 | 360 MHz | 32 MB PSRAM | 320×240 (QVGA) | ~8–12 m |

---

## Team

**Mach Mind** — [machmind.dev](https://machmind.dev)

| Member | Disciplines |
|--------|-------------|
| **Mindaugas Jonauskis** (Founder) | Firmware & Software Engineering · Electronics & Electrical Design |
| **Hauke Renk** | Mechanical Design & Integration |

---

## Repository Structure

```
drone-swarm-challenge-2026/
├── drone-firmware/          # All drone onboard firmware (v1.0 and v2.0)
│   ├── v1.0/                # SDC 2026 Qualifying — single ESP32-S3
│   └── v2.0/                # SDC 2026 Finals — dual ESP32-P4 + ESP32-S3
│       ├── drone-vision-esp32p4/  # Navigation: ArUco, ToF, UART TX
│       ├── drone-comms-esp32s3/   # Communication: MAVLink, micro-ROS
│       └── shared/                # Binary UART protocol header
├── ground-station-software/ # ROS2 ground control station, swarm algorithms & vision
├── hardware/                # Mechanical and electrical design files
├── launchers/               # Platform-specific launch scripts
│   ├── ubuntu-gnome-pc/     # x86_64 Ubuntu GNOME (development GCS)
│   └── ubuntu-xfce-pi5/     # ARM64 Raspberry Pi 5 (field GCS)
└── docs/                    # Documentation and media
```

### drone-firmware

ESP-IDF firmware for all onboard MCUs across both competition versions. See [drone-firmware/README.md](drone-firmware/README.md) for full architecture details and version comparison.

**v1.0** — Single Seeed Studio XIAO ESP32-S3 handling camera, ArUco, ToF sensors, MAVLink, and micro-ROS.

**v2.0** — Dual-MCU split on a custom PCB:
- `drone-vision-esp32p4/` — Waveshare ESP32-P4 (360 MHz): OV5647 MIPI-CSI camera, ArUco detection, 6× VL53L1X ToF, binary UART TX to S3
- `drone-comms-esp32s3/` — ESP32-S3: receives UART frames from P4, forwards `OBSTACLE_DISTANCE` + `VISION_POSITION_ESTIMATE` to PX4 via MAVLink, maintains micro-ROS GCS link

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
- [v2.0 Navigation — ESP32-P4](drone-firmware/v2.0/drone-vision-esp32p4/README.md)
- [v2.0 Communications — ESP32-S3](drone-firmware/v2.0/drone-comms-esp32s3/README.md)

---

## License

Private — All rights reserved. Team Mach Mind, 2026.
