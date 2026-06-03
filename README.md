**Team Mach Mind**

| Member | Disciplines |
|--------|-------------|
| [**Mindaugas Jonauskis**](https://www.linkedin.com/in/mindaugas-jonauskis-11931145/) (Founder) | Firmware & Software Engineering · Electronics & Electrical Design |
| [**Hauke Renk**](https://www.linkedin.com/in/hauke-renk-90832739a/) | Mechanical Design & Integration |

---

# Swarm Drone Challenge 2026

Source code of the solution by **Team Mach Mind** for the Swarm Drone Challenge 2026, organised by [MBDA](https://www.mbda-systems.com) and [brigkAIR](https://www.brigkair.com).

Qualifying rounds took place **20–24 April 2026**. Team Mach Mind qualified and was one of **6 finalists** competing at the Finals on **11 June 2026**.

**Drone Versions**

| Version | Event | MCU | CPU (MHz), PSRAM (MB) | ToF | Vision | ArUco |
|---------|-------|-----|-------|-----|--------|-------|
| [v1.0](drone-firmware/v1.0/) | Qualifying | S3 | 240 MHz, 8 MB | 4 | Offboard | 80×60 px, up to 4 m |
| [v2.0](drone-firmware/v2.0/) | Finals | S3 & P4 | 240 & 360 MHz, 8 & 32 MB | 6 | Onboard | 320×240 px, up to 12 m |


## System Architecture (v2.0)

![System Architecture v2.0](docs/system_architecture.png)

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

