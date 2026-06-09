**Team**

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
| [v1.0](drone-firmware/v1.0/) | Qualifying | S3 | 240 MHz, 8 MB | 4 | Offboard | 80×60px, up to 4 m |
| [v2.0](drone-firmware/v2.0/) | Finals | S3&P4 | 240&360 MHz, 8&32 MB | 6 | Onboard | 320×240px, up to 12 m |


## System Architecture (v2.0)

![System Architecture v2.0](docs/system_architecture.png)

---

## Highlights

### ArUco box detection — GCS view

![ArUco offboard detection](docs/media/gcs/aruco-offboard-1.jpeg)

### RViz — drone position over ROS

![RViz first test](docs/media/gcs/rviz-first-test-1.png)

### Flight test videos

| Video | Description |
|-------|-------------|
| [basic-swarm.mp4](docs/media/flight-testing/basic-swarm.mp4) | First test of basic swarming functionality |
| [five-unit-flight.mp4](docs/media/flight-testing/five-unit-flight.mp4) | First flight of all 5 units together |
| [obstacle-detection.mp4](docs/media/flight-testing/obstacle-detection.mp4) | First test of ToF obstacle detection |
| [first-flight.mp4](docs/media/flight-testing/first-flight.mp4) | First flight on an empty platform |

Full media index: [docs/media/](docs/media/)

---

## Swarm Behaviour

The **SDC26 Commander** (`ground-station-software/swarm/sdc26_commander.py`) runs the swarm by role, streaming waypoints to `/gcs/drone_<id>/control` in the arena frame (`id · x · y · height`).

**Swarm control via LLM — demo only:** Ollama (Gemma) turns natural-language instructions into JSON/coordinate commands, but it is a standalone DEMO and is NOT in live on the finals. Meaningful coordinate tasking needs a reliable absolute position reference (ArUco Navigation), which we couldn't be fully implement in time, so all live flight is run by the deterministic Commander above.

**Waypoint execution (Manhattan):** the firmware converts each arena waypoint to NED and reaches it in axis-aligned legs (one axis at a time, no diagonals), each leg flown as a sequence of discrete steps — keeping motion predictable and obstacle handling simple.

| Role | Behaviour |
|------|-----------|
| **Executor** | Captures opponent boxes based on locations received from a Seeker. |
| **Seeker** | The only role that publishes box locations. If a box is not found within 2 minutes, the GCS publishes a random. |
| **Leader** | Monitors the home base while no boxes have been captured. |

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

---

## To Do

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m. Since navigation via ArUco markers were not implemented a work around took place.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

---

## Open Issues


### ArUco navigation

ArUco-based EKF fusion was developed, but not fully deployed in time for the finals. ArUco is therefore used only for box detection, not navigation.

