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

## Swarm Behaviour

The **SDC26 Commander** (`ground-station-software/swarm/sdc26_commander.py`) runs the swarm by role, streaming waypoints to `/gcs/drone_<id>/control` in the arena frame (`id · x · y · height`). A 5 s start-up grace period precedes any command, and the active team (LH/red or RH/blue) is read live from RQT (`/gcs/system/team_color`).

**Swarm control via LLM — demo only:** Ollama (Gemma) turns natural-language instructions into JSON/coordinate commands, but it is a standalone DEMO and is NOT in live on the finals. Meaningful coordinate tasking needs a reliable absolute position reference (ArUco Navigation), which we couldn't fully implement in time, so all live flight is run by the deterministic Commander above.

**Waypoint execution (Manhattan):** the firmware converts each arena waypoint to NED and reaches it in axis-aligned legs (one axis at a time, no diagonals), each leg flown as a sequence of discrete steps — keeping motion predictable and obstacle handling simple.

### Executor

Capture opponent boxes based on data received from Seeker.

### Seeker

The only role that publishes box locations. If none of the boxes were found, after 2 minutes GCS publishes random box for each not found.

### Leader

Monitors the home base, if no boxes were captured.

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

### Pre-Finals Critical

- [x] **ARM-time inertial anchor fix** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:874`) — `inertial_anchor_valid = true` now set at ARM time; trail tracks live from takeoff.

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

---

## Open Issues


### ArUco navigation

ArUco-based EKF fusion was developed, but not fully deployed in time for the finals. ArUco is therefore used only for box detection, not navigation.

