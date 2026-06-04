> **Known issue (frame-mix):** `drone-comms-esp32s3/main/main.c:931` (`aruco_approach_task`) — `hold_y = vision_pose_valid ? vp_y : px4_pos_y` mixes arena-frame `vp_y` with NED `px4_pos_y`. Pre-existing bug, only in the ArUco-spin approach feature, not the waypoint path. Needs the vision branch converted to NED (`ned_offset_y - vp_y`).

---

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

- [ ] **Lower `C2_FAIL_THRESHOLD` to 1** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:157`) — currently set to 3 (6 s timeout) for WiFi RTT tolerance during testing. Must be lowered to 1 (2 s) before the competition flight.

- [x] **ARM-time inertial anchor fix** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:874`) — `inertial_anchor_valid = true` now set at ARM time; trail tracks live from takeoff.

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m.

### Analysis

- [ ] **Analyse drift test `log_63_UnknownDate.ulg`** (`drone-firmware/v2.0/logs/2026-06-01_2109_drone4/`) — 5-minute aggressive inertial-only flight. Goal: quantify position error accumulation over time to set the mission planning envelope for how long the drone can fly without a position correction.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

### Cleanup

- [ ] **Remove dead heading-seed code** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` are no longer effective. Remove once nav is confirmed stable.

- [x] **Review `simulate_drones.py` active drone list** (`ground-station-software/swarm/algorithm/simulate_drones.py:20`) — all 5 drones now active.
