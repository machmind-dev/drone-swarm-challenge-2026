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

## Manhattan Waypoint Navigation Parameters

All parameters are `#define` constants in `drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `MANHATTAN_STEP_M` | `1.0 m` | Maximum distance between intermediate waypoints. X-leg is broken into steps of this size first, then Y-leg. Smaller values = smoother path but more waypoints and more frequent ToF checks. |
| `MANHATTAN_ARRIVAL_M` | `1.0 m` | Radius within which a waypoint is considered reached and the sequencer advances to the next one. Must be ≤ `MANHATTAN_STEP_M` to avoid skipping waypoints. |
| `MANHATTAN_PASSTHROUGH_M` | `1.0 m` | Setpoints with a Manhattan distance (&#124;Δx&#124; + &#124;Δy&#124;) from the drone's current position at or below this threshold bypass the sequencer entirely and are streamed directly to PX4. Covers keyboard fly-mode (0.5 m increments) and rotation-only commands (0 m). |
| `MANHATTAN_OBSTACLE_MM` | `500 mm` | ToF clearance threshold in the direction of travel. If any horizontal sensor reads below this the sequencer stops, enters obstacle-hold, and waits for a new GCS destination. Does **not** auto-resume when the obstacle clears — requires explicit new setpoint. |
| `MANHATTAN_MAX_WPS` | `40` | Static waypoint array size. At 1 m steps the worst-case path across the full arena (20 m + 10 m) needs 30 entries; 40 gives margin for rounding remainder waypoints. Only increase if `MANHATTAN_STEP_M` is reduced below ~0.7 m. |
| `COLLISION_MARGIN_M` | `0.3 m` | Safety buffer subtracted from ToF clearance in pass-through mode (`clamp_setpoint_for_obstacles`). Does **not** apply during Manhattan sequencing — Manhattan uses `MANHATTAN_OBSTACLE_MM` for a full stop instead. |
| `OFFBOARD_STREAM_PERIOD_MS` | `50 ms` | Tick rate of the mission loop (20 Hz). Controls how frequently the active waypoint setpoint is re-sent to PX4 and how quickly a new GCS destination or obstacle detection is acted on. |

**Notes:**
- `vision_enabled` (ArUco EKF toggle) has **no effect** on Manhattan — `vision_yaw` and `vision_pose_valid` are updated by the ArUco pipeline regardless of this flag. Only the MAVLink relay to PX4 is gated.
- `DRONE_ID` and team colour affect `ned_offset_x/y`, which converts GCS arena coordinates to NED before the Manhattan distance check — the sequencer operates entirely in NED space and is team/drone-ID agnostic.

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
