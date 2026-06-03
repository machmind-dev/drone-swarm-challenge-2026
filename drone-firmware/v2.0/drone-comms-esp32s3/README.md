# Communications Module – ESP32 S3

ESP-IDF firmware for the [Seeed Studio XIAO ESP32-S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) board acting as the communication bridge in the v2.0 dual-MCU architecture. Receives sensor data from the ESP32-P4 over UART, forwards obstacle distances and pose estimates to PX4 via MAVLink, and maintains the micro-ROS link to the Ground Control Station.

## Role in the System

```
ESP32-P4 (Navigation)
  │  UART binary frames @ 115200
  │  GPIO22 TX → GPIO3 RX
  ▼
ESP32-S3 (Communications)
  ├── MAVLink → PX4 (GPIO43 TX / GPIO44 RX, 57600 baud)
  │     OBSTACLE_DISTANCE  — 5 horizontal ToF sensors, 30° FOV each
  │     DISTANCE_SENSOR    — upward sensor (slot 5)
  │     VISION_POSITION_ESTIMATE — ArUco pose (when vision enabled)
  │     HEARTBEAT, SET_MODE, ARM, NAV commands
  └── micro-ROS → GCS (Wi-Fi)
        Publish: /drone_N/state, /drone_N/battery, /drone_N/role
        Subscribe: /gcs/drone_N/command, /gcs/drone_N/config, /gcs/drone_N/control
```

## Hardware

| Component | Part |
|-----------|------|
| MCU | Seeed Studio XIAO ESP32-S3 (240 MHz, 8 MB PSRAM) |
| P4 link | UART2 — GPIO3 RX ← P4 GPIO22, GPIO2 TX → P4 GPIO23 |
| PX4 link | UART1 — GPIO43 TX, GPIO44 RX, 57600 baud |
| USB-UART | USB-CDC → `/dev/ttyACM0` |

## UART Protocol (P4 → S3)

Binary framed, 115200 8N1. Frame layout:

```
SOF(1B) | LEN(1B) | TYPE(1B) | PAYLOAD(47B) | CRC8(1B)  =  51 bytes total
```

- SOF = `0xAB`, TYPE = `0x03` (COMBINED)
- Payload = `p4_tof_t` (18 B: 6× uint16 dist + 6× uint8 status) + `p4_pose_t` (29 B: valid + 7× float)
- CRC-8 (poly 0x07) computed over TYPE + PAYLOAD
- Protocol header: `v2.0/shared/p4_link_protocol.h`

## MAVLink Obstacle Distance Mapping

72-bin circular map at 5°/bin, bin 0 = forward, clockwise:

| Sensor | Direction | Bins |
|--------|-----------|------|
| Slot 0 | Left −90° | 51–56 |
| Slot 1 | L-front −45° | 60–65 |
| Slot 2 | Front 0° | 69, 70, 71, 0, 1, 2 |
| Slot 3 | R-front +45° | 6–11 |
| Slot 4 | Right +90° | 15–20 |
| Slot 5 | Up | separate `DISTANCE_SENSOR` (PITCH_90) |

## Console Output

```
[ToF] L90: 320mm L45: 450mm FWD: 880mm R45:  13mm R90:  12mm UP:   2mm | STATE:disarmed       VIS:N
```

## Build & Flash

Use the desktop launcher (recommended):

```
IDE - Flash S3
```

Or manually via Docker:

```bash
cd drone-firmware/v2.0/drone-comms-esp32s3
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec esp32s3_comms bash
# inside container:
source /opt/esp/idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

First build only — set the target once:
```bash
idf.py set-target esp32s3
```

## ESP-IDF Version

IDF 5.0 (Docker image: `espressif/idf:release-v5.0`)

## micro-ROS Vision Toggle

Send from GCS to enable/disable VISION_POSITION_ESTIMATE relay to PX4:

```
/gcs/drone_1/config  →  "CONFIG_VISION_ENABLE"
/gcs/drone_1/config  →  "CONFIG_VISION_DISABLE"
```

## Navigation Architecture (as of 2026-06-01, commit 9f65d04)

### EKF2 fusion — position only, no yaw

`VISION_POSITION_ESTIMATE` is sent with `cov[20] = NaN` (yaw covariance). EKF2
fuses ArUco **position only**. Gyro owns heading throughout the flight.

**Why:** Arena traversal is 15 m. Gyro drift ~0.4°/10 s → ~1.5° over a full
traverse → ~0.4 m lateral error at 15 m. RFID detection range is 1–2 m, so
heading error is within margin. Disabling yaw fusion eliminates all yaw-flip
risk permanently (IPPE ±180° ambiguity can no longer snap EKF2 heading).

`YAW_DISAMBIG_ENABLE`, `YAW_DISAMBIG_RAD`, `VISION_YAW_COV`, and
`px4_yaw_valid` have been removed. The heading seed (`SEED_YAW_ENABLE`) is
still compiled in but now has no effect — `cov[20] = NaN` means EKF2 ignores
the yaw component of the seeded message. Pending removal.

### EKF position-reset absorption gate

`PX4_RESET_DETECT_M` jump detection is gated on `last_vision_pose_ms < 2 s`.
Only absorbs genuine mid-flight EKF frame resets (vision was live). Does NOT
absorb re-acquisition jumps after a long vision dropout (those are legitimate
optical-flow corrections that should reach EKF2).

### ArUco incidence gate (P4)

`MAX_VIEW_ANGLE_DEG` relaxed from 30° to 45° (commit `bd10a7a`). Prevents the
79 s vision gap that occurred during cross-arena traversals when wall markers
were always viewed obliquely during straight +X flight.

### RViz disc — three-tier live tracking

The drone disc on `/visualization_marker` is always published (was incorrectly
deleted when ArUco became active). Position priority:

1. `vision_pose_valid` → ArUco arena position (accurate)
2. `inertial_anchor_valid && px4_pos_valid` → dead-reckon from last ArUco
   anchor via PX4 NED delta: `arena = map_home + (px4_pos − px4_home)`
3. Pre-flight → frozen at team_color home position

The GCS `_marker_cb` feeds every disc CYLINDER into the trail deque, so the
`/drone_N/trail` Path also stays live during vision gaps.

### Commanding — Ollama GCS

Setpoints arrive as `PoseStamped` on `/gcs/drone_{ID}/control` in **arena
coordinates**. `control_callback` subtracts `ned_offset` before sending to
PX4, so Ollama always speaks arena frame regardless of LH/RH scene. Launch via
`GCS - Swarm Ollama.desktop` (model: gemma3:1b, arena 0–20 × 0–10 m).

### DRONE_ID

`#define DRONE_ID` in `main/main.c` is a **per-flash local value** — set it to
the airframe number before building. It is intentionally not committed (kept at
2 in the repo as a neutral default).

## Pending Work

| # | Item | Decisions |
|---|------|-----------|
| 1 | **Manhattan waypoint navigation** | Implement after basic fly-to-point is confirmed in arena. When a destination arrives: break into 2 m steps, X-leg first then Y-leg. At each waypoint: yaw to face direction of travel, check ToF, proceed or hold until clear. On obstacle: stop and wait for clearance. New destination mid-flight: stop/hover and restart sequence from current position. Arrival tolerance: 1 m. |
| 2 | **Remove dead heading-seed code** | `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` are no longer effective. Remove once nav is confirmed stable. |
