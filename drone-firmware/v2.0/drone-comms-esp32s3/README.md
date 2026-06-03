# Comms Module – ESP32 S3

ESP-IDF firmware for the [Seeed Studio XIAO ESP32-S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) board acting as the communication bridge in the v2.0 dual-MCU architecture. Receives sensor data from the ESP32-P4 over UART, forwards obstacle distances and pose estimates to PX4 via MAVLink, and maintains the micro-ROS link to the Ground Control Station.

## Configuration

### DRONE_ID

`#define DRONE_ID` in `main/main.c` is a **per-flash local value** — set it to the airframe number before building. It is intentionally not committed (kept at `2` in the repo as a neutral default).

### micro-ROS Agent Port

The micro-ROS UDP port is derived automatically from `DRONE_ID`: `port = 8880 + DRONE_ID` (e.g. drone 1 → 8881, drone 2 → 8882). Set the agent IP via `idf.py menuconfig` → **micro-ROS Settings → Agent IP**.

## micro-ROS Vision Toggle

Send from GCS to enable/disable VISION_POSITION_ESTIMATE relay to PX4:

```
/gcs/drone_1/config  →  "CONFIG_VISION_ENABLE"
/gcs/drone_1/config  →  "CONFIG_VISION_DISABLE"
```

## micro-ROS Control Source Toggle

Send from GCS to switch between GCS Offboard control and RC manual control:

```
/gcs/drone_1/config  →  "CONFIG_SOURCE_GCS"
/gcs/drone_1/config  →  "CONFIG_SOURCE_RC"
```

- `CONFIG_SOURCE_GCS` — sets PX4 to Offboard mode; position setpoints accepted from `/gcs/drone_N/control`
- `CONFIG_SOURCE_RC` — sets PX4 to Stabilized mode; RC transmitter takes over

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


## Pending Work

| # | Item | Decisions |
|---|------|-----------|
| 1 | **Manhattan waypoint navigation** | Implement after basic fly-to-point is confirmed in arena. When a destination arrives: break into 2 m steps, X-leg first then Y-leg. At each waypoint: yaw to face direction of travel, check ToF, proceed or hold until clear. On obstacle: stop and wait for clearance. New destination mid-flight: stop/hover and restart sequence from current position. Arrival tolerance: 1 m. |
| 2 | **Remove dead heading-seed code** | `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` are no longer effective. Remove once nav is confirmed stable. |

## micro-ROS Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_N/state` | `std_msgs/String` | Flight state (disarmed / armed / flying / landing) |
| `/drone_N/role` | `std_msgs/String` | Assigned mission role |
| `/drone_N/battery` | `std_msgs/Int8` | Battery level (%) |
| `/drone_N/vision_pose` | `geometry_msgs/PoseStamped` | ArUco world-frame pose |
| `/visualization_marker` | `visualization_msgs/Marker` | RViz drone disc + box markers |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `/gcs/drone_N/command` | `std_msgs/String` | Flight commands (ARM, TAKEOFF, LAND, etc.) |
| `/gcs/drone_N/config` | `std_msgs/String` | Runtime config (CONFIG_VISION_ENABLE / DISABLE) |
| `/gcs/drone_N/control` | `geometry_msgs/PoseStamped` | Position setpoint in arena coordinates |
| `/gcs/system/team_color` | `std_msgs/String` | Scene selection — `red` (LH) or `blue` (RH) |
