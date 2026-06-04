# Comms Module – ESP32 S3

ESP-IDF firmware for the [Seeed Studio XIAO ESP32-S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) board acting as the communication bridge in the v2.0 dual-MCU architecture. Receives sensor data from the ESP32-P4 over UART, forwards obstacle distances and pose estimates to PX4 via MAVLink, and maintains the micro-ROS link to the Ground Control Station.

## Configuration

### DRONE_ID

`#define DRONE_ID` in `main/main.c` is a **per-flash local value** — set it to the airframe number before building. It is intentionally not committed (kept at `2` in the repo as a neutral default).

### micro-ROS Agent Port

The micro-ROS UDP port is derived automatically from `DRONE_ID`: `port = 8880 + DRONE_ID` (e.g. drone 1 → 8881, drone 2 → 8882). Set the agent IP via `idf.py menuconfig` → **micro-ROS Settings → Agent IP**.

## GCS RQT Panel — Vision Toggle

Enable/disable VISION_POSITION_ESTIMATE relay to PX4 via the RQT buttons panel on the GCS:

```
/gcs/drone_N/config  →  "CONFIG_VISION_ENABLE"
/gcs/drone_N/config  →  "CONFIG_VISION_DISABLE"
```

## GCS RQT Panel — Control Source Toggle

Switch between GCS Offboard control and RC manual control via the RQT buttons panel on the GCS:

```
/gcs/drone_N/config  →  "CONFIG_SOURCE_GCS"
/gcs/drone_N/config  →  "CONFIG_SOURCE_RC"
```

- `CONFIG_SOURCE_GCS` — sets PX4 to Offboard mode; position setpoints accepted from `/gcs/drone_N/control`
- `CONFIG_SOURCE_RC` — sets PX4 to Stabilized mode; RC transmitter takes over

## Manhattan Waypoint Navigation Parameters

All parameters are `#define` constants in `main/main.c`.

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `MANHATTAN_STEP_M` | `1.0 m` | Maximum distance between intermediate waypoints. X-leg is broken into steps of this size first, then Y-leg. Smaller values = smoother path but more waypoints and more frequent ToF checks. |
| `MANHATTAN_ARRIVAL_M` | `1.0 m` | Radius within which a waypoint is considered reached and the sequencer advances to the next one. Must be ≤ `MANHATTAN_STEP_M` to avoid skipping waypoints. |
| `MANHATTAN_PASSTHROUGH_M` | `1.0 m` | Setpoints with a Manhattan distance (\|Δx\| + \|Δy\|) from the drone's current position at or below this threshold bypass the sequencer entirely and are streamed directly to PX4. Covers keyboard fly-mode (0.5 m increments) and rotation-only commands (0 m). |
| `MANHATTAN_OBSTACLE_MM` | `500 mm` | ToF clearance threshold in the direction of travel. If any horizontal sensor reads below this the sequencer stops, enters obstacle-hold, and waits for a new GCS destination. Does **not** auto-resume when the obstacle clears — requires explicit new setpoint. |
| `MANHATTAN_MAX_WPS` | `40` | Static waypoint array size. At 1 m steps the worst-case path across the full arena (20 m + 10 m) needs 30 entries; 40 gives margin for rounding remainder waypoints. Only increase if `MANHATTAN_STEP_M` is reduced below ~0.7 m. |
| `COLLISION_MARGIN_M` | `0.3 m` | Safety buffer subtracted from ToF clearance in pass-through mode (`clamp_setpoint_for_obstacles`). Does **not** apply during Manhattan sequencing — Manhattan uses `MANHATTAN_OBSTACLE_MM` for a full stop instead. |
| `OFFBOARD_STREAM_PERIOD_MS` | `50 ms` | Tick rate of the mission loop (20 Hz). Controls how frequently the active waypoint setpoint is re-sent to PX4 and how quickly a new GCS destination or obstacle detection is acted on. |

**Notes:**
- `vision_enabled` (ArUco EKF toggle) has **no effect** on Manhattan — `vision_yaw` and `vision_pose_valid` are updated by the ArUco pipeline regardless of this flag. Only the MAVLink relay to PX4 is gated.
- `DRONE_ID` and team colour affect `ned_offset_x/y`, which converts GCS arena coordinates to NED before the Manhattan distance check — the sequencer operates entirely in NED space and is team/drone-ID agnostic.

## Pending Work

| # | Item | Decisions |
|---|------|-----------|
| 1 | **Manhattan waypoint navigation** | Implement after basic fly-to-point is confirmed in arena. When a destination arrives: break into 2 m steps, X-leg first then Y-leg. At each waypoint: yaw to face direction of travel, check ToF, proceed or hold until clear. On obstacle: stop and wait for clearance. New destination mid-flight: stop/hover and restart sequence from current position. Arrival tolerance: 1 m. |
| 2 | **Remove dead heading-seed code** | `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` are no longer effective. Remove once nav is confirmed stable. |

## micro-ROS Topics

### ESP32-S3 → GCS / RViz (published by drone)

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_N/state` | `std_msgs/String` | Flight state (disarmed / armed / flying / landing) |
| `/drone_N/role` | `std_msgs/String` | Assigned mission role |
| `/drone_N/battery` | `std_msgs/Int8` | Battery level (%) |
| `/drone_N/vision_pose` | `geometry_msgs/PoseStamped` | ArUco world-frame pose |
| `/visualization_marker` | `visualization_msgs/Marker` | RViz drone disc + box markers |

### GCS → ESP32-S3 (published by GCS, received by drone)

| Topic | Type | Description |
|-------|------|-------------|
| `/gcs/drone_N/command` | `std_msgs/String` | Flight commands (ARM, TAKEOFF, LAND, etc.) |
| `/gcs/drone_N/config` | `std_msgs/String` | Runtime config (CONFIG_VISION_ENABLE / DISABLE) |
| `/gcs/drone_N/control` | `geometry_msgs/PoseStamped` | Position setpoint in arena coordinates |
| `/gcs/system/team_color` | `std_msgs/String` | Scene selection — `red` (LH) or `blue` (RH) |
