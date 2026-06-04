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
