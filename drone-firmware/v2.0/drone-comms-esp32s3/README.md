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

## To Do

### Pre-Finals Critical

- [ ] **Lower `C2_FAIL_THRESHOLD` to 1** (`main/main.c:157`) — currently set to 3 (6 s timeout) for WiFi RTT tolerance during testing. Must be lowered to 1 (2 s) before the competition flight.

- [ ] **ARM-time inertial anchor fix** (`main/main.c:~874`) — ARM handler already captures `px4_home_x/y/z` but does not set `inertial_anchor_valid = true` or seed `map_home_x/y` from the known team-colour home. This causes a ~47 s frozen trail at the start of each flight. Fix: set `inertial_anchor_valid = true` and seed `map_home_x/y` at ARM time.

### Navigation

- [ ] **Manhattan waypoint navigation** — When a GCS destination arrives: break into 2 m steps, X-leg first then Y-leg. At each intermediate waypoint: yaw to face direction of travel, check ToF, proceed or hold until clear. On obstacle: stop and wait for clearance. New destination mid-flight: stop/hover and restart sequence from current position. Arrival tolerance: 1 m. Implement after basic fly-to-point is confirmed in arena.

### Analysis

- [ ] **Analyse drift test `log_63_UnknownDate.ulg`** (`logs/2026-06-01_2109_drone4/`) — 5-minute aggressive inertial-only flight. Goal: quantify position error accumulation over time to set the mission planning envelope for how long the drone can fly without a position correction.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

### Cleanup

- [ ] **Remove dead heading-seed code** — `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` in `main/main.c` are no longer effective. Remove once nav is confirmed stable.

- [ ] **Review `simulate_drones.py` active drone list** (`ground-station-software/swarm/algorithm/simulate_drones.py:20`) — `ACTIVE_DRONES = [2]` is hardcoded. Update to reflect the drones used at the finals.

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
