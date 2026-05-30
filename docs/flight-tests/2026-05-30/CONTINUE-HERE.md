# CONTINUE HERE — yaw-flip debugging handoff (2026-05-30)

Self-contained context for a fresh Claude Code session (e.g. on a different machine
with no shared memory). Read this top to bottom; it links the detail docs.

## The problem
Drone (drone_2) **flips ~180° in yaw** when it acquires ArUco markers — most clearly
on the **first** marker of a flight (e.g. marker 12 on the y=0 wall, LH scene), then
behaves once the heading settles. Manual keyboard flight, indoor arena.

## Root cause (confirmed from PX4 ulog — see `FINDINGS.md`)
- The airframe **has no magnetometer** (`EKF2_MAG_TYPE=5`) and no indoor GPS, so
  **ArUco yaw is EKF2's only absolute heading source** (`EKF2_EV_CTRL=15`).
- At poor viewing geometry (low altitude / steep look-up) the two IPPE PnP solutions
  share ~the same position but ~180°-opposite yaw, so a flipped yaw passes the
  position gate and gets fused with nothing to veto it (EV-yaw innovations ~2.5–2.9 rad
  observed). At ≈ marker height, square-on, yaw is unambiguous → stable.
- The vision pose itself is **correct** (visYaw ≈ −90° facing marker 12). The RViz
  heading arrow rendering points −X there — a viz artifact (it renders the quaternion's
  body-X / camera-right axis), NOT a pose error.

## What's committed on `origin/main`
| Commit | What |
|---|---|
| `8ed12e5` | P4 `aruco_pose.cpp`: absolute arena/wall-side gate (fixes the *position*-reflection flip; not the same-position yaw flip) |
| `b9b5d99` | `docs/flight-tests/2026-05-30/` FINDINGS + ulog + rosbags |
| `575f05f` | S3 `main.c`: yaw-continuity reject gate (`MAX_YAW_JUMP_RAD`, drops >90° yaw jumps after the first fix) |
| *(this checkpoint)* | S3 scene-aware **heading seed** + P4 `POSE_MAX_RANGE_M` 8→5; `DRONE_ID=2` (test drone); these docs |

## The heading-seed feature (S3 `main.c`)
Goal: kill the **first-fix flip**. With no mag, seed EKF2's heading at arm from the
known launch heading so the gyro (low drift) carries it and the first ArUco fix agrees.
- Scene-aware: set from `team_color` — **LH/red → +X (0°)**, **RH/blue → −X (180°)**
  (`seed_yaw_rad`, `seed_yaw_valid`; consts `START_YAW_LH_DEG`/`START_YAW_RH_DEG`).
- Sends a **yaw-only** VISION_POSITION_ESTIMATE (position echoes EKF's own estimate at
  loose covariance) for `SEED_YAW_MS=4000` ms after arm, then stops so it can't fight
  later yaw maneuvers. Gated on `vision_enabled && seed_yaw_valid`. Toggle `SEED_YAW_ENABLE`.

## ⚠ Critical: the seed has NEVER actually run on any flight yet
Each flight had a different blocker:
- **20:48, 21:54** — ArUco EKF **disabled at arm** → seed gated off (it requires `vision_enabled`).
- **22:21** — micro-ROS link down → drone never received `team_color` → `seed_yaw_valid` false.
  (The 22:21 *rosbag* has almost no drone telemetry: 60 msgs, 0 `vision_pose`.)

## NEXT STEPS
1. **Get the 22:21 PX4 ulog from drone_2** (onboard, link-independent). See
   `TODO-download-ulogs.md`. Read with pyulog. Check: did the seed VPE arrive (EV-yaw at
   start heading during the arm window)? EKF yaw + EV-yaw innovation at first fix?
2. **Decide the offered robustness fix:** default the seed to **LH (+X) at boot** so it
   no longer depends on `team_color` arriving over the flaky micro-ROS link (team_color
   still overrides for RH). Implement in `team_color`/seed init in S3 `main.c`.
3. **Truly test the seed:** arm with **ArUco EKF ENABLED**, LH scene initiated AND
   received by the drone (micro-ROS up). Record `ros2 bag record -a` AND keep the ulog.
4. Optional: fix the RViz heading arrow to render true forward heading, not camera-right.

## How to read the logs
- **rosbag (mcap):** copy the `.mcap` + rename `metadata(1).yaml`→`metadata.yaml` into a
  temp dir, `source /opt/ros/jazzy/setup.bash`, read with `rosbag2_py.SequentialReader`
  + `rclpy.serialization.deserialize_message`. Drone under test is **drone_2** →
  topics `/drone_2/vision_pose` (PoseStamped), `/drone_2/state`, `/gcs/system/team_color`.
- **vision_yaw formula** (camera +Z = forward, projected to world XY):
  `vision_yaw = atan2( 2(qy·qz − qx·qw), 2(qx·qz + qy·qw) )`.
- **ulog:** `pip install --user --break-system-packages pyulog` (venv/ensurepip
  unavailable on the field laptop); or open in QGroundControl / PX4 Flight Review.
  Key topics: `vehicle_attitude` (yaw from q), `vehicle_attitude_setpoint.yaw_body`,
  `estimator_innovations.heading` (EV-yaw innovation), `actuator_armed`.

## Key facts / params
- No magnetometer (hardware). `EKF2_MAG_TYPE=5`, `EKF2_EV_CTRL=15`, `EKF2_EVA_NOISE=0.10`
  (tight — could loosen to ~0.3–0.5 to smooth bad yaw). `EKF2_HGT_REF=2` (range).
- Firmware files get re-owned to uid 1001 by the Docker build → `sudo chown -R 1000:1000
  drone-firmware/v2.0/<target>` before editing.
- Detail analysis: `FINDINGS.md`. Reminder: `TODO-download-ulogs.md`.
