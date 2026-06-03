# Vision Module – ESP32 P4

ESP-IDF firmware for the Waveshare ESP32-P4 WiFi6 board with OV5647 MIPI-CSI camera.
Performs real-time ArUco marker detection and outputs world-frame pose estimates over UART.

## Hardware

| Component | Part |
|-----------|------|
| MCU | Waveshare ESP32-P4 WiFi6 (360 MHz, 32 MB PSRAM) |
| Camera | OV5647 MIPI-CSI, 800×800 RAW8 @ 50 fps → ISP → RGB565 |
| USB-UART | WCH CH343 bridge → `/dev/ttyACM0` |

Detection pipeline: 800×800 capture → center-crop to 800×600 → resize to 320×240 (QVGA) for ArUco.

## Camera Exposure Tuning

The OV5647 AEC (auto-exposure) brightness target is set via `CONFIG_VISION_AEC_TARGET`
in `main/aruco_pose.cpp`. Range is 0–47; the sensor target is `value × 4.92` out of 255.

```cpp
#define CONFIG_VISION_AEC_TARGET 15   // ← tune per venue
```

| Value | Sensor target | Use when |
|-------|--------------|----------|
| 47 | ~91% (max) | Dark indoor arena, no windows |
| 20 | ~38% | Indoor with some ambient light |
| 15 | ~29% | Daylit venue, windows visible — **current default** |
| 10 | ~19% | Strong window glare / direct sunlight |

To override without editing source, add to `main/CMakeLists.txt`:
```cmake
target_compile_options(${COMPONENT_LIB} PRIVATE ... "-DCONFIG_VISION_AEC_TARGET=10")
```

Confirmed at value 15: ArUco markers detected correctly in daylit conditions (2026-05-25).

## Detection Resolution

Two modes are selectable at the top of `main/aruco_pose.cpp`:

```cpp
#define VISION_RES_QVGA   // 320×240 — ~8 m detection range (default)
// #define VISION_RES_HVGA // 480×320 — ~12 m detection range, slower
```

Stream preview is always 80×60 (10× downscale of the detection crop).

## Pose-Acceptance Tuning

Two stateless filters in the per-marker PnP loop in `main/aruco_pose.cpp` reject
unreliable pose solutions before they reach the averager, `best_R_wc`, or the
S3/PX4 EKF. Both are critical — without them PX4 will physically fly the drone
to chase a wrong pose (long-range noise) or rotate it 180° (PnP planar
ambiguity flips).

### 1. Distance gate — `POSE_MAX_RANGE_M`

```cpp
#define POSE_MAX_RANGE_M  8.0f   // metres; match to detection resolution
```

Markers detected farther than this drop out of the pose averaging entirely.
At QVGA a 50 cm marker projects to ~12 pixels at 8 m and ~7 pixels at 11 m —
beyond ~8 m, sub-pixel corner noise produces PnP solutions metres off truth
(verified by stationary test: at 11 m the reported drone Z was 6 m vs. 1 m
truth). The matching value if switching to HVGA is ~12.

| Resolution | Recommended gate |
|------------|------------------|
| QVGA 320×240 (default) | **8.0 m** |
| HVGA 480×320 | ~12 m |

### 2. Orientation guard — R<sub>l2c</sub>[1][1] threshold

```cpp
if (R_l2c.at<double>(1, 1) > -0.8) continue;   // current value
```

Wall markers are mounted with their printed "+Y" axis pointing world-up. The
correct PnP solution puts marker +Y near image-up in the camera frame, i.e.
`R_l2c[1][1] ≈ -1`. The 90°/180°/270° PnP planar-ambiguity flips that
`SOLVEPNP_IPPE_SQUARE` occasionally returns instead land at `R[1][1] ≈ 0` (90°),
`+1` (180°), or `0` (270°). The guard rejects anything that isn't clearly
right-side up.

| Threshold | Accepts up to | When to use |
|-----------|---------------|-------------|
| `-0.5`    | ~60° camera tilt | Aggressive flight envelope; risk of letting borderline wrong solutions through |
| **`-0.8`** (current default) | ~37° camera tilt | Hover/cruise — recommended; catches the wrong solutions that slipped past `-0.5` during bench testing |
| `-0.95`   | ~18° camera tilt | Very tight; will start rejecting valid solutions when the drone banks |

Tighter (closer to `-1`) → fewer wrong solutions accepted, but more correct
solutions rejected during banking turns. If pose updates become too sparse
during flight, loosen toward `-0.6`.

**Assumes camera is mounted upright on the drone** (image-up = world-up when
drone is level). If the camera is rotated 90° on the mount, the check needs to
move to a different row/column of `R_l2c` accordingly.

## Tools

| Script | Purpose |
|--------|---------|
| `tools/stream_view.py /dev/ttyACM0 921600` | Live 80×60 detection crop with marker outlines and POSE overlay |
| `tools/view_frame.py /dev/ttyACM0 115200` | Two-panel debug viewer — color RGB (left) and grayscale (right) via hex dump protocol |

## Key Design Decisions

- **No per-frame STREAMOFF** — camera streams continuously so ISP AE/AWB/lens-shading
  correction converges. Previous per-frame STREAMOFF caused dark vignette frames.
- **`SPIRAM_MALLOC_ALWAYSINTERNAL=131072`** — forces OpenCV's 76 800-byte adaptive
  threshold workspace into SRAM, eliminating the PSRAM DMA spinlock conflict.
- **5 s ISP warmup** before the first DQBUF, matching `camera_view_mode` behaviour.
- **R_lw world-pose formula** — verified correct for all 4 SDC26 arena walls.

## ArUco Markers — Navigation Inside Arena

Four ArUco markers (IDs 11–14) are mounted one per arena wall at a known world-frame position. During flight the P4 detects visible markers, solves the camera pose via `SOLVEPNP_IPPE_SQUARE`, applies the arena-side geometric gate and reprojection filter, and transmits the averaged world-frame position + quaternion to the S3 over UART. The S3 relays this as a `VISION_POSITION_ESTIMATE` MAVLink message to PX4's EKF2, which fuses it as the primary absolute horizontal position source (no GPS, no magnetometer indoors). Navigation is enabled/disabled from the GCS via `COMMAND_VISION_ON / OFF`.

Detection runs at ~3–4 fps on the ESP32-P4 at 360 MHz. `stream_view.py` shows the ISP-corrected colour view at 80×60 with detected marker outlines in white and the world-pose overlay in cyan (top-left).

![ArUco detection working](docs/aruco_detection_working.png)

Console output format:
```
M1:4.94m POSE:1:15.538:6.171:4.056:0.059:0.736:-0.046:0.673
```
- `POSE:N:x:y:z:qx:qy:qz:qw` — averaged world-frame position (metres) + quaternion sent to PX4 EKF2

**Known issues**

- **OPEN-7 — No position-innovation gate (flyaway risk).** When IPPE planar ambiguity produces a reflected solution that happens to satisfy the arena-bounds check, the EKF receives a position fix with a ~6 m lateral error. PX4 may snap its position estimate to this fix and fly toward the erroneous setpoint. Fix: reject any `VISION_POSITION_ESTIMATE` that disagrees with the current PX4 local position by more than 2–3 m before relaying it from the S3.

- **OPEN-8 — EV yaw fusion 180° flip on first fix.** `cov[20] = NaN` is intended to disable EKF2 yaw fusion, but PX4 may ignore message covariance for external-vision yaw and use `EKF2_EVA_NOISE` instead, leaving yaw fusion active. At poor viewing geometry (low altitude, steep look-up) the two IPPE solutions have nearly the same position but ~180°-opposite yaw; if the wrong one is selected and yaw fusion is still active, EKF2 snaps heading and the drone rotates physically. Fix: set `EKF2_EV_CTRL = 7` in PX4 params (disables yaw bit, keeps position and velocity fusion) so the gyro owns heading throughout the flight.

---

## ArUco Markers — Target Box Position Detection

Target boxes are fitted with ArUco markers at known offsets. During a low-altitude pass the P4 reports each detected marker ID and its distance; the S3 forwards this in the COMBINED frame to the GCS. The ground station uses the marker ID to identify which box the drone is currently above and triggers the scoring sequence (RFID read / payload drop). Detection at close range (< 2 m) is reliable with the current QVGA pipeline and AEC target.

Console output format:
```
M1:4.94m POSE:1:15.538:6.171:4.056:0.059:0.736:-0.046:0.673
```
- `M<id>:<dist>m` — distance to each detected marker; used by the GCS to confirm box identity

No known issues.

---

## ToF — Obstacle Detection

Six VL53L1X time-of-flight sensors provide radial short-range distance measurements around the drone body. The P4 polls all six sensors, packages the readings into the COMBINED UART frame, and the S3 unpacks them and forwards them to PX4 as `OBSTACLE_DISTANCE` and `DISTANCE_SENSOR` MAVLink messages at 20 Hz. PX4 collision prevention (`CP_DIST = 0.5 m`, `CP_GUIDE_ANG = 30°`) uses this data to decelerate and hold the drone before contact. The primary altitude source is a downward-facing LiDAR (baro disabled); ToF covers the horizontal plane only.

No known issues.

---

## Fixed Issues

### 1. Yaw flip on wall markers — IPPE ambiguity + mag-less EKF ⚠ Partial fix / root cause identified 2026-05-30

**Symptom** (flight-tested 2026-05-29/30, LH scene, keyboard manual control):
Facing ArUco 12 (y=0 wall) or ArUco 14 (y=10 wall) head-on, the drone flipped
180° in yaw. A direct approach to ArUco 13 (x=0 wall) as the *first* marker of
the flight sent the drone flying off-axis into the wall — while the *same*
marker stabilised correctly when it was acquired *after* marker 12. That
order-dependence was the tell.

**Root cause.** For a vertical wall marker the IPPE planar ambiguity is a ~180°
rotation about the marker's vertical axis. In world frame this is
`R_wc_flip = Rot(worldZ, 180°)·R_wc`, which reflects the recovered drone
position *across the wall plane*: the wrong solution lands **behind the wall,
outside the arena**, with a 180°-flipped yaw — and both the bad position and bad
yaw get fused into EKF2. The marker-frame upright guard (`R_l2c[1][1] < -0.8`)
cannot see this because a vertical-axis flip keeps the marker upright for both
solutions.

**Superseded first attempt (commit `24eb720`).** Switched to
`solvePnPGeneric(SOLVEPNP_IPPE)` and picked the solution whose world-frame yaw
was closest to the previous accepted yaw (`s_prev_yaw`, temporal continuity).
This **did not work**: temporal continuity has no absolute anchor, so on the
first frame (`s_prev_yaw = NaN`) it blindly took `sol 0` and latched onto it.
Whether that was correct depended on acquisition order — hence marker 13 working
or crashing depending on what was seen before it.

**Fix.** Replaced the temporal heuristic with an **absolute, history-free
geometric gate** in `main/aruco_pose.cpp`. For each IPPE solution the recovered
drone position must (a) lie on the arena-facing side of the marker face
(`(p_w − marker)·normal > 0` — you can only detect a face from in front of it,
so the true solution always satisfies this and the mirror solution never does)
and (b) fall inside the arena envelope (`[−1,21]×[−1,11]` m). Among the
survivors the lowest reprojection error wins. No temporal state, correct on the
very first frame. A rate-limited `AMB …` serial trace logs the surviving
solution and recovered `(x,y)` for in-flight confirmation (remove once
verified). No FPS impact — IPPE computes both solutions internally regardless.

**Deeper root cause — found in the PX4 log (flight 2026-05-30, drone_2).** The gate
above stops the *position*-reflection flip, but flights kept flipping on first
acquisition. The flight-controller log explains why:

- The airframe **has no magnetometer** (`EKF2_MAG_TYPE = 5`) and indoors there is no
  GPS — so **ArUco yaw is EKF2's only absolute heading source** (`EKF2_EV_CTRL = 15`).
- At poor geometry (low altitude / steep look-up) the two IPPE solutions have nearly
  the **same position but ~180°-opposite yaw** — both pass the position gate, so a
  flipped *yaw* can still be selected. With no mag to veto it, EKF2 fuses it
  (observed external-vision heading innovations of ~2.5–2.9 rad ≈ 180°), the heading
  estimate snaps, and the drone rotates. At ~marker height square-on, yaw is
  unambiguous → innovation ≈ 0 → stable. Hence "first bad-angle marker flips, then
  everything works."

**Resolved (2026-06-01, commit `bd10a7a`).** EV yaw fusion has been **disabled
entirely** on the S3 side (`cov[20] = NaN`). EKF2 now fuses ArUco position
only; gyro owns heading throughout the flight. Gyro drift over a 15 m arena
traverse is ~1.5°, well within the 1–2 m RFID capture window. This eliminates
all yaw-flip risk. The yaw-continuity gate (`MAX_YAW_JUMP_RAD`) is retained on
the S3 to protect `vision_yaw` (used only for the continuity gate itself, not
fused). The heading seed is now a no-op. Full analysis and flight logs:
[`docs/flight-tests/2026-05-31/FINDINGS.md`](../../../docs/flight-tests/2026-05-31/FINDINGS.md).

### Incidence gate — relaxed to 45° (commit `bd10a7a`)

`MAX_VIEW_ANGLE_DEG` changed from 30° → 45° (`MIN_VIEW_COS` 0.866 → 0.707).
The 30° gate caused 79 s vision gaps during cross-arena traversals: the drone
flew parallel to the long walls so pillar markers were always viewed obliquely
and rejected. 45° doubles the acceptance cone (90° total) while still excluding
edge-on views where IPPE yaw is ambiguous. Since yaw is no longer fused into
EKF2, the risk of accepting a yaw-ambiguous pose is limited to the S3
continuity gate only.

### 2. Emergency-landing rotates drone to yaw=0 before descending ✓ Fixed 2026-05-30

**Symptom.** Pressing emergency land from rqt caused the drone to rotate
(sometimes 180°) before descending.

**Cause.** `mav_eland()` in `s3-comms/main.c` sent `MAV_CMD_NAV_LAND` with
`param4 = 0` — commanding a yaw-to-North before landing.

**Fix (commit `cc81f6d`).** Changed `param4` from `0` to `NAN` in the
`mavlink_msg_command_long_pack` call. Drone now holds its current heading
throughout the landing.
