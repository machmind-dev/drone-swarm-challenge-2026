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

```cpp
#define CONFIG_VISION_AEC_TARGET 15   // ← tune per venue
```

| Value | Sensor target | Use when |
|-------|--------------|----------|
| 47 | ~91% (max) | Dark indoor arena, no windows |
| 20 | ~38% | Indoor with some ambient light |
| 15 | ~29% | Daylit venue, windows visible — **current default** |
| 10 | ~19% | Strong window glare / direct sunlight |

## Detection Resolution

```cpp
#define VISION_RES_QVGA   // 320×240 — ~8 m detection range (default)
// #define VISION_RES_HVGA // 480×320 — ~12 m detection range, slower
```

## Pose-Acceptance Tuning

### Distance gate — `POSE_MAX_RANGE_M`

```cpp
#define POSE_MAX_RANGE_M  8.0f   // metres; match to detection resolution
```

### Orientation guard — R<sub>l2c</sub>[1][1] threshold

```cpp
if (R_l2c.at<double>(1, 1) > -0.8) continue;   // current value
```

## Troubleshooting & Debugging

| Launcher | Purpose |
|----------|---------|
| <img src="../../../launchers/stream_ico.png" width="48"><br>**IDE — Stream View** | Live camera stream with ArUco marker outlines and ToF overlay |
| *(pending)*<br>**Mach Mind – Arena View** | Two-panel debug viewer — colour RGB (left) and grayscale (right) |

## Key Design Decisions

- **No per-frame STREAMOFF** — camera streams continuously so ISP AE/AWB/lens-shading
  correction converges. Previous per-frame STREAMOFF caused dark vignette frames.
- **`SPIRAM_MALLOC_ALWAYSINTERNAL=131072`** — forces OpenCV's 76 800-byte adaptive
  threshold workspace into SRAM, eliminating the PSRAM DMA spinlock conflict.
- **5 s ISP warmup** before the first DQBUF, matching `camera_view_mode` behaviour.
- **R_lw world-pose formula** — verified correct for all 4 SDC26 arena walls.

## ArUco Markers — Navigation Inside Arena

16 ArUco markers (IDs 1–16) are mounted on arena poles at two heights (4 m and 2 m) with known world-frame positions:

| IDs | Location | Facing |
|-----|----------|--------|
| 1, 9 | x=20 end pole (x=20, y=5) | −X (yaw 270°) |
| 5, 13 | x=0 end pole (x=0, y=5) | +X (yaw 90°) |
| 6, 14 / 7, 15 / 8, 16 | y=10 wall at x=5 / x=10 / x=15 | −Y (yaw 0°) |
| 4, 12 / 3, 11 / 2, 10 | y=0 wall at x=5 / x=10 / x=15 | +Y (yaw 180°) |

Odd-numbered IDs are at z=4 m; even-numbered at z=2 m. All positions are defined in `MARKER_MAP[]` in `main/aruco_pose.cpp`.

During flight the P4 detects visible markers, solves the camera pose via `SOLVEPNP_IPPE_SQUARE`, applies the arena-side geometric gate and reprojection filter, and transmits the averaged world-frame position + quaternion to the S3 over UART. The S3 relays this as a `VISION_POSITION_ESTIMATE` MAVLink message to PX4's EKF2, which fuses it as the primary absolute horizontal position source (no GPS, no magnetometer indoors). Navigation is enabled/disabled from the GCS via `COMMAND_VISION_ON / OFF`.

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

Target boxes are fitted with ArUco markers (IDs 31–36 and 41–46). During a low-altitude pass the P4 detects visible box markers, computes their world positions, and reports each marker ID and distance. The S3 forwards this in the COMBINED frame to the GCS. The ground station uses the marker ID to identify which box the drone is currently above and triggers the scoring sequence (RFID read / payload drop). Detection at close range (< 2 m) is reliable with the current QVGA pipeline and AEC target.

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
