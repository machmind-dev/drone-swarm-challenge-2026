# ESP32-P4 ArUco Vision Board

ESP-IDF firmware for the Waveshare ESP32-P4 WiFi6 board with OV5647 MIPI-CSI camera.
Performs real-time ArUco marker detection and outputs world-frame pose estimates over UART.

## Current State (May 2026)

**ArUco detection and POSE estimation working.**

![ArUco detection working](docs/aruco_detection_working.png)

`stream_view.py` shows the ISP-corrected color view at 80×60 with detected marker
outlines drawn white and the world-pose overlay in cyan (top-left).  Detection runs
at ~3–4 fps on the ESP32-P4 at 360 MHz.

Console output format:
```
M1:4.94m POSE:1:15.538:6.171:4.056:0.059:0.736:-0.046:0.673
```
- `M<id>:<dist>m` — distance to each detected marker
- `POSE:N:x:y:z:qx:qy:qz:qw` — averaged world-frame position (metres) + quaternion

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

## Build & Flash

Use the launcher script (recommended):

```bash
launchers/ubuntu-gnome-pc/launch-drone-vision-p4.sh
```

Or manually via Docker:

```bash
cd drone-vision/ESP32P4
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec esp32p4_vision bash
# inside container:
idf.py set-target esp32p4   # first time only
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

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

## UART Output → ESP32-S3

Binary framed COMBINED frames are transmitted over UART1 at 115200 baud at 20 Hz.

| Signal | P4 GPIO | → | S3 GPIO |
|--------|---------|---|---------|
| TX | GPIO22 | → | GPIO3 (RX) |
| RX | GPIO23 | ← | GPIO2 (TX) |
| GND | GND | — | GND |

Frame: `SOF(0xAB) | LEN | TYPE(0x03) | p4_combined_t(47B) | CRC8` = 51 bytes total.
Protocol defined in `v2.0/shared/p4_link_protocol.h`, transmitted by `main/p4_link_tx.c`.

The S3 decodes the frames and forwards obstacle data as `OBSTACLE_DISTANCE` and
`DISTANCE_SENSOR` MAVLink messages to PX4, and relays ArUco pose as
`VISION_POSITION_ESTIMATE` when vision is enabled from the GCS.

## Known Issues / TODO

### 1. Yaw oscillation on wall markers — IPPE planar ambiguity not fully gated

**Symptom** (flight-tested 2026-05-29, Red scene, keyboard manual control):
After applying the distance gate (`POSE_MAX_RANGE_M`) and the face-normal
orientation guard (`R_l2c[1][1] < -0.8`), drone behaviour is **slower and
more stable** but a deterministic rotation oscillation remains: when the
drone is positioned to face ArUco 12 (y=0 wall) or ArUco 14 (y=10 wall) at
short range, the moment the marker becomes visible the drone rotates 180°,
sees the opposite-wall marker, rotates 180° back, repeats.

**Hypothesis.** The orientation guard was designed for the case where
`SOLVEPNP_IPPE_SQUARE` flips its solution by 180° around the marker's *face
normal*. The actual IPPE planar ambiguity rotates by 180° around an axis
*in the marker plane* perpendicular to the line-of-sight. For a frontal
view of a wall marker this leaves marker +Y mostly upright in the camera
frame (so `R_l2c[1][1] ≈ -1` and the guard accepts), while still producing
an `R_wc` that yields a vision_yaw 180° off the truth.

**Proposed fix (next iteration).** Switch from `cv::solvePnP(...
SOLVEPNP_IPPE_SQUARE)` to `cv::solvePnPGeneric(... SOLVEPNP_IPPE)`, which
returns *both* ambiguous solutions. Compute world pose for each, then pick
the one whose drone position is closer to the previous frame's PX4
inertial position (`px4_pos_x/y/z` on the S3 side, forwarded back to the
P4 over UART, or alternatively keep a P4-local previous pose and use
that). This is the standard remedy for IPPE ambiguity in production
systems and addresses the actual ambiguity axis rather than only the
face-normal subset.

**Alternative diagnostic step.** If we want to *confirm* the hypothesis
before changing the PnP call, add a `printf` on each accepted marker
dumping `id`, `R[0][0..2]`, `R[1][0..2]`, `R[2][2]` and the computed yaw.
Capture the log around a 180° rotation event, then pick the row/column
that distinguishes correct vs. wrong solutions.

### 2. Emergency-landing rotates drone to yaw=0 before descending

**Symptom.** Pressing emergency land from rqt causes the drone to rotate
(sometimes 180°) before descending.

**Cause.** `s3-comms/main.c:303-312`, `mav_eland()` sends
`MAV_CMD_NAV_LAND` with **param4 = 0** instead of `NaN`. Param4 of NAV_LAND
is the target yaw for the landing approach — `0` means "yaw to 0° (North)
before descending"; `NaN` means "hold current heading."

**Fix.** Change param4 from `0` to `NAN` in the `mavlink_msg_command_long_pack`
call. Single-character edit, no behavioural side effects other than the
drone landing in whichever direction it was already pointing.
