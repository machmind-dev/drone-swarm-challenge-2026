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

## Detection Resolution

Two modes are selectable at the top of `main/aruco_pose.cpp`:

```cpp
#define VISION_RES_QVGA   // 320×240 — ~8 m detection range (default)
// #define VISION_RES_HVGA // 480×320 — ~12 m detection range, slower
```

Stream preview is always 80×60 (10× downscale of the detection crop).

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

Next step: wire P4 TX → S3 UART2 (GPIO8).  S3 parses `POSE:` lines and forwards
`VISION_POSITION_ESTIMATE` MAVLink messages to PX4 via the existing
`mav_send_vision_estimate()` call.
