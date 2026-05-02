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
| Camera | OV5647 MIPI-CSI, 800×640 @ 50 fps, RGB565 ISP output |
| USB-UART | WCH CH343 bridge → `/dev/ttyACM0` |

## Build & Flash

```bash
cd drone-vision/ESP32P4
docker compose -f docker/docker-compose.yml run --rm esp32p4_vision idf.py build
docker compose -f docker/docker-compose.yml run --rm esp32p4_vision idf.py -p /dev/ttyACM0 flash
```

## Stream Viewer

```bash
python3 tools/stream_view.py /dev/ttyACM0 921600
```

Displays three panels: RGB565→RGB | RGB565→BGR | Luminance Y.
POSE overlay appears top-left when a known arena marker is in view.

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
