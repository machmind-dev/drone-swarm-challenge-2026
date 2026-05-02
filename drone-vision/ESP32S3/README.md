# ESP32-S3 ArUco Vision Board

ESP-IDF firmware for the Seeed Studio XIAO ESP32-S3 with OV3660 or OV2640 DVP camera.
Performs ArUco marker detection and outputs world-frame pose estimates over USB-Serial.

## Hardware

| Component | Part |
|-----------|------|
| MCU | Seeed Studio XIAO ESP32-S3 Sense (240 MHz, 8 MB PSRAM) |
| Camera | OV3660 or OV2640 DVP (auto-detected at boot) |
| USB-UART | USB-CDC → `/dev/ttyACM0` |

## Modes

Two build modes selectable via `idf.py menuconfig`:

| Mode | Description |
|------|-------------|
| `POSE` | Continuous ArUco detection, world-frame x/y/z + quaternion output |
| `BENCH` | Multi-resolution FPS sweep (QQVGA → QVGA → HVGA → VGA), CSV output |

## Console Output (POSE mode)

```
[aruco]  M7  [known]    dist=2.34m  H=+5.2deg  V=-8.1deg
[pose]   x=5.231  y=3.142  z=1.503  qw=0.9971 qx=0.0104 qy=-0.0198 qz=0.0502  (2 matched)
```

## Build & Flash

Use the launcher script (recommended):

```bash
launchers/ubuntu-gnome-pc/launch-drone-vision-s3.sh
# or on Pi5:
launchers/ubuntu-xfce-pi5/launch-node-ide.sh
```

Or manually via Docker:

```bash
cd drone-vision/ESP32S3
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec esp32s3_vision bash
# inside container:
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Relationship to ESP32-P4

The ESP32-S3 variant was the earlier prototype and benchmark platform. The ESP32-P4 (`ESP32P4/`) is the production vision board used on the drone — it runs at 360 MHz with MIPI-CSI and outputs POSE over UART to the drone firmware (ESP32-S3 node). The ArUco marker map and world-pose math are shared between both variants.
