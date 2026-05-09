# ESP32-S3 Communications Board

ESP-IDF firmware for the Seeed Studio XIAO ESP32-S3 acting as the communication bridge in the v2.0 dual-MCU architecture. Receives sensor data from the ESP32-P4 over UART, forwards obstacle distances and pose estimates to PX4 via MAVLink, and maintains the micro-ROS link to the Ground Control Station.

## Role in the System

```
ESP32-P4 (Navigation)
  │  UART binary frames @ 115200
  │  GPIO22 TX → GPIO3 RX
  ▼
ESP32-S3 (Communications)
  ├── MAVLink → PX4 (GPIO43 TX / GPIO44 RX, 57600 baud)
  │     OBSTACLE_DISTANCE  — 5 horizontal ToF sensors, 30° FOV each
  │     DISTANCE_SENSOR    — upward sensor (slot 5)
  │     VISION_POSITION_ESTIMATE — ArUco pose (when vision enabled)
  │     HEARTBEAT, SET_MODE, ARM, NAV commands
  └── micro-ROS → GCS (Wi-Fi)
        Publish: /drone_N/state, /drone_N/battery, /drone_N/role
        Subscribe: /gcs/drone_N/command, /gcs/drone_N/config, /gcs/drone_N/control
```

## Hardware

| Component | Part |
|-----------|------|
| MCU | Seeed Studio XIAO ESP32-S3 (240 MHz, 8 MB PSRAM) |
| P4 link | UART2 — GPIO3 RX ← P4 GPIO22, GPIO2 TX → P4 GPIO23 |
| PX4 link | UART1 — GPIO43 TX, GPIO44 RX, 57600 baud |
| USB-UART | USB-CDC → `/dev/ttyACM0` |

## UART Protocol (P4 → S3)

Binary framed, 115200 8N1. Frame layout:

```
SOF(1B) | LEN(1B) | TYPE(1B) | PAYLOAD(47B) | CRC8(1B)  =  51 bytes total
```

- SOF = `0xAB`, TYPE = `0x03` (COMBINED)
- Payload = `p4_tof_t` (18 B: 6× uint16 dist + 6× uint8 status) + `p4_pose_t` (29 B: valid + 7× float)
- CRC-8 (poly 0x07) computed over TYPE + PAYLOAD
- Protocol header: `v2.0/shared/p4_link_protocol.h`

## MAVLink Obstacle Distance Mapping

72-bin circular map at 5°/bin, bin 0 = forward, clockwise:

| Sensor | Direction | Bins |
|--------|-----------|------|
| Slot 0 | Left −90° | 51–56 |
| Slot 1 | L-front −45° | 60–65 |
| Slot 2 | Front 0° | 69, 70, 71, 0, 1, 2 |
| Slot 3 | R-front +45° | 6–11 |
| Slot 4 | Right +90° | 15–20 |
| Slot 5 | Up | separate `DISTANCE_SENSOR` (PITCH_90) |

## Console Output

```
[ToF] L90: 320mm L45: 450mm FWD: 880mm R45:  13mm R90:  12mm UP:   2mm | STATE:disarmed       VIS:N
```

## Build & Flash

Use the desktop launcher (recommended):

```
IDE - Flash S3
```

Or manually via Docker:

```bash
cd drone-firmware/v2.0/drone-comms-esp32s3
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec esp32s3_comms bash
# inside container:
source /opt/esp/idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

First build only — set the target once:
```bash
idf.py set-target esp32s3
```

## ESP-IDF Version

IDF 5.0 (Docker image: `espressif/idf:release-v5.0`)

## micro-ROS Vision Toggle

Send from GCS to enable/disable VISION_POSITION_ESTIMATE relay to PX4:

```
/gcs/drone_1/config  →  "CONFIG_VISION_ENABLE"
/gcs/drone_1/config  →  "CONFIG_VISION_DISABLE"
```
