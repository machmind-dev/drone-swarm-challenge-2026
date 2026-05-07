# Mach Mind — Drone Firmware

Onboard firmware for the Mach Mind drone platform, developed for the **Swarm Drone Challenge 2026**.

| Version | Event | Hardware |
|---------|-------|----------|
| [v1.0](v1.0/) | SDC 2026 Qualifying | Single ESP32-S3 (COTS dev board) |
| [v2.0](v2.0/) | SDC 2026 Finals | Dual ESP32-P4 + ESP32-S3 (custom PCB) |

---

## v1.0 — SDC 2026 Qualifying

**Single-MCU architecture.** One Seeed Studio XIAO ESP32-S3 Sense handled all onboard tasks: camera capture, ArUco detection, range sensing, micro-ROS communication, and MAVLink interfacing to the PX4 flight controller.

### Hardware

| Component | Part |
|-----------|------|
| MCU | Seeed Studio XIAO ESP32-S3 Sense (240 MHz, 8 MB PSRAM) |
| Camera | OV2640 / OV3660 DVP (auto-detected at boot) |
| Range sensors | VL53L1X laser rangers |
| Comms | Wi-Fi → micro-ROS → ROS 2 GCS |
| FC link | UART MAVLink → PX4 |

### What it did

- Streamed camera frames to the Ground Control Station via micro-ROS over Wi-Fi
- Detected ArUco markers and published world-frame pose estimates
- Published obstacle distances from VL53L1X sensors
- Forwarded `VISION_POSITION_ESTIMATE` MAVLink messages to PX4
- Handled drone identification signalling

### Limitations that drove v2.0

- ESP32-S3 at 240 MHz with DVP camera could not run ArUco detection and micro-ROS simultaneously without frame-rate compromise
- All tasks competed for the same CPU and PSRAM — vision, comms, and sensor polling blocked each other
- No dedicated hardware for the MIPI-CSI interface, limiting camera resolution and ISP quality
- Single point of failure: a crash or watchdog reset took down all onboard functions at once

---

## v2.0 — SDC 2026 Finals

**Dual-MCU architecture on a custom PCB.** Responsibilities are split between two dedicated processors, connected by UART.

```
OV5647 MIPI-CSI                             PX4 Flight Controller
      │                                              │
      ▼                                              │
┌─────────────┐   UART (POSE + ToF)   ┌─────────────┴──────────┐
│  ESP32-P4   │ ───────────────────►  │       ESP32-S3          │
│  Navigation │                       │     Communication       │
└─────────────┘                       └─────────────────────────┘
  ArUco detection                       MAVLink → PX4
  World-frame pose                       micro-ROS → GCS (Wi-Fi)
  6× VL53L1X ToF                        LED strip control
  Obstacle detection                     Drone identification
```

### Hardware

| Component | Part |
|-----------|------|
| Navigation MCU | Waveshare ESP32-P4 WiFi6 (360 MHz, 32 MB PSRAM) |
| Camera | OV5647 MIPI-CSI, 800×800 RAW8 @ 50 fps → ISP → RGB565 |
| Comm MCU | ESP32-S3 |
| Range sensors | 6× VL53L1X ToF (I2C, on ESP32-P4) |
| Inter-MCU link | UART — P4 TX → S3 RX |
| FC link | UART MAVLink → PX4 |
| Comms | Wi-Fi → micro-ROS → ROS 2 GCS |
| PCB | Custom Mach Mind Sensors Board (EAGLE, rev 07/05/2026) |

### Modules

| Module | Folder | Processor | Role |
|--------|--------|-----------|------|
| drone-vision-esp32p4 | [v2.0/drone-vision-esp32p4](v2.0/drone-vision-esp32p4/) | ESP32-P4 | Camera, ArUco, ToF, pose output over UART |
| drone-comms-esp32s3 | [v2.0/drone-comms-esp32s3](v2.0/drone-comms-esp32s3/) | ESP32-S3 | UART receive, MAVLink, micro-ROS, LED, drone ID |
| shared | [v2.0/shared](v2.0/shared/) | — | UART protocol definitions shared by both modules |

### What changed from v1.0

| Aspect | v1.0 | v2.0 |
|--------|------|------|
| MCU count | 1× ESP32-S3 | 1× ESP32-P4 + 1× ESP32-S3 |
| Camera interface | DVP (OV2640 / OV3660) | MIPI-CSI (OV5647) |
| Camera resolution | Up to QVGA for detection | 800×800 RAW8 with hardware ISP |
| CPU for vision | 240 MHz, shared with comms | 360 MHz, dedicated |
| PSRAM | 8 MB | 32 MB |
| ArUco detection rate | ~2–3 fps (shared CPU) | ~3–4 fps (dedicated, ISP-corrected frames) |
| Range sensors | VL53L1X (count varied) | 6× VL53L1X (fixed layout on PCB) |
| Vision → FC path | S3 direct MAVLink | P4 UART → S3 → MAVLink → FC |
| Failure isolation | Single point of failure | Vision and comms fail independently |
| Hardware | COTS dev board | Custom PCB |

---

## Repository layout

```
drone-firmware/
├── v1.0/                        # SDC 2026 Qualifying — single-MCU ESP32-S3
└── v2.0/
    ├── drone-vision-esp32p4/    # Navigation module — ESP32-P4
    ├── drone-comms-esp32s3/     # Communication module — ESP32-S3
    └── shared/                  # UART protocol header (uart_protocol.h)
```
