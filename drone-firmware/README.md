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

## Camera Comparison — v1.0 vs v2.0

### Sensor specifications

| | OV2640 (v1.0) | OV3660 (v1.0) | OV5647 (v2.0) |
|---|---|---|---|
| Sensor size | 1/4" | 1/5" | 1/4" |
| Native resolution | 2 MP (1600×1200) | 3 MP (2048×1536) | 5 MP (2592×1944) |
| Pixel size | 2.2 µm | 1.75 µm | 1.4 µm |
| Interface | DVP (parallel) | DVP (parallel) | MIPI-CSI2 (2-lane) |
| Hardware ISP | None | Partial | Full (ESP32-P4 ISP block) |
| Lens type | Fixed focus | Fixed focus | **Adjustable focus (M12 mount)** |
| Max frame rate | 15 fps @ SVGA | 15 fps @ SVGA | 50 fps @ 800×800 RAW8 |
| Low-light | Poor | Moderate | Good (ISP AE/AWB/lens-shading) |

### Resolution used for ArUco detection

| | v1.0 (OV2640 / OV3660) | v2.0 (OV5647) |
|---|---|---|
| Capture resolution | 160×120 (QQVGA) | 800×800 RAW8 |
| Detection resolution | 80×60 (2× downsampled) | 320×240 (QVGA, cropped + resized) |
| Detection range achieved | ~3–4 m | ~8 m (QVGA) / ~12 m (HVGA mode) |
| Sensor frame rate | 15 fps (DVP limit) | 50 fps @ 800×800 RAW8 |
| Frame rate at detection res | ~2–3 fps (CPU shared) | ~3 fps at 320×240 (CPU dedicated) |

The OV5647 sensor delivers 50 fps but detection runs at ~3 fps — the camera is not the
bottleneck. Each frame requires the P4 to read and convert the full 800×800 RGB565 buffer
(1.28 MB) from PSRAM to SRAM twice (grayscale crop for ArUco + color thumbnail for the
stream viewer), then run OpenCV `detectMarkers` which internally executes adaptive
thresholding, contour finding, corner refinement, and `solvePnP` on the 320×240 image.
That pipeline costs ~300 ms per frame on the 360 MHz RISC-V core, capping throughput at
~3 fps regardless of sensor speed. The sensor runs continuously so ISP auto-exposure and
lens-shading correction keep converging between detection frames.

The v1.0 firmware had to use 80×60 (QQVGA downsampled) because the S3 shared its CPU
between camera, ArUco, micro-ROS, and sensor polling. The OV5647 on P4 processes 320×240
— 16× more pixels — on a dedicated core, achieving more than double the reliable marker
detection range despite the same ~3 fps throughput.

### OV5647 adjustable focus lens

The OV5647 module used on the Waveshare ESP32-P4 board has an **M12 screw-mount lens with
a manual focus ring**. Rotating the lens barrel moves the lens element along the optical
axis, shifting the focal plane between close range (~0.5 m) and far range (>10 m).

This is a concrete operational advantage over the fixed-focus OV2640 / OV3660:

| Scenario | Fixed-focus (OV2640 / OV3660) | Adjustable-focus (OV5647) |
|----------|-------------------------------|---------------------------|
| Arena with bright overhead lighting | Image overexposed at fixed AEC target — no adjustment possible | ISP AEC + lens focus tuned to arena distance before flight |
| Arena with dim / mixed lighting | Fixed focus may place sharpest plane at wrong distance | Focus ring set for working distance (e.g. 3 m to nearest wall marker) |
| Changing between indoor arenas | Stuck with factory focus | Refocused in ~10 seconds with a small screwdriver |
| Marker at 1 m vs 8 m | One is blurry — no fix | Focus set to the typical operating distance for each competition stage |

The OV5647 also benefits from the ESP32-P4's hardware ISP pipeline (auto-exposure,
auto-white-balance, and lens-shading correction), which converges to stable image
quality after the 5-second warmup. The OV2640/OV3660 had no equivalent — exposure was
managed through software register writes and reacted slowly to lighting changes.

> **SDC 2026 arena note:** Finals arenas typically differ from qualifying arenas in size
> and ceiling height. The adjustable lens was set to the marker working distance
> (wall-to-wall detection range) before each flight, and the ISP warmup was allowed to
> stabilise before arming. This is not possible with a fixed-focus sensor.

---

## Repository layout

```
drone-firmware/
├── v1.0/                        # SDC 2026 Qualifying — single-MCU ESP32-S3
└── v2.0/
    ├── drone-vision-esp32p4/    # Navigation module — ESP32-P4
    ├── drone-comms-esp32s3/     # Communication module — ESP32-S3
    └── shared/                  # UART protocol header (p4_link_protocol.h)
```
