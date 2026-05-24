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

> **Mechanical note — lens protrusion:** The OV5647 M12 lens barrel extends **9.5 mm** in
> front of the PCB mounting surface. This offset must be accounted for in any extrinsic
> calibration: the optical centre is 9.5 mm forward of the board along the camera boresight
> axis. When computing the camera-to-body transform for world-frame pose correction, apply a
> −9.5 mm translation along the camera Z axis (forward). Current firmware does not apply
> this correction — it is noted here for future ChArUco extrinsic calibration.

---

## PX4 Flight Controller Settings

Parameter file: [`PX4_settings.params`](PX4_settings.params)
Platform: PX4 Pro 1.14.3 dev · Quadrotor · HKUST NXT-Dual

Load via **QGroundControl → Vehicle Setup → Parameters → Tools → Load from file**.

The file is a full parameter dump. The table below covers only the parameters that differ from PX4 defaults or are specific to this drone's hardware and mission profile.

### Battery

| Parameter | Value | Reason |
|-----------|-------|--------|
| `BAT1_N_CELLS` | 4 | 4S LiPo pack |
| `BAT1_V_CHARGED` | 4.20 V | Standard LiPo full-charge voltage |
| `BAT1_V_EMPTY` | 3.20 V | Conservative low-voltage cutoff |
| `BAT1_A_PER_V` | 17.0 | Current sensor calibration for the onboard shunt |
| `BAT1_R_INTERNAL` | 0.005 Ω | Pack internal resistance for voltage sag compensation |

### Indoor / GPS-denied operation

| Parameter | Value | Reason |
|-----------|-------|--------|
| `COM_ARM_WO_GPS` | 1 | Allow arming without GPS fix — indoor arena, no GPS |
| `COM_RC_IN_MODE` | 3 | RC input disabled; drone is controlled via MAVLink (GCS / companion) |
| `EKF2_HGT_REF` | 2 (Vision) | Use vision (ArUco) as height reference instead of barometer |
| `EKF2_EV_CTRL` | 15 | Enable all external-vision fusion bits: horizontal pos + vertical pos + velocity + yaw |
| `EKF2_EV_NOISE_MD` | 0 | Trust `EVP_NOISE` / `EVV_NOISE` values set below, not EV message covariance |
| `EKF2_EVP_NOISE` | 0.10 m | Position noise for vision pose input |
| `EKF2_EVV_NOISE` | 0.10 m/s | Velocity noise for vision pose input |
| `EKF2_EVA_NOISE` | 0.10 rad | Angle noise for vision pose input |

### Circuit breakers

These disable PX4 pre-arm checks that do not apply to this custom hardware:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `CBRK_IO_SAFETY` | 22027 | No physical safety switch on custom PCB |
| `CBRK_USB_CHK` | 197848 | Allow arming while USB connected (bench work and tethered testing) |
| `CBRK_SUPPLY_CHK` | 894281 | Voltage divider calibrated separately; PX4 default check gives false fail |
| `CBRK_FLIGHTTERM` | 121212 | Disable flight termination action (no parachute; eland used instead) |

### MAVLink / serial ports

| Parameter | Value | Reason |
|-----------|-------|--------|
| `SER_TEL1_BAUD` | 115200 | TELEM1 — QGroundControl uplink |
| `SER_TEL2_BAUD` | 57600 | TELEM2 — ESP32-S3 companion computer link |
| `MAV_2_CONFIG` | 102 (TELEM2) | MAVLink instance 2 assigned to TELEM2 for S3 |
| `MAV_2_MODE` | 2 (Onboard) | S3 is treated as onboard companion, not GCS |
| `MAV_2_RATE` | 0 (unlimited) | Let the S3 manage its own output rate |

### Collision prevention

| Parameter | Value | Reason |
|-----------|-------|--------|
| `CP_DIST` | −1 | Collision prevention **disabled** — sensor data is being logged and verified (M4); enable in M6 |
| `CP_DELAY` | 0.40 s | Reaction delay when CP is enabled (pre-configured for M6 activation) |
| `CP_GUIDE_ANG` | 30° | Maximum deflection angle CP will apply to avoid obstacle |
| `COM_OBS_AVOID` | 0 | Obstacle avoidance disabled at planner level (also for M6) |

### Motor / actuator mapping

X-frame quadrotor, motors on main PWM outputs:

| Output | `PWM_MAIN_FUNC` | Motor position |
|--------|-----------------|----------------|
| MAIN 1 | 101 (Motor 1) | Front-right |
| MAIN 2 | 104 (Motor 4) | Rear-left |
| MAIN 3 | 103 (Motor 3) | Rear-right |
| MAIN 4 | 102 (Motor 2) | Front-left |

Rotor arm length: 15 cm (±0.15 m in `CA_ROTOR*_PX/PY`).

### Rate and position controller tuning

| Parameter | Value | Note |
|-----------|-------|------|
| `MC_ROLLRATE_P` | 0.150 | Roll rate P gain |
| `MC_ROLLRATE_I` | 0.200 | Roll rate I gain |
| `MC_ROLLRATE_D` | 0.003 | Roll rate D gain |
| `MC_ROLLRATE_K` | 0.350 | Roll rate global gain |
| `MC_PITCHRATE_P` | 0.150 | Pitch rate P gain |
| `MC_PITCHRATE_I` | 0.200 | Pitch rate I gain |
| `MC_PITCHRATE_D` | 0.003 | Pitch rate D gain |
| `MC_PITCHRATE_K` | 0.400 | Pitch rate global gain |
| `MC_YAWRATE_P` | 0.200 | Yaw rate P gain |
| `MC_YAWRATE_I` | 0.100 | Yaw rate I gain |
| `MC_YAWRATE_K` | 1.200 | Yaw rate global gain |
| `MPC_XY_P` | 0.95 | Horizontal position loop gain |
| `MPC_XY_VEL_P_ACC` | 1.80 | Horizontal velocity P |
| `MPC_Z_P` | 1.00 | Vertical position loop gain |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | Maximum climb rate |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | Maximum descent rate |
| `MPC_TKO_SPEED` | 1.5 m/s | Takeoff climb speed |
| `MPC_LAND_SPEED` | 0.7 m/s | Landing descent speed |

---

---

## Fixes & Known Issues

### ArUco false-positive on cold start with covered/dark camera (v2.0)

**Symptom:** `VIS:OK` with plausible-looking coordinates appears on the ESP32-S3 console immediately after boot, even though no ArUco marker is visible (camera lens covered or scene completely dark).

**Root cause — two interacting issues:**

1. **OV5647 AEC at maximum gain on a black scene.**
   The 5-second ISP stabilisation delay lets auto-exposure converge. With the camera covered the AEC ramps to maximum gain trying to brighten the scene, maximising sensor noise in the first frame after stabilisation.

2. **98th-percentile normaliser amplifies noise into a false marker.**
   `aruco_pose.cpp` normalises each frame so its 98th-percentile luma maps to 255 — a technique that handles dim scenes well. With a covered camera the p98 raw luma is 2–8 (pure noise), giving a normalisation scale of `(255 × 256) / 8 = 8160`. Sensor noise is boosted into high-contrast pseudo-edges that `cv::aruco::detectMarkers` can match to a valid marker.

3. **S3 does not immediately clear `vision_pose_valid` on `pose.valid = false`.**
   Even if the false detection is short-lived, the S3 kept showing `VIS:OK` for up to 1 500 ms (the `VISION_TIMEOUT_MS` window) after the last valid packet because the receive loop had no `else` branch.

**Fix applied:**

- **`drone-vision-esp32p4/main/aruco_pose.cpp`** — brightness gate added before `detectMarkers()`:
  ```cpp
  if (norm_max < 20) {          // raw 98th-percentile luma; real scenes are 30+
      s_pose_valid = false;
      continue;                  // skip detection, don't amplify noise
  }
  ```
  `norm_max` is the raw p98 luma **before** normalisation. A well-lit indoor scene reads 50–200+. A covered camera or pitch-dark room reads 2–10. The threshold of 20 gives a safe margin in both directions.

- **`drone-comms-esp32s3/main/main.c`** — immediate clear on the S3 side:
  ```c
  } else {
      vision_pose_valid = false;   // clear immediately, don't wait for timeout
  }
  ```
  The 1 500 ms timeout remains as a safety net for UART dropout, but `pose.valid = false` packets now clear the flag straight away.

**Tuning the threshold:** the diagnostic log printed every 50 frames (`diag frame=N mean=X min=Y max=Z`) shows the actual pixel range. If operating in a genuinely dim environment where real markers are visible, lower the threshold — but keep it above the covered-camera noise floor observed on your hardware.

---

## Repository layout

```
drone-firmware/
├── PX4_settings.params          # PX4 flight controller parameter file
├── v1.0/                        # SDC 2026 Qualifying — single-MCU ESP32-S3
└── v2.0/
    ├── drone-vision-esp32p4/    # Navigation module — ESP32-P4
    ├── drone-comms-esp32s3/     # Communication module — ESP32-S3
    └── shared/                  # UART protocol header (p4_link_protocol.h)
```
