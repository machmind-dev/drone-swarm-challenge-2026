# Drone Firmware

| Version | Event | Hardware | Vision Computing |
|---------|-------|----------|-----------------|
| [v1.0](v1.0/) | SDC 2026 Qualifying | Single ESP32-S3 (COTS dev board) | Off-board (GCS) |
| [v2.0](v2.0/) | SDC 2026 Finals | Dual ESP32-P4 + ESP32-S3 (custom PCB) | On-board (ESP32-P4) |

---

## Drone Hardware

Hardware documentation (PCB, schematics, part list) is located at [hardware/drone/v2.0](https://github.com/machmind-dev/drone-swarm-challenge-2026/tree/main/hardware/drone/v2.0).

**Note:** Maximum speed is intentionally limited via PX4 parameters (`MPC_XY_CRUISE`, `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN`) for safe indoor arena operation.

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
| `BAT1_V_DIV` | 10.1 | Voltage divider ratio for battery voltage measurement |

### Indoor / GPS-denied operation

| Parameter | Value | Reason |
|-----------|-------|--------|
| `COM_ARM_WO_GPS` | 1 | Allow arming without GPS fix — indoor arena, no GPS |
| `COM_RC_IN_MODE` | 3 | RC input disabled; drone is controlled via MAVLink (GCS / companion) |
| `EKF2_HGT_REF` | 2 (Vision) | Height reference set to vision/external source |
| `EKF2_EV_CTRL` | 15 | Enable all external-vision fusion bits: horizontal pos + vertical pos + velocity + yaw |
| `EKF2_EV_NOISE_MD` | 0 | Trust `EVP_NOISE` / `EVV_NOISE` values set below, not EV message covariance |
| `EKF2_EVP_NOISE` | 0.10 m | Position noise for vision pose input |
| `EKF2_EVV_NOISE` | 0.10 m/s | Velocity noise for vision pose input |
| `EKF2_EVA_NOISE` | 0.10 rad | Angle noise for vision pose input |

### Circuit breakers

| Parameter | Value | Reason |
|-----------|-------|--------|
| `CBRK_IO_SAFETY` | 22027 | No physical safety switch on custom PCB |
| `CBRK_USB_CHK` | 197848 | Allow arming while USB connected (bench work and tethered testing) |
| `CBRK_SUPPLY_CHK` | 894281 | Voltage divider calibrated separately; PX4 default check gives false fail |
| `CBRK_FLIGHTTERM` | 0 | Flight termination active (default) |

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
| `CP_DIST` | 0.5 m | Minimum distance to obstacle before collision prevention activates |
| `CP_DELAY` | 0.40 s | Reaction delay |
| `CP_GUIDE_ANG` | 30° | Maximum deflection angle to avoid obstacle |
| `COM_OBS_AVOID` | 0 | Obstacle avoidance disabled at planner level |

### Motor / actuator mapping

X-frame quadrotor, motors on main PWM outputs:

| Output | `PWM_MAIN_FUNC` | Motor position |
|--------|-----------------|----------------|
| MAIN 1 | 101 (Motor 1) | Front-right |
| MAIN 2 | 104 (Motor 4) | Rear-left |
| MAIN 3 | 103 (Motor 3) | Rear-right |
| MAIN 4 | 102 (Motor 2) | Front-left |

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
| `MPC_XY_VEL_I_ACC` | 0.40 | Horizontal velocity I |
| `MPC_XY_VEL_D_ACC` | 0.20 | Horizontal velocity D |
| `MPC_Z_P` | 1.00 | Vertical position loop gain |
| `MPC_Z_VEL_P_ACC` | 4.00 | Vertical velocity P |
| `MPC_Z_VEL_I_ACC` | 2.00 | Vertical velocity I |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | Maximum climb rate |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | Maximum descent rate |
| `MPC_TKO_SPEED` | 1.5 m/s | Takeoff climb speed |
| `MPC_LAND_SPEED` | 0.7 m/s | Landing descent speed |
| `MPC_XY_CRUISE` | 0.5 m/s | Default horizontal cruise speed |
