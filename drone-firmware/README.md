# Mach Mind Drone Firmware  
## ESP32-S3 Sensor and micro-ROS Interface for Swarm Drone Challenge 2026

This firmware is part of the **Mach Mind** system developed for the **Swarm Drone Challenge 2026**.

It runs on the **Seeed Studio XIAO ESP32-S3 Sense** and acts as the onboard sensor and communication node of each drone. The ESP32-S3 interfaces local sensors, publishes data through **micro-ROS**, and exchanges information with the Ground Control Station over Wi-Fi.

---

## Overview

The firmware is designed to support a lightweight onboard architecture for indoor autonomous drone operation in a GPS-denied environment.

Main functions of the ESP32-S3 module:
- acquisition of onboard sensor data
- camera streaming
- range sensor integration
- micro-ROS communication with the Ground Control Station
- support for obstacle-awareness and localization-related functions
- MAVLink-related interfacing support for the PX4-based flight stack

The drone architecture is split into two major parts:
- **Navigation module**: PX4 flight controller, ESCs, optical flow and lidar
- **Skull module**: ESP32-S3, camera, laser rangers, and local sensor-processing functions

The ESP32-S3 is not responsible for low-level flight stabilization. That remains on the PX4 flight controller. Its role is to provide sensing, communication, and lightweight onboard processing.

---

## Hardware Platform

Current target hardware:
- **Seeed Studio XIAO ESP32-S3 Sense**
- **OV2640 or OV3660 camera** (per-drone, auto-detected at boot)
- **VL53L1X laser range sensors**
- optional multiplexer for sensor expansion
- UART link for MAVLink-related communication
- Wi-Fi connection to the Ground Control Station

This firmware is developed for the **Mach Mind drone prototype** used in the Swarm Drone Challenge 2026 qualification phase.

### Camera Sensor Configuration

Both OV2640 and OV3660 are supported. The sensor is auto-detected at boot and the firmware applies per-sensor settings automatically. No rebuild is required when swapping sensors.

The OV3660 is mounted 180° rotated on the drone frame — the firmware corrects this via hardware flip registers (`set_vflip` + `set_hmirror`) so the ArUco detection pipeline requires no changes.

| Parameter | OV2640 | OV3660 | Reason |
|---|---|---|---|
| `errorCorrectionRate` | 0.6 | 0.3 | OV3660 heavy downscaling (2048×1536 → 80×60) produces noisier bit patterns; stricter threshold prevents ghost IDs |
| `adaptiveThreshWinSizeMax` | 15 | 11 | Smaller window matches OV3660 noise profile |
| `minMarkerPerimeterRate` | 0.10 | 0.10 | Rejects noise clusters smaller than 8 px perimeter |
| `aec_value` | 400 | 200 | OV3660 exposure register has different scale |
| Hardware flip | None | vflip + hmirror | OV3660 mounted 180° rotated |

---

## Software Stack

The firmware is based on:
- **ESP-IDF**
- **micro-ROS**
- **FreeRTOS**
- **ESP32 camera driver**
- custom sensor and communication integration
- optional MAVLink support for data exchange with the flight controller

The Ground Control Station side uses:
- **ROS 2**
- **micro_ros_agent**
- visualization and control tools running on the GCS

---

## Functional Scope

Depending on the currently enabled build configuration, the firmware may include:
- camera image publishing
- obstacle distance publishing
- command subscription from the Ground Control Station
- status and heartbeat signaling
- drone identification signaling
- local debug and telemetry output

Typical use cases:
- streaming image data from the drone to the GCS
- publishing obstacle-distance information from laser rangers
- supporting ArUco-based localization workflows
- enabling operator-triggered control features such as video on/off

---

## Project Role in the Full System

Within the Mach Mind system, this firmware supports a broader architecture in which:
- each drone performs local sensing and lightweight data handling,
- flight-critical behavior remains on PX4,
- higher-level coordination and heavy processing are performed on the Ground Control Station,
- multi-drone coordination is managed through the GCS and ROS 2-based infrastructure.

This approach keeps the airborne node lightweight while allowing scalable swarm development.

---

## Build and Run

Open the development container and run:

```bash
sudo chown -R $(whoami) /code
cd /code
source /opt/esp/idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
sudo chmod o+rw /dev/ttyACM0
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
