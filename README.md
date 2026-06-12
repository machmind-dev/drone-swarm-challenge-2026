**Team** · [machmind.dev](http://machmind.dev)

| Member | Disciplines |
|--------|-------------|
| [**Mindaugas Jonauskis**](https://www.linkedin.com/in/mindaugas-jonauskis-11931145/) (Founder) | Firmware & Software Engineering · Electronics & Electrical Design |
| [**Hauke Renk**](https://www.linkedin.com/in/hauke-renk-90832739a/) | Mechanical Design & Integration |

---

# Swarm Drone Challenge 2026

Source code of the solution by **Team Mach Mind** for the [Swarm Drone Challenge 2026](https://swarmdronechallenge.digital/), organised by [MBDA](https://www.mbda-systems.com) and [brigkAIR](https://www.brigk.digital/en/brigkair/).

Qualifying rounds took place **20–24 April 2026**. Team Mach Mind qualified as one of **6 finalists** and finished **4th place** at the Finals on **11 June 2026** at ILA Berlin.

<p align="center"><a href="docs/media"><img src="takeoff.gif" alt="Swarm takeoff"></a></p>

**Drone Versions**

| Version | Event | MCU | CPU (MHz), PSRAM (MB) | ToF | Vision | ArUco |
|---------|-------|-----|-------|-----|--------|-------|
| [v1.0](drone-firmware/v1.0/) | Qualifying | S3 | 240 MHz, 8 MB | 4 | Offboard | 80×60px, up to 4 m |
| [v2.0](drone-firmware/v2.0/) | Finals | S3&P4 | 240&360 MHz, 8&32 MB | 6 | Onboard | 320×240px, up to 12 m |


## System Architecture (v2.0)

![System Architecture v2.0](docs/system_architecture.png)

---

## Swarm Behaviour

The **SDC26 Commander** (`ground-station-software/swarm/sdc26_commander.py`) runs the swarm by role, streaming waypoints to `/gcs/drone_<id>/control` in the arena frame (`id · x · y · height`).

**Swarm control via LLM — demo only:** Ollama (Gemma) turns natural-language instructions into JSON/coordinate commands, but it is a standalone DEMO and is NOT in live on the finals. Meaningful coordinate tasking needs a reliable absolute position reference (ArUco Navigation), which we couldn't be fully implement in time, so all live flight is run by the deterministic Commander above.

**Waypoint execution (Manhattan):** the firmware converts each arena waypoint to NED and reaches it in axis-aligned legs (one axis at a time, no diagonals), each leg flown as a sequence of discrete steps — keeping motion predictable and obstacle handling simple.

| Role | Behaviour |
|------|-----------|
| **Executor** | Captures opponent boxes based on locations received from a Seeker. |
| **Seeker** | The only role that publishes box locations. If a box is not found within 2 minutes, the GCS publishes a random. |
| **Leader** | Monitors the home base while no boxes have been captured. |

---

## Repository Structure

```
drone-swarm-challenge-2026/
├── drone-firmware/          # All drone onboard firmware (v1.0 and v2.0)
│   ├── v1.0/                # SDC 2026 Qualifying — single ESP32-S3
│   └── v2.0/                # SDC 2026 Finals — dual ESP32-P4 + ESP32-S3
│       ├── drone-vision-esp32p4/  # Navigation: ArUco, ToF, UART TX
│       ├── drone-comms-esp32s3/   # Communication: MAVLink, micro-ROS
│       └── shared/                # Binary UART protocol header
├── ground-station-software/ # ROS2 ground control station, swarm algorithms & vision
├── hardware/                # Mechanical and electrical design files
├── launchers/               # Platform-specific launch scripts
│   ├── ubuntu-gnome-pc/     # x86_64 Ubuntu GNOME (development GCS)
│   └── ubuntu-xfce-pi5/     # ARM64 Raspberry Pi 5 (field GCS)
└── docs/                    # Documentation and media
```

---

## To Do

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m. Since navigation via ArUco markers were not implemented a work around took place.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

---

## Open Issues


### ArUco navigation

ArUco-based EKF fusion was developed, but not fully deployed in time for the finals. ArUco is therefore used only for box detection, not navigation.

---

## Hardware

### Drone Version 2.0

| Swarm Stack Bracket | Base for LiDAR & Flow Motion | FC adapter 25.5 to 20 mm |
|---|---|---|
| [<img src="hardware/drone/v2.0/swarm_stack_bracket.png" width="260">](hardware/drone/v2.0/swarm_stack_bracket.stl) | [<img src="hardware/drone/v2.0/base_mount_lidar_flow_sensor_mk4.png" width="260">](hardware/drone/v2.0/base_mount_lidar_flow_sensor_mk4.stl) | [<img src="hardware/drone/v2.0/fc_mount_adapter_25_5 _to_20_mm.png" width="260">](hardware/drone/v2.0/fc_mount_adapter_25_5%20_to_20_mm.stl) |

| Swarm Stack PCB | Spoiler Block (for RFID) | Spoiler Block Bracket |
|---|---|---|
| [<img src="hardware/drone/v2.0/PCB/pcb_board.png" width="260">](https://github.com/machmind-dev/drone-swarm-challenge-2026/tree/main/hardware/drone/v2.0/PCB) | [<img src="hardware/drone/v2.0/spoiler_block_mk3.png" width="260">](hardware/drone/v2.0/spoiler_block_mk3.stl) | [<img src="hardware/drone/v2.0/spoiler_block_bracket.png" width="260">](hardware/drone/v2.0/spoiler_block_bracket.stl) |

---

Drones assembled and ready for first flight test as formation of five units

<img src="hardware/drone/all_drones_being_prepared_for_flight.jpg" width="868">

<img src="hardware/drone/all_drones_performing_connectivity_test.jpg" width="868">

<table><tr>
<td><a href="hardware/drone/assembled_drone_version_2_0_photo_1.jpg"><img src="hardware/drone/assembled_drone_version_2_0_photo_1.jpg" width="260"></a></td>
<td><a href="hardware/drone/assembled_drone_version_2_0_photo_2.jpg"><img src="hardware/drone/assembled_drone_version_2_0_photo_2.jpg" width="260"></a></td>
<td><a href="hardware/drone/assembled_drone_version_2_0_photo_3.jpg"><img src="hardware/drone/assembled_drone_version_2_0_photo_3.jpg" width="260"></a></td>
</tr><tr>
<td><a href="hardware/drone/assembling_drone_version_2_0_photo_4.jpg"><img src="hardware/drone/assembling_drone_version_2_0_photo_4.jpg" width="260"></a></td>
<td><a href="hardware/drone/assembling_drone_version_2_0_photo_5.jpg"><img src="hardware/drone/assembling_drone_version_2_0_photo_5.jpg" width="260"></a></td>
<td><a href="hardware/drone/assembling_drone_version_2_0_photo_7.jpg"><img src="hardware/drone/assembling_drone_version_2_0_photo_7.jpg" width="260"></a></td>
</tr><tr>
<td><a href="hardware/drone/testing_idea_after_qualifyings.jpg"><img src="hardware/drone/testing_idea_after_qualifyings.jpg" width="260"></a></td>
<td><a href="hardware/drone/assembling_first_pcb.jpg"><img src="hardware/drone/assembling_first_pcb.jpg" width="260"></a></td>
<td><a href="hardware/drone/old_swarm_stack_vs_new.jpg"><img src="hardware/drone/old_swarm_stack_vs_new.jpg" width="260"></a></td>
</tr></table>

---

### Ground Control Station

| Bezel Mk2 | Panel Bottom | Bottom Frame Mk2 |
|---|---|---|
| [<img src="hardware/ground-station/bezel_mk2.png" width="260">](hardware/ground-station/bezel_mk2.stl) | [<img src="hardware/ground-station/panel_bottom.png" width="260">](hardware/ground-station/panel_bottom.stl) | [<img src="hardware/ground-station/bottom_frame_mk2.png" width="260">](hardware/ground-station/bottom_frame_mk2.stl) |

| Exhaust | Top Connector | Top Lower LH Mk3 |
|---|---|---|
| [<img src="hardware/ground-station/exaust.png" width="260">](hardware/ground-station/exaust.stl) | [<img src="hardware/ground-station/top_connector.png" width="260">](hardware/ground-station/top_connector_mk3.stl) | [<img src="hardware/ground-station/top_lower_lh_mk3.png" width="260">](hardware/ground-station/top_lower_lh_mk3.stl) |

| Top Lower RH Mk3 | Top Upper LH Mk3 | Top Upper RH Mk3 |
|---|---|---|
| [<img src="hardware/ground-station/top_lower_rh_mk3.png" width="260">](hardware/ground-station/top_lower_rh_mk3.stl) | [<img src="hardware/ground-station/top_upper_lh_mk3.png" width="260">](hardware/ground-station/top_upper_lh_mk3.stl) | [<img src="hardware/ground-station/top_upper_rh_mk3.png" width="260">](hardware/ground-station/top_upper_rh_mk3.stl) |

Ground Control Station assembled and in use for flight testing

<img src="hardware/ground-station/ground_contro_station_assembled.png" width="868">

<img src="hardware/ground-station/Screenshot_2026-06-07_22-23-24.png" width="868">

| Monitor trial fit | Assembling 1 | Assembling 2 |
|---|---|---|
| <img src="hardware/ground-station/gcs_assembly_1.jpg" width="260"> | <img src="hardware/ground-station/gcs_assembly_2.jpg" width="260"> | <img src="hardware/ground-station/gcs_assembly_3.jpg" width="260"> |

---

### Transportation

| Transport Box Version 2.0 | Transport Box Version 1.0 |
|---|---|
| [<img src="hardware/transportation/Version 2.0/transport_box_mk4.png" width="400">](hardware/transportation/Version%202.0/transport_box_mk4.stl) | [<img src="hardware/transportation/Version 1.0/transport_box_mk2.png" width="400">](hardware/transportation/Version%201.0/transport_box_mk2.stl) |

---

Drones packed for trip to ILA Berlin 2026

<img src="hardware/transportation/drones_packed_for_ILA_berlin.png" width="868">

---

## License

This project is licensed under the [MIT License](LICENSE).

