---

**Team**

| Member | Disciplines |
|--------|-------------|
| [**Mindaugas Jonauskis**](https://www.linkedin.com/in/mindaugas-jonauskis-11931145/) (Founder) | Firmware & Software Engineering · Electronics & Electrical Design |
| [**Hauke Renk**](https://www.linkedin.com/in/hauke-renk-90832739a/) | Mechanical Design & Integration |

---

# Swarm Drone Challenge 2026

Source code of the solution by **Team Mach Mind** for the Swarm Drone Challenge 2026, organised by [MBDA](https://www.mbda-systems.com) and [brigkAIR](https://www.brigkair.com).

Qualifying rounds took place **20–24 April 2026**. Team Mach Mind qualified and was one of **6 finalists** competing at the Finals on **11 June 2026**.

**Drone Versions**

| Version | Event | MCU | CPU (MHz), PSRAM (MB) | ToF | Vision | ArUco |
|---------|-------|-----|-------|-----|--------|-------|
| [v1.0](drone-firmware/v1.0/) | Qualifying | S3 | 240 MHz, 8 MB | 4 | Offboard | 80×60px, up to 4 m |
| [v2.0](drone-firmware/v2.0/) | Finals | S3&P4 | 240&360 MHz, 8&32 MB | 6 | Onboard | 320×240px, up to 12 m |


## System Architecture (v2.0)

![System Architecture v2.0](docs/system_architecture.png)

---

## Swarm Information

Each drone is assigned a **fixed swarm role** at firmware compile time, keyed on
its `DRONE_ID`. The role is published on `/drone_<ID>/role` and drives the
mission behaviour (which drone publishes box locations, which captures boxes, and
which watches the home base).

### Initial role assignment

| Drone ID | Role | Responsibility |
|----------|----------|----------------|
| 1 | **Seeker** | Only role allowed to publish box locations |
| 2 | **Executor** | Only role sent to capture a discovered opponent box |
| 3 | **Seeker** | Only role allowed to publish box locations |
| 4 | **Executor** | Only role sent to capture a discovered opponent box |
| 5 | **Leader** | Checks the home base while no boxes have been captured |

### How roles can be changed

Roles are **hard-coded at compile time** in
[`drone-comms-esp32s3/main/main.c`](drone-firmware/v2.0/drone-comms-esp32s3/main/main.c)
— the `DRONE_ID → DRONE_ROLE` mapping (`ROLE_SEEKER` / `ROLE_EXECUTOR` /
`ROLE_LEADER`) sits directly under the `#define DRONE_ID` identity block. The
role always follows the ID, so:

- **To repurpose a drone during the challenge**, change its `#define DRONE_ID`
  to an ID that carries the desired role and re-flash — e.g. flashing a spare as
  `DRONE_ID 5` makes it the Leader. No separate role flag to keep in sync.
- **To change the mapping itself** (e.g. make ID 4 a Seeker), edit the
  `#if (DRONE_ID == …)` role block and re-flash the affected drone(s).

#### Override a role from the GCS at runtime (no re-flash)

The firmware emits its role on `/drone_<ID>/role` **once at boot**. Because ROS 2
allows multiple publishers on a topic, the GCS can publish onto the same topic to
override the role seen by every consumer — RViz, the RQT panel, and the planned
SDC26 Commander (which keys its box→executor assignment off this topic):

```bash
# One-shot override — make drone 5 act as a Seeker for the swarm logic
ros2 topic pub --once /drone_5/role std_msgs/msg/String "{data: seeker}"

# Keep it asserted for late-joining subscribers (publishes continuously; Ctrl-C to stop)
ros2 topic pub --rate 1 /drone_5/role std_msgs/msg/String "{data: seeker}"
```

> **Caveat:** this overrides only what the *ground station* consumes — it does
> **not** change the drone's compile-time `DRONE_ROLE`. Any behaviour gated
> on-board by role (e.g. the future Seeker-only box-publish gate) still follows
> the flashed value; permanent changes need a re-flash. There is no runtime role
> switch inside the firmware yet.

### Quick test

After flashing, confirm a drone reports the expected role over ROS 2:

```bash
ros2 topic echo /drone_5/role
# → data: leader
```

Substitute the drone number to check the others (e.g. `/drone_1/role` → `seeker`,
`/drone_2/role` → `executor`).

---

## Swarm Behaviour

The **SDC26 Commander** (`ground-station-software/swarm/sdc26_commander.py`)
orchestrates the swarm: it watches box detections, knows each drone's role, and
drives role-specific behaviour. A 5 s start-up grace period runs before any drone
command is sent, and the active team (LH/red or RH/blue) is taken live from the
RQT panel (`/gcs/system/team_color`).

### Executor (drones 2 & 4)

Executors capture opponent boxes. For each one, the Commander streams waypoints to
`/gcs/drone_<id>/control` (same arena-frame `id · x · y · height` convention as the
Waypoint Commander / Swarm Mission tools — the firmware handles arena→NED and
Manhattan stepping). The per-executor sequence is:

1. **Fly to the box** — when an opponent box appears, the nearest **idle** executor
   is sent to its coordinates. The two executors never take the same box (boxes are
   claimed exclusively), so they always work separate targets.
2. **Dwell (cooldown)** — on reaching the box it holds for the 5 s cooldown.
3. **Return to the team-zone border** — flies back keeping the **same Y** as the
   box, to the zone border X: **LH/red = 5**, **RH/blue = 15**.
4. **Step out of the zone** — moves **3 m out**, same Y: **LH/red = 8**,
   **RH/blue = 12**, then **hovers until the next command**.

The dashboard's `WP` column shows each executor's current waypoint (or `hover`),
and `COOLDOWN` shows the dwell timer at the box.

### Seeker (drones 1 & 3)

Seekers are the only role allowed to publish box locations (see
[Swarm Information](#swarm-information)). Detected boxes are published by the
firmware; the Commander consumes them and never overwrites found boxes. If a box
is still undiscovered at the `--boxes-timeout` (default 2 min), the Commander fills
it with a predefined random position (marked `[RND]` on the terminal only).

### Leader (drone 5)

The Leader checks the home base while no boxes have been captured *(behaviour
in progress)*.

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

### Pre-Finals Critical

- [x] **ARM-time inertial anchor fix** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:874`) — `inertial_anchor_valid = true` now set at ARM time; trail tracks live from takeoff.

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.
