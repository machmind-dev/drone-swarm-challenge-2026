> **TODO (box detection):** box detection needs to be double-checked for correctness.
>
> **TODO (hardware mod):** planned board/wiring revisions —
> - establish a **common ground** between the BEC and the sensors (shared ground reference);
> - add a **series current-limiting resistor** on the LED-strip control/data line(s);
> - (firmware) implement **per-drone roles** in the ESP32-S3 firmware — **Seeker**, **Executor**, **Lead**.
>
> **TODO (obstacle, hover):** add a 1 s hover/debounce on obstacle detection in the Manhattan leg path — the ToF (`MANHATTAN_OBSTACLE_MM`, now 1.0 m) fluctuates against the net wall's holes, so a single flickering reading can trigger a stop. Not yet implemented; decide whether the 1 s should debounce the trigger (confirm before halting) or settle before the await-GCS hold.
>
> **TODO (obstacle, pass-through):** the pass-through clamp (`clamp_setpoint_for_obstacles`, keyboard-fly mode — not the leg path) still uses only the **forward** sensor, because it clamps motion along the travel vector; an all-5 min there would wrongly shorten forward motion when passing a side wall. Decide whether that mode should also stop on any-side proximity.
>
> **Known issue (frame-mix):** `drone-comms-esp32s3/main/main.c:931` (`aruco_approach_task`) — `hold_y = vision_pose_valid ? vp_y : px4_pos_y` mixes arena-frame `vp_y` with NED `px4_pos_y`. Pre-existing bug, only in the ArUco-spin approach feature, not the waypoint path. Needs the vision branch converted to NED (`ned_offset_y - vp_y`).

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

- [ ] **Lower `C2_FAIL_THRESHOLD` to 1** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:157`) — currently set to 3 (6 s timeout) for WiFi RTT tolerance during testing. Must be lowered to 1 (2 s) before the competition flight.

- [x] **ARM-time inertial anchor fix** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c:874`) — `inertial_anchor_valid = true` now set at ARM time; trail tracks live from takeoff.

### Navigation

- [x] **Manhattan waypoint navigation** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — 1 m steps X-leg first then Y-leg; yaw to face travel direction; ToF obstacle stop at 500 mm; awaits new GCS command on obstacle; keyboard pass-through for steps ≤ 1 m; arrival tolerance 1 m.

### Analysis

- [ ] **Analyse drift test `log_63_UnknownDate.ulg`** (`drone-firmware/v2.0/logs/2026-06-01_2109_drone4/`) — 5-minute aggressive inertial-only flight. Goal: quantify position error accumulation over time to set the mission planning envelope for how long the drone can fly without a position correction.

### Calibration

- [ ] **Full ChArUco calibration on ESP32-P4** (`drone-firmware/v2.0/drone-vision-esp32p4/`) — barrel distortion not yet corrected. Current focal-length correction (fx=438.6 px) achieves ~1% range error but sub-cm accuracy requires a full calibration run with a ChArUco board.

### Cleanup

- [ ] **Remove dead heading-seed code** (`drone-firmware/v2.0/drone-comms-esp32s3/main/main.c`) — `SEED_YAW_ENABLE`, `seed_yaw_rad`, `seed_yaw_valid` are no longer effective. Remove once nav is confirmed stable.

- [x] **Review `simulate_drones.py` active drone list** (`ground-station-software/swarm/algorithm/simulate_drones.py:20`) — all 5 drones now active.
