# Flight-test logs & findings — v2.0 firmware (2026-05-31 → 06-01)

Indoor arena, manual keyboard control, no magnetometer, no GPS
(`EKF2_MAG_TYPE=5`, vision is PX4's only absolute pos/heading source, `EKF2_EV_CTRL=15`).
Each folder holds the PX4 `.ulg`, the ROS 2 `.mcap` + `metadata.yaml`, and (where captured) the
keyboard/remote-control log. Folders are `YYYY-MM-DD_HHMM_droneN`.

> **2026-06-01 flights** (`2026-06-01_*`) were flown on the post-`bd10a7a`/`4fdc15b` build
> (relaxed 45° incidence gate, EV-yaw-fusion *intended* off, live-trail RViz). They surfaced
> **three regressions** — see the 06-01 findings and open items OPEN-7…OPEN-9 below.

Firmware split: **S3** = `drone-comms-esp32s3` (MAVLink bridge: EV send, setpoints, seed,
gates). **P4** = `drone-vision-esp32p4` (ArUco detection + pose). They are separate binaries
and are flashed independently.

---

## Flight findings

| Flight (folder) | Drone | Log | Result | Root cause / notes |
|---|---|---|---|---|
| 17:42 `..._1742_drone2` | 2 | `log_171` | ✅ **Yaw flip FIXED** | First-fix heading innovation 21° (was ~170°). Confirmed gyro tracks heading correctly (no mag); ArUco was still flipping pre-fix. |
| 18:31 `..._1831_drone2` | 2 | `log_173` | ❌ Free-fall crash @2 m | **Physical rotor failure — rear-left motor (output idx 1 / "Motor 2").** Not software: setpoint stayed ~2–3° while actual attitude diverged to >250°/s; mixer maxed one corner, 40% thrust unallocatable. Ruled out battery, failsafe, `CBRK_FLIGHTTERM`, vision. |
| 19:44 `..._1944_drone1` | 1 | `log_24` | ❌ Flew into wall | **EKF position reset after a 19 s vision dropout.** Re-acquisition force-reset pos by Δ(+1.58,−3.68) m; controller chased the 4 m phantom error into the wall. No yaw flip. |
| 20:43 `..._2043_drone3` | 3 | `log_28` | ❌ Flew +X / LH, landed | **First vision fix after 79 s of flow-only flight reset pos ~5 m in x** (Δ(−2.8,−0.75)+(−2.43,−1.08)); drone chased phantom error +X and −Y. `cs_ev_pos=0` until 79 s. **The ±30° P4 incidence gate WAS flashed** and is the likely cause of the 79 s no-fusion — it rejected the obliquely-viewed wall markers during the +X traverse until the drone got square-on to one at x≈6. So the gate *caused* the coverage gap → flow drift → reset. Keyboard log confirms commanded arena (1,3)→(6.5,3); `ned_offset≈(1,3)` correct during flow phase. |

Detailed write-up: `../../../docs/flight-tests/2026-05-31/FINDINGS.md`.

### 2026-06-01 flights (post-`bd10a7a` build)

| Flight (folder) | Drone | Scene | Log | Result | Root cause / notes (Claude analysis from ulog + mcap) |
|---|---|---|---|---|---|
| 18:55 `..._1855_drone3` | 3 | LH | `log_29` | ❌ Flew away in East | **Bad-East ArUco fix → flyaway.** +X(North) leg flown cleanly on flow (East≈0); at t≈153 s the *first & only* vision fixes (17, all in last 10 s) arrived with a **~6 m East error** (fix `(2.5,5.6)` vs actual y≈0). PX4 `reset_pos_to_vision` snapped East, which ran to **+35 m** → drone chased it out. Marker 13 (rear wall x=0) was the trigger, as reported. |
| 19:42 `..._1942_drone4` | 4 | LH | `log_58` | ❌ Got "crazy" after 90° turn | **Same bad-East flyaway.** L-path: +X to x≈18, yaw −90°, then East exploded. `reset_pos_to_vision` @t93 s; East **0→14→27 m**. Vision fix `(18.4, 6.39)` vs actual y≈0 (~6 m error). 3 fixes total, all at the end. `cs_opt_flow`=100 %, `cs_gps`=never. |
| 20:07 `..._2007_drone3` | 3 | RH | `log_60` | ❌ **180° yaw flip on hover** | **Yaw-flip regression.** Armed at correct yaw 180° (RH/blue); first ArUco fix @t1.6 s fused a 180°-flipped heading → yaw estimate 180°→−9°, **attitude setpoint followed** → physical 180° flip. `cs_ev_yaw` active despite firmware `cov[20]=NaN` — see OPEN-8. |
| 21:09 `..._2109_drone4` | 4 | ? | _(no `.ulg`)_ | ⏳ **Not yet analyzed** | **Archived for later investigation (workstation switch).** Has rosbag `rosbag2_2026_06_01-20_55_45_0.mcap` + `metadata.yaml` + cockpit screenshot only — **PX4 `.ulg` was not pulled off the FC** and still needs to be exported and added here. Scene (LH/RH) and result TBD. |

**Cross-cutting conclusion (06-01):** vision is the common failure. The drone gets **near-zero ArUco fusion during flight** (OPEN-3 *not* fixed by the 45° gate), and the few fixes it does get are **flipped/reflected** (wrong East by ~6 m, or wrong yaw by 180°). PX4 trusts them (`reset_pos_to_vision`, EV-yaw) and flies into the error. Net: the 45° relax (`bd10a7a`) did **not** restore coverage and **re-opened** the flip ambiguity that `513cd8a` had closed.

---

## Firmware changes (what / which commit)

| Commit | FW | Change | Flight-verified? |
|---|---|---|---|
| `8ed12e5` | P4 | Absolute arena/wall-side gate (fixes position-reflection flip) | partial |
| `575f05f` | S3 | Yaw-continuity reject gate (`MAX_YAW_JUMP_RAD`, post-first-fix) | — |
| `22736a3` | S3/P4 | Scene-aware heading seed (S3) + 5 m range limiter (P4) | seed never fused; red herring |
| `5ee89a1` | S3 | Heading seed defaults to LH at boot | superseded |
| `513cd8a` | S3 | **`px4_yaw` disambiguation** — pick IPPE yaw closest to gyro heading; fires on first fix | ✅ **Yes** (17:42, 18:31, 19:44, 20:43 — no flips) |
| `cca8686` | S3 | **(1)** Absorb PX4 pos-reset Δ into `ned_offset`; **(2)** re-acquisition EV covariance ramp | ❌ flashed on drone 3 but did **not** prevent the 20:43 reset — see OPEN-1 |
| `c242bd3` | P4 | **±30° viewing-incidence gate** (`MAX_VIEW_ANGLE_DEG`) — reject oblique markers | ⚠️ **was flashed for 20:43 (drone 3)** — too aggressive: caused the 79 s vision gap → reset/runaway. Relax (OPEN-3). |
| `bd10a7a` | S3/P4 | **(1)** Disable EV yaw fusion via `cov[20]=NaN` + remove `px4_yaw` disambiguation; **(2)** gate `ned_offset` EKF-reset absorption on vision recency; **(3)** relax incidence gate **30°→45°** | ❌ **06-01: all three regressed.** `cov[20]=NaN` does **NOT** disable EV yaw (PX4 uses `EKF2_EVA_NOISE`, msg cov ignored) → 180° flip returned (20:07, OPEN-8). 45° gate still gave ~0 coverage **and** admitted flipped fixes → East flyaway (18:55/19:42, OPEN-7). Removing `513cd8a` undid the one verified flip fix. |
| `9f65d04` | S3 | RViz live disc via NED dead-reckoning; always publish | ⚠️ disc still **frozen at home** in flight — dead-reckon is gated on first ArUco fix, which never comes until landing (OPEN-9). |
| `4fdc15b` | GCS | `scene_map_objects.rviz` — per-drone `Path`/`heading_marker`/`vision_pose` displays | ✅ trail now renders; needs OPEN-9 firmware fix to track live. |

---

## Open items

| # | Item | Status / next step |
|---|---|---|
| OPEN-1 | **Did `cca8686` Fix 1 backfire?** Drone 3 had the fixes flashed yet still hard-reset & flew +X. If the ~5 m was real optical-flow drift (likely, over a 79 s unaided traverse), the vision fix was a *legitimate* correction and absorbing it into `ned_offset` *cancels* it → overshoot. Fix 1 helps when vision is wrong, hurts when flow drifted. | **Investigate first.** Decide: gate Fix 1 (only absorb when vision is established/self-consistent) or drop it and rely on Fix 2 alone. |
| OPEN-2 | **Re-acquisition ramp (Fix 2) did not soften the reset.** | Verify it was actually in the flashed build; check for a bug; check whether PX4's rejection-timeout reset fires regardless of reported covariance on a ~5 m disagreement. |
| OPEN-3 | **Vision coverage is the root issue** — no fusion for 79 s (20:43) / 19 s gap (19:44). **Confirmed:** the ±30° incidence gate (flashed for 20:43) *caused* the 79 s gap by rejecting oblique wall markers during the traverse. | **User decision: fix coverage first.** Relax the incidence gate (≥45°, or revert toward the old behaviour) AND investigate marker layout / camera FOV / flight path. The ±30° gate is empirically too aggressive for this arena/flight geometry — tighter → longer flow-only stretches → bigger resets. |
| OPEN-4 | **Rear-left rotor (drone 2) physical failure.** | Inspect rear-left arm: prop seating/balance/crack, motor bearings/bell, ESC, solder, mount. Hover showed it running ~400 PWM low (marginal) before failure. |
| OPEN-5 | **`CBRK_FLIGHTTERM=0`** (termination enabled, cuts motors on >60°/0.3 s). | Did not cause any crash, but consider `121212` for indoor low-altitude tests. User decision pending. |
| OPEN-6 | **`DRONE_ID`** is a per-drone local flashing value in S3 `main.c` (kept out of version control). | Set it for the airframe being flashed each time; not committed. |
| **OPEN-7** | **Bad-East ArUco fix → flyaway (18:55, 19:42).** The first/only fixes have a ~6 m lateral (East) error — an IPPE planar-flip whose reflected solution lands *inside* the arena, so the bounds gate (`aruco_pose.cpp:92-97`) misses it. PX4 `reset_pos_to_vision` snaps East and runs to 27–35 m. | **Add a position-innovation gate:** reject any EV fix disagreeing with the current PX4 position by > ~2–3 m (flow holds short-term position, so a 6 m jump is provably wrong). Plus stronger flip disambiguation (use px4 yaw/position, not just arena bounds). Would have stopped *both* flyaways. |
| **OPEN-8** | **180° yaw flip on first fix RETURNED (20:07).** `bd10a7a` set `cov[20]=NaN` to kill EV-yaw fusion, but **PX4 ignores the message yaw covariance** (`EKF2_EV_NOISE_MD=0` → uses `EKF2_EVA_NOISE`) and fuses EV yaw because `EKF2_EV_CTRL=15` has the yaw bit. First fix's flipped yaw → physical 180° flip. | **Either** set `EKF2_EV_CTRL=7` (drop yaw bit 8; gyro owns heading, ~1.5°/traverse drift — acceptable) **or** restore the verified `513cd8a` px4_yaw disambiguation. Param fix is cleanest and matches `bd10a7a`'s stated intent. |
| **OPEN-9** | **Live trail frozen during flight.** Disc dead-reckon (`main.c:1089`) is gated on `inertial_anchor_valid`, set only after the first ArUco fix — which never arrives until landing → trail stuck at home the whole flight. | **Anchor dead-reckon at ARM** from the already-known home + `px4_home` (`main.c:871-877`), so the disc tracks live from takeoff (drifts with flow, snaps on ArUco). RViz side already done (`4fdc15b`). |

> **To-do review (Claude, 2026-06-01):** OPEN-3 is the trunk — **coverage is still broken at 45°**, so relaxing the gate did not help; the real question is *why P4 detects nothing during the traverse* (camera FOV/exposure, marker layout vs flight path, detection pipeline), not the gate angle. OPEN-7/8 are both **flip** failures and likely share a cause (oblique-view IPPE ambiguity admitted by the 45° gate); fixing detection quality + adding an innovation gate addresses both. OPEN-1/OPEN-2 remain valid but are downstream of OPEN-3/7. Recommended order: **OPEN-8 (param, instant, stops flips) → OPEN-9 (so we can *see* the drone) → OPEN-7 (innovation gate) → OPEN-3 (root coverage).**

---

## How to read the logs
- **ulog:** `pip install --user --break-system-packages pyulog`; key topics `vehicle_attitude(_setpoint)`,
  `vehicle_local_position(_setpoint)` (incl. `xy_reset_counter`/`delta_xy`), `actuator_motors`,
  `vehicle_angular_velocity`, `estimator_innovations.heading`, `estimator_status_flags.cs_ev_pos/cs_ev_yaw`,
  `estimator_ev_pos_bias`, `control_allocator_status`, `vehicle_status`, `actuator_armed`.
- **rosbag (mcap):** `source /opt/ros/jazzy/setup.bash`, read with `rosbag2_py` + `rclpy.serialization`.
  Topics under `/drone_N/` (`vision_pose`, `state`); NB rosbag clock ≠ ulog clock.
- **vision_yaw** (camera +Z fwd → world XY): `atan2(2(qy·qz − qx·qw), 2(qx·qz + qy·qw))`.
