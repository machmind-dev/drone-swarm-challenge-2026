# Flight-test logs & findings — v2.0 firmware (2026-05-31)

Indoor arena, LH scene, manual keyboard control, no magnetometer, no GPS
(`EKF2_MAG_TYPE=5`, vision is PX4's only absolute pos/heading source, `EKF2_EV_CTRL=15`).
Each folder holds the PX4 `.ulg`, the ROS 2 `.mcap` + `metadata.yaml`, and (20:43) the
keyboard-control capture.

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

---

## How to read the logs
- **ulog:** `pip install --user --break-system-packages pyulog`; key topics `vehicle_attitude(_setpoint)`,
  `vehicle_local_position(_setpoint)` (incl. `xy_reset_counter`/`delta_xy`), `actuator_motors`,
  `vehicle_angular_velocity`, `estimator_innovations.heading`, `estimator_status_flags.cs_ev_pos/cs_ev_yaw`,
  `estimator_ev_pos_bias`, `control_allocator_status`, `vehicle_status`, `actuator_armed`.
- **rosbag (mcap):** `source /opt/ros/jazzy/setup.bash`, read with `rosbag2_py` + `rclpy.serialization`.
  Topics under `/drone_N/` (`vision_pose`, `state`); NB rosbag clock ≠ ulog clock.
- **vision_yaw** (camera +Z fwd → world XY): `atan2(2(qy·qz − qx·qw), 2(qx·qz + qy·qw))`.
