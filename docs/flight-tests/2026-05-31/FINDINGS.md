# Flight tests 2026-05-31 (LH scene, drone_2) — yaw fix confirmed + free-fall crash

**TL;DR.**
1. The **180° first-fix yaw flip is FIXED.** The `px4_yaw` disambiguation (commit
   `513cd8a`) worked in flight: heading innovation at first ArUco fix peaked at **21°**
   instead of the previous **~163–176°**. No EKF heading snap.
2. The **free-fall crash was a physical propulsion failure**, not software / vision /
   control logic. The **rear-left rotor (PX4 output index 1, "Motor 2") lost thrust**;
   the airframe tumbled at >250 °/s and fell from 2 m. PX4 responded correctly.
3. `CBRK_FLIGHTTERM` and the vision-position estimate were **ruled out** as the cause.

Evidence: PX4 ulogs `log_171` (17:42) and `log_173` (18:31), kept by the user under
`~/Desktop/Flgiht Test Record/`. Parsed with pyulog; rosbags read with `rosbag2_py`.

---

## 1. Yaw-flip fix — CONFIRMED working (log_171, 17:42)

The earlier "vision pose is correct, only the RViz arrow is a viz artifact" conclusion
(2026-05-30) was **wrong for the first fix**. log_171 settled it with the raw gyro:

- Integrated body yaw-rate from arm: flat 0° for 48 s, then **+89.9°** when the pilot
  rotated to face marker 12. **EKF2 tracked it correctly to +89°** — so the no-mag
  gyro heading *mean* is accurate (drift ≈ 0.4°/10 s). Heading tracking is not the issue.
- First ArUco-12 fix: EV yaw measured **≈ −80°** vs EKF **+89°** → ~170° disagreement
  → heading reset + oscillation = the flip. So **the ArUco yaw itself was still flipping
  ~180° on the first fix** (the IPPE same-position ambiguity), not a viz artifact.
- It *succeeds* because with no mag the EKF heading is unobservable: over a long loiter
  the heading *variance* grows huge (mean stays right via gyro), so a tight-covariance
  (`VISION_YAW_COV = 0.05`) flipped EV yaw easily overrides the correct-but-uncertain prior.

### The fix (commit `513cd8a`, `drone-comms-esp32s3/main.c`)
`px4_yaw` disambiguation: after deriving the heading from the incoming ArUco quaternion,
if it disagrees with the gyro-propagated EKF heading (`px4_yaw`, from the ATTITUDE
message) by more than 90°, add 180°. Unlike the prior-vision continuity gate
(`MAX_YAW_JUMP_RAD`), this fires on the **first** fix after arm/dropout — where the flip
actually happens. New `YAW_DISAMBIG_ENABLE`/`YAW_DISAMBIG_RAD`, `px4_yaw_valid` flag.

### Result (log_173, 18:31, marker 12 → 11)
Heading innovation peaked at **21°** (max), first nonzero 17° then 7–12°. **No flip.**
The seed (`SEED_YAW_*`) turned out to be a red herring — it never fused in flight and
wouldn't have helped anyway (EKF mean already correct via gyro). It is left in place.

---

## 2. Free-fall crash — physical rotor failure (log_173, 18:31)

Flight: manual control to ArUco 12, then to ArUco 11; crashed by free fall from ~2 m at
**t = 54.76 s post-arm**.

### Timeline (post-arm)
| t | State |
|---|---|
| ≤ 54.74 s | Stable: roll/pitch ~1°, body rates ~0–4 °/s, hover 2 m, tracking toward sp (4.5, 0), thrust −0.64 |
| 54.76 s | Body rates begin diverging (pitch 5, yaw −15 °/s) |
| 54.78 s | Rates explode: roll −32, pitch +34 °/s |
| 54.8–54.9 s | Tumble: roll −240, pitch +262 °/s; motors saturate asymmetric `[0.0, 1.0, 0.30, 0.15]`; thrust commanded to **max (−1.0)**; net lift collapses |
| 55.1 s | Roll −102° (inverted), free-falling at 2.3 m/s |

### Decisive test: commanded vs actual attitude
The setpoint stayed calm while the actual attitude ran away — proving the controller was
a **victim**, not the instigator (a vision/termination-driven event would jump the
*setpoint* first):

| t | roll/pitch **setpoint** | roll/pitch **actual** |
|---|---|---|
| 54.75 s | (2.8°, 1.5°) | (1.2°, 1.0°) |
| 54.85 s | (2.0°, 3.3°) — calm | (−8°, 9.5°) |
| 54.90 s | (3.8°, 3.0°) — calm | (−22°, 21°) |
| 55.05 s | (16°, −7°) — *now* reacting | (−79°, 39°) — already gone |

### Which rotor — rear-left (output index 1 / "Motor 2")
`CA_ROTOR` geometry (body FRD, +X fwd / +Y right):

| index | (PX, PY) | corner |
|---|---|---|
| 0 (M1) | (+0.15, +0.15) | front-right |
| **1 (M2)** | **(−0.15, −0.15)** | **rear-left** |
| 2 (M3) | (+0.15, −0.15) | front-left |
| 3 (M4) | (−0.15, +0.15) | rear-right |

Two independent signatures both indict the rear-left rotor:
1. **Direction of fall:** pitch +21° (nose up → rear drops) + roll −22° (left down) ⇒
   the **rear-left corner was lowest** ⇒ that corner lost lift.
2. **Mixer response:** in the first 50 ms the controller ramped **M1 (rear-left) 0.56 →
   0.91 → 1.0** while cutting its diagonal **M0 (front-right) → 0.0**. It slammed the
   rear-left motor to max to lift the dropping corner; the corner kept dropping — the
   definition of a failed rotor.

Corroborating: during stable hover the rear pair (M1/M3) ran ~400 PWM *lower* than the
front pair (M0/M2), M1 lowest — that rotor/prop was underperforming before it let go.
At the crash `unallocated_thrust = −0.4` (40% of demanded thrust undeliverable).

### Ruled out
- **Battery:** healthy throughout (15.8 V, ~3 A).
- **Failsafe / disarm / lockdown:** none. `arming_state` stayed ARMED (2),
  `nav_state = 14` (OFFBOARD), `force_failsafe = 0`.
- **`CBRK_FLIGHTTERM`:** enabled (`= 0`, not the `121212` bypass) with `FD_FAIL_R/P = 60°`,
  TTRI `0.3 s`. But the tumble began at 54.76 s while attitude was still ~1–2°; roll did
  not reach 60° until ~55.05 s, so termination's 0.3 s timer couldn't elapse until
  ~55.35 s — past the end of the log. Motors were asymmetric, not all-min. **Termination
  never fired**; if it had, it would have been downstream of the failure, not its cause.
- **Vision position estimate:** EKF velocity was smooth right through the event (vx
  decelerating 0.92 → 0.56 as it neared the setpoint, vz ≈ 0). The EV-position innovation
  swing at the marker 12→11 handoff (+1.48 → −1.55 m) was absorbed cleanly — `ev_pos_bias`
  fused on its normal cadence with no velocity spike. `EKF2_EVP_NOISE = 0.1` is tight but
  did not destabilise anything here.

---

## Recommendations
1. **Hardware first:** inspect the **rear-left arm** — prop (seating/balance/crack),
   motor (bearings, bell, windings), ESC, solder joints, arm/mount. It was marginal
   (low hover PWM) before failing.
2. **For indoor low-altitude tests, consider `CBRK_FLIGHTTERM = 121212`** — it did not
   cause this crash, but while enabled any future >60°/0.3 s excursion will cut all
   motors and guarantee a crash with no recovery attempt. Trade-off, user's call.
3. Optional secondary hardening: loosen `EKF2_EVA_NOISE` and/or relax `VISION_YAW_COV`
   so a single bad yaw can't override an established heading as violently.

## How to reproduce the analysis
- ulog: `pip install --user --break-system-packages pyulog`; key topics
  `vehicle_attitude`, `vehicle_attitude_setpoint`, `vehicle_local_position(_setpoint)`,
  `actuator_motors`/`actuator_outputs`, `vehicle_angular_velocity`,
  `estimator_innovations.heading`, `estimator_status_flags.cs_ev_yaw`,
  `control_allocator_status`, `vehicle_status`, `actuator_armed`.
- rosbag (mcap): `source /opt/ros/jazzy/setup.bash`, read with `rosbag2_py` +
  `rclpy.serialization`. NB: the 18:31 rosbag captured a *different* segment (controlled
  ELAND+DISARM), not the log_173 free-fall.
