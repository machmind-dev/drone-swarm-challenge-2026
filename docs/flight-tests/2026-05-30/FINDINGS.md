# Yaw-flip root cause — flight tests 2026-05-30 (LH scene, drone_2)

**TL;DR.** The drone's 180° yaw "flip" on first marker acquisition is **not** a pure
ArUco bug. The root cause is that **the airframe has no magnetometer**
(`EKF2_MAG_TYPE = 5`) and, indoors, no GPS — so **ArUco yaw is PX4's only absolute
heading source** (`EKF2_EV_CTRL = 15`, yaw bit set). When ArUco yaw is ambiguous at
poor viewing geometry (low altitude / steep look-up angle), a ~180°-wrong yaw is fed
to EKF2 with nothing to veto it, the heading estimate snaps, and the drone — chasing
its yaw setpoint — physically rotates. At good geometry (≈ marker height, square-on)
the yaw is unambiguous and everything is stable.

This took several flights to isolate; the evidence is preserved here.

## Artifacts in this folder

- `px4_log_159.ulg` — PX4 flight-controller log (the decisive evidence).
- `bag_2048/` — `ros2 bag` of `/drone_2/vision_pose`, `/state`, markers, with an
  ArUco-EKF **disable → enable** toggle mid-flight. 195 s.
- `bag_2025/` — earlier flight, vision valid only ~2 s (drone stayed < 0.9 m).

## Evidence

### 1. No magnetometer → ArUco yaw is the sole heading reference
From `px4_log_159.ulg` initial parameters:
```
EKF2_MAG_TYPE = 5    # magnetometer OFF (no mag heading)
EKF2_EV_CTRL  = 15   # external vision = pos + vel + YAW (bit 8 set)
```
The airframe physically has no magnetometer. Indoors there is no GPS course either.
So EKF2 cannot cross-check the ArUco heading against anything.

### 2. Flips coincide with huge external-vision yaw innovations
In the log the yaw **estimate tracks the yaw setpoint** cleanly — except two bursts
where it jerks while the setpoint is held, each coinciding with a large EV heading
innovation (~2.5–2.9 rad ≈ 145–165°):
```
t≈55–58s : yaw est 89→39→2→−62→84 (setpoint held 90)  EVhdg innov 2.56,1.36,2.88 rad
t≈127–135s: yaw est 92→59→−123→163→−32 (near landing)  EVhdg innov 2.84,2.27 rad
```
A ~2.8 rad innovation means the EV yaw measurement arrived ~160–180° away from the
EKF's (correct) estimate. With no mag to reject it, EKF2 fused it and the heading
snapped. Between the bursts (good geometry) innovation is 0.0 and yaw is stable.

### 3. The vision pose itself is stable at altitude
From `bag_2048/`, every accepted `/drone_2/vision_pose` is stable, e.g. at ~2 m
square-on to marker 12 (at world (5,0)): position ≈ (5.0, 3.9), `visYaw ≈ −90°`
(pointing −Y straight at the marker) — correct to within a degree, no oscillation.
The flips happen in the *gaps* (e.g. a 70 s window with **zero** published poses)
because the S3 reject gates (`reproj > 10 px`, `jump > 1 m`, `main.c:1519,1526`) drop
the flipped/ambiguous frames before publishing — so the bag shows the flip's shadow
(vision rejected) rather than the flip values. The PX4 log is what exposes them.

## Why the earlier `aruco_pose.cpp` gate only partly helped
The absolute arena/wall-side gate (commit `8ed12e5`) rejects the IPPE solution that
reflects the drone *behind the wall* (a position flip). But at poor geometry the two
IPPE solutions can have **nearly the same position with ~180°-opposite yaw** — both
pass a position gate, so the wrong-yaw one can still be selected and sent as EV yaw.
A position gate cannot fix a same-position yaw flip.

## Fix directions (ranked)
1. **S3 yaw-continuity reject gate** — mirror the existing 1 m position-jump gate:
   reject a frame whose `vision_yaw` differs > ~90° from the last accepted yaw. Drops
   180° flips before they reach EKF2. Catches everything **after** the first fix.
2. **Loosen PX4 trust in EV yaw** — `EKF2_EVA_NOISE` is `0.10` (very tight); raising to
   ~0.3–0.5 lets EKF2 smooth a single bad yaw instead of snapping. Param-only.
3. **Operational** — acquire the *first* marker at altitude (≈ 2 m, square-on). The
   very first fix at a bad angle is genuinely undecidable with no mag and no prior yaw.
4. **Add a heading reference** — a magnetometer, or seed EKF yaw at arm from the known
   start orientation (drone's start pose in the arena is known). Either lets EKF2
   reject a flipped ArUco yaw outright. Note: indoor magnetometers are often unreliable
   (steel/motors), and measured gyro drift over a 90° turn is low — so seeding heading
   + the yaw gate may be preferable to adding a mag.

## Reading the logs
```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play docs/flight-tests/2026-05-30/bag_2048      # replay
# PX4 ulog: open in QGroundControl / Flight Review, or parse with pyulog
```
