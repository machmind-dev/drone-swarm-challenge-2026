# TODO — download PX4 ulogs from Drone 2

**Reminder (set at the flying field, 2026-05-30):** pull the PX4 flight-controller
`.ulog` files off **Drone 2** for the yaw-flip / heading-seed diagnosis.

## Why
The onboard PX4 `.ulog` is the only reliable source for this: it records EKF yaw,
external-vision yaw innovations, and fusion **independent of the micro-ROS / GCS
link**. The `22:21` rosbag captured almost no drone telemetry (micro-ROS was not
delivering — 60 msgs, 0 `vision_pose`), so the bag can't show the flip; the ulog can.

## Which logs
- **Priority:** the **22:21 flight** ulog (LH scene, takeoff → climb 2 m → marker 12,
  still flipped). Drop it in `docs/flight-tests/2026-05-30/22-21/` (or alongside this file).
- Helpful but lower priority: ulogs from the other late-evening flights.
- Skip the two invalid runs (camera cap left on / LH scene not initiated).

## How
- QGroundControl → Analyze Tools → **Log Download** → newest entries, or
- pull `*.ulg` from the SD card under `/fs/microsd/log/<date>/`.

## What we'll check in the ulog
1. Did the **heading seed** arrive? (EV-yaw measurements at the start heading
   during the ~4 s arm window — confirms `team_color` reached the drone and
   `seed_yaw_valid` was true.)
2. EKF yaw + EV-yaw **innovation** at the first marker fix — small (seed worked) vs
   ~2.8 rad / 180° (un-seeded flip).

## Open decision
Make the seed **default to LH (+X)** at boot so it no longer depends on `team_color`
arriving over the (flaky) micro-ROS link. See `FINDINGS.md` and project memory.
