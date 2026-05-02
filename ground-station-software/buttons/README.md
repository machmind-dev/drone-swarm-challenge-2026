# Physical Button Handler

ROS2 publisher node for the ground station physical control buttons (Raspberry Pi 5).

## Hardware

Three push-buttons wired to Raspberry Pi 5 GPIO (gpiochip4):

| Button | GPIO pin | Action |
|--------|----------|--------|
| ARM | 5 | Hold → COMMAND_ARM to all drones; release → COMMAND_DISARM |
| MISSION | 6 | Press → COMMAND_MISSION_START to armed drones |
| EMERG | 13 | Short press → COMMAND_ELAND; hold ≥ 3 s → COMMAND_KILL |

## Entry Point

`btn_test.py` — reads GPIO via `libgpiod`, publishes commands over ROS2.

## Launch

```bash
# Via launcher script (recommended):
launchers/ubuntu-xfce-pi5/launch-buttons.sh

# Manually:
source /opt/ros/jazzy/setup.bash
python3 buttons/btn_test.py
```

## Dependencies

```bash
sudo apt install python3-gpiod
```
