# Launcher — Ubuntu GNOME PC (x86_64)

Development and home-lab ground control station.

| Property | Value |
|----------|-------|
| Platform | x86_64 |
| OS | Ubuntu 24.04 LTS (GNOME) |
| ROS2 | Jazzy |
| Network role | Wired to GL-AX1800 Flint router |

---

## Network Setup

The ground station connects to the swarm network via a wired Ethernet link to the **GL-AX1800 Flint** router. Drones join the same network over WiFi.

All ROS2 nodes communicate over this shared LAN. No VPN or bridge is required.

Recommended: assign this machine a static IP in the Flint DHCP reservation table so launch scripts can rely on a fixed address.

---

## Prerequisites

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# micro-ROS agent (runs as a Docker container — see drone-firmware/docker/)
sudo apt install docker.io docker-compose-plugin

# RViz / rqt (included in ros-jazzy-desktop)
# OpenCV Python bindings
sudo apt install python3-opencv

# Source ROS2 in your shell (add to ~/.bashrc)
source /opt/ros/jazzy/setup.bash
```

---

## Scripts

| Script | Purpose |
|--------|---------|
| `launch.sh` | Full GCS stack — micro-ROS agent, ROS2 topic monitor, RViz2, rqt control panel |
| `launch-swarm-mission.sh` | Swarm L-loop mission — runs `mission_forward_back.py` for drones 1 & 2 in parallel |
| `launch-vision.sh` | GCS-side ArUco vision node (`aruco_node.py`) |
| `launch-drone-vision-p4.sh` | Docker dev shell for ESP32-P4 vision firmware (IDF 5.4, MIPI-CSI) |
| `launch-drone-vision-s3.sh` | Docker dev shell for ESP32-S3 vision firmware (IDF 5.0, DVP) |
| `launch-idf-docker.sh` | Docker dev shell for drone firmware (ESP32-S3, micro-ROS) |
| `launch-node-vision-ide.sh` | Docker dev shell for ESP32-S3 ArUco benchmark |

---

## Quick Start

```bash
# Launch full ground station
./launch.sh

# (Optional) GCS-side ArUco vision only
./launch-vision.sh

# (Optional) Firmware development — drone node
./launch-idf-docker.sh

# (Optional) Firmware development — ESP32-P4 vision board
./launch-drone-vision-p4.sh

# (Optional) Run swarm mission (requires drones armed via rqt first)
./launch-swarm-mission.sh
```

---

## Desktop Shortcuts

`.desktop` launcher files are in the `desktop/` subfolder. Copy them to `~/.local/share/applications/` or your desktop to get one-click launchers in GNOME.

```bash
cp desktop/*.desktop ~/.local/share/applications/
```
