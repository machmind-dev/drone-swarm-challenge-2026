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
| `setup.sh` | One-time dependency install and workspace build |
| `launch.sh` | Start the full GCS stack (micro-ROS agent + ROS2 nodes + RViz) |
| `launch-idf-docker.sh` | Start the ESP-IDF Docker environment for firmware development |
| `launch-vision.sh` | Start the ArUco vision node only |

---

## Quick Start

```bash
# 1. First-time setup
./setup.sh

# 2. Launch full ground station
./launch.sh

# 3. (Optional) Firmware development environment
./launch-idf-docker.sh
```

---

## Desktop Shortcuts

`.desktop` launcher files are in the `desktop/` subfolder. Copy them to `~/.local/share/applications/` or your desktop to get one-click launchers in GNOME.

```bash
cp desktop/*.desktop ~/.local/share/applications/
```
