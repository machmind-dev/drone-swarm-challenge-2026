# Launcher — Ubuntu XFCE Pi5 (ARM64)

Field ground control station — deployed at the competition site.

| Property | Value |
|----------|-------|
| Platform | ARM64 (Raspberry Pi 5) |
| OS | Ubuntu 24.04 LTS (XFCE) |
| ROS2 | Jazzy |
| Network role | Wired to GL-AX1800 Flint router |

---

## Network Setup

The Pi5 connects to the swarm network via a wired Ethernet link to the **GL-AX1800 Flint** router. Drones join the same network over WiFi.

Recommended: assign the Pi5 a static IP in the Flint DHCP reservation table so it is always reachable at a known address from other nodes.

---

## Prerequisites

```bash
# ROS2 Jazzy (ARM64 — use the standard apt repo)
sudo apt install ros-jazzy-desktop

# micro-ROS agent (Docker)
sudo apt install docker.io docker-compose-plugin
sudo usermod -aG docker $USER   # re-login after this

# OpenCV Python bindings
sudo apt install python3-opencv

# Source ROS2 in your shell (add to ~/.bashrc)
source /opt/ros/jazzy/setup.bash
```

> **Note:** The Pi5 runs a lighter XFCE desktop to preserve resources for ROS2 nodes. Avoid running RViz on the Pi5 in the field — use it only for diagnostics.

---

## Scripts

| Script | Purpose |
|--------|---------|
| `setup.sh` | One-time dependency install and workspace build |
| `launch.sh` | Start the full GCS stack (micro-ROS agent + ROS2 nodes) |
| `launch-vision.sh` | Start the ArUco vision node only |
| `launch-buttons.sh` | Start the physical button handler ROS2 publisher |

---

## Quick Start

```bash
# 1. First-time setup (run once after imaging the SD card)
./setup.sh

# 2. Launch full ground station
./launch.sh

# 3. (Optional) Vision only
./launch-vision.sh

# 4. (Optional) Button handler only
./launch-buttons.sh
```

---

## Desktop Shortcuts

`.desktop` launcher files are in the `desktop/` subfolder. In XFCE, copy them to `~/Desktop` for one-click launchers, or to `~/.local/share/applications/` to appear in the application menu.

```bash
cp desktop/*.desktop ~/Desktop/
chmod +x ~/Desktop/*.desktop
```
