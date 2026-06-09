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

# GPIO library (for physical button handler)
sudo apt install python3-gpiod

# Source ROS2 in your shell (add to ~/.bashrc)
source /opt/ros/jazzy/setup.bash
```

> **Note:** The Pi5 runs a lighter XFCE desktop to preserve resources for ROS2 nodes. Avoid running RViz on the Pi5 in the field — use it only for diagnostics.

---

## Scripts

| Script | Purpose |
|--------|---------|
| `launch.sh` | Full GCS stack — micro-ROS agent, ROS2 topic monitor, rqt, RViz2 (xfce4-terminal tabs) |
| `launch-swarm-mission.sh` | Swarm L-loop mission — runs `mission_forward_back.py` for drones 1 & 2 in parallel |
| `launch-vision.sh` | GCS-side ArUco vision node (`aruco_node.py`) |
| `launch-buttons.sh` | Physical GPIO button handler — ARM / MISSION / EMERG buttons via gpiochip4 |
| `launch-node-ide.sh` | Docker dev shell for drone firmware (ESP32-S3, micro-ROS) |
| `launch-sdc26-commander.sh` | SDC26 swarm orchestrator — assigns boxes to executors, monitors leader home zone |
| `launch-waypoint-commander.sh` | Manual waypoint sender — `<drone_id\|all> <x> <y> [z] [yaw_deg]` in arena frame |
| `launch-arena-view.sh` | Top-down arena position map — serial port selector for P4 (ArUco pose) and S3 (LOCAL_NED) |
| `launch-swarm-ollama.sh` | LLM swarm control (DEMO) — Gemma3 1B natural-language → waypoint commands via Ollama |

---

## Quick Start

```bash
# 1. Launch full ground station
./launch.sh

# 2. (Optional) Physical button handler
./launch-buttons.sh

# 3. (Optional) ArUco vision only
./launch-vision.sh

# 4. (Optional) Run swarm mission (requires drones armed via rqt first)
./launch-swarm-mission.sh
```

---

## Desktop Shortcuts

`.desktop` launcher files are in the `desktop/` subfolder. In XFCE, copy them to `~/Desktop` for one-click launchers, or to `~/.local/share/applications/` to appear in the application menu.

```bash
cp desktop/*.desktop ~/Desktop/
chmod +x ~/Desktop/*.desktop
```
