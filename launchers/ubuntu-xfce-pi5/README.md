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

<table>
<tr>
<td align="center"><img src="../GCS_GUI_ico.png" width="150"><br><b>GCS - Mach Mind</b></td>
<td><code>launch.sh</code></td>
<td>Launches ROS2, uROS, RViz, rqt, troubleshooting</td>
</tr>
<tr>
<td align="center"><img src="../GCS_Buttons_ico.png" width="150"><br><b>Mach Mind - Buttons</b></td>
<td><code>launch-buttons.sh</code></td>
<td>Physical buttons publishing state over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../swam_commander_ico.png" width="150"><br><b>GCS — SDC26 Commander</b></td>
<td><code>launch-sdc26-commander.sh</code></td>
<td>Swarm orchestrator — assigns boxes to executors, leader home-check</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="150"><br><b>GCS — Waypoint Commander</b></td>
<td><code>launch-waypoint-commander.sh</code></td>
<td>Send arena coordinates to drones via Manhattan navigation</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="150"><br><b>GCS — Keyboard Control</b></td>
<td><code>launch-swarm-mission.sh</code></td>
<td>Real-time keyboard control of a single drone</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="150"><br><b>GCS — Swarm Ollama</b></td>
<td><code>launch-swarm-ollama.sh</code></td>
<td>Launches Ollama services over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../stream_ico.png" width="150"><br><b>GCS — Arena View</b></td>
<td><code>launch-arena-view.sh</code></td>
<td>Camera and ToF data live stream over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../vision_ico.png" width="150"><br><b>GCS — Vision</b></td>
<td><code>launch-vision.sh</code></td>
<td>GCS ArUco detection via OpenCV (not used)</td>
</tr>
<tr>
<td align="center"><img src="../idf_py_ico.png" width="150"><br><b>Mach Mind Node IDE</b></td>
<td><code>launch-node-ide.sh</code></td>
<td>Firmware flashing for ESP32 inside Docker</td>
</tr>
</table>

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
