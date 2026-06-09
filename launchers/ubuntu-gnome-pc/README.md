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

<table>
<tr>
<td align="center"><img src="../gui_ico.png" width="100"><br><b>GCS — Control Station</b></td>
<td><code>launch.sh</code></td>
<td>Launches ROS2, uROS, RViz, rqt, troubleshooting</td>
</tr>
<tr>
<td align="center"><img src="../swam_commander_ico.png" width="100"><br><b>GCS — SDC26 Commander</b></td>
<td><code>launch-sdc26-commander.sh</code></td>
<td>Swarm orchestrator — assigns boxes to executors, leader home-check</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="100"><br><b>Mach Mind — Waypoint Commander</b></td>
<td><code>launch-waypoint-commander.sh</code></td>
<td>Send arena coordinates to drones via Manhattan navigation</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="100"><br><b>GCS — Manual Flight</b></td>
<td><code>launch-swarm-mission.sh</code></td>
<td>Real-time keyboard control of a single drone</td>
</tr>
<tr>
<td align="center"><img src="../swarm_ico.png" width="100"><br><b>GCS — Swarm Ollama</b></td>
<td><code>launch-swarm-ollama.sh</code></td>
<td>Launches Ollama services over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../stream_ico.png" width="100"><br><b>GCS — Arena View</b></td>
<td><code>launch-arena-view.sh</code></td>
<td>Camera and ToF data live stream over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../stream_ico.png" width="100"><br><b>IDE — Stream View</b></td>
<td><code>launch-stream-view.sh</code></td>
<td>Camera and ToF data live stream over ROS2</td>
</tr>
<tr>
<td align="center"><img src="../vision_ico.png" width="100"><br><b>GCS — Vision</b></td>
<td><code>launch-vision.sh</code></td>
<td>GCS ArUco detection via OpenCV (not used)</td>
</tr>
<tr>
<td align="center"><img src="../flash_ico.png" width="100"><br><b>IDE — Flash P4</b></td>
<td><code>launch-drone-vision-p4.sh</code></td>
<td>Firmware flashing for ESP32-P4 inside Docker</td>
</tr>
<tr>
<td align="center"><img src="../flash_ico.png" width="100"><br><b>IDE — Flash S3</b></td>
<td><code>launch-drone-vision-s3.sh</code></td>
<td>Firmware flashing for ESP32-S3 inside Docker</td>
</tr>
<tr>
<td align="center"><img src="../flash_ico.png" width="100"><br><b>IDE — Node v1.0</b></td>
<td><code>launch-idf-docker.sh</code></td>
<td>Firmware flashing for ESP32 inside Docker</td>
</tr>
<tr>
<td align="center"><img src="../flash_ico.png" width="100"><br><b>IDE — Node Vision IDE</b></td>
<td><code>launch-node-vision-ide.sh</code></td>
<td>Docker dev shell for ESP32-S3 ArUco benchmark</td>
</tr>
</table>
