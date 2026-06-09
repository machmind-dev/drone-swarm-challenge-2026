# Drone Firmware v2.0

| Folder | Description |
|--------|-------------|
| [drone-vision-esp32p4](drone-vision-esp32p4/) | ESP32-P4 firmware — MIPI-CSI camera capture, ArUco marker detection (OpenCV), 6× VL53L1X ToF sensors, transmits pose + ToF data to S3 over UART |
| [drone-comms-esp32s3](drone-comms-esp32s3/) | ESP32-S3 firmware — receives vision data from P4 over UART, bridges MAVLink between PX4 and the GCS over ROS2 micro-ROS |
| [shared](shared/) | UART binary frame protocol header (`p4_link_protocol.h`) shared by both firmwares |
| [logs](logs/) | PX4 flight logs (`.ulg`) and ROS2 bag files (`.mcap`) from test sessions |

---

## Building and Flashing

Firmware is compiled and flashed from desktop shortcuts on the GCS PC.

<table>
<tr>
<td align="center"><img src="../../launchers/flash_ico.png" width="100"><br><b>IDE — Flash P4</b></td>
<td><code>launch-drone-vision-p4.sh</code></td>
<td>Firmware flashing for ESP32-P4 inside Docker</td>
</tr>
<tr>
<td align="center"><img src="../../launchers/flash_ico.png" width="100"><br><b>IDE — Flash S3</b></td>
<td><code>launch-drone-vision-s3.sh</code></td>
<td>Firmware flashing for ESP32-S3 inside Docker</td>
</tr>
</table>

### idf.py commands

| Command | Description |
|---------|-------------|
| `idf.py set-target <target>` | Set the chip target before building. Use `esp32p4` for the vision module or `esp32s3` for the comms module. Run once after the first container start or after `fullclean`. |
| `idf.py flash` | Flash the compiled firmware to the board |
