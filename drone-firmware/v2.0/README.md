# Drone Firmware v2.0

- **[drone-vision-esp32p4](drone-vision-esp32p4/)** — ESP32-P4 firmware — MIPI-CSI camera capture, ArUco marker detection (OpenCV), 6× VL53L1X ToF sensors, transmits pose + ToF data to S3 over UART
- **[drone-comms-esp32s3](drone-comms-esp32s3/)** — ESP32-S3 firmware — receives vision data from P4 over UART, bridges MAVLink between PX4 and the GCS over ROS2 micro-ROS
- **[shared](shared/)** — UART binary frame protocol header (`p4_link_protocol.h`) shared by both firmwares
- **[logs](logs/)** — PX4 flight logs (`.ulg`) and ROS2 bag files (`.mcap`) from test sessions

---

## Building and Flashing

Firmware is compiled and flashed from desktop shortcuts on the PC.

<table>
<tr>
<td align="center"><img src="../../launchers/flash_ico.png" width="100"><br><b>IDE — Flash P4</b></td>
<td>Firmware flashing for ESP32-P4 inside Docker (<a href="../../launchers/ubuntu-gnome-pc/launch-drone-vision-p4.sh">launch-drone-vision-p4.sh</a>)</td>
</tr>
<tr>
<td align="center"><img src="../../launchers/flash_ico.png" width="100"><br><b>IDE — Flash S3</b></td>
<td>Firmware flashing for ESP32-S3 inside Docker (<a href="../../launchers/ubuntu-gnome-pc/launch-drone-vision-s3.sh">launch-drone-vision-s3.sh</a>)</td>
</tr>
</table>

> **Note:** To avoid versioning issues, code is built and compiled inside Docker.

### idf.py commands

| Command | Description |
|---------|-------------|
| `idf.py set-target <target>` | Set the chip target — `esp32p4` for the vision module or `esp32s3` for the comms module |
| `idf.py fullclean` | Delete build directory |
| `idf.py menuconfig` | Open settings menu for firmware settings (important for ESP32-S3) |
| `idf.py build` | Compile firmware |
| `idf.py flash` | Flash to board |
| `idf.py monitor` | Open serial console for debugging and troubleshooting |
