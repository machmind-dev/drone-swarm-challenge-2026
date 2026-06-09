# Drone Firmware v2.0

| Folder | Description |
|--------|-------------|
| [drone-vision-esp32p4](drone-vision-esp32p4/) | ESP32-P4 firmware — MIPI-CSI camera capture, ArUco marker detection (OpenCV), 6× VL53L1X ToF sensors, transmits pose + ToF data to S3 over UART |
| [drone-comms-esp32s3](drone-comms-esp32s3/) | ESP32-S3 firmware — receives vision data from P4 over UART, bridges MAVLink between PX4 and the GCS over ROS2 micro-ROS |
| [shared](shared/) | UART binary frame protocol header (`p4_link_protocol.h`) shared by both firmwares |
| [logs](logs/) | PX4 flight logs (`.ulg`) and ROS2 bag files (`.mcap`) from test sessions |
