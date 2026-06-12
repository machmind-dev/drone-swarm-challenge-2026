# Drone Firmware v1.0 – ESP32-S3

ESP-IDF firmware for the [**Seeed XIAO ESP32S3 Sense**](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) used in the qualifying rounds of the Swarm Drone Challenge 2026.

## Features

- **ArUco detection** — OV2640 camera (DVP), QQVGA 160×120 downsampled to 80×60, `DICT_4X4_50`, pose via `cv::solvePnP` (`espressif/opencv ^4.10.0~3`)
- **ToF ranging** — VL53L1X via `VL53L1-ULD-ESP` component
- **MAVLink to FC** — sends `VISION_POSITION_ESTIMATE` and `DISTANCE_SENSOR` to PX4 EKF2 over UART
- **micro-ROS over WiFi** — publishes sensor data to ROS 2 via `micro_ros_espidf_component`

## Key parameters

```cpp
#define DET_W  80   // detection resolution (2× downsampled from 160×120)
#define DET_H  60
#define ARUCO_DICT    cv::aruco::DICT_4X4_50
#define MARKER_SIZE_M 0.50f
```

## Related

- [v2.0 Vision Module (ESP32-P4)](../v2.0/drone-vision-esp32p4/) — upgraded to MIPI-CSI, 320×240, OV5647, 6× ToF

---

<!-- SEO: ESP32-S3 ArUco detection · XIAO ESP32S3 Sense ArUco · ESP32-S3 OpenCV solvePnP · ArUco marker pose estimation ESP32 · espressif/opencv ArUco example · ESP-IDF ArUco FreeRTOS · OV2640 ArUco ESP32 · ESP32-S3 camera ArUco · ESP32-S3 VL53L1 ToF MAVLink · micro-ROS ESP32-S3 WiFi -->
