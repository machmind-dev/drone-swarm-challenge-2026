# Drone Firmware v2.0

| Folder | Description |
|--------|-------------|
| [drone-vision-esp32p4](drone-vision-esp32p4/) | ESP32-P4 firmware — MIPI-CSI camera capture, ArUco marker detection (OpenCV), 6× VL53L1X ToF sensors, transmits pose + ToF data to S3 over UART |
| [drone-comms-esp32s3](drone-comms-esp32s3/) | ESP32-S3 firmware — receives vision data from P4 over UART, bridges MAVLink between PX4 and the GCS over ROS2 micro-ROS |
| [shared](shared/) | UART binary frame protocol header (`p4_link_protocol.h`) shared by both firmwares |
| [logs](logs/) | PX4 flight logs (`.ulg`) and ROS2 bag files (`.mcap`) from test sessions |

---

## Building and Flashing

Both firmwares use **ESP-IDF** via Docker. Each project has a `docker/` folder with a `docker-compose.yml`. Run all `idf.py` commands through `docker compose exec`.

### Start the container

```bash
# From drone-vision-esp32p4/docker/ or drone-comms-esp32s3/docker/
docker compose up -d
```

### idf.py commands

| Command | Description |
|---------|-------------|
| `idf.py set-target <target>` | Set the chip target before building. Use `esp32p4` for the vision module or `esp32s3` for the comms module. Run once after the first container start or after `fullclean`. |
| `idf.py flash` | Flash the compiled firmware to the board |

### ESP32-P4 example

```bash
cd drone-vision-esp32p4/docker
docker compose up -d
docker compose exec esp32p4_vision idf.py set-target esp32p4   # first time only
docker compose exec esp32p4_vision idf.py build
docker compose exec esp32p4_vision idf.py -p /dev/ttyACM0 flash monitor
```

### ESP32-S3 example

```bash
cd drone-comms-esp32s3/docker
docker compose up -d
docker compose exec esp32s3_comms idf.py build
docker compose exec esp32s3_comms idf.py -p /dev/ttyACM0 flash monitor
```
