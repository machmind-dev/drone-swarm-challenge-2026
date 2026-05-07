#!/bin/bash

clear

TEAL='\033[38;2;51;117;110m'
WHITE='\033[38;2;220;220;220m'
RESET='\033[0m'

echo -e "${TEAL}"
cat << "EOF"
   ███╗   ███╗ █████╗  ██████╗██╗  ██╗    ███╗   ███╗██╗███╗   ██╗██████╗
   ████╗ ████║██╔══██╗██╔════╝██║  ██║    ████╗ ████║██║████╗  ██║██╔══██╗
   ██╔████╔██║███████║██║     ███████║    ██╔████╔██║██║██╔██╗ ██║██║  ██║
   ██║╚██╔╝██║██╔══██║██║     ██╔══██║    ██║╚██╔╝██║██║██║╚██╗██║██║  ██║
   ██║ ╚═╝ ██║██║  ██║╚██████╗██║  ██║    ██║ ╚═╝ ██║██║██║ ╚████║██████╔╝
   ╚═╝     ╚═╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝    ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝
   http://machmind.dev                                Team Mach Mind (c) 2026
EOF

echo -e "${WHITE}"
echo -e "               [Drone Vision — ESP32-P4 + OV5647 — IDF 5.4 — MIPI-CSI]"
echo -e "${RESET}"
echo ""
echo "[INFO] Commands once inside the container:"
echo "         idf.py set-target esp32p4   (first time only)"
echo "         idf.py build"
echo "         idf.py -p /dev/ttyACM0 flash monitor"
echo ""
sleep 1

PROJECT_DIR="$HOME/drone-swarm-challenge-2026/drone-firmware/v2.0/drone-vision-esp32p4"
COMPOSE_FILE="docker/docker-compose.yml"
SERVICE_NAME="esp32p4_vision"

if [ ! -d "$PROJECT_DIR" ]; then
    echo "[ERROR] Project directory not found: $PROJECT_DIR"
    exit 1
fi

cd "$PROJECT_DIR" || exit 1

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed."
    exit 1
fi

mkdir -p "$PROJECT_DIR/managed_components"

if [ "${REBUILD:-0}" = "1" ]; then
    echo "[INFO] Building docker container (IDF 5.3) ..."
    docker compose -f "$COMPOSE_FILE" up -d --build
else
    echo "[INFO] Starting docker container ..."
    docker compose -f "$COMPOSE_FILE" up -d
fi

if [ $? -ne 0 ]; then
    echo "[ERROR] Docker compose start failed."
    exit 1
fi

echo "[INFO] Fixing /code permissions ..."
docker compose -f "$COMPOSE_FILE" exec --user root "$SERVICE_NAME" \
    chown -R espidf:espidf /code

echo "[INFO] Entering container ..."
sleep 1

docker compose -f "$COMPOSE_FILE" exec -it "$SERVICE_NAME" bash -ic '
cd /code || exit 1

if [ -f /opt/esp/idf/export.sh ]; then
    source /opt/esp/idf/export.sh
    echo "[INFO] ESP-IDF 5.3 environment loaded"
else
    echo "[WARN] /opt/esp/idf/export.sh not found"
fi

export HISTFILE=/code/.bash_history
touch "$HISTFILE"
export HISTSIZE=5000
export HISTFILESIZE=10000
history -r 2>/dev/null || true

echo "[INFO] Target: ESP32-P4  IDF: 5.4  Camera: OV5647 MIPI-CSI"
echo "[INFO] Working directory: $(pwd)"
echo ""

# Check if target has been set for this build directory
if [ ! -f sdkconfig ] || ! grep -q "CONFIG_IDF_TARGET_ESP32P4=y" sdkconfig 2>/dev/null; then
    echo "[WARN] sdkconfig not set for ESP32-P4 — run:"
    echo "         idf.py set-target esp32p4"
    echo ""
fi

echo "[INFO] Build:  idf.py build"
echo "[INFO] Flash:  idf.py -p /dev/ttyACM0 flash monitor"
echo ""

exec bash -i
'
