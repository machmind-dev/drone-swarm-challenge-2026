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
   http://machmind.dev                               Team Mach Mind (c) 2026
EOF

echo -e "${WHITE}"
echo -e "                   [Node Vision IDE — OpenCV ArUco Benchmark]"
echo -e "${RESET}"

echo ""
echo "[INFO] Initializing ESP-IDF + OpenCV environment ..."
sleep 1

PROJECT_DIR="$HOME/drone-swarm-challenge-2026/drone-firmware/v2.0/drone-comms-esp32s3"
COMPOSE_FILE="docker/docker-compose.yml"
SERVICE_NAME="esp32s3_vision"

if [ ! -d "$PROJECT_DIR" ]; then
    echo "[ERROR] Project directory not found: $PROJECT_DIR"
    exit 1
fi

cd "$PROJECT_DIR" || exit 1

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed."
    exit 1
fi

# Force rebuild with: REBUILD=1 bash launch-node-vision-ide.sh
if [ "${REBUILD:-0}" = "1" ]; then
    echo "[INFO] Building Docker image (first run may take a few minutes) ..."
    sleep 1
    docker compose -f "$COMPOSE_FILE" build
fi

echo "[INFO] Starting container ..."
docker compose -f "$COMPOSE_FILE" up -d "$SERVICE_NAME"

if [ $? -ne 0 ]; then
    echo "[ERROR] Docker compose up failed."
    exit 1
fi

echo "[INFO] Fixing /code permissions ..."
docker compose -f "$COMPOSE_FILE" exec --user root "$SERVICE_NAME" \
    chown -R espidf:espidf /code 2>/dev/null || true

echo "[INFO] Entering container ..."
sleep 1

docker compose -f "$COMPOSE_FILE" exec -it "$SERVICE_NAME" bash -ic '
cd /code || exit 1

if [ -f /opt/esp/idf/export.sh ]; then
    source /opt/esp/idf/export.sh
    echo "[INFO] ESP-IDF environment loaded"
else
    echo "[WARN] /opt/esp/idf/export.sh not found"
fi

export HISTFILE=/code/.bash_history
touch "$HISTFILE"
export HISTSIZE=5000
export HISTFILESIZE=10000
history -r 2>/dev/null || true

echo ""
echo "[INFO] Working directory: $(pwd)"
echo "[INFO] Project: drone-vision — ESP32S3 ArUco FPS Benchmark"
echo ""
echo "  Quick commands:"
echo "    idf.py build                              — compile firmware"
echo "    idf.py -p /dev/ttyACM0 flash monitor     — flash + serial monitor"
echo "    idf.py menuconfig                        — change board / benchmark settings"
echo "    idf.py fullclean                          — clean build artefacts"
echo ""
echo "  Benchmark stages run automatically after boot:"
echo "    Stage 0: QQVGA  160x120"
echo "    Stage 1: QVGA   320x240"
echo "    Stage 2: HVGA   480x320"
echo "    Stage 3: VGA    640x480"
echo ""

exec bash -i
'
