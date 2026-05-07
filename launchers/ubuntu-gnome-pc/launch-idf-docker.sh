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
   http://machmind.dev		      	           Team Mach Mind (c) 2026
EOF

echo -e "${WHITE}"
echo -e "		   [Node Development Environment]"
echo -e "${RESET}"

echo ""
echo "[INFO] Initializing micro-ROS environment ..."
sleep 1

PROJECT_DIR="$HOME/drone-swarm-challenge-2026/drone-firmware/v1.0"
COMPOSE_FILE="docker/docker-compose.yml"
SERVICE_NAME="esp32s3_camera"

if [ ! -d "$PROJECT_DIR" ]; then
    echo "[ERROR] Project directory not found: $PROJECT_DIR"
    exit 1
fi

cd "$PROJECT_DIR" || exit 1

if ! command -v docker >/dev/null 2>&1; then
    echo "[ERROR] Docker is not installed."
    exit 1
fi

# Rebuild only if explicitly requested:
# REBUILD=1 bash launch-idf-docker.sh
if [ "${REBUILD:-0}" = "1" ]; then
    echo "[INFO] Building docker container ..."
    sleep 1
    docker compose -f "$COMPOSE_FILE" up -d --build
else
    echo "[INFO] Starting existing docker container ..."
    sleep 1
    docker compose -f "$COMPOSE_FILE" up -d
fi

if [ $? -ne 0 ]; then
    echo "[ERROR] Docker compose start failed."
    exit 1
fi

echo "[INFO] Fixing /code permissions ..."
docker compose -f "$COMPOSE_FILE" exec --user root "$SERVICE_NAME" \
    chown -R espidf:espidf /code

echo "[INFO] Entering running container ..."
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

echo "[INFO] Working directory: $(pwd)"
echo "[INFO] Available commands: idf.py build | idf.py -p /dev/ttyACM0 flash | idf.py -p /dev/ttyACM0 monitor"

exec bash -i
'



