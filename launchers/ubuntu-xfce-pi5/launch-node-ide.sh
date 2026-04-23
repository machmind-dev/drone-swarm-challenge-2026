#!/bin/bash

if [ -z "$MACHMIND_NODE_DEV_TERMINAL" ]; then
    export NO_AT_BRIDGE=1
    SCRIPT_ABS="$(readlink -f "$0")"

    if command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind Node Development" \
            -e "bash -c 'export MACHMIND_NODE_DEV_TERMINAL=1 NO_AT_BRIDGE=1; bash \"$SCRIPT_ABS\"; exec bash'"
    fi
fi

clear
export NO_AT_BRIDGE=1
trap 'rc=$?; echo ""; read -rp "[Exited with code $rc. Press Enter to close] " _' EXIT

TEAL=$'\033[38;2;51;117;110m'
WHITE=$'\033[38;2;220;220;220m'
RESET=$'\033[0m'

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
echo -e "                   [Node Development Environment]"
echo -e "${RESET}"
echo ""

echo "[INFO] Initializing micro-ROS environment ..."
sleep 1

PROJECT_DIR="$HOME/drone-swarm-challenge-2026/drone-firmware"
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

if ! docker compose version >/dev/null 2>&1; then
    echo "[ERROR] docker compose plugin is not available."
    exit 1
fi

if [ ! -f "$COMPOSE_FILE" ]; then
    echo "[ERROR] Compose file not found: $PROJECT_DIR/$COMPOSE_FILE"
    exit 1
fi

# Rebuild only if explicitly requested:
# REBUILD=1 bash start_machmind_node_development.sh
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

echo "[INFO] Entering running container ..."
sleep 1

TTY_FLAG=""
[ -t 1 ] && TTY_FLAG="-t"

docker compose -f "$COMPOSE_FILE" exec -i $TTY_FLAG "$SERVICE_NAME" bash -ic '
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
echo "[INFO] History file: $HISTFILE"
echo "[INFO] Use arrow-up for previous commands"

exec bash -i
'