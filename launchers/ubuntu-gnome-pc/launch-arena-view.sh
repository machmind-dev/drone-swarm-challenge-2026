#!/bin/bash

clear

TEAL='\033[38;2;51;117;110m'
WHITE='\033[38;2;220;220;220m'
YELLOW='\033[38;2;255;200;60m'
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
echo -e "              [ESP32-P4 Arena View — Top-Down Position Map]"
echo -e "${RESET}"
echo ""

SCRIPT="$HOME/drone-swarm-challenge-2026/drone-firmware/v2.0/drone-vision-esp32p4/tools/arena_view.py"
PORT="${1:-/dev/ttyACM0}"
BAUD="${2:-921600}"

if [ ! -f "$SCRIPT" ]; then
    echo "[ERROR] arena_view.py not found: $SCRIPT"
    read -rp "Press Enter to close..."
    exit 1
fi

if [ ! -e "$PORT" ]; then
    echo -e "${YELLOW}[WARN] Serial port not found: $PORT${RESET}"
    echo "       Is the ESP32-P4 board connected?"
    echo "       Override port:  bash launch-arena-view.sh /dev/ttyUSB0"
    echo ""
fi

echo "[INFO] Port: $PORT @ $BAUD baud"
echo "[INFO] Starting arena view ..."
echo ""
sleep 1

python3 "$SCRIPT" "$PORT" "$BAUD"

echo ""
read -rp "Arena view closed. Press Enter to exit..."
