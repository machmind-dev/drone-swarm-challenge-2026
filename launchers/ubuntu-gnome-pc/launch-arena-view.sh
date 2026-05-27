#!/bin/bash

clear

TEAL='\033[38;2;51;117;110m'
WHITE='\033[38;2;220;220;220m'
YELLOW='\033[38;2;255;200;60m'
CYAN='\033[38;2;0;220;200m'
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
echo -e "              [Arena View — Top-Down Position Map]"
echo -e "${RESET}"
echo ""

SCRIPT="$HOME/drone-swarm-challenge-2026/drone-firmware/v2.0/drone-vision-esp32p4/tools/arena_view.py"

if [ ! -f "$SCRIPT" ]; then
    echo "[ERROR] arena_view.py not found: $SCRIPT"
    read -rp "Press Enter to close..."
    exit 1
fi

# ── Port selection menu ───────────────────────────────────────────────────────
# Both P4 and S3 appear as /dev/ttyACM0 when connected alone.
# When both are plugged in, assignment depends on connection order.

echo -e "${CYAN}  What is connected?${RESET}"
echo ""
echo "  1)  P4 only          — ttyACM0=P4,   S3=none"
echo "  2)  S3 only          — P4=none,       ttyACM0=S3"
echo "  3)  Both  (P4 first) — ttyACM0=P4,   ttyACM1=S3"
echo "  4)  Both  (S3 first) — ttyACM0=S3,   ttyACM1=P4"
echo ""
read -rp "  Choice [1-4]: " CHOICE

case "$CHOICE" in
    1) P4_PORT="/dev/ttyACM0"; S3_PORT="none" ;;
    2) P4_PORT="none";         S3_PORT="/dev/ttyACM0" ;;
    3) P4_PORT="/dev/ttyACM0"; S3_PORT="/dev/ttyACM1" ;;
    4) P4_PORT="/dev/ttyACM1"; S3_PORT="/dev/ttyACM0" ;;
    *)
        echo -e "${YELLOW}[WARN] Invalid choice — defaulting to P4 only on ttyACM0${RESET}"
        P4_PORT="/dev/ttyACM0"; S3_PORT="none"
        ;;
esac

echo ""
echo "[INFO] P4 port: $P4_PORT  (ArUco POSE)"
echo "[INFO] S3 port: $S3_PORT  (LOCAL_NED comparison)"
echo "[INFO] Starting arena view ..."
echo ""
sleep 1

python3 "$SCRIPT" "$P4_PORT" "$S3_PORT"

echo ""
read -rp "Arena view closed. Press Enter to exit..."
