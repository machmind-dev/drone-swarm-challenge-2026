#!/bin/bash

MISSION_SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/swarm/mission_forward_back.py"

if [ -z "$MACHMIND_SWARM_TERMINAL" ]; then
    export MACHMIND_SWARM_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --title="Mach Mind - Manual Flight" -- bash "$0" "$@"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind - Manual Flight" --hold --command="env MACHMIND_SWARM_TERMINAL=1 bash '$0' $*"
    fi
fi

clear
export NO_AT_BRIDGE=1

TEAL=$'\033[38;2;51;117;110m'
WHITE=$'\033[38;2;220;220;220m'
YELLOW=$'\033[38;2;255;200;60m'
CYAN=$'\033[38;2;0;220;200m'
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
echo -e "                   [Manual Flight — Keyboard Control]"
echo -e "${RESET}"
echo ""

ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$MISSION_SCRIPT" ]; then
    echo "[ERROR] Mission script not found: $MISSION_SCRIPT"
    read -rp "Press Enter to close..."
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS 2 not found at $ROS_SETUP"
    read -rp "Press Enter to close..."
    exit 1
fi

# ── Drone selection menu ──────────────────────────────────────────────────────
echo -e "${CYAN}  Select drone to control:${RESET}"
echo ""
echo "  1)  Drone 1"
echo "  2)  Drone 2"
echo "  3)  Drone 3"
echo "  4)  Drone 4"
echo "  5)  Drone 5"
echo ""
read -rp "  Choice [1-5]: " CHOICE

case "$CHOICE" in
    1|2|3|4|5) DRONE_ID="$CHOICE" ;;
    *)
        echo -e "${YELLOW}[WARN] Invalid choice — defaulting to Drone 1${RESET}"
        DRONE_ID=1
        ;;
esac

echo ""

# ── Team selection menu ───────────────────────────────────────────────────────
echo -e "${CYAN}  Select team / starting side:${RESET}"
echo ""
echo "  1)  Red  (LH — facing +X)"
echo "  2)  Blue (RH — facing -X)"
echo ""
read -rp "  Choice [1/2]: " TEAM_CHOICE

case "$TEAM_CHOICE" in
    2) TEAM="blue" ;;
    *) TEAM="red"  ;;
esac

echo ""

# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "[INFO] ROS 2 sourced: $ROS_DISTRO"
echo "[INFO] Drone: $DRONE_ID  Team: $TEAM"
echo "[INFO] ARM the drone and press MISSION in rqt to start keyboard control."
echo ""

python3 "$MISSION_SCRIPT" "$DRONE_ID" fly "$TEAM"

echo ""
read -rp "Manual flight ended. Press Enter to close..."
