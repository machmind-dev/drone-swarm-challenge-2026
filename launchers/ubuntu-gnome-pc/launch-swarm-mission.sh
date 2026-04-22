#!/bin/bash

MISSION_SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/swarm/mission_forward_back.py"

if [ -z "$MACHMIND_SWARM_TERMINAL" ]; then
    export MACHMIND_SWARM_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --title="Mach Mind - Swarm Mission" -- bash "$0" "$@"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind - Swarm Mission" --hold --command="env MACHMIND_SWARM_TERMINAL=1 bash '$0' $*"
    fi
fi

clear
export NO_AT_BRIDGE=1

TEAL=$'\033[38;2;51;117;110m'
WHITE=$'\033[38;2;220;220;220m'
YELLOW=$'\033[38;2;255;200;60m'
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
echo -e "                   [Swarm Mission — Forward / Rotate / Back]"
echo -e "${RESET}"
echo ""

ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONE_ID="${1:-1}"

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

# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "[INFO] ROS 2 sourced: $ROS_DISTRO"
echo ""
echo -e "${YELLOW}[INFO] Drone ID: ${DRONE_ID}${RESET}"
echo "[INFO] Mission: fly 1 m forward → rotate 180° → fly back 1 m"
echo "[INFO] ARM the drone and press MISSION in rqt to start the flight path."
echo ""

python3 "$MISSION_SCRIPT" "$DRONE_ID"

echo ""
read -rp "Mission finished. Press Enter to close..."
