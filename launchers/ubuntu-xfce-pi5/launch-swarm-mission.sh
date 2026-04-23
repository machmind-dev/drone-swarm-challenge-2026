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
echo -e "                   [Swarm Mission — L-Loop Flight Path]"
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

# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "[INFO] ROS 2 sourced: $ROS_DISTRO"
echo ""
echo -e "${YELLOW}[INFO] Drones: 1 and 2 (running in parallel)${RESET}"
echo "[INFO] Mission: L-loop — fwd 1m → left 90° → fwd 2m → climb 3m → left 90° → fwd 1m → left 90° → fwd 2m → descend 0.5m"
echo "[INFO] ARM both drones and press MISSION in rqt to start the flight path."
echo ""

# Launch both drone missions in parallel; prefix each line so output is readable
python3 "$MISSION_SCRIPT" 1 2>&1 | sed -u 's/^/[D1] /' &
PID1=$!
python3 "$MISSION_SCRIPT" 2 2>&1 | sed -u 's/^/[D2] /' &
PID2=$!

wait $PID1
wait $PID2

echo ""
read -rp "Mission finished. Press Enter to close..."
