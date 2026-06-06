#!/bin/bash

SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/swarm/sdc26_commander.py"

if [ -z "$MACHMIND_CMD_TERMINAL" ]; then
    export MACHMIND_CMD_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="GCS - SDC26 Commander" --hold --command="env MACHMIND_CMD_TERMINAL=1 bash '$0' $*"
    elif command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --title="GCS - SDC26 Commander" -- bash "$0" "$@"
    fi
fi

clear
export NO_AT_BRIDGE=1

TEAL=$'\033[38;2;51;117;110m'
WHITE=$'\033[38;2;220;220;220m'
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
echo -e "                     [SDC26 Commander]"
echo -e "${RESET}"
echo ""
echo -e "${CYAN}  Swarm orchestrator — boxes → executors, leader home-check${RESET}"
echo ""
echo "  Roles:  D1,D3 Seeker   D2,D4 Executor   D5 Leader"
echo "  Team:   from RQT (/gcs/system/team_color)"
echo "  Args:   --boxes-timeout SECONDS  --start-altitude METRES  --dry-run"
echo ""

ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$SCRIPT" ]; then
    echo "[ERROR] Script not found: $SCRIPT"
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

python3 "$SCRIPT" "$@"

echo ""
read -rp "SDC26 Commander closed. Press Enter to exit..."
