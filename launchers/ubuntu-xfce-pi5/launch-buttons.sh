#!/bin/bash

BUTTONS_SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/buttons/btn_test.py"

if [ -z "$MACHMIND_BUTTONS_TERMINAL" ]; then
    export MACHMIND_BUTTONS_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind - Buttons" --hold --command="env MACHMIND_BUTTONS_TERMINAL=1 bash '$0'"
    fi
fi

clear
export NO_AT_BRIDGE=1

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
echo -e "                   [Ground Station Physical Buttons]"
echo -e "${RESET}"
echo ""

ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$BUTTONS_SCRIPT" ]; then
    echo "[ERROR] Button script not found: $BUTTONS_SCRIPT"
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "[ERROR] ROS 2 not found at $ROS_SETUP"
    read -p "Press Enter to close..."
    exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "[INFO] ROS 2 sourced: $ROS_DISTRO"

echo "[INFO] Starting GPIO button monitor..."
echo ""
python3 "$BUTTONS_SCRIPT"
