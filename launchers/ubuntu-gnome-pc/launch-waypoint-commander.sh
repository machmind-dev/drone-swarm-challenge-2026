#!/bin/bash

SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/swarm/waypoint_commander.py"

if [ -z "$MACHMIND_WP_TERMINAL" ]; then
    export MACHMIND_WP_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --title="Mach Mind - Waypoint Commander" -- bash "$0" "$@"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind - Waypoint Commander" --hold --command="env MACHMIND_WP_TERMINAL=1 bash '$0' $*"
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
echo -e "                     [Waypoint Commander]"
echo -e "${RESET}"
echo ""
echo -e "${CYAN}  Format:  <drone_id|all>  <x>  <y>  [z]  [yaw_deg]${RESET}"
echo -e "${CYAN}  Arena:   x=0-20 m   y=0-10 m   z default=1.0 m${RESET}"
echo ""
echo "  Examples:"
echo "    3 10 5          drone 3 → (10, 5) at 1.5 m"
echo "    3 10 5 2.0      drone 3 → (10, 5) at 2.0 m"
echo "    3 10 5 1.5 90   drone 3 → (10, 5) facing +Y"
echo "    all 10 5        all drones → (10, 5)"
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

python3 "$SCRIPT"

echo ""
read -rp "Waypoint Commander closed. Press Enter to exit..."
