#!/bin/bash

clear

print_logo() {
    local TEAL='\033[38;2;51;117;110m'
    local WHITE='\033[38;2;220;220;220m'
    local RESET='\033[0m'

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
    echo -e "                         [ArUco Vision Node]"
    echo -e "${RESET}"
    echo ""
}

print_logo
echo "[INFO] Starting ArUco Vision Node..."
sleep 1

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

VISION_DIR="/home/pihas/drone-swarm-challenge-2026/ground-station-software/vision"

python3 "$VISION_DIR/aruco_node.py"

exec bash
