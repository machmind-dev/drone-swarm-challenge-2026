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
    echo -e "                 [Ground Station Graphical Interface]"
    echo -e "${RESET}"
    echo ""
}

print_logo
echo "[INFO] Activate common environment ..."
sleep 2

ROS_SETUP="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0"
UROS_SETUP="source ~/uros_ws/install/local_setup.bash"
RVIZ_CONFIG="/home/pihas/drone-swarm-challenge-2026/ground-station-software/gcs/rviz/scene_map_objects.rviz"

echo "[INFO] Launch Terminal 1 ..."
sleep 2

gnome-terminal --title="micro-ROS Agent" -- bash -lc '
print_logo() {
    local TEAL='\''\033[38;2;51;117;110m'\''
    local WHITE='\''\033[38;2;220;220;220m'\''
    local RESET='\''\033[0m'\''
    clear
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
    echo -e "                            [micro-ROS Agent]"
    echo -e "${RESET}"
    echo ""
}

print_logo
'"$ROS_SETUP"'
'"$UROS_SETUP"'
echo "[INFO] Starting micro-ROS Agent..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
exec bash
' &

echo "[INFO] Launch Terminal 2 ..."
sleep 2

gnome-terminal --title="ROS2 Topics / Publisher" -- bash -lc '
print_logo() {
    local TEAL='\''\033[38;2;51;117;110m'\''
    local WHITE='\''\033[38;2;220;220;220m'\''
    local RESET='\''\033[0m'\''
    clear
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
    echo -e "                        [ROS2 Topics / Publisher]"
    echo -e "${RESET}"
    echo ""
}

print_logo
'"$ROS_SETUP"'

echo "[INFO] Monitoring topics..."
watch -n 1 ros2 topic list -t

exec bash
' &

echo "[INFO] Launch Terminal 3 ..."
sleep 2

gnome-terminal --title="RViz2" -- bash -lc '
print_logo() {
    local TEAL='\''\033[38;2;51;117;110m'\''
    local WHITE='\''\033[38;2;220;220;220m'\''
    local RESET='\''\033[0m'\''
    clear
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
    echo -e "                                [RViz2]"
    echo -e "${RESET}"
    echo ""
}

print_logo
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
echo "[INFO] Launching RViz2..."
rviz2 -d '"$RVIZ_CONFIG"'
exec bash
' &

echo "[INFO] Launch Terminal 4 (RQT Control Panel) ..."
sleep 2

# Scene is no longer auto-published on launch.
# Use LH Scene / RH Scene buttons in the RQT panel to load it manually.

gnome-terminal --title="RQT Control Panel" -- bash -lc '
print_logo() {
    local TEAL='\''\033[38;2;51;117;110m'\''
    local WHITE='\''\033[38;2;220;220;220m'\''
    local RESET='\''\033[0m'\''
    clear
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
    echo -e "                          [RQT Control Panel]"
    echo -e "${RESET}"
    echo ""
}

print_logo
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

echo "[INFO] Waiting for ROS graph..."
sleep 3

echo "[INFO] Launching rqt..."
source ~/drone-swarm-challenge-2026/ground-station-software/gcs/rqt/install/setup.bash
rqt --force-discover

exec bash
' &

wait
