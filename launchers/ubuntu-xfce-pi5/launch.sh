#!/bin/bash

clear
export NO_AT_BRIDGE=1

if ! command -v xfce4-terminal >/dev/null 2>&1; then
    echo "[ERROR] xfce4-terminal is not installed."
    echo "Install it with: sudo apt update && sudo apt install xfce4-terminal"
    exit 1
fi

print_logo() {
    local TEAL=$'\033[38;2;51;117;110m'
    local WHITE=$'\033[38;2;220;220;220m'
    local RESET=$'\033[0m'

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
sleep 1

RVIZ_CONFIG="$HOME/drone-swarm-challenge-2026/ground-station-software/gcs/rviz/scene_map_objects.rviz"
TMPDIR="$(mktemp -d /tmp/machmind_gui.XXXXXX)"

cleanup() {
    rm -rf "$TMPDIR"
}
trap cleanup EXIT

cat > "$TMPDIR/common_logo.sh" <<'EOF'
print_logo() {
    local TEAL=$'\033[38;2;51;117;110m'
    local WHITE=$'\033[38;2;220;220;220m'
    local RESET=$'\033[0m'
    clear
    echo -e "${TEAL}"
    cat << "EOLOGO"
   ███╗   ███╗ █████╗  ██████╗██╗  ██╗    ███╗   ███╗██╗███╗   ██╗██████╗
   ████╗ ████║██╔══██╗██╔════╝██║  ██║    ████╗ ████║██║████╗  ██║██╔══██╗
   ██╔████╔██║███████║██║     ███████║    ██╔████╔██║██║██╔██╗ ██║██║  ██║
   ██║╚██╔╝██║██╔══██║██║     ██╔══██║    ██║╚██╔╝██║██║██║╚██╗██║██║  ██║
   ██║ ╚═╝ ██║██║  ██║╚██████╗██║  ██║    ██║ ╚═╝ ██║██║██║ ╚████║██████╔╝
   ╚═╝     ╚═╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝    ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝
   http://machmind.dev                               Team Mach Mind (c) 2026
EOLOGO
    echo -e "${WHITE}"
    echo -e "${1}"
    echo -e "${RESET}"
    echo ""
}
EOF

for _ID in 1 2 3 4 5; do
    _PORT=$((8880 + _ID))
    cat > "$TMPDIR/tab_agent_d${_ID}.sh" << AGENTEOF
#!/bin/bash
source /tmp/\$(basename "\$(dirname "\$0")")/common_logo.sh
print_logo "           [micro-ROS Agent — Drone ${_ID} :${_PORT}]"
source /opt/ros/jazzy/setup.bash
[ -f ~/uros_ws/install/local_setup.bash ] && source ~/uros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
export NO_AT_BRIDGE=1
echo "[INFO] Starting micro-ROS Agent — Drone ${_ID} port ${_PORT}..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port ${_PORT}
exec bash
AGENTEOF
done

cat > "$TMPDIR/tab_topics.sh" <<'EOF'
#!/bin/bash
source /tmp/$(basename "$(dirname "$0")")/common_logo.sh
print_logo "                        [ROS2 Topics / Publisher]"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export NO_AT_BRIDGE=1
echo "[INFO] Monitoring topics..."
watch -n 1 ros2 topic list -t
exec bash
EOF

cat > "$TMPDIR/tab_rqt.sh" <<'EOF'
#!/bin/bash
source /tmp/$(basename "$(dirname "$0")")/common_logo.sh
print_logo "                        [RQT Control Panel]"
source /opt/ros/jazzy/setup.bash
[ -f ~/drone-swarm-challenge-2026/install/local_setup.bash ] && source ~/drone-swarm-challenge-2026/install/local_setup.bash
export ROS_DOMAIN_ID=0
export NO_AT_BRIDGE=1
export QT_QPA_PLATFORMTHEME=qt5ct
echo "[INFO] Waiting for ROS graph..."
sleep 3
echo "[INFO] Launching rqt..."
rqt --force-discover
exec bash
EOF

chmod +x "$TMPDIR"/tab_*.sh

echo "[INFO] Launching tabbed terminal ..."
sleep 1

xfce4-terminal \
  --title="Mach Mind GCS" \
  --tab --title="Agent D1 :8881" --command="bash '$TMPDIR/tab_agent_d1.sh'" \
  --tab --title="Agent D2 :8882" --command="bash '$TMPDIR/tab_agent_d2.sh'" \
  --tab --title="Agent D3 :8883" --command="bash '$TMPDIR/tab_agent_d3.sh'" \
  --tab --title="Agent D4 :8884" --command="bash '$TMPDIR/tab_agent_d4.sh'" \
  --tab --title="Agent D5 :8885" --command="bash '$TMPDIR/tab_agent_d5.sh'" \
  --tab --title="ROS2 Topics / Publisher" --command="bash '$TMPDIR/tab_topics.sh'" \
  --tab --title="RQT Control Panel" --command="bash '$TMPDIR/tab_rqt.sh'" &

echo "[INFO] Launching RViz2 ..."
sleep 1

bash -lc "
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export NO_AT_BRIDGE=1
export QT_QPA_PLATFORMTHEME=qt5ct
rviz2 -d '$RVIZ_CONFIG'
" &

wait