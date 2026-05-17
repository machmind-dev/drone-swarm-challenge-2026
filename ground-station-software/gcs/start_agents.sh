#!/bin/bash
# ============================================================
#  Mach Mind GCS — micro-ROS Agent Launcher
#  Starts one dedicated agent per drone on its own port:
#    Drone 1 → port 8881
#    Drone 2 → port 8882
#    Drone 3 → port 8883
#    Drone 4 → port 8884
#    Drone 5 → port 8885
#
#  Usage:  ./start_agents.sh [1|2|3|4|5]
#    No argument → launch all 5 agents (one terminal tab each)
#    With ID     → launch only that drone's agent (foreground)
# ============================================================

ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONE_COUNT=5
BASE_PORT=8880

# ── Single drone (foreground, useful for debugging) ───────────
if [[ -n "$1" ]]; then
    ID="$1"
    PORT=$((BASE_PORT + ID))
    echo "=== Mach Mind micro-ROS Agent — Drone $ID (port $PORT) ==="
    source "$ROS_SETUP"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$PORT"
fi

# ── All drones — one terminal tab per agent ───────────────────
if ! command -v xfce4-terminal &>/dev/null && \
   ! command -v gnome-terminal &>/dev/null && \
   ! command -v lxterminal &>/dev/null && \
   ! command -v xterm &>/dev/null; then
    echo "ERROR: No terminal emulator found"
    exit 1
fi

SCRIPT="$(realpath "$0")"

for ID in $(seq 1 $DRONE_COUNT); do
    PORT=$((BASE_PORT + ID))
    TITLE="Agent D${ID} :${PORT}"
    CMD="bash -c 'source $ROS_SETUP && ros2 run micro_ros_agent micro_ros_agent udp4 --port $PORT; echo Agent exited; read -p \"Press Enter to close...\"'"

    if command -v xfce4-terminal &>/dev/null; then
        xfce4-terminal --tab --title="$TITLE" -e "bash -c '$SCRIPT $ID; read -p \"Press Enter...\"'" &
    elif command -v gnome-terminal &>/dev/null; then
        gnome-terminal --tab --title="$TITLE" -- bash -c "$SCRIPT $ID; read -p 'Press Enter...'" &
    elif command -v lxterminal &>/dev/null; then
        lxterminal --title="$TITLE" -e "bash -c '$SCRIPT $ID; read -p \"Press Enter...\"'" &
    else
        xterm -title "$TITLE" -e "bash -c '$SCRIPT $ID; read -p \"Press Enter...\"'" &
    fi

    sleep 0.3   # stagger launches so tabs open in order
done

echo "All $DRONE_COUNT agents launched (ports ${BASE_PORT}1–${BASE_PORT}${DRONE_COUNT})."
