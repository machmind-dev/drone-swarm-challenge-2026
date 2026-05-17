#!/bin/bash
# ============================================================
#  Mach Mind GCS — micro-ROS Agent Launcher
#  One dedicated agent per drone in one terminal window:
#    Drone 1 → port 8881
#    Drone 2 → port 8882
#    Drone 3 → port 8883
#    Drone 4 → port 8884
#    Drone 5 → port 8885
#
#  Usage:  ./start_agents.sh [1|2|3|4|5]
#    No argument → all 5 agents, one tab each
#    With ID     → that drone's agent in the foreground (debug)
# ============================================================

ROS_SETUP="/opt/ros/jazzy/setup.bash"
DRONE_COUNT=5
BASE_PORT=8880
RUNNER_DIR="/tmp/mach_mind_agents"

# ── Single-drone foreground mode (debug) ──────────────────────
if [[ -n "$1" ]]; then
    ID="$1"
    PORT=$((BASE_PORT + ID))
    echo "=== Mach Mind micro-ROS Agent — Drone $ID (port $PORT) ==="
    source "$ROS_SETUP"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$PORT"
fi

# ── Write one runner script per drone ────────────────────────
mkdir -p "$RUNNER_DIR"

for ID in $(seq 1 $DRONE_COUNT); do
    PORT=$((BASE_PORT + ID))
    RUNNER="$RUNNER_DIR/agent_d${ID}.sh"
    cat > "$RUNNER" << RUNNER_EOF
#!/bin/bash
echo "=== Mach Mind micro-ROS Agent — Drone ${ID} (port ${PORT}) ==="
echo "Date: \$(date)"
echo ""
source "${ROS_SETUP}"
echo "ROS 2: \$ROS_DISTRO"
echo ""
ros2 run micro_ros_agent micro_ros_agent udp4 --port ${PORT}
echo ""
echo "Agent D${ID} exited (port ${PORT})."
read -p "Press Enter to close..."
RUNNER_EOF
    chmod +x "$RUNNER"
done

# ── Launch all tabs in one terminal window ────────────────────
if command -v xfce4-terminal &>/dev/null; then
    CMD=(xfce4-terminal)
    for ID in $(seq 1 $DRONE_COUNT); do
        PORT=$((BASE_PORT + ID))
        CMD+=(--tab --title="Agent D${ID} :${PORT}" --command="$RUNNER_DIR/agent_d${ID}.sh")
    done
    "${CMD[@]}" &

elif command -v gnome-terminal &>/dev/null; then
    CMD=(gnome-terminal)
    for ID in $(seq 1 $DRONE_COUNT); do
        PORT=$((BASE_PORT + ID))
        CMD+=(--tab --title="Agent D${ID} :${PORT}" -- "$RUNNER_DIR/agent_d${ID}.sh")
    done
    "${CMD[@]}" &

elif command -v lxterminal &>/dev/null; then
    for ID in $(seq 1 $DRONE_COUNT); do
        PORT=$((BASE_PORT + ID))
        lxterminal --title="Agent D${ID} :${PORT}" -e "$RUNNER_DIR/agent_d${ID}.sh" &
        sleep 0.3
    done

else
    for ID in $(seq 1 $DRONE_COUNT); do
        PORT=$((BASE_PORT + ID))
        xterm -title "Agent D${ID} :${PORT}" -e "$RUNNER_DIR/agent_d${ID}.sh" &
        sleep 0.3
    done
fi

echo "All $DRONE_COUNT agents launched (ports ${BASE_PORT}1–${BASE_PORT}${DRONE_COUNT})."
