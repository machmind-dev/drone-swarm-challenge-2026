#!/bin/bash
# ============================================================
#  Mach Mind GCS — ArUco Vision Launcher
#  Writes a temp runner script then opens it in a terminal.
#  This avoids all shell quoting / multi-line issues.
# ============================================================

VISION_DIR="$HOME/drone-swarm-challenge-2026/ground-station-software/vision"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
NODE_SCRIPT="$VISION_DIR/aruco_node.py"
PARAMS_FILE="$VISION_DIR/aruco_params.yaml"
LOG_FILE="/tmp/mach_mind_gcs.log"
RUNNER="/tmp/mach_mind_runner.sh"

# ── Write runner script to a temp file ────────────────────────
cat > "$RUNNER" << RUNNER_EOF
#!/bin/bash
exec > >(tee -a "$LOG_FILE") 2>&1
echo "=== Mach Mind Vision GCS ==="
echo "Date: \$(date)"
echo "User: \$(whoami)"
echo ""

if [ ! -f "$ROS_SETUP" ]; then
    echo "ERROR: ROS 2 not found at $ROS_SETUP"
    read -p "Press Enter to close..."
    exit 1
fi
source "$ROS_SETUP"
echo "ROS 2 sourced: \$ROS_DISTRO"

if [ ! -f "$NODE_SCRIPT" ]; then
    echo "ERROR: aruco_node.py not found at $NODE_SCRIPT"
    read -p "Press Enter to close..."
    exit 1
fi

if [ ! -f "$PARAMS_FILE" ]; then
    echo "ERROR: aruco_params.yaml not found at $PARAMS_FILE"
    read -p "Press Enter to close..."
    exit 1
fi

cd "$VISION_DIR"
echo "Working dir: \$(pwd)"
echo "Starting node..."
echo ""

python3 "$NODE_SCRIPT" --ros-args --params-file "$PARAMS_FILE"

EXIT_CODE=\$?
echo ""
echo "Node exited with code: \$EXIT_CODE"
echo "Log saved to: $LOG_FILE"
read -p "Press Enter to close..."
RUNNER_EOF

chmod +x "$RUNNER"

# ── Pick terminal and launch ───────────────────────────────────
if command -v xfce4-terminal &>/dev/null; then
    xfce4-terminal --title="Mach Mind Vision GCS" --hold -e "$RUNNER"

elif command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="Mach Mind Vision GCS" -- "$RUNNER"

elif command -v lxterminal &>/dev/null; then
    lxterminal --title="Mach Mind Vision GCS" -e "$RUNNER"

elif command -v xterm &>/dev/null; then
    xterm -title "Mach Mind Vision GCS" -e "$RUNNER"

else
    echo "ERROR: No terminal emulator found"
    exit 1
fi