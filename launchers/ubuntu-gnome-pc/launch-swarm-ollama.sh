#!/bin/bash

OLLAMA_SCRIPT="$HOME/drone-swarm-challenge-2026/ground-station-software/swarm/algorithm/ollama_commander.py"

if [ -z "$MACHMIND_SWARM_TERMINAL" ]; then
    export MACHMIND_SWARM_TERMINAL=1
    export NO_AT_BRIDGE=1
    if command -v gnome-terminal >/dev/null 2>&1; then
        exec gnome-terminal --title="Mach Mind - GCS Swarm Ollama" -- bash "$0" "$@"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        exec xfce4-terminal --title="Mach Mind - GCS Swarm Ollama" --hold --command="env MACHMIND_SWARM_TERMINAL=1 bash '$0' $*"
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
echo -e "                   [GCS — Swarm Ollama — Gemma3 4B LLM Control]"
echo -e "${RESET}"
echo ""

ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [ ! -f "$OLLAMA_SCRIPT" ]; then
    echo "[ERROR] Ollama commander not found: $OLLAMA_SCRIPT"
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
echo -e "${YELLOW}[INFO] Ollama must be running: ollama serve${RESET}"
echo "[INFO] Model: gemma3:4b  |  Arena: x=0-20m  y=0-10m  z=0-5m"
echo "[INFO] Type natural-language commands, e.g. 'Send drone 1 to position 10, 5'"
echo ""

python3 "$OLLAMA_SCRIPT" "$@"

echo ""
read -rp "Session ended. Press Enter to close..."
