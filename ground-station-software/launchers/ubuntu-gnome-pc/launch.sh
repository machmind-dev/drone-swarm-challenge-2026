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

echo "[INFO] Launch Terminal 4 ..."
sleep 2

gnome-terminal --title="Scene Launcher" -- bash -lc '
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
    echo -e "                           [Scene Launcher]"
    echo -e "${RESET}"
    echo ""
}

print_logo
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

echo "[INFO] Waiting for RViz to start..."
sleep 5

echo "[INFO] Publishing base ground plane..."
ros2 topic pub --once /visualization_marker visualization_msgs/msg/Marker "
header:
  frame_id: '\''map'\''
ns: '\''arena'\''
id: 100
type: 1
action: 0
pose:
  position: {x: 10.0, y: 5.0, z: -0.02}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
scale: {x: 20.0, y: 10.0, z: 0.02}
color: {r: 0.22, g: 0.22, b: 0.22, a: 1.0}
"

sleep 1

echo "[INFO] Publishing arena decoration..."
ros2 topic pub --once /visualization_marker_array visualization_msgs/msg/MarkerArray "
markers:

# --- Team zone ---
- header: {frame_id: 'map'}
  ns: 'arena'
  id: 101
  type: 1
  action: 0
  pose:
    position: {x: 3.3333, y: 5.0, z: -0.005}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 6.6667, y: 10.0, z: 0.01}
  color: {r: 0.2, g: 0.4, b: 0.8, a: 0.30}

# --- No-man'\''s-land ---
- header: {frame_id: 'map'}
  ns: 'arena'
  id: 102
  type: 1
  action: 0
  pose:
    position: {x: 10.0, y: 5.0, z: -0.004}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 6.6667, y: 10.0, z: 0.01}
  color: {r: 0.5, g: 0.5, b: 0.5, a: 0.25}

# --- Opponent zone ---
- header: {frame_id: 'map'}
  ns: 'arena'
  id: 103
  type: 1
  action: 0
  pose:
    position: {x: 16.6667, y: 5.0, z: -0.003}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 6.6667, y: 10.0, z: 0.01}
  color: {r: 0.8, g: 0.3, b: 0.3, a: 0.30}

# --- Team label ---
- header: {frame_id: 'map'}
  ns: 'arena_labels'
  id: 201
  type: 9
  action: 0
  pose:
    position: {x: 3.3333, y: 5.0, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.0, y: 0.0, z: 0.6}
  color: {r: 0.68, g: 0.68, b: 0.68, a: 1.0}
  text: '\''TEAM-ZONE'\''

# --- No-man'\''s-land label ---
- header: {frame_id: 'map'}
  ns: 'arena_labels'
  id: 202
  type: 9
  action: 0
  pose:
    position: {x: 10.0, y: 5.0, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.0, y: 0.0, z: 0.6}
  color: {r: 0.68, g: 0.68, b: 0.68, a: 1.0}
  text: '\''NO-MAN'\'''\''S-LAND'\''

# --- Opponent label ---
- header: {frame_id: 'map'}
  ns: 'arena_labels'
  id: 203
  type: 9
  action: 0
  pose:
    position: {x: 16.6667, y: 5.0, z: 0.3}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.0, y: 0.0, z: 0.6}
  color: {r: 0.68, g: 0.68, b: 0.68, a: 1.0}
  text: '\''OPPONENT-ZONE'\''
"

sleep 1

echo "[INFO] Publishing monuments..."
ros2 topic pub --once /visualization_marker_array visualization_msgs/msg/MarkerArray "
markers:

# --- Row y = 0 (NOW rotation 180°) ---
# monument 12_11
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 1
  type: 10
  action: 0
  pose:
    position: {x: 4.0, y: 0.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 10_9
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 2
  type: 10
  action: 0
  pose:
    position: {x: 8.0, y: 0.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 8_7
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 3
  type: 10
  action: 0
  pose:
    position: {x: 12.0, y: 0.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 6_5
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 4
  type: 10
  action: 0
  pose:
    position: {x: 16.0, y: 0.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# --- Row y = 10 (NOW rotation 0°) ---
# monument 18_17
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 5
  type: 10
  action: 0
  pose:
    position: {x: 4.0, y: 10.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 20_19
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 6
  type: 10
  action: 0
  pose:
    position: {x: 8.0, y: 10.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 22_21
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 7
  type: 10
  action: 0
  pose:
    position: {x: 12.0, y: 10.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 24_23
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 8
  type: 10
  action: 0
  pose:
    position: {x: 16.0, y: 10.0, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# --- Left side (rotation 90° unchanged) ---
# monument 16_15
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 9
  type: 10
  action: 0
  pose:
    position: {x: 0.0, y: 6.66, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 14_13
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 10
  type: 10
  action: 0
  pose:
    position: {x: 0.0, y: 3.33, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# --- Right side (rotation 270° unchanged) ---
# monument 2_1
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 11
  type: 10
  action: 0
  pose:
    position: {x: 20.0, y: 6.66, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: -0.70710678, w: 0.70710678}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true

# monument 4_3
- header: {frame_id: 'map'}
  ns: 'monuments'
  id: 12
  type: 10
  action: 0
  pose:
    position: {x: 20.0, y: 3.33, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: -0.70710678, w: 0.70710678}
  scale: {x: 0.001, y: 0.001, z: 0.001}
  color: {r: 1.0, g: 1.0, b: 1.0, a: 1.0}
  mesh_resource: '\''file:///home/pihas/drone-swarm-challenge-2026/docs/media/software/marker_2_1.dae'\''
  mesh_use_embedded_materials: true
"

echo "[INFO] Publishing monument labels..."

ros2 topic pub --once /visualization_marker_array visualization_msgs/msg/MarkerArray "
markers:

########################################
# ===== MONUMENT LABELS ONLY ==========
########################################

# --- Bottom row ---

# 12 / 11
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 101
  type: 9
  action: 0
  pose: {position: {x: 4.0, y: -0.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '12'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 102
  type: 9
  action: 0
  pose: {position: {x: 4.0, y: -0.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '11'

# 10 / 9
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 103
  type: 9
  pose: {position: {x: 8.0, y: -0.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '10'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 104
  type: 9
  pose: {position: {x: 8.0, y: -0.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '9'

# 8 / 7
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 105
  type: 9
  pose: {position: {x: 12.0, y: -0.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '8'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 106
  type: 9
  pose: {position: {x: 12.0, y: -0.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '7'

# 6 / 5
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 107
  type: 9
  pose: {position: {x: 16.0, y: -0.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '6'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 108
  type: 9
  pose: {position: {x: 16.0, y: -0.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '5'


# --- Top row ---

# 18 / 17
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 109
  type: 9
  pose: {position: {x: 4.0, y: 10.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '18'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 110
  type: 9
  pose: {position: {x: 4.0, y: 10.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '17'

# 20 / 19
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 111
  type: 9
  pose: {position: {x: 8.0, y: 10.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '20'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 112
  type: 9
  pose: {position: {x: 8.0, y: 10.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '19'

# 22 / 21
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 113
  type: 9
  pose: {position: {x: 12.0, y: 10.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '22'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 114
  type: 9
  pose: {position: {x: 12.0, y: 10.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '21'

# 24 / 23
- header: {frame_id: 'map'}
  ns: 'labels_top'
  id: 115
  type: 9
  pose: {position: {x: 16.0, y: 10.9, z: 4.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '24'

- header: {frame_id: 'map'}
  ns: 'labels_bottom'
  id: 116
  type: 9
  pose: {position: {x: 16.0, y: 10.9, z: 2.0}, orientation: {w: 1.0}}
  scale: {z: 0.4}
  color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}
  text: '23'


# --- Left side ---

- {header: {frame_id: 'map'}, ns: 'labels_top', id: 117, type: 9,
   pose: {position: {x: -0.9, y: 6.66, z: 4.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '16'}

- {header: {frame_id: 'map'}, ns: 'labels_bottom', id: 118, type: 9,
   pose: {position: {x: -0.9, y: 6.66, z: 2.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '15'}

- {header: {frame_id: 'map'}, ns: 'labels_top', id: 119, type: 9,
   pose: {position: {x: -0.9, y: 3.33, z: 4.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '14'}

- {header: {frame_id: 'map'}, ns: 'labels_bottom', id: 120, type: 9,
   pose: {position: {x: -0.9, y: 3.33, z: 2.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '13'}


# --- Right side ---

- {header: {frame_id: 'map'}, ns: 'labels_top', id: 121, type: 9,
   pose: {position: {x: 20.9, y: 6.66, z: 4.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '2'}

- {header: {frame_id: 'map'}, ns: 'labels_bottom', id: 122, type: 9,
   pose: {position: {x: 20.9, y: 6.66, z: 2.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '1'}

- {header: {frame_id: 'map'}, ns: 'labels_top', id: 123, type: 9,
   pose: {position: {x: 20.9, y: 3.33, z: 4.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '4'}

- {header: {frame_id: 'map'}, ns: 'labels_bottom', id: 124, type: 9,
   pose: {position: {x: 20.9, y: 3.33, z: 2.0}, orientation: {w: 1.0}},
   scale: {z: 0.4}, color: {r: 0.95, g: 0.95, b: 0.95, a: 1.0}, text: '3'}

"


echo "[INFO] Scene published."
exec bash
' &


echo "[INFO] Launch Terminal 5 (RQT Control Panel) ..."
sleep 2

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
    echo -e "                        [RQT Control Panel]"
    echo -e "${RESET}"
    echo ""
}

print_logo

source /opt/ros/jazzy/setup.bash
source ~/drone-swarm-challenge-2026/ground-station-software/gcs/install/local_setup.bash
export ROS_DOMAIN_ID=0

echo "[INFO] Waiting for ROS graph..."
sleep 3

echo "[INFO] Launching rqt..."
rqt

exec bash
' &

wait
