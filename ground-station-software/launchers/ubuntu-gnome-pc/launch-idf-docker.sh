#!/bin/bash

clear

TEAL='\033[38;2;51;117;110m'
WHITE='\033[38;2;220;220;220m'
RESET='\033[0m'

echo -e "${TEAL}"
cat << "EOF" 
   ███╗   ███╗ █████╗  ██████╗██╗  ██╗    ███╗   ███╗██╗███╗   ██╗██████╗
   ████╗ ████║██╔══██╗██╔════╝██║  ██║    ████╗ ████║██║████╗  ██║██╔══██╗
   ██╔████╔██║███████║██║     ███████║    ██╔████╔██║██║██╔██╗ ██║██║  ██║
   ██║╚██╔╝██║██╔══██║██║     ██╔══██║    ██║╚██╔╝██║██║██║╚██╗██║██║  ██║
   ██║ ╚═╝ ██║██║  ██║╚██████╗██║  ██║    ██║ ╚═╝ ██║██║██║ ╚████║██████╔╝
   ╚═╝     ╚═╝╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝    ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝╚═════╝
   http://machmind.dev		      	           Team Mach Mind (c) 2026
EOF

echo -e "${WHITE}"
echo -e "		   [Node Development Environment]"
echo -e "${RESET}"

echo ""
echo "[INFO] Initializing micro-ROS environment ..."
sleep 2

cd "$HOME/esp32s3-microros" 

echo "[INFO] Building docker container ..."
sleep 2

docker compose -f docker/docker-compose.yml up -d --build

echo "[INFO] Start docker, then exec into it ..."
sleep 2

docker compose -f docker/docker-compose.yml exec -it docker-esp32s3_camera bash

echo "[INFO] Enter running container ..."
sleep 2

docker compose -f docker/docker-compose.yml exec -it esp32s3_camera \
bash -c 'sudo chown -R "$(whoami)":"$(whoami)" /code && cd /code && source /opt/esp/idf/export.sh && exec bash'

sudo chown -R "$(whoami)":"$(whoami)" /code
cd /code
source /opt/esp/idf/export.sh



