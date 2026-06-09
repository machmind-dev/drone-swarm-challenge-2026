# Swarm

**Waypoint Commander** — [waypoint_commander.py](waypoint_commander.py)

Sends arena coordinates to individual drones or all drones via Manhattan navigation. Format: `<drone_id|all> <x> <y> [z] [yaw_deg]`. Arena: x=0–20 m, y=0–10 m.

<img src="../../launchers/waypoint_commander_terminal.jpg" width="868">

**Keyboard Drone Control** — [mission_forward_back.py](mission_forward_back.py)

Real-time manual keyboard control of a single drone. Selects drone and team/starting side interactively before flight.

<img src="../../launchers/keyboard_control_terminal.jpg" width="868">

**Ollama Control of Swarm** — [algorithm/ollama_commander.py](algorithm/ollama_commander.py)

Natural-language swarm control via Gemma3 1B LLM running locally in Ollama. Converts text commands into arena coordinates published over ROS2.

<img src="../../launchers/ollama_swarm_control_terminal.jpg" width="868">
