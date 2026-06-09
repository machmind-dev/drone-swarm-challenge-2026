# Ground Control Station Software

Launch scripts are located in [launchers/](../launchers/).

**SDC26 Commander**

<img src="../launchers/Screenshot_2026-06-06_21-34-13(1).png" width="868">

## Software

| Software | Version | Description |
|----------|---------|-------------|
| Ubuntu | 24.04 LTS | Operating system |
| ROS2 | Jazzy | Robot middleware — all node communication |
| RViz2 | Jazzy | 3D visualisation of arena, drone positions and paths |
| rqt + machmind_rqt_buttons | 1.4.0 | Custom GCS control panel — arm, mission, emergency, scene |
| micro-ROS agent | Jazzy (Docker) | Bridge between ESP32-S3 micro-ROS and ROS2 |
| Python | 3.12 | Swarm logic, vision, button handler scripts |
| OpenCV | 4.x | ArUco marker detection in GCS vision node |
| NumPy | — | Matrix and vector math for vision and navigation |
| gpiod | — | Physical GPIO button handler (Raspberry Pi 5) |
| Docker | — | Containerised micro-ROS agent and firmware dev environment |
| Ollama | — | Local LLM runtime for natural-language swarm control |
| Gemma3 | 1B | LLM model used by Ollama swarm commander |

---

| Shortcut | Description | Source |
|----------|-------------|--------|
| Waypoint Commander | Sends arena coordinates to individual drones or all drones via Manhattan navigation. Format: `<drone_id\|all> <x> <y> [z] [yaw_deg]` | [Link](swarm/) |
| Keyboard Drone Control | Real-time manual keyboard control of a single drone. Selects drone and team/starting side interactively. | [Link](swarm/) |
| Ollama Control of Swarm | Natural-language swarm control via Gemma3 1B LLM running locally in Ollama. Converts text commands to arena coordinates over ROS2. | [Link](swarm/algorithm/) |
