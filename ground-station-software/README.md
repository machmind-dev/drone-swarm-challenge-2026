# Ground Control Station Software

Launch scripts are located in [launchers/](../launchers/).

| GCS Desktop Screenshot | SDC26 Commander |
|---|---|
| <img src="../launchers/Screenshot_2026-06-09_16-37-10.png" width="400"> | <img src="../launchers/Screenshot_2026-06-06_21-34-13(1).png" width="400"> |

| Shortcut | Description | Source |
|----------|-------------|--------|
| Waypoint Commander | Sends arena coordinates to individual drones or all drones via Manhattan navigation. Format: `<drone_id\|all> <x> <y> [z] [yaw_deg]` | [swarm/](swarm/) |
| Keyboard Drone Control | Real-time manual keyboard control of a single drone. Selects drone and team/starting side interactively. | [swarm/](swarm/) |
| Ollama Control of Swarm | Natural-language swarm control via Gemma3 1B LLM running locally in Ollama. Converts text commands to arena coordinates over ROS2. | [swarm/algorithm/](swarm/algorithm/) |
