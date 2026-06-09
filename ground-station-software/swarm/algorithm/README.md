# Swarm Algorithm — LLM Control (DEMO)

This directory contains the Ollama-based natural-language swarm controller (`ollama_commander.py`). It was developed as an experiment for high-level tasking — converting natural-language commands into arena coordinates published over ROS2 using the Gemma3 1B model running locally via Ollama.

**This was NOT used in the finals.** All live flight at the finals was run by the deterministic SDC26 Commander (`../sdc26_commander.py`).

## Why it was not used

**Reliability:** The LLM output is non-deterministic. During testing, command interpretation was inconsistent — the model occasionally produced incorrect coordinates or malformed responses, which is unacceptable in a time-critical competition environment.

**Computing power:** The controller runs on a Raspberry Pi 5 CPU. The Pi5 does not have sufficient CPU performance to run Gemma3 inference at a useful speed alongside the full ROS2 GCS stack (micro-ROS agent, RViz2, rqt). Response latency was too high for real-time swarm control — during testing, a single command took around 2 seconds to process with the 1B parameter model.

**Path planning:** PX4 runs in Offboard mode receiving position setpoints computed by the deterministic GCS algorithm — not by the LLM. The LLM was only ever intended for high-level tasking, not low-level trajectory generation.

<img src="../../../launchers/ollama_swarm_control_terminal.jpg" width="868">

The functionality is fully implemented and can be demonstrated independently. To run it, Ollama must be running separately (`ollama serve`) with the `gemma3:1b` model pulled.
