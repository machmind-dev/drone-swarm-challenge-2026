#!/usr/bin/env python3
"""
ollama_commander.py — Natural-language swarm control via local Ollama LLM.

Usage:
    python3 ollama_commander.py [--model MODEL] [--ollama URL]

Defaults:
    model  : gemma3:4b
    ollama : http://localhost:11434

Type a plain-English command at the prompt.  The LLM translates it to JSON,
which is validated and forwarded to each drone via ROS 2 topics.

Examples:
    "Send drone 1 to position 10, 5"
    "Move drone 3 to x=15 y=7 at 2 metres"
    "Return drone 2 home"
    "Send drones 1 and 4 to x=5 y=3"

Arena limits:  x 0-20 m,  y 0-10 m,  z 0-5 m
Default alt:   1.2 m

ROS 2 topics (per drone):
    Publish:   /gcs/drone_{ID}/control   geometry_msgs/PoseStamped
               /gcs/drone_{ID}/command   std_msgs/String
    Subscribe: /drone_{ID}/state         std_msgs/String
"""

import argparse
import json
import math
import sys
import threading
import time

import requests
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

# ── Constants ─────────────────────────────────────────────────────────────────
NUM_DRONES     = 5
DEFAULT_ALT_M  = 1.2
ARENA_X_MAX    = 20.0
ARENA_Y_MAX    = 10.0
ARENA_Z_MAX    = 5.0
ARENA_Z_MIN    = 0.1

OLLAMA_TIMEOUT = 30   # seconds

SYSTEM_PROMPT = (
    "You control a drone swarm. Arena: x=0-20m, y=0-10m, z=0-5m. "
    "5 drones (1-5). Default altitude 1.2m.\n"
    "Output JSON only.\n"
    'Move:  {"commands": [{"drone_id": 1, "x": 10.0, "y": 5.0, "z": 1.2}]}\n'
    'Home:  {"commands": [{"drone_id": 1, "action": "return_home"}]}\n'
    'Multi: {"commands": [...]}\n'
    'Error: {"error": "reason"}'
)

# ── ROS 2 Node ─────────────────────────────────────────────────────────────────
class OllamaCommanderNode(Node):

    def __init__(self):
        super().__init__('ollama_commander')
        self._states = {i: 'unknown' for i in range(1, NUM_DRONES + 1)}
        self._control_pubs = {}
        self._command_pubs = {}

        for i in range(1, NUM_DRONES + 1):
            self._control_pubs[i] = self.create_publisher(
                PoseStamped, f'/gcs/drone_{i}/control', 10)
            self._command_pubs[i] = self.create_publisher(
                String, f'/gcs/drone_{i}/command', 10)
            self.create_subscription(
                String, f'/drone_{i}/state',
                lambda msg, d=i: self._state_cb(d, msg), 10)

    def _state_cb(self, drone_id: int, msg: String):
        self._states[drone_id] = msg.data

    def states_summary(self) -> str:
        parts = [f"D{i}:{self._states[i]}" for i in range(1, NUM_DRONES + 1)]
        return '  '.join(parts)

    def send_setpoint(self, drone_id: int, x: float, y: float, z: float,
                      yaw_deg: float = 0.0):
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._control_pubs[drone_id].publish(msg)
        self.get_logger().info(
            f'D{drone_id} → setpoint x={x:.2f} y={y:.2f} z={z:.2f}')

    def send_command(self, drone_id: int, cmd: str):
        msg = String()
        msg.data = cmd
        self._command_pubs[drone_id].publish(msg)
        self.get_logger().info(f'D{drone_id} → command "{cmd}"')


# ── Ollama client ──────────────────────────────────────────────────────────────
def query_ollama(prompt: str, model: str, ollama_url: str) -> dict:
    payload = {
        "model": model,
        "prompt": SYSTEM_PROMPT + "\n\nUser: " + prompt,
        "format": "json",
        "stream": False,
        "options": {
            "temperature": 0.1,
            "num_predict": 150,
        },
    }
    resp = requests.post(
        f"{ollama_url}/api/generate",
        json=payload,
        timeout=OLLAMA_TIMEOUT,
    )
    resp.raise_for_status()
    raw = resp.json().get("response", "")
    return json.loads(raw)


# ── Validation ─────────────────────────────────────────────────────────────────
def validate_and_clamp(cmd: dict) -> dict | None:
    """Return cleaned command dict or None on unrecoverable error."""
    drone_id = cmd.get("drone_id")
    if not isinstance(drone_id, int) or drone_id < 1 or drone_id > NUM_DRONES:
        print(f"  [skip] invalid drone_id: {drone_id!r}")
        return None

    action = cmd.get("action")
    if action == "return_home":
        return {"drone_id": drone_id, "action": "return_home"}

    x = cmd.get("x")
    y = cmd.get("y")
    z = cmd.get("z", DEFAULT_ALT_M)

    if x is None or y is None:
        print(f"  [skip] D{drone_id}: missing x or y")
        return None

    try:
        x, y, z = float(x), float(y), float(z)
    except (TypeError, ValueError):
        print(f"  [skip] D{drone_id}: non-numeric coordinates")
        return None

    # Clamp to arena bounds
    x_c = max(0.0, min(ARENA_X_MAX, x))
    y_c = max(0.0, min(ARENA_Y_MAX, y))
    z_c = max(ARENA_Z_MIN, min(ARENA_Z_MAX, z))

    if (x_c, y_c, z_c) != (x, y, z):
        print(f"  [clamp] D{drone_id}: ({x:.2f},{y:.2f},{z:.2f})"
              f" → ({x_c:.2f},{y_c:.2f},{z_c:.2f})")

    return {"drone_id": drone_id, "x": x_c, "y": y_c, "z": z_c}


# ── REPL ───────────────────────────────────────────────────────────────────────
def run_repl(node: OllamaCommanderNode, model: str, ollama_url: str):
    print(f"\nOllama Commander  model={model}  arena=20×10×5 m")
    print("Type a command, or 'quit' to exit.\n")

    while True:
        try:
            print(f"[{node.states_summary()}]")
            user_input = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not user_input:
            continue
        if user_input.lower() in ("quit", "exit", "q"):
            break

        print(f"  Querying {model}...", flush=True)
        try:
            result = query_ollama(user_input, model, ollama_url)
        except requests.exceptions.ConnectionError:
            print(f"  [error] Cannot reach Ollama at {ollama_url}. Is it running?")
            continue
        except requests.exceptions.Timeout:
            print(f"  [error] Ollama timed out after {OLLAMA_TIMEOUT}s.")
            continue
        except json.JSONDecodeError as e:
            print(f"  [error] LLM returned non-JSON: {e}")
            continue
        except Exception as e:
            print(f"  [error] {e}")
            continue

        if "error" in result:
            print(f"  [LLM error] {result['error']}")
            continue

        commands = result.get("commands")
        if not isinstance(commands, list) or not commands:
            print(f"  [error] Unexpected response: {result}")
            continue

        for raw_cmd in commands:
            cleaned = validate_and_clamp(raw_cmd)
            if cleaned is None:
                continue

            d = cleaned["drone_id"]
            if "action" in cleaned:
                node.send_command(d, "COMMAND_RETURN_HOME")
                print(f"  D{d} → return home")
            else:
                node.send_setpoint(d, cleaned["x"], cleaned["y"], cleaned["z"])
                print(f"  D{d} → x={cleaned['x']:.2f} y={cleaned['y']:.2f}"
                      f" z={cleaned['z']:.2f}")

        print()


# ── Entry point ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Ollama LLM swarm commander")
    parser.add_argument("--model",  default="gemma3:4b",
                        help="Ollama model name (default: gemma3:4b)")
    parser.add_argument("--ollama", default="http://localhost:11434",
                        help="Ollama base URL (default: http://localhost:11434)")
    args = parser.parse_args()

    rclpy.init()
    node = OllamaCommanderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Give subscriptions a moment to receive current states
    time.sleep(0.5)

    try:
        run_repl(node, args.model, args.ollama)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
