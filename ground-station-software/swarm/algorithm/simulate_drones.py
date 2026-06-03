#!/usr/bin/env python3
"""Simulate 5 drones online — publishes /drone_N/state at 1 Hz."""

import os
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

NUM_DRONES = 5
PUBLISH_HZ  = 1.0
ACTIVE_DRONES = [2]


class DroneSimulator(Node):

    def __init__(self):
        super().__init__('drone_simulator')
        self._state_pubs   = {}
        self._battery_pubs = {}
        self._role_pubs    = {}

        for i in ACTIVE_DRONES:
            self._state_pubs[i]   = self.create_publisher(String, f'/drone_{i}/state',   10)
            self._battery_pubs[i] = self.create_publisher(Int8,   f'/drone_{i}/battery', 10)
            self._role_pubs[i]    = self.create_publisher(String, f'/drone_{i}/role',    10)

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def _publish(self):
        for i in ACTIVE_DRONES:
            state_msg = String(); state_msg.data = 'mission'
            self._state_pubs[i].publish(state_msg)

            bat_msg = Int8(); bat_msg.data = 100
            self._battery_pubs[i].publish(bat_msg)

            role_msg = String(); role_msg.data = 'seeker'
            self._role_pubs[i].publish(role_msg)


def main():
    rclpy.init()
    node = DroneSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
