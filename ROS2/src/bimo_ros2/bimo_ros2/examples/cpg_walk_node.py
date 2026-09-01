"""
Copyright (c) 2026, Mekion
SPDX-License-Identifier: Apache-2.0

CPG-based walking example node for the Bimo ROS2 comms node
"""

import math

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from bimo_ros2_msgs.msg import ActionCommand, StateData
from bimo_ros2_msgs.srv import PerformRoutine
from bimo_ros2.cpg import BimoCPG


CENTER_VALUE = 0.9  # Calibration value for straight walking
MAX_TURN = 2.0  # Maximum turn rate for stable walking


class CpgWalkNode(Node):
    def __init__(self):
        super().__init__('cpg_walk_node')
        self.declare_parameter('step_freq_hz', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('ramp_cycles', 0.8)

        self.nominal_freq = self.get_parameter('step_freq_hz').value
        rate = self.get_parameter('control_rate_hz').value
        self.ramp_cycles = self.get_parameter('ramp_cycles').value
        self.dt = 1.0 / rate

        self._startup_sequence()

        self.cpg = BimoCPG(step_freq_hz=self.nominal_freq)
        self.t = 0.0
        self.latest_state = None

        self.action_pub = self.create_publisher(ActionCommand, 'bimo/cmd_action', 10)
        self.create_subscription(StateData, 'bimo/state', self.on_state, 10)
        self.create_timer(self.dt, self.on_timer)

    def _startup_sequence(self):
        """Stand up and lock heading before the CPG loop starts"""
        routine_client = self.create_client(PerformRoutine, 'bimo/perform_routine')
        heading_client = self.create_client(Trigger, 'bimo/lock_heading')

        routine_client.wait_for_service()
        req = PerformRoutine.Request()
        req.name = 'stand'
        fut = routine_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        if not fut.result().success:
            raise RuntimeError(f"stand routine failed: {fut.result().message}")

        heading_client.wait_for_service()
        fut = heading_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut)

        if not fut.result().success:
            raise RuntimeError(f"lock_heading failed: {fut.result().message}")

        self.get_logger().info('Standing and heading locked, starting CPG walk.')

    def on_state(self, msg):
        self.latest_state = msg

    def on_timer(self):
        # Ramp-up: avoids feet dragging on startup
        cycles_elapsed = self.t * self.nominal_freq
        freq_scale = min(1.0, cycles_elapsed / self.ramp_cycles)
        self.cpg.omega = 2 * math.pi * self.nominal_freq * freq_scale

        new_actions = self.cpg.angles()
        self.cpg.step(self.dt)
        self.t += self.dt

        # Heading-based turn correction using the last known yaw deviation
        z_dev = self.latest_state.orient[2] if self.latest_state else 0.0
        ctn = max(min((-z_dev / 0.4) * MAX_TURN, MAX_TURN), -MAX_TURN)
        new_actions[0] += CENTER_VALUE + ctn  # R Hip
        new_actions[1] -= CENTER_VALUE + ctn  # L Hip

        out = ActionCommand()
        out.positions = new_actions
        self.action_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CpgWalkNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
