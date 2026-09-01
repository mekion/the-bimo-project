"""
Copyright (c) 2026, Mekion
SPDX-License-Identifier: Apache-2.0

RL model-based walking example node for the Bimo ROS2 comms node
"""

import os

import cv2
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import onnxruntime as onx
import numpy as np

from bimo_ros2_msgs.msg import ActionCommand, StateData
from bimo_ros2_msgs.srv import PerformRoutine, CaptureImage
from bimo_ros2.bimo_core import Bimo


class NnWalkNode(Node):
    def __init__(self):
        super().__init__('nn_walk_node')
        self.declare_parameter('model_path', 'policy.onnx')
        self.declare_parameter('image_output_dir', '/tmp/bimo_captures')
        model_path = self.get_parameter('model_path').value

        self.bimo = Bimo()
        self._bridge = CvBridge()

        self.session = onx.InferenceSession(
            model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])

        self._startup_sequence()
        self._save_debug_images()

        self.last_actions = list(self.bimo.stand_pose)
        self.action_pub = self.create_publisher(ActionCommand, 'bimo/cmd_action', 10)
        self.create_subscription(StateData, 'bimo/state', self.on_state, 10)

    def _startup_sequence(self):
        """Stand up and lock heading before inference starts"""
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

        self.get_logger().info('Stood up and locked heading, starting NN inference.')

    def _save_debug_images(self):
        """Saves front/top snapshots at startup as example usage"""
        out_dir = self.get_parameter('image_output_dir').value
        os.makedirs(out_dir, exist_ok=True)

        capture_client = self.create_client(CaptureImage, 'bimo/capture_image')
        capture_client.wait_for_service()

        for camera in ('front', 'top'):
            req = CaptureImage.Request()
            req.camera = camera
            fut = capture_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            result = fut.result()

            if not result.success:
                self.get_logger().warn(f'Could not capture {camera}: {result.message}')
                continue

            frame = self._bridge.imgmsg_to_cv2(result.image, desired_encoding='bgr8')
            path = os.path.join(out_dir, f'{camera}.png')
            cv2.imwrite(path, frame)
            self.get_logger().info(f'Saved {path}')

    def on_state(self, msg):
        obs = self._process_observations(list(msg.orient), self.last_actions)

        inputs = {self.session.get_inputs()[0].name: obs.reshape(1, -1)}
        model_actions = self.session.run(None, inputs)

        new_actions = self._process_actions(model_actions, self.last_actions)
        self.last_actions = new_actions

        out = ActionCommand()
        out.positions = new_actions
        self.action_pub.publish(out)

    def _process_actions(self, model_actions, last_actions):
        actions = np.clip(model_actions[0].reshape(-1), -3.0, 3.0)
        actions = np.array(last_actions) + (actions * 4 / 3)
        actions = actions.tolist()

        self.bimo.clip_actions(actions)

        return actions

    def _process_observations(self, orient, actions_deg):
        scaled_orient = self.bimo.scale_value(orient, -1, 1)
        scaled_act = self.bimo.scale_value(actions_deg, self.bimo.servo_min, self.bimo.servo_max)

        obs = scaled_orient + scaled_act
        obs = [round(val, 4) for val in obs]

        return np.clip(np.array(obs, dtype=np.float32), -1.0, 1.0)


def main(args=None):
    rclpy.init(args=args)
    node = NnWalkNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
