"""
Copyright (c) 2025-2026, Mekion
SPDX-License-Identifier: Apache-2.0

Bimo Robotics Kit example API usage with RL ONXX model
"""

from time import sleep, perf_counter
import onnxruntime as onx
import numpy as np
import cv2

from bimo import Bimo


def main():
    # Initalize ONNX session
    ses = onx.InferenceSession(
        "policy.onnx",
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )

    # Initialize Bimo and stand up
    bimo = Bimo()
    bimo.initialize(calibrate=False)
    bimo.perform("stand")
    bimo.lock_heading()  # Required for heading correction

    # Saves two images in current directory as example
    cv2.imwrite("front.png", bimo.capture_image("front"))
    cv2.imwrite("top.png", bimo.capture_image("top"))

    # Initialize last actions buffer
    last_actions = list(bimo.stand_pose)

    # Main control loop (50ms)
    period = 0.05
    t_next = perf_counter()

    while True:
        # Get state data
        state = bimo.request_state_data()

        # Calculate observations array
        obs = process_observations(bimo, state["orient"], last_actions)

        # Perform NN inference
        inputs = {ses.get_inputs()[0].name: obs.reshape(1, -1)}
        model_actions = ses.run(None, inputs)

        # Process new actions
        new_act = process_actions(bimo, model_actions, last_actions)
        last_actions = new_act

        # Execute actions
        bimo.send_positions(new_act)

        # Timestep sync
        t_next += period
        sleep_dt = max(0, t_next - perf_counter())
        sleep(sleep_dt)


def process_actions(bimo, model_actions, last_actions):
    """Converts NN action to degrees"""
    actions = np.clip(model_actions[0].reshape(-1), -3.0, 3.0)
    actions = np.array(last_actions) + (actions * 4 / 3)
    actions = actions.tolist()

    bimo.clip_actions(actions)

    return actions


def process_observations(bimo, orient, actions_deg):
    """Scales values and builds observation vector"""
    scaled_orient = bimo.scale_value(orient, -1, 1)
    scaled_act = bimo.scale_value(actions_deg, bimo.servo_min, bimo.servo_max)

    obs = scaled_orient + scaled_act

    # Round values to reduce noise
    obs = [round(val, 4) for val in obs]

    return np.clip(np.array(obs, dtype=np.float32), -1.0, 1.0)


if __name__ == '__main__':
    main()
