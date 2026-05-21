# Copyright (c) 2025-2026, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Bimo Robotics Kit example API usage"""

from time import sleep, time
import onnxruntime as onx
import numpy as np
import cv2

from bimo import Bimo


def main():
    # Initialize ONNX session
    ses = onx.InferenceSession(
        "policy.onnx",
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )

    # Initialize Bimo and stand up
    bimo = Bimo()
    bimo.initialize()

    bimo.perform("stand")
    bimo.lock_heading()  # Required for walking model

    # Saves two images in current directory as example
    cv2.imwrite("front.png", bimo.capture_image("front"))
    cv2.imwrite("top.png", bimo.capture_image("top"))

    # Initialize buffers
    start_scaled = bimo.scale_value(bimo.stand_pose, bimo.servo_min, bimo.servo_max)

    # History kept for experimentation, not needed for current walking model
    action_history = [start_scaled for _ in range(4)]  # Bimo starts inference standing
    orient_history = [[0.0, 0.0, 0.0] for _ in range(4)]
    last_actions = list(bimo.stand_pose)

    # Main control loop 50ms
    period = 0.05
    t1 = time()

    while True:
        # Get state data
        state = bimo.request_state_data()

        # Update buffers
        update_buffers(bimo, state["orient"], last_actions, orient_history, action_history)

        # Calculate observations array
        observations = process_observations(bimo, orient_history, action_history)

        # Perform NN inference
        inputs = {ses.get_inputs()[0].name: observations.reshape(1, -1)}
        model_actions = ses.run(None, inputs)

        # Process new actions
        new_actions = process_actions(bimo, model_actions, last_actions)
        last_actions = new_actions

        # Execute actions
        bimo.send_positions(new_actions)

        # Timestep sync
        t1 += period
        sleep_dt = max(0, t1 - time())
        sleep(sleep_dt)


def process_actions(bimo, model_actions, last_actions):
    """Converts NN actions to degrees"""
    actions = np.clip(model_actions[0].reshape(-1), -3.0, 3.0)
    actions = np.array(last_actions) + (actions * 4 / 3)
    actions = actions.tolist()

    bimo.clip_actions(actions)

    return actions


def update_buffers(bimo, state_orient, last_actions, orient_hist, act_hist):
    """Updates orientation and action history buffers with latest state."""

    # Scale new readings
    orient = bimo.scale_value(state_orient, -1, 1)
    scaled_act = bimo.scale_value(last_actions, bimo.servo_min, bimo.servo_max)

    # Store into buffers
    orient_hist.pop(0)
    orient_hist.append(orient)

    act_hist.pop(0)
    act_hist.append(scaled_act)


def process_observations(bimo, orient_history, action_history):
    """Builds flat observation array from orientation and action histories."""

    observations = orient_history[-1] + action_history[-1]

    # Round values to reduce noise
    observations = [round(val, 4) for val in observations]

    return np.clip(np.array(observations, dtype=np.float32), -1.0, 1.0)


if __name__ == '__main__':
    main()
