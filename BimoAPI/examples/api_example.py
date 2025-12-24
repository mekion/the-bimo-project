# Copyright (c) 2025, Mekion
# SPDX-License-Identifier: Apache-2.0
"""Bimo Robotics Kit example API usage"""

from time import sleep, time
import onnxruntime as onx
import numpy as np

from bimo import Bimo, BimoRoutines


def main():
    # Initalize ONNX session
    ses = onx.InferenceSession(
        "policy.onnx",
        providers=["CUDAExecutionProvider", "CPUExecutionProvider"],
    )

    # Initialize Bimo and stand up
    bimo = Bimo()
    bimo_r = BimoRoutines()

    bimo.initialize()
    bimo_r.perform(bimo, "stand")

    # Initialize buffers
    orient_history = [[0.0, 0.0] for _ in range(4)]
    gyro_history = [[0.0, 0.0, 0.0] for _ in range(4)]

    start_scaled = bimo.scale_value(bimo.stand_pose, bimo.servo_min, bimo.servo_max)
    action_history = [start_scaled for _ in range(4)]
    last_actions = list(bimo.stand_pose)  # Asumes bimo starts inference standing

    new_actions = [0.0 for _ in range(8)]

    # Main control loop 50ms
    period = 0.05
    t1 = time()

    while True:
        # Get state data
        imu, _, _ = bimo.request_state_data()

        # Update buffers
        update_buffers(bimo, imu, last_actions, orient_history, gyro_history, action_history)

        # Calculate obervations array
        observations = process_observations(bimo, orient_history, gyro_history, action_history)

        # Perform NN inference
        inputs = {ses.get_inputs()[0].name: observations.reshape(1, -1)}
        model_actions = ses.run(None, inputs)

        # Process new actions
        new_actions = process_actions(bimo, model_actions, last_actions)
        last_actions = new_actions

        # Execute
        bimo.send_positions(new_actions)

        # Timestep sync
        t1 += period
        sleep_dt = max(0, t1 - time())
        sleep(sleep_dt)


def process_actions(bimo, model_actions, last_actions):
    """Converts NN action to degrees"""
    actions = np.clip(model_actions[0].reshape(-1), -3.0, 3.0)
    actions = np.array(last_actions) + (actions * 1.6 / 3)  # Limit actions for stability
    actions = actions.tolist()

    bimo.clip_actions(actions)

    return actions


def update_buffers(bimo, imu, last_actions, orient_hist, gyro_hist, act_hist):
    # Scale new readings
    euler = bimo.quaternion_to_euler(imu[:4])
    orient = bimo.scale_value(euler[:2], -1, 1)  # X, Y orient only
    gyro = bimo.scale_value(imu[4:7], -2, 2)
    scaled_act = bimo.scale_value(last_actions, bimo.servo_min, bimo.servo_max)

    # Store into buffers
    orient_hist.pop(0)
    orient_hist.append(orient)

    gyro_hist.pop(0)
    gyro_hist.append(gyro)

    act_hist.pop(0)
    act_hist.append(scaled_act)


def process_observations(bimo, orient_history, gyro_history, action_history):
    observations = []

    # Interleave: orient and gyro for each frame
    for orient, gyro in zip(orient_history, gyro_history):
        observations.extend(orient)  # Add orient (X, Y)
        observations.extend(gyro)    # Add gyro (X, Y, Z)

    # Add action history
    for actions in action_history:
        observations.extend(actions)

    # Round values to reduce noise
    observations = [round(val, 4) for val in observations]

    return np.array(observations, dtype=np.float32)


if __name__ == '__main__':
    main()
