"""
Copyright (c) 2026, Mekion
SPDX-License-Identifier: Apache-2.0

Bimo Robotics Kit example API usage with CPG
"""

from time import sleep, perf_counter
from bimo import Bimo, BimoCPG
import math


CENTER_VALUE = 0.9  # Calibration value for straight walking
MAX_TURN = 2.0  # Maximum turn rate for stable walking


def main():
    # Initialize Bimo and stand up
    bimo = Bimo()
    bimo.initialize(calibrate=False)
    bimo.perform("stand")
    bimo.lock_heading()  # Required for heading correction
    sleep(2)  # Allows robot to settle after standing

    # Main control loop parameters
    period = 0.05
    nominal_freq = 1.5  # Normal CPG frequency
    ramp_cycles = 0.8  # Prevents feet dragging when starting to walk
    t_next = perf_counter()  # Main loop time keeping
    t = 0.0  # CPG time keeping

    # Initialize CPG
    cpg = BimoCPG(step_freq_hz=nominal_freq)

    # Main control loop (50ms)
    while True:
        # Get Z deviation value
        z_dev = bimo.request_state_data()["orient"][2]

        # Get new actions (with ramp-up on startup)
        cycles_elapsed = t * nominal_freq
        freq_scale = min(1.0, cycles_elapsed / ramp_cycles)
        cpg.omega = 2 * math.pi * nominal_freq * freq_scale

        new_actions = cpg.angles()
        cpg.step(period)
        t += period

        # Apply turn correction based on Z deviation, max trun and center value
        ctn = max(min((-z_dev / 0.4) * MAX_TURN, MAX_TURN), - MAX_TURN)
        new_actions[0] += CENTER_VALUE + ctn  # R Hip
        new_actions[1] -= CENTER_VALUE + ctn  # L Hip

        bimo.send_positions(new_actions)

        # Timestep sync
        t_next += period
        sleep_dt = max(0, t_next - perf_counter())
        sleep(sleep_dt)


if __name__ == '__main__':
    main()
