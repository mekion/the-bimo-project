"""
Copyright (c) 2026, Mekion
SPDX-License-Identifier: Apache-2.0

Bimo Central Pattern Generator (CPG) - Fourier-series gait approximation
with per-joint amplitude gain correction.
"""

import math

K = 6

# coeffs[j] = [a0, a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6]
COEFFS = {
    0: [-30.698467, -1.145966, -0.197057, -0.006928, -0.073806, 0.022461, -0.055625, 0.004423, 0.00642, -0.003602, -0.11218, -0.005505, -0.008125],
    1: [-30.698467, 1.145966, 0.197057, -0.006928, -0.073806, -0.022461, 0.055625, 0.004423, 0.00642, 0.003602, 0.11218, -0.005505, -0.008125],
    2: [3.804532, 0.070275, 0.269269, -0.298154, -0.0102, -0.113301, 0.318696, -0.051545, 0.129605, -0.003596, 0.135895, -0.015997, 0.074676],
    3: [-2.195468, -0.070275, -0.269269, -0.298154, -0.0102, 0.113301, -0.318696, -0.051545, 0.129605, 0.003596, -0.135895, -0.015997, 0.074676],
    4: [60.710089, -1.7391, 1.153817, -0.13511, -0.102376, 0.01105, -0.236465, -0.029411, 0.00445, -0.046513, -0.130492, -0.007796, 0.000703],
    5: [60.710089, 1.7391, -1.153817, -0.13511, -0.102376, -0.01105, 0.236465, -0.029411, 0.00445, 0.046513, 0.130492, -0.007796, 0.000703],
    6: [26.439194, -1.967726, -0.057765, 0.050809, 0.136169, 0.026425, 0.076218, 0.024923, -0.059639, -0.015267, 0.156544, 0.027614, 0.011717],
    7: [26.439194, 1.967726, 0.057765, 0.050809, 0.136169, -0.026425, -0.076218, 0.024923, -0.059639, 0.015267, -0.156544, 0.027614, 0.011717],
}

# Amplitude-correction gains
AMP_GAIN = {0: 1.7, 1: 1.7, 2: 1.0, 3: 1.0, 4: 2, 5: 2, 6: 1.5, 7: 1.5}


class BimoCPG:
    def __init__(self, step_freq_hz=1.5, phase_offset=0.0, amp_gain=None):
        """
        step_freq_hz: cycles per second of the gait (1 / step_period).
        phase_offset: global phase shift (rad).
        amp_gain: (optional) dict {joint_idx: gain} overriding AMP_GAIN,
                  e.g. to push knees further if they still undershoot:
                  amp_gain={4: 2.5, 5: 2.5}
        """
        self.omega = 2 * math.pi * step_freq_hz
        self.phase_offset = phase_offset
        self.phi = phase_offset % (2 * math.pi)
        self.amp_gain = dict(AMP_GAIN)

        if amp_gain:
            self.amp_gain.update(amp_gain)

    def step(self, dt):
        """Advance the internal phase clock by dt seconds."""

        self.phi = (self.phi + self.omega * dt) % (2 * math.pi)

    def joint_angle(self, j):
        """Calculates next position for joint (j)"""

        c = COEFFS[j]
        a0 = c[0]
        gain = self.amp_gain[j]
        val = 0.0

        for n in range(1, K + 1):
            an, bn = c[2 * n - 1], c[2 * n]
            val += an * math.cos(n * self.phi) + bn * math.sin(n * self.phi)

        return a0 + gain * val

    def angles(self):
        """Returns next joint positions"""
        return [self.joint_angle(j) for j in range(8)]
