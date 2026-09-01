"""
Copyright (c) 2026, Mekion
Copyright (c) 2022-2026, The Isaac Lab Project Developers.
SPDX-License-Identifier: Apache-2.0

Bimo Robotics Kit custom actuator class for STS3215 12V 30Kg·cm servos.
"""

import torch
from isaaclab.actuators.actuator_base import ActuatorBase
from isaaclab.actuators.actuator_cfg import ActuatorBaseCfg
from isaaclab.utils import configclass
import math


class STS3215Actuator(ActuatorBase):
    cfg: "STS3215Cfg"

    def __init__(self, cfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        n = self._num_envs
        nj = self.num_joints
        d = self._device

        default_pos = torch.tensor(cfg.init_pose_deg, device=d, dtype=torch.float32)
        default_pos = default_pos.unsqueeze(0).expand(n, -1)
        default_pos = torch.deg2rad(default_pos)

        # Recompute physical constants
        self._servo_tracked = default_pos.clone()
        self.backlash_rad = torch.zeros(n, nj, device=d)

        # Contact-point state for directional backlash (gear-engagement position)
        self._contact_pos = default_pos.clone()

        alpha_val = 1.0 - math.exp(-2.0 * math.pi * cfg.bandwidth_hz * cfg.sim_dt)
        self._alpha = torch.full((n, nj), alpha_val, device=d)

        # Write fixed stiffness/damping into base class tensors (shape: num_envs, num_joints)
        self.stiffness[:] = cfg.stiffness
        self.damping[:] = cfg.damping

        # Per-env, per-joint mutable params, all (num_envs, num_joints)
        self._friction = torch.full((n, nj), cfg.friction_nm, device=d)
        self._acc_limit = torch.zeros(n, nj, device=d)

        # Internal state buffers
        self._torque_prev = torch.zeros(n, nj, device=d)

        # Delay ring buffer, shape: (max_delay+1, num_envs, num_joints)
        max_delay = cfg.bus_delay_steps + 2
        self._cmd_buffer = default_pos.clone().unsqueeze(0).expand(max_delay + 1, -1, -1).clone()
        self._delay_offset = torch.full((n,), cfg.bus_delay_steps, dtype=torch.long, device=d)
        self._buf_idx = 0

        # Track last delayed command to detect new action arrivals
        self._last_delayed_cmd = default_pos.clone()

    def reset(self, env_ids):
        """Reset actuator internal states for given env indices."""
        default_pos = torch.tensor(self.cfg.init_pose_deg, device=self._device, dtype=torch.float32)
        default_pos = torch.deg2rad(default_pos)

        self._servo_tracked[env_ids] = default_pos
        self._cmd_buffer[:, env_ids, :] = default_pos
        self._torque_prev[env_ids] = 0.0

        # Sample backlash before placing the contact point
        low_rad = math.radians(self.cfg.backlash_range_deg[0])
        high_rad = math.radians(self.cfg.backlash_range_deg[1])
        self.backlash_rad[env_ids] = torch.empty(
            len(env_ids), self.num_joints, device=self._device
        ).uniform_(low_rad, high_rad)

        # Randomize where the gear happens to be resting within the band at power-on
        half_bl = self.backlash_rad[env_ids] / 2.0
        offset = torch.empty_like(half_bl).uniform_(-1.0, 1.0) * half_bl
        self._contact_pos[env_ids] = default_pos + offset

        self._last_delayed_cmd[env_ids] = default_pos

    def compute(self, control_action, joint_pos, joint_vel):
        # Bus delay
        self._cmd_buffer[self._buf_idx] = control_action.joint_positions
        env_idx = torch.arange(joint_pos.shape[0], device=joint_pos.device)
        read_idx = (self._buf_idx - self._delay_offset) % self._cmd_buffer.shape[0]
        delayed_cmd = self._cmd_buffer[read_idx, env_idx, :]
        self._buf_idx = (self._buf_idx + 1) % self._cmd_buffer.shape[0]

        # First-order lag
        self._servo_tracked = (
            self._servo_tracked + self._alpha * (delayed_cmd - self._servo_tracked)
        )

        self._last_delayed_cmd = delayed_cmd.clone()

        # Directional backlash via classic backlash operator.
        # The "contact point" (gear-engaged position) only moves once the
        # driving side (_servo_tracked) travels outside the current backlash
        # band around it. This means: as long as motion continues in the same
        # direction, the gears stay meshed and there is zero extra lag (contact
        # point tracks the input 1:1, offset by half the gap). Lag/deadband is
        # only introduced right after a direction reversal, when the gears must
        # first cross the backlash gap before re-engaging, exactly matching
        # real gear backlash behavior.
        half_bl = self.backlash_rad / 2.0
        lower_bound = self._servo_tracked - half_bl
        upper_bound = self._servo_tracked + half_bl
        new_contact_pos = torch.clamp(self._contact_pos, lower_bound, upper_bound)

        # Re-engagement happens where the contact point had to move to
        # stay within the band, i.e. exactly at a direction reversal. Since
        # measured backlash varies with joint loading/pressure on the real
        # robot, resample the gap width for those joints at the moment of
        # re-engagement, to represent a random new pressure condition.
        reengaged = (new_contact_pos != self._contact_pos)

        if reengaged.any():
            low_rad = math.radians(self.cfg.backlash_range_deg[0])
            high_rad = math.radians(self.cfg.backlash_range_deg[1])
            new_bl = torch.empty_like(self.backlash_rad).uniform_(low_rad, high_rad)
            self.backlash_rad = torch.where(reengaged, new_bl, self.backlash_rad)

        self._contact_pos = new_contact_pos

        # Backlash deadband
        raw_err = self._contact_pos - joint_pos
        effective_err = raw_err

        # PD torque
        tau = self.stiffness * effective_err - self.damping * joint_vel

        # Friction
        tau = tau - self._friction * torch.sign(joint_vel)

        # Effort & velocity limit
        tau_sat = self.cfg.stall_torque_nm
        v_max = self.cfg.max_velocity

        tau_max_v = torch.clamp(tau_sat * (1 - joint_vel / v_max), 0.0, tau_sat)
        tau_min_v = torch.clamp(tau_sat * (-1 - joint_vel / v_max), -tau_sat, 0.0)

        tau = torch.clamp(tau, tau_min_v, tau_max_v)

        self.computed_effort[:] = tau
        self.applied_effort[:] = tau
        control_action.joint_efforts = tau
        control_action.joint_positions = None
        control_action.joint_velocities = None

        return control_action


@configclass
class STS3215Cfg(ActuatorBaseCfg):
    class_type: type = STS3215Actuator

    # Mechanical params
    stall_torque_nm: float = 2.943  # Nm | 30 kg·cm @ 12V, gearbox variant 1:345
    max_velocity: float = 4.712  # Rad/s | @ 12V = 45 RPM
    stiffness: float = 57.386  # Nm/rad | k = I_refl * wn^2, wn = 2*pi*f_bw (~3.113 Hz sys-ID) | Measured with servo firmware P=30
    damping: float = 1.339  # Nm*s/rad | c = 2*zeta*I_refl*wn, zeta≈0.228 (fit from step-response decay) | Measured with servo firmware D=35

    friction_nm: float = 0.044  # Nm | m·g·l_CoM·sin(theta_hold), l_CoM=0.19m | Measured on hip actuator using leg properties
    armature: float = 0.15  # Kg*m^2 | I_rotor(1.26e-6) x N^2(345^2)
    bandwidth_hz: float = 3.113  # Based on sys-ID

    bus_delay_steps: int = 1  # Physics Steps | Data Request + Inference + Action Exec = ~5ms = 1 step
    backlash_range_deg: tuple = (1.0, 2.4)  # Degrees | Measured on real robot

    init_pose_deg: list = [0, 0, 0, 0, 0, 0, 0, 0] # Degrees

    # Sim physics step (must match env)
    sim_dt: float = 0.005  # Seconds
