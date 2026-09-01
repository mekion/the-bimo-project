"""
Copyright (c) 2025-2026, Mekion
Copyright (c) 2022-2026, The Isaac Lab Project Developers.
SPDX-License-Identifier: Apache-2.0

Bimo Robotics Kit task environment for Isaac Lab.

Walking Behavior RL Model
"""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.sim.utils import bind_physics_material
from isaaclab.utils.noise import GaussianNoiseCfg, gaussian_noise
from isaaclab.sensors import Imu, ImuCfg, ContactSensor, ContactSensorCfg
from isaaclab.sim.spawners import RigidBodyMaterialCfg
from isaaclab.terrains import (
    TerrainImporterCfg,
    TerrainGeneratorCfg,
    TerrainImporter
)
from isaaclab.terrains.height_field import HfRandomUniformTerrainCfg
from .bimo_config import BIMO_CFG
from collections.abc import Sequence
from random import uniform
import math


@configclass
class BimoEnvCfg(DirectRLEnvCfg):
    # Environment settings
    decimation = 10
    episode_length_s = 10
    observation_space = 11  # Overwritten by rsl_rl at runtime
    action_space = 8
    state_space = 0
    dt = 0.005  # Physics step in seconds

    # Reward weights [orientation, height, joint pos, feet height, velocity, deviation]
    weights = [1, 1, 1, 1, 1, 1]

    # Head X CoM shift (adjust based on payload position)
    com_shift = 0.01  # Forward (X) Positive

    # Simulation
    sim: SimulationCfg = SimulationCfg(dt=dt)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        env_spacing=1,
        replicate_physics=True
    )

    # Bimo robot configuration
    bimo_cfg: ArticulationCfg = BIMO_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # Sensors configuration
    imu: ImuCfg = ImuCfg(
        prim_path="/World/envs/env_.*/Robot/Bimo/Head",
        offset=ImuCfg.OffsetCfg(
            pos=(-0.006, 0.0, -0.5175),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
        debug_vis=False,
        update_period=0.012,
    )

    contact: ContactSensorCfg = ContactSensorCfg(
        prim_path="/World/envs/env_.*/Robot/Bimo/Foot.*",
        history_length=0,
        update_period=dt,
        debug_vis=False,
        track_pose=True,
        track_air_time=True,
        force_threshold=0.001,
    )

    # Link mass randomization and pushes
    events = {
        "randomize_link_mass": EventTermCfg(
            func="isaaclab.envs.mdp.events:randomize_rigid_body_mass",
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("bimo"),
                "mass_distribution_params": (0.95, 1.05),  # +-5%
                "operation": "scale",
                "distribution": "uniform",
                "recompute_inertia": True,
            }
        ),
        "periodic_push": EventTermCfg(
            func="isaaclab.envs.mdp.events:push_by_setting_velocity",
            mode="interval",
            params={
                "asset_cfg": SceneEntityCfg("bimo", body_names="Head"),
                # Set to 0.0, which overwrites velocity buffers randomly, causing randomizing effect
                "velocity_range": {"x": (-0.0, 0.0), "y": (-0.0, 0.0), "z": (-0.0, 0.0)},
            },
            interval_range_s=(2.0, 4.0),
        )
    }


class BimoEnv(DirectRLEnv):
    def __init__(self, cfg: BimoEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # R Hip, L Hip, R Shoulder, L Shoulder... Max and Min positions in degrees
        self.servo_max = torch.tensor([90, 90, 90, 12, 140, 140, 93, 93], device=self.device, dtype=torch.int)
        self.servo_min = torch.tensor([-90, -90, -12, -90, 0, 0, -93, -93], device=self.device, dtype=torch.int)

        # Base pose
        start_pos = [-30, -30, 0, 0, 60, 60, 30, 30]
        self.base_pose = torch.tensor([start_pos for _ in range(self.scene.num_envs)], device=self.device, dtype=torch.float32)

        # Commanded actions by NN (degrees)
        self.cmd_actions = self.base_pose.clone()

        # Torque parameter range for randomization
        self.n_torques = 24
        self.torque_range = torch.linspace(
            2.7,   # 2.7 Nm (min voltage ~10V)
            2.94,  # 2.94 Nm (max voltage ~12V)
            self.n_torques,  # 0.01 steps
            device=self.device
        )
        self.torque_idx = torch.zeros(self.scene.num_envs, dtype=torch.long, device=self.device)

        # Noise settings
        self.orient_noise = GaussianNoiseCfg(mean=0.0, std=0.015, operation="add")
        self.actuator_noise = GaussianNoiseCfg(mean=0.0, std=0.5, operation="add")

        # Reward weights
        self.weights = torch.tensor(self.cfg.weights, device=self.device).repeat(self.scene.num_envs, 1)

        # Single config-set flag for COM
        self.com_set = False

        # Bimo last world position log (used in reward calculation)
        self.last_position = self.scene.env_origins.clone()

    def _setup_scene(self):
        # Articulations setup
        self.bimo = Articulation(self.cfg.bimo_cfg)
        self.scene.articulations["bimo"] = self.bimo

        # IMU sensor setup
        self.imu = Imu(self.cfg.imu)
        self.scene.sensors["imu"] = self.imu

        # Contact sensors setup
        self.contact = ContactSensor(self.cfg.contact)
        self.scene.sensors["contact"] = self.contact

        # Randomized ground properties
        num_cols = max(math.ceil(math.sqrt(self.num_envs)), 15)
        num_rows = max(math.ceil(self.num_envs / num_cols), 15)

        self.terrain = TerrainImporter(TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="generator",
            use_terrain_origins=True,
            terrain_generator=TerrainGeneratorCfg(
                seed=42,
                curriculum=False,
                size=(1.25, 1.25),  # Matches 1.25 x env_spacing
                border_width=0.0,
                border_height=0.5,
                num_rows=num_rows,
                num_cols=num_cols,
                horizontal_scale=0.05,
                vertical_scale=0.001,
                slope_threshold=0.75,
                color_scheme="height",
                sub_terrains={
                    "flat": HfRandomUniformTerrainCfg(  # Flat with slight deviations
                        proportion=1.0,
                        noise_range=(-0.001, 0.001),
                        noise_step=0.001,
                        downsampled_scale=0.2,
                    ),
                },
            ),
            # Simulates floor contact properties
            physics_material=RigidBodyMaterialCfg(
                static_friction=0.6,
                dynamic_friction=0.5,
                restitution=0.02,
                friction_combine_mode="min",
            ),
            debug_vis=False,
        ))

        # Clone environments
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])

        # Randomize foot pad material properties: TPU
        for i in range(self.scene.num_envs):
            for foot_name in ["FootLeft", "FootRight"]:
                static = round(uniform(0.3, 0.7) * 10) / 10
                dynamic = static - 0.1
                restitution = round(uniform(0.0, 0.05) * 100) / 100

                mat_cfg = RigidBodyMaterialCfg(
                    static_friction=static,
                    dynamic_friction=dynamic,
                    restitution=restitution,
                    compliant_contact_stiffness=5e4,
                    compliant_contact_damping=8e2,
                    friction_combine_mode="min",
                )

                mat_cfg.func(f"/World/ContactMaterials/env_{i}/{foot_name}_mat", mat_cfg)
                prim_path = f"/World/envs/env_{i}/Robot/Bimo/{foot_name}"
                mat_path = f"/World/ContactMaterials/env_{i}/{foot_name}_mat"

                bind_physics_material(prim_path, mat_path)

        # Light
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

        # Camera
        self.cfg.viewer.eye = (5.0, -5.0, 4.0)
        self.cfg.viewer.lookat = (0.0, 0.0, 0.0)

    def _get_observations(self):
        # Get IMU orientation data, add noise and scale to [-1, 1]
        self.imu_data = self.scene.sensors["imu"].data
        orient = quaternion_to_euler(self.imu_data.quat_w)
        orient = gaussian_noise(orient, self.orient_noise)
        orient = scale_value(orient, -1.0, 1.0)

        # Get last commanded position and scale to [-1, 1]
        cmd_act = torch.clamp((self.cmd_actions - self.servo_min) / (self.servo_max - self.servo_min) * 2 - 1, -1, 1)

        # Create observation buffer
        obs_buffer = torch.cat((orient, cmd_act), dim=1)
        obs_buffer = torch.round(obs_buffer, decimals=4)

        return {"policy": obs_buffer}

    def _pre_physics_step(self, actions):
        # Calculates action delta
        actions_cpy = torch.clamp(actions.clone(), -3.0, 3.0)
        self.cmd_actions = torch.clamp(self.cmd_actions + actions_cpy * 4 / 3, self.servo_min, self.servo_max)

        # Adds noise to actions
        self.noisy_act = torch.clamp(gaussian_noise(self.cmd_actions, self.actuator_noise), self.servo_min, self.servo_max)

    def _apply_action(self):
        # Applies NN action
        self.bimo.set_joint_position_target(torch.deg2rad(self.noisy_act))

    def _get_rewards(self):
        # Get data for reward
        euler_imu_orient = quaternion_to_euler(self.imu_data.quat_w)
        bimo_root_pos = self.bimo.data.root_pos_w
        lin_vel = self.bimo.data.root_com_vel_w
        contact_pos = self.scene.sensors["contact"].data.pos_w
        air_time = self.scene.sensors["contact"].data.current_air_time

        # Compute reward components
        orientation_rew = orientation_reward(euler_imu_orient, self.device)
        height_rew = height_reward(bimo_root_pos)
        position_rew = joint_position_reward(self.cmd_actions, self.base_pose, self.device)
        vel_rew = velocity_reward(lin_vel)
        feet_h_rew = feet_height_reward(air_time, contact_pos, 0.02, 150)
        dev_rew = deviation_reward(self.scene.env_origins, bimo_root_pos)

        # Compute weighted reward
        w = self.weights / torch.sum(self.weights, dim=1, keepdim=True)

        total_reward = (orientation_rew * w[:, 0] + height_rew * w[:, 1]
                        + position_rew * w[:, 2] + feet_h_rew * w[:, 3]
                        + vel_rew * w[:, 4] + dev_rew * w[:, 5])

        # Update buffers
        self.last_position[:] = self.bimo.data.root_pos_w

        return total_reward

    def _get_dones(self):
        # Check for time-out (episode length exceeded)
        truncated = self.episode_length_buf >= self.max_episode_length - 1

        # Check for height termination
        head_heights = self.bimo.data.root_pos_w[:, 2]
        height_termination = head_heights < 0.1

        # Get root orientations and check if robot tilted over
        root_orientations = self.bimo.data.root_quat_w
        euler_angles = quaternion_to_euler(root_orientations)
        x_rotation = torch.abs(euler_angles[:, 0])
        y_rotation = torch.abs(euler_angles[:, 1])
        orientation_termination = (x_rotation > 0.95) | (y_rotation > 0.95)

        # Combine termination conditions
        terminated = height_termination | orientation_termination

        return terminated, truncated

    def _reset_idx(self, env_ids: Sequence[int] | None):
        # Reset specified environments
        if env_ids is None:
            env_ids = self.bimo._ALL_INDICES
        super()._reset_idx(env_ids)

        # Set COM shift once
        if not self.com_set:
            physx_view = self.bimo.root_physx_view
            coms = physx_view.get_coms()
            coms[:, 0, 0] -= self.cfg.com_shift

            physx_view.set_coms(coms, indices=env_ids.cpu())
            self.com_set = True

        # Get default root pose and add env origin position (for spacing)
        root_state = self.bimo.data.default_root_state[env_ids]
        root_state[:, :3] += self.scene.env_origins[env_ids]

        # Get default joint positions and velocities
        joint_pos = self.bimo.data.default_joint_pos[env_ids].clone()
        joint_vel = self.bimo.data.default_joint_vel[env_ids].clone()

        # Choose new torque parameters
        torque_vals = self.torque_range[self.torque_idx[env_ids]]
        torque_vals = torque_vals.unsqueeze(1).expand(-1, 8)

        # Advance index wrapping at n_torques
        self.torque_idx[env_ids] = (self.torque_idx[env_ids] + 1) % self.n_torques

        # Write parameters to sim
        self.bimo.write_joint_effort_limit_to_sim(torque_vals, joint_ids=None, env_ids=env_ids)
        self.bimo.write_root_link_pose_to_sim(root_state[:, :7], env_ids)
        self.bimo.write_root_com_velocity_to_sim(root_state[:, 7:], env_ids)
        self.bimo.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        # Reset buffers
        self.cmd_actions[env_ids] = self.base_pose[env_ids]
        self.last_position[env_ids] = self.scene.env_origins[env_ids]


@torch.jit.script
def quaternion_to_euler(quat: torch.Tensor):
    """Convert quaternion to Euler angles"""

    # Normalize quaternion
    quat = quat / torch.norm(quat, dim=-1, keepdim=True)

    # Extract quaternion components
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = torch.where(
        torch.abs(sinp) >= 1,
        torch.sign(sinp) * (torch.pi / 2),
        torch.asin(sinp)
    )

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)

    return torch.stack([roll, pitch, yaw], dim=1)


@torch.jit.script
def scale_value(value: torch.Tensor, min_val: float, max_val: float):
    """Scale a value to the range [-1, 1]"""

    return torch.clamp((value - min_val) / (max_val - min_val) * 2 - 1, -1, 1)


@torch.jit.script
def orientation_reward(euler_imu_orient, device: str):
    """Orientation reward based on IMU data and task"""

    angle_sums = torch.zeros(euler_imu_orient.shape[0], device=device)

    # Scales Z for softer heading correction
    absolute = torch.abs(euler_imu_orient)
    absolute[:, 2] *= 0.6
    angle_sums = torch.sum(absolute, dim=1)

    # Full reward
    reward = torch.where(
        angle_sums <= 0.95,
        1 - torch.sqrt(angle_sums / 0.95),
        torch.ones_like(angle_sums) * -1
    )

    return reward


@torch.jit.script
def deviation_reward(og_pose, curr_pose):
    """Y linear distance deviation reward"""

    y_dev = torch.abs(og_pose[:, 1] - curr_pose[:, 1])
    y_raw = torch.clamp(1 - y_dev / 0.2, 0.0, 1.0)

    reward = y_raw * 2 - 1

    return reward


@torch.jit.script
def height_reward(bimo_root_pos):
    """Reward based on root height"""

    # Extract the z-coordinate (height) for all environments
    heights = bimo_root_pos[:, 2]

    # Define the ideal height and maximum deviation
    ideal_height = 0.381
    max_deviation = 0.3

    # Calculate the absolute difference between current and ideal height
    height_diff = torch.abs(heights - ideal_height)

    # Clip the difference to the maximum allowed deviation
    clipped_diff = torch.clamp(height_diff, 0, max_deviation)
    height_rew = scale_value(clipped_diff, 0.3, 0.0)

    # Scales to [0, 1]
    height_rew = (height_rew + 1) / 2

    return height_rew


@torch.jit.script
def joint_position_reward(pos_buff, start_pos, device: str):
    """Calculates how far the joint position is from the ideal position (standing)"""

    # Define max differences
    max_diff = torch.tensor([90, 90, 90, 90, 75, 75, 90, 90], device=device)

    # Calculate absolute differences
    diff = torch.abs(pos_buff - start_pos)

    # Scale differences
    diff_scaled = 1 - torch.sqrt(torch.clamp(diff / max_diff.unsqueeze(0), 0, 1))

    # Calculate mean for each environment
    pos_rew = torch.mean(diff_scaled, dim=1)

    # Scale reward from [0, 1] to [-1, 1]
    pos_rew = pos_rew * 2 - 1

    return pos_rew


@torch.jit.script
def velocity_reward(lin_vel_d):
    """Calculates reward based on forward linear velocity"""

    lin_v_x = lin_vel_d[:, 0]
    target_speed = 0.06  # m/s

    sigma = target_speed * 0.5
    reward = torch.clamp(1.0 - ((lin_v_x - target_speed) / sigma) ** 2, min=-1.0, max=1.0)

    return reward


@torch.jit.script
def feet_height_reward(air_time, feet_pos, target_h: float, scale: float = 25.0):
    """Feet clearance reward"""

    in_air = (air_time > 0)  # [num_envs, 2]
    num_in_air = in_air.sum(dim=1)  # [num_envs]

    both_in_air = (num_in_air == 2)
    both_on_ground = (num_in_air == 0)

    # Foot Z positions
    z_pos = feet_pos[..., 2]  # [num_envs, 2]
    z_err = z_pos - target_h

    # Reward is 1.0 if z = threshold, else exponential decay both ways
    reward_per_leg = torch.exp(-scale * z_err ** 2) * in_air.float()
    reward = reward_per_leg.sum(dim=1)  # sum both legs

    # If both feet in air or both on ground, set reward to 0
    reward = torch.where(
        both_in_air | both_on_ground,
        torch.zeros_like(reward),
        reward
    )

    return reward
