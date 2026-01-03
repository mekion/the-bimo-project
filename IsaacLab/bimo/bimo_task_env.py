# Copyright (c) 2025-2026, Mekion
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: Apache-2.0

"""Bimo Robotics Kit task environment for Isaac Lab."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils import configclass
from isaaclab.sim.utils import bind_physics_material
from isaaclab.utils.noise import GaussianNoiseCfg, gaussian_noise

from .bimo_config import BIMO_CFG
from isaaclab.sensors import Imu, ImuCfg, ContactSensor, ContactSensorCfg
from isaaclab.sim.spawners import RigidBodyMaterialCfg

from collections.abc import Sequence
from random import uniform


@configclass
class BimoEnvCfg(DirectRLEnvCfg):
    # Environment settings
    decimation = 10
    episode_length_s = 10
    observation_space = 13  # Overwritten by rsl_rl on startup
    action_space = 8
    state_space = 0
    dt = 0.005

    # Training objective: walk | turn (experimental) | stop (experimental)
    obj = "walk"

    # Reward weights
    # [orientation, height, joint pos, joint pos sigmoid, feet height, velocity, deviation]
    weights = {
        "stop": [1, 1, 1, 1, 1, 0, 0],  # Learns to stop and sustain pushes (Experimental)
        "walk": [1, 1, 1, 0, 2, 1, 1],  # Learns to walk
        "turn": [1, 1, 1, 1, 2, 1, 2],  # Learns to turn (Experimental)
    }

    # Head COM shift forward (X axis)
    com_shift = 0.0175

    # Actuator settings
    actuator_delay_max = 4  # Physics steps
    actuator_delay_min = 1  # Physiscs steps
    backlash = 1.6  # Degrees

    # Simulation
    sim: SimulationCfg = SimulationCfg(dt=dt)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(env_spacing=2, replicate_physics=True)

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

    # Randomization events: link mass and push forces
    push_velocities = {
        "stop": {"x": (-0.4, 0.4), "y": (-0.4, 0.4), "z": (-0.4, 0.4)},
        "walk": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.2, 0.2)},
        "turn": {"x": (-0.2, 0.2), "y": (-0.2, 0.2), "z": (-0.2, 0.2)},
    }

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
                "velocity_range": push_velocities[obj],
            },
            interval_range_s=(0.0, 2.0),
        )
    }


class BimoEnv(DirectRLEnv):
    def __init__(self, cfg: BimoEnvCfg, **kwargs):
        super().__init__(cfg, **kwargs)
        # Weights
        self.weights = torch.tensor(self.cfg.weights[self.cfg.obj], device=self.device).repeat(self.scene.num_envs, 1)
        self.obj = self.cfg.obj

        # R Hip, L Hip, R Shoulder, L Shoulder... Max and Min positions in degrees
        self.servo_max = torch.tensor([90, 90, 90, 12, 140, 140, 93, 93], device=self.device, dtype=torch.int)
        self.servo_min = torch.tensor([-90, -90, -12, -90, 0, 0, -93, -93], device=self.device, dtype=torch.int)

        # Buffers for joint positions
        start_pos = [-30, -30, 0, 0, 60, 60, 30, 30]
        self.base_pose = torch.tensor([start_pos for _ in range(self.scene.num_envs)], device=self.device, dtype=torch.float32)

        self.cmd_actions = self.base_pose.clone()  # Commanded by NN (degrees)
        self.last_direction = torch.zeros(self.scene.num_envs, 8, device=self.device)
        self.gear_position = self.base_pose.clone()  # Applied joint targets (degrees)

        # Action direction: turn left (-1) / right (+1)
        half = self.scene.num_envs // 2

        self.act_direction = torch.cat((
            torch.ones(half, device=self.device),
            -torch.ones(self.scene.num_envs - half, device=self.device)
        ), dim=0)

        # Parameter ranges for joints
        self.frictions = torch.tensor([0.1 + x / 1000 for x in range(0, 201)], device=self.device)
        self.torques = torch.tensor([2.7 + x / 1000 for x in range(0, 241)], device=self.device)
        self.dampings = torch.tensor([0.6 + x / 1000 for x in range(0, 101)], device=self.device)

        # Noise settings
        self.orient_noise = GaussianNoiseCfg(mean=0.0, std=0.015, operation="add")
        self.gyro_noise = GaussianNoiseCfg(mean=0.0, std=0.01, operation="add")
        self.actuator_noise = GaussianNoiseCfg(mean=0.0, std=0.5, operation="add")

        # Aactuator delays
        self.act_timer = 0
        self.act_delay = 0

        # COM setting
        self.com_set = False

        # History keeping
        self.orient_h = torch.zeros(self.scene.num_envs, 4, 3, device=self.device)
        self.gyro_h = torch.zeros(self.scene.num_envs, 4, 3, device=self.device)

        self.act_hist = torch.zeros(self.scene.num_envs, 4, 8, device=self.device)
        self.act_hist[:, :] = torch.clamp((self.base_pose[0] - self.servo_min) / (self.servo_max - self.servo_min) * 2 - 1, -1, 1)

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

        # Clone environments
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=[])

        # Randomize foot pad material properties: TPU
        for i in range(self.scene.num_envs):
            for foot_name in ["FootLeft", "FootRight"]:
                static = round(uniform(0.5, 0.8) * 10) / 10
                dynamic = static - 0.2
                restitution = round(uniform(0.05, 0.15) * 100) / 100

                mat_cfg = RigidBodyMaterialCfg(
                    static_friction=static,
                    dynamic_friction=dynamic,
                    restitution=restitution,
                    compliant_contact_stiffness=5e4,
                    compliant_contact_damping=8e2,
                    improve_patch_friction=True,
                    friction_combine_mode="average",
                )

                mat_cfg.func(f"/World/ContactMaterials/env_{i}/{foot_name}_mat", mat_cfg)
                prim_path = f"/World/envs/env_{i}/Robot/BimoSmall/{foot_name}"
                mat_path = f"/World/ContactMaterials/env_{i}/{foot_name}_mat"

                bind_physics_material(prim_path, mat_path)

        # Ground properties: ceramic tiles (adjust based on your surface type)
        ground_cfg = RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=0.5,
            restitution=0.05,
            improve_patch_friction=True,
            friction_combine_mode="average",
        )
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg(physics_material=ground_cfg))

        # Light
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

        # Camera
        self.cfg.viewer.eye = (5.0, -5.0, 4.0)
        self.cfg.viewer.lookat = (0.0, 0.0, 0.0)

    def _get_observations(self):
        """Compute and return observations"""

        # Get IMU data, add noise and scale to [-1, 1]
        self.imu_data = self.scene.sensors["imu"].data
        orient = quaternion_to_euler(self.imu_data.quat_w)

        orient = gaussian_noise(orient, self.orient_noise)
        angular_vel = gaussian_noise(self.imu_data.ang_vel_b, self.gyro_noise)

        orient = scale_value(orient, -1.0, 1.0)
        angular_vel = scale_value(angular_vel, -2, 2)

        # Update IMU history and arrange for observations
        self.update_imu_history(orient, angular_vel)

        imu_data = torch.cat((self.orient_h[:, :, :2], self.gyro_h), dim=2)
        imu_data = imu_data.reshape(self.scene.num_envs, 20)

        # Get last commanded position and scale to [-1, 1]
        cmd_act = torch.clamp((self.cmd_actions - self.servo_min) / (self.servo_max - self.servo_min) * 2 - 1, -1, 1)

        # Update position history and arrange for observations
        self.act_hist[:, :-1] = self.act_hist[:, 1:].clone()
        self.act_hist[:, -1] = cmd_act
        proc_act = self.act_hist.reshape(self.scene.num_envs, 32)

        # Create observation buffer
        obs_buffer = None

        if self.obj != "turn":
            obs_buffer = torch.cat((imu_data, proc_act), dim=1)

        else:
            # Includes direction observation (turn i.e. left/right)
            obs_buffer = torch.cat((self.act_direction.unsqueeze(1), imu_data, proc_act), dim=1)

        obs_buffer = torch.round(obs_buffer, decimals=4)

        return {"policy": obs_buffer}

    def _pre_physics_step(self, actions):
        # Calculates action delta
        actions_cpy = torch.clamp(actions.clone(), -3.0, 3.0)
        self.cmd_actions += actions_cpy * 2 / 3

        # Simulates backlash ~1.59 DEG
        delta = self.cmd_actions - self.gear_position
        direction = torch.sign(delta)
        direction_changed = (direction != self.last_direction) & (self.last_direction != 0)

        movement = torch.where(
            direction_changed,
            torch.clamp(torch.abs(delta) - 1.6, min=0) * direction,
            delta,
        )

        self.gear_position += movement
        self.last_direction = torch.where(delta != 0, direction, self.last_direction)

        # Adds noise
        self.noisy_act = torch.clamp(gaussian_noise(self.gear_position, self.actuator_noise), self.servo_min, self.servo_max)

        # Action delay 5ms - 20ms
        self.act_timer = 0
        self.act_delay = torch.randint(
            low=self.cfg.actuator_delay_min,
            high=self.cfg.actuator_delay_max + 1,
            size=(1,)
        ).item()

    def _apply_action(self):
        # Applies NN action
        if self.act_timer >= self.act_delay:
            self.bimo.set_joint_position_target(torch.deg2rad(self.noisy_act))

        else:
            self.act_timer += 1

    def _get_rewards(self):
        # Get data for reward
        euler_imu_orient = quaternion_to_euler(self.imu_data.quat_w)
        bimo_root_pos = self.bimo.data.root_pos_w
        lin_vel = self.bimo.data.root_com_vel_w
        contact_pos = self.scene.sensors["contact"].data.pos_w
        air_time = self.scene.sensors["contact"].data.current_air_time

        # Compute reward components
        orientation_rew = orientation_reward(euler_imu_orient, self.obj, self.device)
        height_rew = height_reward(bimo_root_pos)
        position_rew = joint_position_reward(self.cmd_actions, self.base_pose, self.device)
        sig_extra = sigmoid_extra(self.cmd_actions, self.base_pose)
        vel_rew = velocity_reward(lin_vel, self.act_direction, self.obj)
        feet_h_rew = feet_height_reward(air_time, contact_pos, 0.03, 150)
        dev_rew = deviation_reward(self.scene.env_origins, bimo_root_pos, self.obj)

        # Compute weighted reward
        w = self.weights / torch.sum(self.weights, dim=1, keepdim=True)

        total_reward = (orientation_rew * w[:, 0] + height_rew * w[:, 1]
                        + position_rew * w[:, 2] + sig_extra * w[:, 3]
                        + feet_h_rew * w[:, 4] + vel_rew * w[:, 5]
                        + dev_rew * w[:, 6])

        return total_reward

    def _get_dones(self):
        # Compute and return done flags
        terminated = torch.zeros(self.scene.num_envs, dtype=torch.bool, device=self.device)
        truncated = torch.zeros(self.scene.num_envs, dtype=torch.bool, device=self.device)

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

        # Shift COM if not done
        if not self.com_set:
            physx_view = self.bimo.root_physx_view
            coms = physx_view.get_coms()
            coms[:, 0, 0] -= self.cfg.com_shift

            physx_view.set_coms(coms, indices=env_ids.cpu())
            self.com_set = True

        # Get default root pose and adds env origin position (for spacing)
        root_state = self.bimo.data.default_root_state[env_ids]
        root_state[:, :3] += self.scene.env_origins[env_ids]

        # Get default joint positions and velocities
        joint_pos = self.bimo.data.default_joint_pos[env_ids].clone()
        joint_vel = self.bimo.data.default_joint_vel[env_ids].clone()

        # Randomze joint parameters
        reset_ids = env_ids.flatten().long()
        n_reset = reset_ids.shape[0]
        n_joints = 8

        fric_idx = torch.randint(0, self.frictions.size(0), (n_reset, n_joints), device=self.device)
        torque_idx = torch.randint(0, self.torques.size(0), (n_reset, n_joints), device=self.device)
        damp_idx = torch.randint(0, self.dampings.size(0), (n_reset, n_joints), device=self.device)

        fric_samples = self.frictions[fric_idx]
        torque_samples = self.torques[torque_idx]
        damp_samples = self.dampings[damp_idx]

        # Write data to sim
        self.bimo.write_joint_friction_coefficient_to_sim(fric_samples, joint_ids=None, env_ids=env_ids)
        self.bimo.write_joint_effort_limit_to_sim(torque_samples, joint_ids=None, env_ids=env_ids)
        self.bimo.write_joint_damping_to_sim(damp_samples, joint_ids=None, env_ids=env_ids)

        self.bimo.write_root_link_pose_to_sim(root_state[:, :7], env_ids)
        self.bimo.write_root_com_velocity_to_sim(root_state[:, 7:], env_ids)
        self.bimo.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        # Reset buffers
        self.orient_h[env_ids] = 0.0
        self.gyro_h[env_ids] = 0.0
        self.act_hist[env_ids, :] = self.base_pose[0]
        self.cmd_actions[env_ids] = self.base_pose[0]

        # Not reset on purpose: makes model learn a better first step action
        # self.gear_position[env_ids] = self.base_pose[env_ids]
        # self.last_direction[env_ids] = 0

    def update_imu_history(self, new_orient, new_gyro):
        self.orient_h[:, :-1] = self.orient_h[:, 1:].clone()
        self.gyro_h[:, :-1] = self.gyro_h[:, 1:].clone()

        self.orient_h[:, -1] = new_orient
        self.gyro_h[:, -1] = new_gyro


@torch.jit.script
def quaternion_to_euler(quat: torch.Tensor):
    if not isinstance(quat, torch.Tensor):
        quat = torch.tensor(quat)

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
        torch.sign(sinp) * torch.tensor(torch.pi / 2),
        torch.asin(sinp)
    )

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)

    return torch.stack([roll, pitch, yaw], dim=1)


@torch.jit.script
def scale_value(value: torch.Tensor, min_val: float, max_val: float):
    # Scale a value to the range [-1, 1]
    return torch.clamp((value - min_val) / (max_val - min_val) * 2 - 1, -1, 1)


@torch.jit.script
def scale_to_range(value: float, min_val: float, max_val: float):
    # Gets original value from a number in range [-1, 1]
    return min_val + (value + 1) * 0.5 * (max_val - min_val)


@torch.jit.script
def orientation_reward(euler_imu_orient, action: str, device: str):
    # Calculate the sum of absolute Euler angles
    angle_sums = torch.zeros(euler_imu_orient.shape[0], device=device)

    if action == "walk":
        angle_sums = torch.sum(torch.abs(euler_imu_orient), dim=1)

    else:
        # Excludes Z from reward to aid in turning
        angle_sums = torch.sum(torch.abs(euler_imu_orient[:, :2]), dim=1)

    # Calculate the reward
    orientation_rew = torch.where(
        angle_sums <= 0.95,
        1 - torch.sqrt(angle_sums / 0.95),
        torch.ones_like(angle_sums) * -1
    )

    return orientation_rew


@torch.jit.script
def deviation_reward(og_pose, curr_pose, action: str = "walk"):
    # X, Y distance deviation reward
    x_dev = torch.abs(og_pose[:, 0] - curr_pose[:, 0])
    y_dev = torch.abs(og_pose[:, 1] - curr_pose[:, 1])

    reward = torch.zeros_like(x_dev)

    # Calculate reward
    if action == "walk":
        # Y distance only for walking
        reward = torch.where(
            y_dev <= 0.3,
            1 - torch.sqrt(y_dev / 0.3),
            torch.ones_like(y_dev) * -1
        )

    else:
        # X + Y distance for turning
        dist = x_dev + y_dev
        reward = torch.where(
            dist <= 0.3,
            1 - torch.sqrt(dist / 0.3),
            torch.ones_like(dist) * -1
        )

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
    """Calculates how far the joints position is from the ideal position"""
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
def velocity_reward(vel_data, direction, action: str = "walk"):
    """Calculates reward based on linear and angualr velocities"""
    reward = torch.zeros_like(direction)

    if action == "walk":
        vx = vel_data[:, 0]
        vy = torch.abs(vel_data[:, 1])

        rew_lin = torch.where(
            vx > 0,
            torch.clamp(vx / (vx + vy + 1e-8), 0, 1.0),
            0,
        )

        rew_ang = torch.clamp(- torch.abs(vel_data[:, 4]) / 2, -1, 0)

        reward = 0.5 * rew_lin + 0.5 * rew_ang

    else:
        z_ang_vel = vel_data[:, 5]

        # -1 = Turn Left, +1 Turn Right | Turning left velocity +Z
        correct_vel = torch.sign(z_ang_vel) != torch.sign(direction)

        rew_ang = torch.where(
            correct_vel,
            torch.clamp(torch.abs(z_ang_vel) / 0.2, 0, 1.0),
            -1,
        )

        rew_ang_penalty = torch.clamp(- torch.abs(vel_data[:, 4]) / 2, -1, 0)

        reward = 0.5 * rew_ang + 0.5 * rew_ang_penalty

    return reward


@torch.jit.script
def sigmoid_extra(pos_buff, start_pos):
    """Extra reward when actuators < 10 DEG difference from ideal"""
    diff = torch.abs(pos_buff - start_pos)
    greatest_diff, _ = torch.max(diff, dim=1)

    sigmoid_values = 1 / (1 + torch.exp(0.8 * greatest_diff - 6))

    return sigmoid_values


@torch.jit.script
def feet_height_reward(air_time, feet_pos, target_h: float, scale: float = 25.0):
    """Feet clearance reward"""
    in_air = (air_time > 0)  # [num_envs, 2]
    num_in_air = in_air.sum(dim=1)  # [num_envs]

    both_in_air = (num_in_air == 2)
    both_on_ground = (num_in_air == 0)

    # Foot Z positions
    z_pos = feet_pos[..., 2]  # [num_envs, 2]
    z_err = torch.abs(z_pos - target_h)  # [num_envs, 2]

    # Reward is 1.0 if z >= threshold, else exponential decay
    reward_per_leg = torch.where(
        z_pos >= target_h,
        torch.ones_like(z_pos),         # reward 1.0 if above clearance
        torch.exp(-scale * z_err)       # otherwise decays
    ) * in_air.float()                  # only airborne feet count

    reward = reward_per_leg.sum(dim=1)  # sum both legs

    # If both feet in air or both on ground, set reward to 0
    reward = torch.where(
        both_in_air | both_on_ground,
        torch.zeros_like(reward),
        reward
    )

    return reward
