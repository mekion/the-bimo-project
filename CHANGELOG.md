

# Changelog

All notable changes to the Bimo Robotics Kit software will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.1.0] - 2026-09-01

### ROS2
#### Added
- Initial ROS2 package for Bimo, wrapping the Bimo API for use with the ROS2 ecosystem.


### IsaacLab
#### Added
- `STS3215Actuator` (`bimo/actuators/STS3215.py`): new explicit actuator model replacing the built-in `DCMotorCfg`, simulating the real STS3215 servo's bandwidth-limited tracking (sys-ID derived), directional gear backlash, fixed bus/inference delay, Coulomb friction, and torque-speed-clipped PD control. See the IsaacLab README's Actuator Model section for the full breakdown.

- `agents/rsl_rl.py`: added `BimoDistillationRunnerCfg` (wrapping `RslRlDistillationRunnerCfg`, student `[64, 32]` / teacher `[256, 128, 64]` hidden dims) as an independent class alongside `BimoPPORunnerCfg`. Enables training a small student policy for onboard RP2040 inference without a tethered PC.
- `__init__.py`: registered a second Gym task ID, `Bimo-Distillation`, pointing at `BimoDistillationRunnerCfg`. Switching between PPO and distillation training is a `--task` flag.

#### Fixed
- `bimo_task_env.py`, `_pre_physics_step()`: fixed actions "winding up" as `self.cmd_actions` accumulated the policy's action delta every step with no clamp to `[servo_min, servo_max]`, letting the commanded position drift arbitrarily far past the physical joint limits before enough opposite-direction steps could "unwind" it back into range. Now clamped every step: `self.cmd_actions = torch.clamp(self.cmd_actions + actions_cpy * 4/3, self.servo_min, self.servo_max)`.

- `bimo_task_env.py`, `_reset_idx()`: fixed `self.cmd_actions[env_ids] = self.base_pose[0]` incorrectly indexing row `0` instead of `self.base_pose[env_ids]` on reset. Harmless today since every env shares an identical base pose, but was incorrect per-env indexing and would silently break if base pose ever varied per env.
- `bimo_task_env.py`, `_reset_idx()`: `self.last_position` is now reset to the environment's spawn position on reset (`self.scene.env_origins[env_ids]`). Previously it kept whatever value the just-terminated episode last wrote, so a freshly reset env's state briefly referenced its old episode's end position instead of its new spawn point.

#### Changed
- Updated Isaac Lab project to version 2.3.2.

- `bimo_task_env.py`: backlash and actuator command delay simulation moved out of the environment file entirely and into `STS3215Actuator`. Replaces the ad-hoc `self.gear_position` / `self.backlash` / `self.last_direction` / `resample_backlash()` integration logic and the per-step `torch.randint` action delay with the actuator's own bandwidth-limited PD + backlash + fixed bus-delay model.

- `velocity_reward`: reworked from a 30/70 blend of forward-speed (`tanh`-shaped) and yaw-angular-velocity penalty into a single parabolic target-speed-tracking term (`1 - ((v − target)/σ)²`); target speed lowered from 0.1 m/s to 0.06 m/s. The explicit wobble/oscillation penalty is gone from this function.
- `deviation_reward`: no longer rewards forward per-step progress (previously computed as `curr_pos − last_position` on the X axis). Now only penalizes lateral (Y-axis) drift from the spawn point. Forward progress is handled exclusively by `velocity_reward`.
- `feet_height_reward`: changed from an asymmetric shape ("full reward for any height ≥ target, exponential decay only below it") to a symmetric decay centered on the target height, penalizing overshoot as well as undershoot. Target foot height lowered from 0.03 m to 0.02 m.
- `orientation_reward`: yaw (Z-axis) softening factor increased from ×0.5 to ×0.6.
- Terrain generation: `rough` and `slope` sub-terrain types (previously 20% proportion each) removed; training now runs on `flat` terrain only (100% proportion). Height-field noise also tightened (`vertical_scale` 0.002 → 0.001, flat `noise_step` 0.002 → 0.001).

- `BimoEnvCfg`: `weights` list shrank from 7 entries (including a zero-weighted `sigmoid_extra` slot) to 6, matching the reward components actually in use.

#### Removed
- `sigmoid_extra()`: zero-weighted "bonus reward near ideal standing pose" term. Dead weight, since its config weight was always `0`.

- `self.orient_h` / `self.act_hist` (4-step observation history buffers): shifted and appended every step, but only the newest entry was ever read. Observations now compute directly without buffering.
- Env-level backlash/delay state: `self.gear_position`, `self.backlash`, `self.bk_values`, `self.last_direction`, `resample_backlash()`, `self.act_timer`, `self.act_delay`, and the `oscillation_lim` / `actuator_delay_max` / `actuator_delay_min` / `backlash_range` config fields, all superseded by `STS3215Actuator`/`STS3215Cfg`.


### BimoAPI
#### Added
- `BimoCPG`: new Fourier-series CPG gait generator class with per-joint amplitude gain correction, packaged as a submodule of `bimo` (`from bimo import BimoCPG`).

- `cpg_walk.py`, `cpg_walk_keyboard.py`: new example control loops driving `BimoCPG` directly, the latter adding real-time `Q`/`E` keyboard steering.
#### Fixed
- `Bimo.calibrate()`: corrected ankle target pose to use negative angle values, fixing servos not reaching the physical calibration position.

#### Changed
- `api_example.py`: observation vector updated to 11 elements (3 IMU orientation + 8 last actions), matching the Isaac Lab task environment.

- `api_example.py` renamed to `nn_walk.py`.


### MCU
#### Added
- RP2040 internal temperature (`rp_temp`) exposed in `StateData` struct and available via `state["rp_temp"]` in the Python API. Struct size is now 118 bytes.

- `micro_bimo_nn.ino`: new standalone firmware for onboard NN inference. Drops the host comms protocol entirely and runs a distilled ONNX-to-C walking policy directly on the RP2040, closing the 50ms control loop with no external PC in the loop. Replicates the IMU pitch/roll offset calibration and stand-up routine that BimoAPI normally handles on the host, plus heading lock, since there's no host present to do it.

- `micro_bimo_cpg.ino`: new standalone firmware driving the CPG (Central Pattern Generator) walking gait directly on the RP2040, with the same standalone structure as `micro_bimo_nn.ino` (auto stand-up, IMU calibration, heading lock, no host comms) but no distillation or `onnx2c` step required, running the gait coefficients as-is.

#### Changed
- Distance sensor init now retries up to 3 times on startup before continuing silently.
