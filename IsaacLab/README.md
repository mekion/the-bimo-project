

# Bimo Isaac Lab Environment

Reinforcement learning training environment for Bimo robot with sim-to-real transfer using Isaac Lab.


## Overview

This folder contains the **Isaac Lab task definition** for training locomotion policies for Bimo using RL algorithms. The current environment provides:

- **Full Implementation**: to quickly train and deploy, including an optional distillation pass for onboard MCU inference.

- **Walk Task**: learns how to walk and **directly transfers to real robot**.
- **Vectorized Approach**: parallel environments for fast, under 6 min training time (depends on hardware)
- **Custom Actuator Model**: `STS3215Actuator` simulates real servo dynamics (backlash, bus delay, bandwidth-limited tracking) instead of an idealized motor.
- **Domain Randomization**: foot pad friction (TPU material variance), link mass (±5%), periodic velocity resets, sensor/actuator noise, actuator backlash, and voltage-driven torque cycling.


## Setup

### Prerequisites

- Install **Isaac Lab 2.3.2** following the [official docs](https://isaac-sim.github.io/IsaacLab/v2.3.2/).

### Installation

From `the-bimo-project/IsaacLab`, copy the `bimo` folder into:

```bash
cp -r bimo /path/to/your/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/
```

### Task Definition

The task files and configurations are defined in `bimo/` as:

- **`bimo_task_env.py`**: environment class with observations, reward functions, termination conditions, and domain randomization at reset.

- **`bimo_config.py`**: robot articulation properties and the `STS3215Cfg` actuator parameters.
- **`actuators/STS3215.py`**: custom **explicit actuator** model.
- **`agents/rsl_rl.py`**: RSL-RL PPO (`BimoPPORunnerCfg`) and distillation (`BimoDistillationRunnerCfg`) hyperparameters. Two independent classes, each registered as its own Gym task ID. See [Train Distilled Policy](#train-distilled-policy-useful-for-native-mcu-deployment) below.
- **`__init__.py`**: registers the `Bimo` and `Bimo-Distillation` Gym task IDs, each pointing at its own `rsl_rl_cfg_entry_point`.


## Training

### Train Walking Policy (PPO)

To start an efficient training run in headless mode, use the following command:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless
```
>Adjust the number of parallel environments based on your hardware capabilities.

You can record videos automatically at certain intervals during training, to minimally impact performance and get visual confirmation of the training process:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless --video --video_interval 2000 --video_length 200
```

These commands train the default **walking model** that transfers directly to the real robot.


### Train Distilled Policy (useful for native MCU deployment)

`agents/rsl_rl.py` defines two independent runner configs: `BimoPPORunnerCfg` (wrapping `RslRlOnPolicyRunnerCfg`) and `BimoDistillationRunnerCfg` (wrapping `RslRlDistillationRunnerCfg`). Both are registered as separate Gym task IDs in `__init__.py`, so switching between them is a `--task` flag:

```bash
# PPO (teacher)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless

# Distillation (student)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo-Distillation --num_envs 2048 --headless
```

Distillation trains a small student network (`[64, 32]` hidden dims) to imitate the PPO teacher (`[256, 128, 64]` hidden dims) trained in the previous step.  Point it at the teacher checkpoint via `load_run`/`load_checkpoint` in `BimoDistillationRunnerCfg`. The resulting student policy is small enough to run directly on the RP2040 without a tethered PC. See [Deployment](#deployment) for more details.

### Monitor Training

TensorBoard is used by default for training logs. Run it using the following command:

```bash
tensorboard --logdir=/path/to/your/IsaacLab/logs/rsl_rl/bimo_ppo_rsrl
```


## Deployment

Once training converges:

1. Export the ONNX policy by running the play script (it will automatically create the exported policy in the `bimo_ppo_rsrl` directory):
   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Bimo --num_envs 1 --headless
   ```

2. Copy the `.onnx` file to your project folder or where your main loop script is located (see `nn_walk.py` in the BimoAPI)

3. Run the inference script.

The **walking policy** will transfer directly. Other behaviors may require hardware tuning, including the walking policy when adding extra payload to Bimo.

**Running policies onboard the RP2040 (no tethered PC):** if you trained a distilled student policy, its small `.onnx` export can be converted to C with [onnx2c](https://github.com/kraiskil/onnx2c) and compiled directly into the robot's MCU firmware, letting Bimo walk without an external PC in the loop. Those conversion and RP2040 build steps live in the [MCU README](../MCU/README.md).


## Environment Design

**Observations (11 values)**
- IMU orientation (roll, pitch, yaw)
- Last commanded actions, scaled to [-1, 1] (8 values)

### Reward Function

The reward is the most complex part of the environment and the main driver of Bimo's walking behavior. Everything else here (observation construction, termination conditions, reset logic) is fairly standard for an Isaac Lab direct-workflow task.

All six reward components below are weighted equally by default (`weights = [1, 1, 1, 1, 1, 1]` in `BimoEnvCfg`, normalized to 1/6 each), and can be re-weighted without touching the reward math itself.

| Component | What It Rewards | Details |
| :-- | :-- | :-- |
| Orientation | Staying upright: penalizes roll/pitch/yaw deviation from level, with yaw weighted softer (×0.6) since heading correction should be gentler than tipping correction. | Full-reward threshold at combined tilt ≤ 0.95 rad; falls to -1 beyond that. |
| Height | Keeping the head at the ideal standing height. | Ideal height 0.381 m, tolerant up to ±0.3 m deviation. |
| Joint Position | Staying close to the standing base pose (`[-30,-30,0,0,60,60,30,30]°`) rather than drifting to extreme joint angles. | Per-joint max allowed deviation, e.g. ±90° hips, ±75° knees... |
| Feet Clearance | Lifting the swing foot to a target height during its air phase. | Target foot height 0.02 m; zero reward if both feet are airborne or both grounded simultaneously (i.e. only scored during a proper single-support swing). |
| Velocity | Forward walking speed matching a target pace. | Target 0.06 m/s; parabolic (inverted-quadratic) falloff `1 - ((v − target)/σ)²`, clamped to [-1, 1]. Peaks at 1 at the target speed, crosses 0 at ±σ, floors at -1 beyond that. |
| Deviation | Staying on a straight line laterally (Y-axis) from the spawn point, so the robot doesn't drift sideways while walking forward. | Full penalty beyond 0.2 m of lateral drift. |

### Domain Randomization

| Randomized Property | Mechanism | Range | Applied |
| :-- | :-- | :-- | :-- |
| Terrain micro-relief | `HfRandomUniformTerrainCfg` height-field noise on an otherwise flat floor | ±1 mm noise | Once, at scene setup. |
| Link mass | `randomize_rigid_body_mass` event | ±5% scale | Every episode reset. |
| Foot pad friction (TPU) | Per-env `RigidBodyMaterialCfg` bound to each foot | Static 0.3–0.7, dynamic = static − 0.1, restitution 0–0.05 | Once per env, at scene setup (fixed for that env's entire training run). |
| Motor torque limit (voltage cycling) | `write_joint_effort_limit_to_sim`, cycled sequentially per env | 2.7–2.94 Nm across 24 discrete steps | Every episode reset (deterministic round-robin, not random). |
| Head velocity reset | `push_by_setting_velocity` event, `mode="interval"` | Forces velocity to exactly 0 m/s on x/y/z | Randomly every 2–4 s during an episode. The interval timing is random even though the target velocity is fixed, so it still acts as an unpredictable momentum-zeroing disturbance. |
| Actuator backlash gap | `STS3215Actuator` internal state | 1.0°–2.4° | Resampled at reset, and again at every direction reversal mid-episode (simulates load-dependent backlash). |
| Actuator gear resting offset | `STS3215Actuator` internal state | Random position within the half-backlash band | At reset only. |
| IMU orientation noise | Additive Gaussian | σ = 0.015 rad | Every observation step. |
| Actuator command noise | Additive Gaussian | σ = 0.5° | Every physics step. |

### Actuator Model: `STS3215Actuator`

Bimo's joints previously used Isaac Lab's built-in `DCMotorCfg`, an idealized motor model. The current config replaces it with a custom explicit `STS3215Actuator` (in `bimo/actuators/STS3215.py`), modeling the actual STS3215 12V 30 kg·cm servo instead of a generic servo actuator.

It does **not** literally subclass `IdealPDActuator` or `DCMotorCfg` in code. It subclasses `ActuatorBase` directly. Conceptually, though, its core torque law is the same PD-with-torque-speed-clipping law Isaac Lab's own `DCMotorCfg` implements (itself an extension of `IdealPDActuator`'s `τ = kp·(q − q_des) + kd·(q̇ − q̇_des) + τ_ff`, clipped to a motor's effort limit).

`STS3215Actuator` reimplements that same PD-plus-clip shape from scratch specifically so it can layer three additional effects on top:

- **Bandwidth-limited tracking**: the driving position isn't the raw commanded target. It's a first-order-lag-filtered version of it (`_servo_tracked`), with the lag time constant derived from a measured ~3.1 Hz system-identification bandwidth. This reproduces the real servo's own finite response speed, not just the simulated PD gains.

- **Directional backlash**: a classic backlash operator tracks a "gear contact point" that only moves once the driving position exits a band around it. As long as motion continues in one direction, the gears stay meshed with zero extra lag; a deadband/lag only appears right after a direction reversal, exactly matching how mechanical gear backlash behaves. The band width is resampled at each reversal to represent varying load pressure.
- **Bus/inference delay**: commands pass through a small ring buffer before reaching the actuator, modeling the real request → inference → action-execution round trip (~5 ms, fixed at 1 physics step). This delay is a constant simulated latency, not randomized.
- **Coulomb friction and torque-speed clipping**: a constant friction torque opposing motion, then the same linear torque-speed clip `DCMotorCfg` uses (max torque falls off linearly to zero as joint velocity approaches `max_velocity`).

All physical constants (`stall_torque_nm`, `stiffness`, `damping`, `friction_nm`, `bandwidth_hz`, `backlash_range_deg`) are documented inline in `STS3215Cfg` with the measurement method used to derive each one.


## References

- [Isaac Lab Docs](https://isaac-sim.github.io/IsaacLab/v2.3.2/)
- [RSL-RL GitHub](https://github.com/leggedrobotics/rsl_rl)
- [onnx2c GitHub](https://github.com/kraiskil/onnx2c)


## Support

For questions or issues:
1. Check Isaac Lab documentation.
2. Review `bimo_task_env.py` for task implementation details.
3. Open an issue on GitHub: https://github.com/mekion/the-bimo-project/issues.

---
