# Bimo Isaac Lab Environment

Reinforcement learning training environment for Bimo robot with sim-to-real transfer using Isaac Lab.


## Overview

This folder contains the **Isaac Lab task definition** for training locomotion policies for Bimo using RL algorithms. The current environment provides:

- **Full Implementation**: to quickly train and deploy.
- **Walk Task**: learns how to walk and **directly transfers to real robot**.
- **2 Experimental Task Variants**: turn, and stop (keep standing when pushed)
- **Vectorized Approach**: parallel environments for fast, under 15 min training time (depends on hardware)
- **Domain Randomization**: foot pad friction (TPU material variance), link mass (±5%),
  periodic pushes, sensor/actuator noise, backlash simulation, and voltage-driven torque cycling.


## Setup

### Prerequisites

- **Install Isaac Lab 2.3.0** (follow the [official docs](https://isaac-sim.github.io/IsaacLab/v2.3.0/))
- **Install TensorBoard** inside the IsaacLab environment created during its installation. TensorBoard will allow tracking training progress of RL tasks.

### Installation

From `the-bimo-project/IsaacLab`, copy the `bimo` folder into:

```bash
cp -r bimo /path/to/your/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/
```


## Training

### Train Walking Policy

To start an efficient training run in headless mode, use the following command:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless
```
**NOTE:** adjust the number of parallel environments based on your hardware capabilities.

You can record videos automatically at certain intervals during training, to minimally impact performance and get visual confirmation of the training process:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless --video --video_interval 2000 --video_length 200
```

These commands train the default **walking model** that transfers directly to the real robot.


### Experimental Task Variants

The following tasks are available for research and experimentation, but **do not currently transfer to hardware**:

**1. Turn**: learns directional control via extra observation signal (turn left/right)  
**2. Stop**: learns to keep the robot standing when pushes are applied

These are included to explore alternative behaviors and can be trained, but hardware validation is needed before deployment. They can be selected by changing the `obj` field in `BimoEnvCfg` inside `bimo_task_env.py`.


### Monitor Training

TensorBoard is used by default for training logs. Install TensorBoard inside your Python environment for visualization:

```bash
tensorboard --logdir=/path/to/your/IsaacLab/logs/rsl_rl/bimo_ppo_rsrl
```


## Deployment on Hardware

Once training converges:

1. Export the ONNX policy by running the play script (it will automatically create the exported policy in the `bimo_ppo_rsrl` directory):
   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Bimo --num_envs 1 --headless
   ```

2. Copy the `.onnx` file to your project folder or where your main loop script is located (see `api_example.py` in BimoAPI)

3. Run the inference script.

The **walking policy** will transfer directly. Other behaviors may require hardware tuning.


## Environment Design

**Observations (44 values - walk & stop / 41 - turn)**
- 4-step IMU orientation history (roll, pitch, yaw × 4 = 12 values)
- 4-step action history, scaled to [-1, 1] (8 joints × 4 = 32 values)
- Direction: meant for turn selection signal (+1 right / -1 left)

**Reward Components** (`obj` key selects weight preset):

| Component | Walk | Turn | Stop |
|---|---|---|---|
| Orientation | ✓ | ✓ | ✓ |
| Height | ✓ | ✓ | ✓ |
| Joint position | ✓ | ✓ | ✓ |
| Sigmoid extra | — | ✓ | ✓ |
| Feet clearance | ✓ | ✓ | ✓ |
| Velocity | ✓ | ✓ | — |
| Deviation | ✓ | — | — |

**Sim-to-Real Design Choices:**
- Backlash simulation: 2.4° deadzone on direction reversal.
- Actuator noise: Gaussian (σ=0.5°) on commanded position.
- Actuator delay: 0-1 physics steps (0-5ms), randomly per step.
- Action history initialized with raw degree values (outside [-1,1]) for reset robustness.


## Task Definition

The task files and configurations are defined in `bimo/`:

- **`bimo_task_env.py`**:  environment class with observations, backlash and sensor/actuator noise simulation, reward functions, termination conditions, and domain randomization at reset.
- **`bimo_config.py`**: robot USD path, actuator PD gains, and joint configuration.
- **`agents/rsl_rl.py`**: RSL-RL PPO hyperparameters.


## Current Locomotion Status

**Walking**: tested and verified sim-to-real transfer.
**Turning/Stopping**: experimental, training works, hardware transfer pending.


## Next Steps

- Perform sim-to-real parameter matching to improve transfer quality.
- Validate stop and turn on hardware.
- Increase overall training robustness.


## References

- [Isaac Lab Docs](https://isaac-sim.github.io/IsaacLab/v2.3.0/)
- [RSL-RL GitHub](https://github.com/leggedrobotics/rsl_rl)
- [Bimo BimoAPI](../BimoAPI/) for hardware control and inference


## Support

For questions or issues:
1. Check Isaac Lab documentation
2. Review `bimo_task_env.py` for task implementation details
3. Open an issue on GitHub: https://github.com/mekion/the-bimo-project

---