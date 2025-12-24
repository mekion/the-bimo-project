# Bimo Isaac Lab Environment

Reinforcement learning training environment for Bimo robot with sim-to-real transfer using Isaac Lab.

## Quick Description

This folder contains the **Isaac Lab task definition** for training bipedal walking policies on Bimo using modern RL algorithms. The environment provides:

- **Full Implementation**: to quickly train and deploy.
- **Walk Task**: learns how to walk and **directly transfers to real robot**
- **2 Experimental Task Variants**: turn, and stop (keep standing when pushed)
- **Vectorized Approach**: parallel environments for fast, under 20 min training time (depends on hardware)

## Setup

### Prerequisites

**Install Isaac Lab 2.0.2** (follow [official docs](https://isaac-sim.github.io/IsaacLab/v2.0.2/))

Project will be migrated soon to the latest version, after ensuring compatibility.

### Installation

From `the-bimo-project/IsaacLab`, copy the `bimo` folder into:

```bash
cp -r bimo /path/to/your/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/
```

## Training

### Train Walking Policy

You can use video to monitor training. Disable for maximum performance.

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Bimo --num_envs 2048 --headless --video --video_interval 2000 --video_length 200
```

This trains the **walking model** that transfers directly to the real robot.

### Experimental Task Variants

The following tasks are available for research and experimentation, but **do not currently transfer to hardware**:

**1. Turn**: learns directional control via extra observation signal (turn left/right)  
**2. Stop**: learns to keep the robot standing when pushes are applied

These are included to explore alternative behaviors and can be trained, but hardware validation is needed before deployment.

### Monitor Training

TensorBoard logs are saved during training:

```bash
tensorboard --logdir=/path/to/your/BimoIsaacLab/IsaacLab/logs/rsl_rl/bimo_ppo_rsrl
```

## Deployment on Hardware

Once training converges:

1. Export the ONNX policy by running the play script (it will automatically create the exported policy in the `bimo_ppo_rsrl` directory):
   ```bash
   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Bimo --num_envs 1 --headless
   ```

2. Copy the `.onnx` file to your project folder or where your main loop script is located (see `api_example.py` in `examples` folder)

3. Run the inference script.

The **walking policy** will transfer directly. Other behaviors may require hardware tuning.

## Task Definition

The task files and configurations are defined in `bimo/`:

- **`bimo_task_env.py`**: environment class with observation/action/reward logic
- **`bimo_config.py`**: robot and actuator configuration
- **`agents/rsl_rl.py`**: RSL_RL PPO hyperparameters

Detailed information about reward and environment design will be added in a future update to this README.

## Current Status

✅ **Walking**: tested and verified sim-to-real transfer  
🔬 **Turning/Stopping**: experimental, training works, hardware transfer pending

## Next Steps

- **(In Progress)** Update to latest IsaacLab version
- Perform sim-to-real parameter matching to improve transfer quality
- Validate stop, turn on hardware
- Increase overall training robustness


## References

- [Isaac Lab Docs](https://isaac-sim.github.io/IsaacLab/main/)
- [RSL-RL GitHub](https://github.com/leggedrobotics/rsl_rl)
- [Bimo BimoAPI](../BimoAPI/) for hardware control and inference

## Support

For questions or issues:
1. Check Isaac Lab documentation
2. Review `bimo_task_env.py` for task implementation details
3. Open an issue on GitHub: https://github.com/mekion/the-bimo-project

---

**Note**: Extended documentation on reward functions, observation design, and hyperparameter tuning coming soon.
