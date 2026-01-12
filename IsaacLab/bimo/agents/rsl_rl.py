# Copyright (c) 2025-2026, Mekion
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: Apache-2.0

"""RSL-RL PPO training configuration for Bimo Robotics Kit."""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class BimoPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 16
    max_iterations = 1200
    save_interval = 50
    experiment_name = "bimo_ppo_rsrl"
    empirical_normalization = True
    fp16 = True  # Needed for RKNN for example, can be removed

    obs_groups = {
        "policy": ["policy"],
        "critic": ["policy"],
    }

    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        actor_obs_normalization=True,
        critic_obs_normalization=False,
        state_dependent_std=False,
        activation="elu",
    )

    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1e-3,
        gamma=0.99,
        lam=0.95,
        max_grad_norm=1.0,
        schedule="adaptive",
        desired_kl=0.01,
    )
