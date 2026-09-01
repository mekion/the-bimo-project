"""
Copyright (c) 2025-2026, Mekion
Copyright (c) 2022-2026, The Isaac Lab Project Developers.
SPDX-License-Identifier: Apache-2.0

RSL-RL PPO training and distillation configuration for Bimo Robotics Kit.
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)
from isaaclab_rl.rsl_rl import (
    RslRlDistillationRunnerCfg,
    RslRlDistillationStudentTeacherCfg,
    RslRlDistillationAlgorithmCfg,
)


@configclass
class BimoPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 16
    max_iterations = 500
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
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
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
        num_learning_epochs=10,
        num_mini_batches=4,
        learning_rate=3e-3,
        gamma=0.99,
        lam=0.95,
        max_grad_norm=1.0,
        schedule="adaptive",
        desired_kl=0.01,
    )


@configclass
class BimoDistillationRunnerCfg(RslRlDistillationRunnerCfg):
    # Experiment settings
    seed: int = 42
    device: str = "cuda:0"
    num_steps_per_env: int = 16
    max_iterations: int = 500

    # Saving settings
    save_interval: int = 50
    experiment_name: str = "bimo_ppo_rsrl"
    run_name: str = "distillation"
    logger: str = "tensorboard"

    # Loading settings
    resume: bool = False
    load_run: str = ".*"  # Default: latest run (regex pattern)
    load_checkpoint: str = "model_.*.pt"  # Default: latest checkpoint

    # Student-Teacher Network Configuration
    policy: RslRlDistillationStudentTeacherCfg = RslRlDistillationStudentTeacherCfg(
        class_name="StudentTeacher",
        init_noise_std=1.0,  # For student policy
        student_obs_normalization=True,
        teacher_obs_normalization=True,

        # Network architectures
        student_hidden_dims=[64, 32],
        teacher_hidden_dims=[256, 128, 64],
        activation="elu",
    )

    # Distillation Algorithm Configuration
    algorithm: RslRlDistillationAlgorithmCfg = RslRlDistillationAlgorithmCfg(
        class_name="Distillation",
        num_learning_epochs=10,
        learning_rate=3e-3,
        gradient_length=16,  # How many steps gradient flows back
        max_grad_norm=1.0,
        optimizer="adam",
        loss_type="mse",  # or "huber"
    )

