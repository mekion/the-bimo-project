"""
Copyright (c) 2025-2026, Mekion
Copyright (c) 2022-2026, The Isaac Lab Project Developers.
SPDX-License-Identifier: Apache-2.0

Bimo Robotics Kit articulation configuration and actuator parameters for Isaac Lab.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from pathlib import Path
from math import pi
from .actuators.STS3215 import STS3215Cfg


RAD = pi / 180

BIMO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{Path(__file__).parent.resolve()}/assets/Bimo.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            enable_gyroscopic_forces=True,
            max_depenetration_velocity=1.0,
            retain_accelerations=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
            fix_root_link=False,
        ),
        copy_from_source=False,
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.38),
        rot=(0.0, 0.0, 0.0, 1.0),
        joint_pos={
            "RHip": -30 * RAD,
            "LHip": -30 * RAD,
            "RShoulder": 0.0,
            "LShoulder": 0.0,
            "RKnee": 60 * RAD,
            "LKnee": 60 * RAD,
            "RAnkle": 30 * RAD,
            "LAnkle": 30 * RAD,
        },
    ),

    actuators={
        "joints": STS3215Cfg(
            # See STS3215 actuator @configclass for more details & settings
            joint_names_expr=[".*"],
            effort_limit_sim=2.943,
            velocity_limit_sim=4.712,
            init_pose_deg=[-30, -30, 0, 0, 60, 60, 30, 30],
        ),
    },
)
