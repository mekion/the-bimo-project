
import gymnasium as gym
from . import agents

gym.register(
    id="Bimo",
    entry_point=f"{__name__}.bimo_task_env:BimoEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bimo_task_env:BimoEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl:BimoPPORunnerCfg",
    },
)
