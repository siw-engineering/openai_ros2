from gym.envs.registration import register

from openai_ros2.tasks import LobotArmFixedGoal
from openai_ros2.robots import LobotArmSim


register(
    id='LobotArmContinuous-v0',  # Continuous action space
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmFixedGoal,
            'robot_cls': robots.LobotArmSim
           }
)
register(
    id='LobotArmContinuous-v1',  # Continuous action space with noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmFixedGoal,
            'robot_cls': robots.LobotArmSim,
            'state_noise_mu': 0,
            'state_noise_sigma': 0.075
            }
)
register(
    id='LobotArmContinuous-v2',  # Continuous action space without noise, with random goal
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmRandomGoal,
            'robot_cls': robots.LobotArmSim
            }
)
register(
    id='LobotArmContinuous-v3',  # Continuous action space with noise, with random goal
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmRandomGoal,
            'robot_cls': robots.LobotArmSim,
            'state_noise_mu': 0,
            'state_noise_sigma': 0.075
            }
)
register(
    id='LobotArmDiscrete-v0',  # Discrete action space no noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmFixedGoal,
            'robot_cls': robots.LobotArmSimDiscrete
            }
)
register(
    id='LobotArmDiscrete-v1',  # Discrete action space with noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmFixedGoal,
            'robot_cls': robots.LobotArmSimDiscrete,
            'state_noise_mu': 0,
            'state_noise_sigma': 0.075
            }
)