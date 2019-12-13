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
    id='LobotArmDiscrete-v0',  # Discrete action space
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': tasks.LobotArmFixedGoal,
            'robot_cls': robots.LobotArmSimDiscrete
            }
)
