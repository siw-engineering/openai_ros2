from gym.envs.registration import register
from openai_ros2.envs.task_envs.lobot_arm.arm_move_simple import LobotArmMoveSimpleEnv
register(
    id='LobotArmSimple-v0',
    entry_point='openai_ros2.envs.task_envs.lobot_arm.arm_move_simple:LobotArmTaskEnv',
)