from gym.envs.registration import register

register(
    id='LobotArmMoveSimple-v0',
    entry_point='openai_ros2.envs:LobotArmMoveSimpleEnv',
)