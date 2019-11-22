from gym.envs.registration import register

register(
    id='LobotArmMoveSimple-v0', #Move to the sample goal after all reset
    entry_point='openai_ros2.envs:LobotArmMoveSimpleEnv',
)

register(
    id='LobotArmMoveSimple-v1', #Move to the random goal after all reset
    entry_point='openai_ros2.envs:LobotArmMoveSimpleRandomGoalEnv',
)