from gym.envs.registration import register

register(
    id='Biped-v0',
    entry_point='biped_gym.envs:BipedEnv',
)

register(
    id='RobotArm-v0',
    entry_point='biped_gym.envs:RobotArmEnv',
)