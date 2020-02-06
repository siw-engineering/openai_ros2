from gym.envs.registration import register

from openai_ros2.tasks import LobotArmFixedGoal, LobotArmRandomGoal
from openai_ros2.robots import LobotArmSim, LobotArmSimDiscrete, LobotArmPybullet

register(
    id='LobotArmContinuousBullet-v0',  # Continuous action space
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmFixedGoal,
            'robot_cls': LobotArmPybullet
            }
)
register(
    id='LobotArmContinuous-v0',  # Continuous action space
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmFixedGoal,
            'robot_cls': LobotArmSim
            }
)
register(
    id='LobotArmContinuous-v1',  # Continuous action space with noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmFixedGoal,
            'robot_cls': LobotArmSim,
            'robot_kwargs': {
                'state_noise_mu': 0,
                'state_noise_sigma': 0.075
                }
            }
)
register(
    id='LobotArmContinuous-v2',  # Continuous action space without noise, with random goal
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmRandomGoal,
            'robot_cls': LobotArmSim
            }
)
register(
    id='LobotArmContinuous-v3',  # Continuous action space with noise, with random goal
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmRandomGoal,
            'robot_cls': LobotArmSim,
            'robot_kwargs': {
                'state_noise_mu': 0,
                'state_noise_sigma': 0.075
                }
            }
)
register(
    id='LobotArmContinuous-v4',  # Continuous action space with noise, with random goal
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmRandomGoal,
            'robot_cls': LobotArmSim,
            'robot_kwargs': {
                'random_init_pos': True
                }
            }
)
register(
    id='LobotArmContinuous-v6',
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmRandomGoal,
            'robot_cls': LobotArmSim,
            'robot_kwargs': {
                'random_init_pos': True
            },
            'task_kwargs': {
                'accepted_dist_to_bounds': 0.001,
                'accepted_error': 0.001,
                'reach_target_bonus_reward': 15.0,
                'reach_bounds_penalty': 10.0,
                'contact_penalty': 10.0,
                'episodes_per_goal': 1,
                'goal_buffer_size': 5,
                'goal_from_buffer_prob': 1.0,
                'num_adjacent_goals': 0,
                'use_fixed_goal_buffer': True
            }
            }
)
register(
    id='LobotArmContinuousJoann-v0',
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmRandomGoal,
            'robot_cls': LobotArmSim,
            'robot_kwargs': {
                'random_init_pos': False
            },
            'task_kwargs': {
                'accepted_dist_to_bounds': 0.001,
                'accepted_error': 0.001,
                'reach_target_bonus_reward': 0.0,
                'reach_bounds_penalty': 0.0,
                'contact_penalty': 0.0,
                'episodes_per_goal': 1,
                'goal_buffer_size': 5,
                'goal_from_buffer_prob': 1.0,
                'num_adjacent_goals': 0,
                'use_fixed_goal_buffer': True
            }
            }
)
register(
    id='LobotArmDiscrete-v0',  # Discrete action space no noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmFixedGoal,
            'robot_cls': LobotArmSimDiscrete
            }
)
register(
    id='LobotArmDiscrete-v1',  # Discrete action space with noise
    entry_point='openai_ros2.envs:LobotArmEnv',
    kwargs={'task_cls': LobotArmFixedGoal,
            'robot_cls': LobotArmSimDiscrete,
            'robot_kwargs': {
                'state_noise_mu': 0,
                'state_noise_sigma': 0.075
                }
            }
)