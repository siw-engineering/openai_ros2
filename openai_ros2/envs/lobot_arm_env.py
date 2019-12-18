import os
from typing import Tuple

import gym
from gym.spaces import Box

import numpy

from openai_ros2.robots import LobotArmSim, LobotArmBase
from openai_ros2.utils import ut_launch

import psutil

import rclpy


class LobotArmEnv(gym.Env):
    """OpenAI Gym environment for Lobot Arm, utilises continuous action space."""

    def __init__(self, robot_cls: type, task_cls: type, state_noise_mu: float = None, state_noise_sigma: float = None):
        ut_launch.set_network_env_vars()
        os.environ["RMW_IMPLEMENTATION"] = 'rmw_opensplice_cpp'
        context = rclpy.get_default_context()
        if not context.ok():
            rclpy.init()
        sim_time_param= rclpy.parameter.Parameter("use_sim_time", value=True)
        self.node = rclpy.node.Node(robot_cls.__name__, parameter_overrides=[sim_time_param])
        # self.node.set_parameters([sim_time])
        self.__robot: LobotArmBase = robot_cls(self.node)
        self.__robot.state_noise_mu = state_noise_mu
        self.__robot.state_noise_sigma = state_noise_sigma
        self.__task = task_cls(self.node, self.__robot)
        self.action_space = self.__robot.get_action_space()
        self.observation_space = self.__robot.get_observation_space()
        # Set up ROS related variables
        self.__episode_num = 0
        self.__cumulated_episode_reward = 0
        self.__step_num = 0
        self.reset()

    def step(self, action: numpy.ndarray) -> Tuple[numpy.ndarray, float, bool, dict]:
        self.__robot.set_action(action)
        robot_state: LobotArmBase.Observation = self.__robot.get_observations()
        obs = numpy.concatenate((robot_state.position_data, robot_state.velocity_data))

        reward = self.__task.compute_reward(robot_state.noiseless_position_data, self.__step_num)
        done = self.__task.is_done(robot_state.noiseless_position_data, robot_state.contact_count, self.__step_num)
        info: dict = {}
        self.__cumulated_episode_reward += reward
        self.__step_num += 1

        # print(f"Reward for step {self.__step_num}: {reward}, \t cumulated reward: {self.__cumulated_episode_reward}")
        return obs, reward, done, info

    def reset(self):
        print(f'Episode {self.__episode_num} concluded with total reward of: {self.__cumulated_episode_reward}')
        self.__robot.reset()
        self.__task.reset()
        self.__step_num = 0
        self.__episode_num += 1
        self.__cumulated_episode_reward = 0
        return numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def close(self):
        print('Closing ' + self.__class__.__name__ + ' environment.')
        self.node.destroy_node()
        rclpy.shutdown()

    def render(self, mode='human'):
        pass

    def set_state_noise(self, mu: float, sigma: float) -> None:
        self.__robot.state_noise_mu = mu
        self.__robot.state_noise_sigma = sigma
