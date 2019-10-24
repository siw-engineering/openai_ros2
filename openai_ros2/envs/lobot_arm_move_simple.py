import gym
import rclpy
import random
from typing import Sequence, Tuple
from openai_ros2.robots.lobot.lobot_arm_sim import LobotArmSim
from openai_ros2.robots.lobot.tasks.basic_movement import LobotArmBasicMovement


class LobotArmMoveSimpleEnv(gym.Env):
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)
        self.__robot = LobotArmSim(self.node)
        self.__task = LobotArmBasicMovement(self.node)
        # Set up ROS related variables
        self.__episode_num = 0
        self.__cumulated_episode_reward = 0
        self.__step_num = 0

    def step(self, action: Sequence[LobotArmSim.Action]) -> Tuple[Sequence[float], float, bool, str]:
        self.__robot.set_action(action)
        obs = self.__robot.get_observations()
        joint_states = obs[0:3]
        reward = self.__task.compute_reward(joint_states, self.__step_num)
        done = self.__task.is_done(obs, self.__step_num)
        info = ""
        self.__cumulated_episode_reward += reward
        self.__step_num += 1

        print(f"Reward for step {self.__step_num}: {reward}, \t cumulated reward: {self.__cumulated_episode_reward}")
        return obs, reward, done, info

    def reset(self):
        print(f"Episode {self.__episode_num} concluded with total reward of: {self.__cumulated_episode_reward}")
        self.__robot.reset()
        self.__step_num = 0
        self.__episode_num += 1
        self.__cumulated_episode_reward = 0
        # Maybe publish to topic?
        pass

    def render(self, mode='human'):
        pass

