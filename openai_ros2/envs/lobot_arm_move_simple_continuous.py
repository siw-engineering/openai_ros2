import gym
from gym.spaces import Box
import rclpy
import random
import numpy
from typing import Sequence, Tuple, Type
from openai_ros2.robots.lobot.lobot_arm_sim_continuous import LobotArmConActSim
from openai_ros2.robots.lobot.tasks.basic_movement import LobotArmBasicMovement

class LobotArmMoveSimpleConActEnv(gym.Env):
    class ObservationData:
        position_data: numpy.ndarray = numpy.array([])
        velocity_data: numpy.ndarray = numpy.array([])
        step_count: int = 0
        contact_count: int = 0
        # the data in this contacts array is an object of gazebo_msgs.msg.ContactState
        contacts: numpy.ndarray = numpy.array([])

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)
        possible_action_count = len(LobotArmConActSim.Action)
        self.action_space = Box(-1.57079632679, 1.57079632679, shape=(3,1))  #TODO more dimension limit, i.e. -2 to 2 for joint1, -1 to 1 for joint2
        self.__robot = LobotArmConActSim(self.node)
        self.__task = LobotArmBasicMovement(self.node)
        # Set up ROS related variables
        self.__episode_num = 0
        self.__cumulated_episode_reward = 0
        self.__step_num = 0

    def step(self, action: numpy.ndarray) -> Tuple[ObservationData, float, bool, str]:
        self.__robot.set_action(action)
        robot_state: LobotArmConActSim.Observation = self.__robot.get_observations()
        obs = LobotArmMoveSimpleConActEnv.ObservationData()
        obs.step_count = self.__step_num
        obs.position_data = robot_state.position_data
        obs.velocity_data = robot_state.velocity_data
        obs.contact_count = robot_state.contact_count
        obs.contacts = robot_state.contacts
        reward = self.__task.compute_reward(obs.position_data, self.__step_num)
        done = self.__task.is_done(obs.position_data, robot_state.contact_count, self.__step_num)
        info = ""
        self.__cumulated_episode_reward += reward
        self.__step_num += 1

        print(f"Reward for step {self.__step_num}: {reward}, \t cumulated reward: {self.__cumulated_episode_reward}")
        return obs, reward, done, info

    def reset(self):
        print(f"Episode {self.__episode_num} concluded with total reward of: {self.__cumulated_episode_reward}")
        self.__robot.reset()
        self.__task.reset()
        self.__step_num = 0
        self.__episode_num += 1
        self.__cumulated_episode_reward = 0
        # Maybe publish to topic?
        pass

    def render(self, mode='human'):
        pass

