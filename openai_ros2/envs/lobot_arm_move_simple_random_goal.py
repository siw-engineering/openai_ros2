import gym
from gym.spaces import MultiDiscrete
import rclpy
import random
import numpy
from typing import Sequence, Tuple, Type
from openai_ros2.robots.lobot.lobot_arm_sim import LobotArmSim
from openai_ros2.robots.lobot.tasks.basic_movement_random_goal import LobotArmBasicMovementRandomGoal

class LobotArmMoveSimpleRandomGoalEnv(gym.Env):
    class ObservationData:
        position_data: numpy.ndarray = numpy.array([])
        velocity_data: numpy.ndarray = numpy.array([])
        step_count: int = 0
        contact_count: int = 0
        # the data in this contacts array is an object of gazebo_msgs.msg.ContactState
        #TODO include reward coordnte
        contacts: numpy.ndarray = numpy.array([])

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node(self.__class__.__name__)
        possible_action_count = len(LobotArmSim.Action)
        self.action_space = MultiDiscrete([possible_action_count, possible_action_count, possible_action_count])
        self.__robot = LobotArmSim(self.node)
        self.__task = LobotArmBasicMovementRandomGoal(self.node)
        # Set up ROS related variables
        self.__episode_num = 0
        self.__cumulated_episode_reward = 0
        self.__step_num = 0

    def step(self, action: numpy.ndarray) -> Tuple[ObservationData, float, bool, str]:
        self.__robot.set_action(action)
        robot_state: LobotArmSim.Observation = self.__robot.get_observations()
        obs = LobotArmMoveSimpleRandomGoalEnv.ObservationData()
        obs.step_count = self.__step_num
        obs.position_data = robot_state.position_data
        obs.velocity_data = robot_state.velocity_data
        obs.contact_count = robot_state.contact_count
        obs.contacts = robot_state.contacts
        reward, obs.random_target_coord = self.__task.compute_reward(obs.position_data, self.__step_num)  #TODO add in random target for trainig
        done = self.__task.is_done(obs.position_data, robot_state.contact_count, self.__step_num)
        info = ""
        self.__cumulated_episode_reward += reward
        self.__step_num += 1

        print(f"Reward for step {self.__step_num}: {reward}, \t cumulated reward: {self.__cumulated_episode_reward}")
        return obs, reward, done, info         #TODO add in random target for trainig

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

