import gym
from gym.spaces import Box
import rclpy
import random
import numpy
from typing import Sequence, Tuple, Type
from openai_ros2.robots.lobot.lobot_arm_sim_continuous import LobotArmSimContinuous
from openai_ros2.robots.lobot.tasks.basic_movement import LobotArmBasicMovement

class LobotArmContinuousEnv(gym.Env):
    class ObservationData:
        position_data: numpy.ndarray = numpy.array([])
        velocity_data: numpy.ndarray = numpy.array([])
        # step_count: int = 0
        # contact_count: int = 0
        # the data in this contacts array is an object of gazebo_msgs.msg.ContactState
        # contacts: numpy.ndarray = numpy.array([])

    def __init__(self):
        context = rclpy.get_default_context()
        if(not context.ok()):
            rclpy.init() 
        self.node = rclpy.create_node(self.__class__.__name__)
        self.action_space = Box(-1.57079632679, 1.57079632679, shape=(3, ))
        self.observation_space = Box(numpy.array([-2.37, -1.57, -1.57, -3, -3, -3]),
                                    numpy.array([2.37, 0.5, 1.57, 3, 3, 3]))
        # TODO more dimension limit, i.e. -2 to 2 for joint1, -1 to 1 for joint2
        self.__robot = LobotArmSimContinuous(self.node)
        self.__task = LobotArmBasicMovement(self.node)
        # Set up ROS related variables
        self.__episode_num = 0
        self.__cumulated_episode_reward = 0
        self.__step_num = 0
        self.reset()

    def step(self, action: numpy.ndarray) -> Tuple[numpy.ndarray, float, bool, str]:
        self.__robot.set_action(action)
        robot_state: LobotArmContinuousEnv.Observation = self.__robot.get_observations()
        obs = numpy.concatenate((robot_state.position_data, robot_state.velocity_data))

        reward = self.__task.compute_reward(robot_state.position_data, self.__step_num)
        done = self.__task.is_done(robot_state.position_data, robot_state.contact_count, self.__step_num)
        info = ""
        self.__cumulated_episode_reward += reward
        self.__step_num += 1

        # print(f"Reward for step {self.__step_num}: {reward}, \t cumulated reward: {self.__cumulated_episode_reward}")
        return obs, reward, done, info

    def reset(self):
        print(f"Episode {self.__episode_num} concluded with total reward of: {self.__cumulated_episode_reward}")
        self.__robot.reset()
        self.__task.reset()
        self.__step_num = 0
        self.__episode_num += 1
        self.__cumulated_episode_reward = 0
        return numpy.array([0,0,0,0,0,0])

    def render(self, mode='human'):
        pass
