import gym
from gym.spaces import Box
import rclpy
import random
import numpy
import psutil
from typing import Sequence, Tuple, Type
from openai_ros2.utils import ut_launch
from openai_ros2.robots.lobot.lobot_arm_sim_continuous import LobotArmSimContinuous
from openai_ros2.robots.lobot.tasks.basic_movement import LobotArmBasicMovement

class LobotArmContinuousEnv(gym.Env):

    def __init__(self):

        # Launch gazebo with the arm in a new Process
        self.__pid_list = []
        self.launch_subp = ut_launch.startLaunchServiceProcess(ut_launch.generateLaunchDescriptionLobotArm(True))
        # TODO: Best to use dictionary for the PID such that it doesn't accidentally kill another process that takes
        #  a previous process's PID
        print(f"Adding process with pid {self.launch_subp.pid} to list")
        self.__pid_list.append(self.launch_subp.pid)
        parent = psutil.Process(self.launch_subp.pid)
        for child in parent.children(recursive=True):
            self.__pid_list.append(child.pid)
            print(f"Adding process with pid {self.launch_subp.pid} to list")

        context = rclpy.get_default_context()
        if(not context.ok()):
            rclpy.init() 
        self.node = rclpy.create_node(self.__class__.__name__)
        self.action_space = Box(-1.57079632679, 1.57079632679, shape=(3,))
        self.observation_space = Box(numpy.array([-2.37, -1.57, -1.57, -3, -3, -3]),
                            numpy.array([2.37, 0.5, 1.57, 3, 3, 3]))
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

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")
        self.node.destroy_node()
        rclpy.shutdown()

    def __del__(self):
        print("Destructor of " + self.__class__.__name__ + " environment.")
        for pid in self.__pid_list:
            try:
                p = psutil.Process(pid)
                print(f"Killing process {pid}, name: {p.name()}")
                p.kill()
            except psutil.NoSuchProcess as nsp:
                print(f"No such process {pid} {nsp}")

    def render(self, mode='human'):
        pass
