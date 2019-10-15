
import rclpy
import numpy
import time
import math
from gym import spaces
from biped_gym.envs.robot_arm_env import RobotArmEnv
from gym.envs.registration import register
from std_msgs.msg import Header
from std_srvs.srv import Empty
from ros2_control_interfaces.msg import JointControl
from rclpy.qos import qos_profile_sensor_data
from biped_gym.utils import ut_param_server

class RobotArmTaskEnv(RobotArmEnv):
    def __init__(self):
        
        # Only variable needed to be set here
        number_actions = 100
        self.action_space = spaces.Discrete(number_actions)
        
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
        # # Topics for the robot publisher
        # Get Joint names from the parameter server
        robot_names = ut_param_server.getRobots(self.node)
        self.robot_name = robot_names[0]
        self._joint_control_topic = '/' + self.robot_name + '/control'
        
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(RobotArmTaskEnv, self).__init__()
        
        # Rewards
        self.cumulated_steps = 0.0

    def _init_pose(self): #TODO init pose

        return True


    def _execute_action(self, action):
        self._pub = self.node.create_publisher(JointControl, self._joint_control_topic, qos_profile=qos_profile_sensor_data)

        return True
        

    def _get_obs(self):  #TODOSimplify obersevation
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot2Env API DOCS
        :return:
        """
        return True
        

    def _check_done(self, observations): #TODO check done

        return True

    def _compute_reward(self, observations, done):  #TODO Compute Reward

        '''if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points'''
        reward=0
        #self.cumulated_reward += reward
        #self.cumulated_steps += 1

        return reward
