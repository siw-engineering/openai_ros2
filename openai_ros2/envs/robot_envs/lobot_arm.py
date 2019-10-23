import abc
from typing import Sequence

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from openai_ros2.envs.robot_gazebo_env import RobotGazeboEnv
from openai_ros2.utils import ut_param_server
from openai_ros2.utils.lobot_arm_helper import LobotArmHelper
import threading
import time
from rclpy.time import Time as rclpyTime


class LobotArmEnv(RobotGazeboEnv):

    def __init__(self):
        # Initialises the gazebo connections and the node from the base class
        super(LobotArmEnv, self).__init__()
        self.node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self._joint_state_sub = self.node.create_subscription(JointState, "/joint_states",
                                                              self._joint_state_subscription_callback,
                                                              qos_profile=qos_profile_parameters)
        self.gazebo.pause_sim()
        self._latest_joint_state_msg = None

        # Set update period and related variables
        self._time_lock = threading.Lock()
        self._update_period_ns = 1000000000 / ut_param_server.get_update_rate(self.node)
        self.node.get_logger().info(f"Gym update period set to {self._update_period_ns}ns")
        self._previous_update_sim_time: rclpyTime = rclpyTime()
        self._current_sim_time: rclpyTime = rclpyTime()

    def reset(self) -> None:
        super(RobotGazeboEnv, self).reset()
        self._latest_joint_state_msg = None

    def _joint_state_subscription_callback(self, message: JointState) -> None:
        self._latest_joint_state_msg = message
        with self._time_lock:
            self._current_sim_time = rclpyTime(seconds=message.header.stamp.sec, nanoseconds=message.header.stamp.nanosec)
        # print("Joint state message received")

    def _reset_controller(self) -> None:
        while not self._robot_resetter.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/' + self.robot_name + '/reset service not available, waiting again...')
        reset_robot_future = self._robot_resetter.call_async(Empty.Request())
        print("Resetting controller to initial positions")
        rclpy.spin_until_future_complete(self.node, reset_robot_future)

    @abc.abstractmethod
    def close(self) -> None:
        pass

    @abc.abstractmethod
    def render(self, mode='human') -> None:
        pass

    @abc.abstractmethod
    def _get_observations(self) -> Sequence[float]:
        pass

    @abc.abstractmethod
    def _get_info(self) -> str:
        pass

    @abc.abstractmethod
    def _set_action(self, action: Sequence[float]) -> None:
        pass

    @abc.abstractmethod
    def _is_done(self, observations: Sequence[float]) -> bool:
        pass

    @abc.abstractmethod
    def _compute_reward(self, observations: Sequence[float], done: bool):
        pass
