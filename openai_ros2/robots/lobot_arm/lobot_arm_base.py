import abc

from gazebo_msgs.msg import ContactsState

import numpy

from openai_ros2.utils import ut_param_server

import rclpy
from rclpy.qos import qos_profile_parameters
from rclpy.time import Time as rclpyTime

from ros2_control_interfaces.srv import GetCurrentSimTime

from sensor_msgs.msg import JointState


class LobotArmBase(abc.ABC):
    """Base Lobot Arm"""

    class Observation:
        position_data: numpy.ndarray = numpy.array([0.0, 0.0, 0.0])
        velocity_data: numpy.ndarray = numpy.array([0.0, 0.0, 0.0])
        noiseless_position_data: numpy.ndarray = numpy.array([0.0, 0.0, 0.0])
        noiseless_velocity_data: numpy.ndarray = numpy.array([0.0, 0.0, 0.0])
        contact_count: int = 0
        # the data in this contacts array is an object of gazebo_msgs.msg.ContactState
        contacts: numpy.ndarray = numpy.array([])

    '''-------------PUBLIC METHODS START-------------'''

    def __init__(self, node, state_noise_mu: float = None, state_noise_sigma: float = None):
        self.node: rclpy.Node = node
        self.state_noise_mu = state_noise_mu
        self.state_noise_sigma = state_noise_sigma
        self.__joint_state_sub = self.node.create_subscription(JointState, "/joint_states",
                                                               self.__joint_state_subscription_callback,
                                                               qos_profile=qos_profile_parameters)
        self._latest_joint_state_msg = None
        self._target_joint_state = numpy.array([0.0, 0.0, 0.0])

        self._previous_update_sim_time = rclpyTime()
        self._current_sim_time = rclpyTime()
        self._update_period_ns = 1000000000 / ut_param_server.get_update_rate(self.node)

    def get_observations(self) -> Observation:

        message: JointState = self._latest_joint_state_msg
        obs = LobotArmBase.Observation()
        if message is None:
            print(f"Latest joint state message is None")
            return obs
            
        if not isinstance(message, JointState):
            print(f"Latest joint state message wrong type")
            return obs
        pos_arr = numpy.array(message.position)
        vel_arr = numpy.array(message.velocity)
        obs.noiseless_position_data = pos_arr
        obs.noiseless_velocity_data = vel_arr
        if self.state_noise_sigma is not None and self.state_noise_mu is not None:
            pos_noise = numpy.random.normal(self.state_noise_mu, self.state_noise_sigma, pos_arr.size)
            vel_noise = numpy.random.normal(0.0, self.state_noise_sigma*2, vel_arr.size)
            obs.position_data = pos_arr + pos_noise
            obs.velocity_data = vel_arr + vel_noise
        else:
            obs.position_data = pos_arr
            obs.velocity_data = vel_arr
        contact_msg: ContactsState = self._latest_contact_msg
        if contact_msg is None:
            obs.contact_count = 0
        else:
            obs.contact_count = len(contact_msg.states)
            obs.contacts = numpy.array(contact_msg.states)
        return obs

    @abc.abstractmethod
    def set_action(self, action: numpy.ndarray):
        pass

    @abc.abstractmethod
    def get_action_space(self):
        pass

    @abc.abstractmethod
    def get_observation_space(self):
        pass

    @abc.abstractmethod
    def reset(self) -> None:
        pass

    '''-------------PUBLIC METHODS END-------------'''

    '''-------------PRIVATE METHODS START-------------'''

    def _reset_state(self) -> None:
        self._latest_contact_msg = None
        self._latest_joint_state_msg = None
        self._target_joint_state = numpy.array([0.0, 0.0, 0.0])

        self._current_sim_time = rclpyTime()

    def __joint_state_subscription_callback(self, message: JointState) -> None:
        self._latest_joint_state_msg = message
        latest_msg_time = rclpyTime(seconds=message.header.stamp.sec,
                                    nanoseconds=message.header.stamp.nanosec)
        if self._current_sim_time < latest_msg_time:
            self._current_sim_time = latest_msg_time

    def _get_current_sim_time_from_srv(self) -> rclpyTime:
        client = self.node.create_client(GetCurrentSimTime, "/get_current_sim_time")
        req = GetCurrentSimTime.Request()
        retry_count = 0
        while not client.wait_for_service(timeout_sec=1.0) and retry_count < 10:
            self.node.get_logger().info('/get_current_sim_time service not available, waiting again...')
            retry_count += 1

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            current_sim_time_sec = future.result().sec
            current_sim_time_nsec = future.result().nanosec
            current_sim_time = rclpyTime(seconds=current_sim_time_sec, nanoseconds=current_sim_time_nsec)
            return current_sim_time
        else:
            self.node.get_logger().warn('/get_current_sim_time service call failed')
            return rclpyTime()
