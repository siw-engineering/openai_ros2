import rclpy
from openai_ros2.utils.gazebo_connection import GazeboConnection
from openai_ros2.utils import ut_param_server
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import numpy
from typing import Sequence, Type
from enum import Enum
from ros2_control_interfaces.msg import JointControl
from ros2_control_interfaces.srv import GetCurrentSimTime
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameters
from rclpy.time import Time as rclpyTime
import threading
import copy
import time


class LobotArmSim:
    class Action(Enum):
        BigPositive = 0.02
        SmallPositive = 0.005
        Remain = 0
        SmallNegative = -0.005
        BigNegative = -0.02

    '''-------------PUBLIC METHODS START-------------'''

    def __init__(self, node):
        self.node = node

        # Get robot name from parameter server, this is to ensure that the gazebo plugin subscribing to the control
        # reads the same name as this code, because the topic depends on the robot name
        robot_names = ut_param_server.get_robots(self.node)
        self.robot_name = robot_names[0]
        self.__joint_names = ut_param_server.get_joints(self.node, self.robot_name)
        joint_control_topic = '/' + self.robot_name + '/control'
        self.__control_pub = self.node.create_publisher(JointControl, joint_control_topic, qos_profile_sensor_data)
        self.__joint_state_sub = self.node.create_subscription(JointState, "/joint_states",
                                                               self.__joint_state_subscription_callback,
                                                               qos_profile=qos_profile_parameters)
        self.__latest_joint_state_msg = None
        self.__target_joint_state = numpy.array([0.0, 0.0, 0.0])

        self.__previous_update_sim_time = rclpyTime()
        self.__current_sim_time = rclpyTime()
        self.__update_period_ns = 1000000000 / ut_param_server.get_update_rate(self.node)

        # Create reset client to call the reset service
        self.__gazebo = GazeboConnection(self.node)
        self.__time_lock = threading.Lock()

    def set_action(self, action: Sequence[Action]) -> None:
        """
        Sets the action, unpauses the simulation and then waits until the update period of openai gym is over.
        The simulation is also expected to pause at the same time.
        This is to create a deterministic time step for the gym environment such that the agent can properly evaluate
        each action
        :param action:
        :return: obs, reward, done, info
        """
        if len(action) != 3:
            print(f"{len(action)} actions passed to LobotArmSim, expected: 3")

        action_values = [x.value for x in action]
        self.__target_joint_state += action_values

        msg = JointControl()
        msg.joints = self.__joint_names
        msg.goals = self.__target_joint_state.tolist()
        msg.header.stamp = self.__current_sim_time.to_msg()
        self.__control_pub.publish(msg)
        # Assume the simulation is paused due to the gym training plugin when set_action is called
        self.__gazebo.unpause_sim()
        self.__spin_until_update_period_over()

    def reset(self) -> None:
        self.__gazebo.pause_sim()
        self.__gazebo.reset_sim()
        self.__reset_controller()
        self.__reset_state()
        # No unpause here because it is assumed that the set_action will unpause it

    def get_observations(self) -> numpy.ndarray:

        message: JointState = self.__latest_joint_state_msg
        if not isinstance(message, JointState):
            print(f"Latest joint state message wrong type")
            return numpy.array([])
        if message is None:
            print(f"Latest joint state message is None")
            return numpy.array([])

        pos_arr = numpy.array(message.position)
        vel_arr = numpy.array(message.velocity)
        state_arr = numpy.concatenate((pos_arr, vel_arr))

        return state_arr

    '''-------------PUBLIC METHODS END-------------'''

    '''-------------PRIVATE METHODS START-------------'''

    def __reset_controller(self) -> None:
        reset_client = self.node.create_client(Empty, '/' + self.robot_name + '/reset')
        while not reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/' + self.robot_name + '/reset service not available, waiting again...')
        reset_robot_future = reset_client.call_async(Empty.Request())
        print("Resetting controller to initial positions")
        rclpy.spin_until_future_complete(self.node, reset_robot_future)

    def __reset_state(self) -> None:
        self.__latest_joint_state_msg = None
        self.__target_joint_state = numpy.array([0.0, 0.0, 0.0])

        self.__previous_update_sim_time = rclpyTime()
        self.__current_sim_time = rclpyTime()

    def __joint_state_subscription_callback(self, message: JointState) -> None:
        self.__latest_joint_state_msg = message
        with self.__time_lock:
            self.__current_sim_time = rclpyTime(seconds=message.header.stamp.sec,
                                               nanoseconds=message.header.stamp.nanosec)

    def __spin_until_update_period_over(self) -> None:
        # Loop to block such that when we take observation it is the latest observation when the
        # simulation is paused due to the gym training plugin
        # Also have a timeout such that when the loop gets stuck it will break itself out
        timeout_duration = 0.3
        loop_start_time = time.time()
        while True:
            # spinning the node will cause self.__current_sim_time to be updated
            rclpy.spin_once(self.node, timeout_sec=0.3)

            with self.__time_lock:
                current_sim_time = copy.copy(self.__current_sim_time)
            time_diff_ns = current_sim_time.nanoseconds - self.__previous_update_sim_time.nanoseconds
            if time_diff_ns < 0:
                print("Negative time difference detected, probably due to a reset")
                self.__previous_update_sim_time = rclpyTime()
                time_diff_ns = current_sim_time.nanoseconds
            loop_end_time = time.time()
            loop_duration = loop_end_time - loop_start_time

            if time_diff_ns >= self.__update_period_ns or loop_duration > timeout_duration:
                break

        if loop_duration >= timeout_duration:
            self.node.get_logger().warn(f"Wait for simulation loop timeout, getting time from service instead")
            current_sim_time = self.__get_current_sim_time_from_srv()
        self.__previous_update_sim_time = current_sim_time

    def __get_current_sim_time_from_srv(self) -> rclpyTime:
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