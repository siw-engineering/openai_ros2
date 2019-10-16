from abc import ABC

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from openai_ros2.envs.robot_gazebo_env import RobotGazeboEnv
from openai_ros2.utils.lobot_arm_helper import LobotArmHelper


class LobotArmEnv(RobotGazeboEnv, ABC):
    def __init__(self):
        # Initialises the gazebo connections and the node from the base class
        super(LobotArmEnv, self).__init__()
        self.node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self._joint_state_sub = self.node.create_subscription(JointState, "/joint_states",
                                                              self._joint_state_subscription_callback,
                                                              qos_profile_sensor_data)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.gazebo.pause_sim()
        self._latest_joint_state_msg = None

    def _joint_state_subscription_callback(self, message: JointState) -> None:
        self._latest_joint_state_msg = message
        self._latest_joint_state_time = LobotArmHelper.get_time_from_time_msg(message.header.stamp)

    def _reset_controller(self) -> None:
        while not self._robot_resetter.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('/' + self.robot_name + '/reset service not available, waiting again...')
        reset_robot_future = self._robot_resetter.call_async(Empty.Request())
        print("Resetting controller to initial positions")
        rclpy.spin_until_future_complete(self.node, reset_robot_future)
