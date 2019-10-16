from collections import Sequence, Iterable

import numpy
import rclpy
from gym import spaces
from rclpy.qos import qos_profile_sensor_data
from ros2_control_interfaces.msg import JointControl
from sensor_msgs.msg import JointState

from openai_ros2.envs.robot_envs.lobot_arm import LobotArmEnv
from openai_ros2.utils import ut_param_server
import tf2_ros
from builtin_interfaces.msg import Time
from rclpy.duration import Duration


class LobotArmMoveSimpleEnv(LobotArmEnv):

    def __init__(self):
        super(LobotArmMoveSimpleEnv, self).__init__()

        # Basically joint 1,2,3 have upper limits of 135,90,90 degrees respectively
        high = numpy.pi * [0.75, 0.5, 0.5]
        # Lower limit is just negative of upper limit, symmetrical about 0
        low = high * -1
        self.action_space = spaces.Box(low, high)

        # Get robot name from the parameter server and create the publisher
        robot_names = ut_param_server.get_robots(self.node)
        self.robot_name = robot_names[0]
        joint_control_topic = '/' + self.robot_name + '/control'
        self._control_pub = self.node.create_publisher(JointControl, joint_control_topic, qos_profile_sensor_data)

        # Get the joint names from parameter server
        self._joint_names = ut_param_server.get_joints(self.node, self.robot_name)

        # Here we will add any init functions prior to starting the MyRobotEnv

        # Rewards
        self.cumulated_steps = 0.0
        # The target coords is currently arbitrarily set to some point achievable
        self.target_coords = [-0.098, -0.022, 0.078]
        self.previous_coords = [0, 0, 0]
        self.current_coords = [0, 0, 0]

    def _init_pose(self):
        raise NotImplementedError("Current Gazebo API is not yet made to support setting initial pose")

    def _set_action(self, action: Sequence[float]):
        # here the action we receive is the same action that is passed to the Step function from gym
        msg = JointControl()
        # TODO: add data to the header of the message, such as time stamp
        if len(self._joint_names) != len(msg.goals):
            self.node.get_logger().warn(f"Number of joints don't match number of goals, "
                                        f"n_joints: {len(self._joint_names)}, n_goals: {len(msg.goals)}")
            return
        msg.joints = self._joint_names
        msg.goals = action
        self._control_pub.publish(msg)

        return True

    # This probably requires some other interface to be exposed by gazebo for collision checking,
    # unless we perform our own collision checking from TF and urdf data which is tedious
    def _check_done(self, observations) -> bool:  # TODO check done
        raise NotImplementedError(
            "Very challenging to check for collisions from joint state data, so not implemented yet")

    def _compute_reward(self, observations, done):

        '''if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points'''
        reward = self.__calc_dist_change(self.previous_coords, self.current_coords)
        # self.cumulated_reward += reward
        # self.cumulated_steps += 1

        return reward

    def close(self) -> None:
        raise NotImplementedError("Gazebo not launched from this environment, so has no control over the processes")

    # For render to work, the gazebo process must be launched from the environment, which is not the case right now
    # This is mostly due to debugging concerns when it comes to multiprocess causing the debugger to not work correctly
    def render(self, mode='human'):
        raise NotImplementedError("Gazebo not launched from this environment, so has no control over the GUI")

    async def _get_observations(self) -> Sequence[float]:
        if self._latest_joint_state_msg is not None and isinstance(JointState, self._latest_joint_state_msg):
            message: JointState = self._latest_joint_state_msg
            pos_arr = numpy.array(message.position)
            vel_arr = numpy.array(message.velocity)
            state_arr = numpy.concatenate((pos_arr, vel_arr))

            self.current_coords = self.__get_coords()
            return state_arr

    def _get_info(self) -> str:
        raise NotImplementedError("Info not implemented yet")

    def _is_done(self, observations: Sequence[float]) -> bool:
        pass

    def __get_coords(self, time_msg: Time = None) -> Iterable[float, float, float]:
        """
        Gets the coordinates of the end effector from the TF buffer
        :param time_msg: defaults no None, if None then get latest coords
        :return:
        """
        from_frame = 'grip_link_3'
        to_frame = 'world'

        # When time is set to 0 it will get the latest transform
        if time_msg is None:
            tf2_time = rclpy.time.Time()
        else:
            time_nsec = time_msg.sec * 1000000000 + time_msg.nanosec
            tf2_time = tf2_ros.Time(nanoseconds=time_nsec)
        current_pos = None
        try:
            current_pos = self._tf_buffer.lookup_transform(to_frame, from_frame, tf2_time, timeout=Duration(seconds=0.02))
        except Exception as e:
            self.node.get_logger().warn('failed to get transform {}'.format(repr(e)))

        # if fail to get coords, fallback to the previous coordinates data
        if current_pos is None:
            return self.previous_coords

        translation = current_pos.transform.translation
        x = translation.x
        y = translation.y
        z = translation.z
        return [x, y, z]

    def __calc_dist_change(self, coords_init: Iterable[float, float, float],
                           coords_next: Iterable[float, float, float]) -> float:
        diff_x_init = coords_init[0] - self.target_coords[0]
        diff_y_init = coords_init[1] - self.target_coords[1]
        diff_z_init = coords_init[2] - self.target_coords[2]
        diff_abs_init = (diff_x_init ** 2 + diff_y_init ** 2 + diff_z_init ** 2) ** 0.5

        diff_x_next = coords_next[0] - self.target_coords[0]
        diff_y_next = coords_next[1] - self.target_coords[1]
        diff_z_next = coords_next[2] - self.target_coords[2]
        diff_abs_next = (diff_x_next ** 2 + diff_y_next ** 2 + diff_z_next ** 2) ** 0.5

        return diff_abs_init-diff_abs_next
