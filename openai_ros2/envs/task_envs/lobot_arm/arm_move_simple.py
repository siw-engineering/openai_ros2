from typing import Sequence

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
from arm_fk_cpp.srv import ForwardKinematics


class LobotArmMoveSimpleEnv(LobotArmEnv):

    def __init__(self):
        super(LobotArmMoveSimpleEnv, self).__init__()

        # Basically joint 1,2,3 have upper limits of 135,90,90 degrees respectively
        high = numpy.pi * numpy.array([0.75, 0.5, 0.5])
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
        self.cumulated_reward = 0.0
        # The target coords is currently arbitrarily set to some point achievable
        self.target_coords = numpy.array([-0.098, -0.022, 0.078])
        self.previous_coords = numpy.array([0, 0, 0, 0, 0])
        self.current_coords = numpy.array([0, 0, 0, 0, 0])

        # Get forward kinematics service
        self._fk_service = self.node.create_client(ForwardKinematics, "ArmForwardKine")

    def _init_pose(self):
        raise NotImplementedError("Current Gazebo API is not yet made to support setting initial pose")

    '''
    Function overrides
    '''

    def reset(self) -> None:
        super(LobotArmEnv, self).reset()
        self.cumulated_steps = 0

    def close(self) -> None:
        raise NotImplementedError("Gazebo not launched from this environment, so has no control over the processes")

    # For render to work, the gazebo process must be launched from the environment, which is not the case right now
    # This is mostly due to debugging concerns when it comes to multiprocess causing the debugger to not work correctly
    def render(self, mode='human'):
        raise NotImplementedError("Gazebo not launched from this environment, so has no control over the GUI")

    def _get_observations(self) -> Sequence[float]:

        # with self._lock:
        message: JointState = self._latest_joint_state_msg
        if not isinstance(message, JointState):
            print(f"Latest joint state message wrong type")
            return []
        if message is None:
            print(f"Latest joint state message is None")
            return []

        pos_arr = numpy.array(message.position)
        vel_arr = numpy.array(message.velocity)
        state_arr = numpy.concatenate((pos_arr, vel_arr))

        # self.current_coords = self.__get_coords()
        self.current_coords = self.__get_coords2(message)
        # print(f"coords x diff: {self.current_coords[0] - current_coords2[0]}")
        # print(f"coords y diff: {self.current_coords[1] - current_coords2[1]}")
        # print(f"coords z diff: {self.current_coords[2] - current_coords2[2]}")
        # timeDiff = (self.current_coords[3] - message.header.stamp.sec) * 1000000000 + self.current_coords[
        #     4] - message.header.stamp.nanosec
        # print(f"Time diff: {timeDiff / 1000000}ms")
        return state_arr

    def _get_info(self) -> str:
        return ""
        # raise NotImplementedError("Info not implemented yet")

    def _set_action(self, action: numpy.ndarray):
        # here the action we receive is the same action that is passed to the Step function from gym
        msg = JointControl()
        # TODO: add data to the header of the message, such as time stamp
        if len(self._joint_names) != len(action):
            self.node.get_logger().warn(f"Number of joints don't match number of goals, "
                                        f"n_joints: {len(self._joint_names)}, n_goals: {len(action)}")
            return
        msg.joints = self._joint_names
        msg.goals = action.tolist()
        self._control_pub.publish(msg)

        return True

    # This probably requires some other interface to be exposed by gazebo for collision checking,
    # unless we perform our own collision checking from TF and urdf data which is tedious
    def _is_done(self, observations: Sequence[float]) -> bool:  # TODO check done
        return False
        # raise NotImplementedError(
        #     "Very challenging to check for collisions from joint state data, so not implemented yet")

    def _compute_reward(self, observations, done):

        '''if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points'''
        reward = self.__calc_dist_change(self.previous_coords, self.current_coords)
        self.previous_coords = self.current_coords
        self.cumulated_episode_reward += reward
        self.cumulated_steps += 1
        print(f"Reward for step {self.cumulated_steps}: {reward},\t cumulated reward: {self.cumulated_episode_reward}")

        return reward

    '''
    Function override end
    '''

    def __get_coords(self, time_msg: Time = None) -> Sequence[float]:
        """
        Gets the coordinates of the end effector from the TF buffer
        :param time_msg: defaults no None, if None then get latest coords
        :return:
        """
        from_frame = 'arm_3_link'
        to_frame = 'world'
        # When time is set to 0 it will get the latest transform
        if time_msg is None:
            # This should get time = 0
            tf2_time = rclpy.time.Time()
        else:
            time_nsec = time_msg.sec * 1000000000 + time_msg.nanosec
            tf2_time = tf2_ros.Time(nanoseconds=time_nsec)
        current_pos = None
        try:
            current_pos = self._tf_buffer.lookup_transform(to_frame, from_frame, tf2_time,
                                                           timeout=Duration(nanoseconds=1000000000))
        except Exception as e:
            self.node.get_logger().warn('failed to get transform {}'.format(repr(e)))

        # if fail to get coords, fallback to the previous coordinates data
        if current_pos is None:
            return self.previous_coords

        translation = current_pos.transform.translation
        x = translation.x
        y = translation.y
        z = translation.z
        sec = current_pos.header.stamp.sec
        nsec = current_pos.header.stamp.nanosec
        print(f'get coords called with coords: [{x}, {y}, {z}]')
        return [x, y, z, sec, nsec]

    def __get_coords2(self, joint_state_msg: JointState = None) -> Sequence[float]:
        req = ForwardKinematics.Request()
        req.joint_states = joint_state_msg.position.tolist()
        future = self._fk_service.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            fk_response: ForwardKinematics.Response = future.result()
            self.node.get_logger().info(f'FK successful: {fk_response.success}')
            if fk_response.success:
                return [fk_response.x, fk_response.y, fk_response.z]
            else:
                return []
        else:
            self.node.get_logger().info('Service call failed %r' % (future.exception(),))

    def __calc_dist_change(self, coords_init: Sequence[float],
                           coords_next: Sequence[float]) -> float:
        diff_x_init = coords_init[0] - self.target_coords[0]
        diff_y_init = coords_init[1] - self.target_coords[1]
        diff_z_init = coords_init[2] - self.target_coords[2]
        diff_abs_init = (diff_x_init ** 2 + diff_y_init ** 2 + diff_z_init ** 2) ** 0.5

        diff_x_next = coords_next[0] - self.target_coords[0]
        diff_y_next = coords_next[1] - self.target_coords[1]
        diff_z_next = coords_next[2] - self.target_coords[2]
        diff_abs_next = (diff_x_next ** 2 + diff_y_next ** 2 + diff_z_next ** 2) ** 0.5

        return diff_abs_init - diff_abs_next
