import numpy
import math
from openai_ros2.utils import forward_kinematics_py as fk
from openai_ros2.utils import ut_gazebo
from openai_ros2.robots import LobotArmSim
from ament_index_python.packages import get_package_share_directory
import os
import rclpy


class LobotArmFixedGoal:
    def __init__(self, node: rclpy.node.Node, robot, max_time_step: int = 500):
        self.node = node
        self.robot = robot

        # The target coords is currently arbitrarily set to some point achievable
        # This is the target for grip_end_point when target joint values are: [1.00, -1.01, 1.01]
        target_x = 0.10175
        target_y = -0.05533
        target_z = 0.1223
        if isinstance(robot, LobotArmSim):  # Check if is simulator or real
            # Repawn the target marker if it is simulated
            success, status = ut_gazebo.remove_target_marker(node)
            node.get_logger().info(f'Delete marker success: {success}, status: {status}')
            spawn_success = ut_gazebo.spawn_target_marker(node, target_x, target_y, target_z)
        self.target_coords = numpy.array([target_x, target_y, target_z])
        self.previous_coords = numpy.array([0.0, 0.0, 0.0])
        self.__max_time_step = max_time_step
        lobot_desc_share_path = get_package_share_directory('lobot_description')
        arm_urdf_path = os.path.join(lobot_desc_share_path, "robots/arm_standalone.urdf")
        self.__fk = fk.ForwardKinematics(arm_urdf_path)

    def is_done(self, joint_states: numpy.ndarray, contact_count: int, time_step: int = -1) -> bool:
        # If there is any contact (collision), we consider the episode done
        if contact_count > 0:
            return True

        current_coords = self.__get_coords(joint_states)
        accepted_error = 0.001

        # Highest done priority is if time step exceeds limit, so we check this first
        if time_step > self.__max_time_step:
            return True

        # If time step still within limits, as long as any coordinate is out of acceptance range, we are not done
        for i in range(3):
            if abs(self.target_coords[i] - current_coords[i]) > accepted_error:
                return False
        # If all coordinates within acceptance range AND time step within limits, we are done
        print(f"Reached destination, target coords: {self.target_coords}, current coords: {current_coords}")
        return True

    def compute_reward(self, joint_states: numpy.ndarray, time_step: int) -> float:
        if len(joint_states) != 3:
            print(f"Expected 3 values for joint states, but got {len(joint_states)} values instead")
            return -1
        coords_get_result = self.__get_coords(joint_states)
        if len(coords_get_result) != 3:
            print(f"Expected 3 values after getting coordinates, but got {len(coords_get_result)} values instead")
            return -1
        current_coords = coords_get_result

        # Give 0 reward on initial state
        if numpy.array_equal(self.previous_coords, numpy.array([0.0, 0.0, 0.0])):
            # print("Initial state detected, giving 0 reward")
            reward = 0.0
        else:
            reward = self.__calc_dist_change(self.previous_coords, current_coords)
        self.previous_coords = current_coords

        # Scale up reward so that it is not so small
        reward = reward * 100
        return reward

    def reset(self):
        self.previous_coords = numpy.array([0.0, 0.0, 0.0])

    def __calc_dist_change(self, coords_init: numpy.ndarray,
                           coords_next: numpy.ndarray) -> float:
        # Efficient euclidean distance calculation by numpy, most likely uses SIMD
        diff_abs_init = numpy.linalg.norm(coords_init - self.target_coords)
        diff_abs_next = numpy.linalg.norm(coords_next - self.target_coords)

        return diff_abs_init - diff_abs_next

    def __get_coords(self, joint_states: numpy.ndarray) -> numpy.ndarray:
        if len(joint_states) != 3:
            print(f"Expected 3 values for joint states, but got {len(joint_states)} values instead")
            return numpy.array([0.0, 0.0, 0.0])

        res = self.__fk.calculate('world', 'grip_end_point', joint_states)
        return numpy.array([res.translation.x, res.translation.y, res.translation.z])
