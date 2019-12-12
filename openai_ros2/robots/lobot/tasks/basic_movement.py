import numpy
import math
from openai_ros2.utils import forward_kinematics_py as fk
from ament_index_python.packages import get_package_share_directory
import os


class LobotArmBasicMovement:
    def __init__(self, node, max_time_step: int = 500):
        self.node = node

        # This time decay constant makes it such that rewards at later time steps are diminished
        # This is based on the exponential decay equation e^(-lambda * t), in this case t is time step
        # At lambda = -0.005,time_step = 100, proportion of reward = 0.6065
        # At time step = 200, proportion of reward = 0.36788
        self.time_decay_constant = -0.005

        # The target coords is currently arbitrarily set to some point achievable
        # This is the target for arm_3_link when target joint values are: [1.00, -1.01, 1.01]
        self.target_coords = numpy.array([0.026975, -0.007283, 0.132731])
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
        acceptedError = 0.01

        # Highest done priority is if time step exceeds limit, so we check this first
        if time_step > self.__max_time_step:
            return True

        # If time step still within limits, as long as any coordinate is out of acceptance range, we are not done
        for i in range(3):
            if abs(self.target_coords[i] - current_coords[i]) > acceptedError:
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
            print("Initial state detected, giving 0 reward")
            reward = 0
        else:
            reward = self.__calc_dist_change(self.previous_coords, current_coords)
        self.previous_coords = current_coords

        # Apply time decay to reward, also scale up reward so that it is not so small
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
            return numpy.array([0, 0, 0])

        # req = ForwardKinematics.Request()
        # req.joint_states = joint_states.tolist()

        res = self.__fk.calculate('world', 'arm_3_link', joint_states)
        return numpy.array([res.translation.x, res.translation.y, res.translation.z])
        # future = self.__fk_service.call_async(req)
        # rclpy.spin_until_future_complete(self.node, future)
        # if future.result() is not None:
        #     fk_response: ForwardKinematics.Response = future.result()
        #     # self.node.get_logger().info(f'FK successful: {fk_response.success}')
        #     if fk_response.success:
        #         return numpy.array([fk_response.x, fk_response.y, fk_response.z])
        #     else:
        #         return numpy.array([])
        # else:
        #     self.node.get_logger().info('Service call failed %r' % (future.exception(),))
