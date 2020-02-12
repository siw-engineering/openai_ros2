import random
from collections import deque
from typing import Dict, Tuple
from enum import Enum, auto
import numpy
import forward_kinematics_py as fk
from openai_ros2.utils import ut_launch, ut_gazebo
from openai_ros2.robots import LobotArmSim
from ament_index_python.packages import get_package_share_directory
from gym.spaces import Box
import os
import rclpy


class ArmState(Enum):
    Reached = auto()
    InProgress = auto()
    ApproachJointLimits = auto()
    Collision = auto()
    Timeout = auto()
    Undefined = auto()

class LobotArmRandomGoal:
    def __init__(self, node: rclpy.node.Node, robot, task_kwargs: Dict = None, max_time_step: int = 500):
        if task_kwargs is None:
            task_kwargs = {}
        self.node = node
        self.robot = robot
        self.accepted_dist_to_bounds = task_kwargs.get('accepted_dist_to_bounds', 0.001)
        self.accepted_error = task_kwargs.get('accepted_error', 0.001)
        self.reach_target_bonus_reward = task_kwargs.get('reach_target_bonus_reward', 0.0)
        self.reach_bounds_penalty = task_kwargs.get('reach_bounds_penalty', 0.0)
        self.contact_penalty = task_kwargs.get('contact_penalty', 0.0)
        self.episodes_per_goal = task_kwargs.get('episodes_per_goal', 1)
        self.goal_buffer_size = task_kwargs.get('goal_buffer_size', 20)
        self.goal_from_buffer_prob = task_kwargs.get('goal_from_buffer_prob', 0.0)
        self.num_adjacent_goals = task_kwargs.get('num_adjacent_goals', 0)
        self.is_validation = task_kwargs.get('is_validation', False)
        self.random_goal_seed = task_kwargs.get('random_goal_seed', None)
        self.normalise_reward = task_kwargs.get('normalise_reward', False)
        print(f'-------------------------------Setting task parameters-------------------------------')
        print('accepted_dist_to_bounds: %f   # Allowable distance to joint limits' % self.accepted_dist_to_bounds)
        print('accepted_error: %f            # Allowable distance from target coordinates' % self.accepted_error)
        print('reach_target_bonus_reward: %f # Bonus reward upon reaching target' % self.reach_target_bonus_reward)
        print('reach_bounds_penalty: %f      # Reward penalty when reaching joint limit' % self.reach_bounds_penalty)
        print('contact_penalty: %f           # Reward penalty for collision' % self.contact_penalty)
        print('episodes_per_goal: %d         # Number of episodes before generating another random goal' % self.episodes_per_goal)
        print('goal_buffer_size: %d          # Number goals to store in buffer to be reused later' % self.goal_buffer_size)
        print('goal_from_buffer_prob: %f     # Probability of selecting a random goal from the goal buffer, value between 0 and 1' % self.goal_from_buffer_prob)
        print('num_adjacent_goals: %d        # Number of nearby goals to be generated for each randomly generated goal ' % self.num_adjacent_goals)
        print(f'random_goal_seed: {self.random_goal_seed}          # Seed used to generate the random goals')
        print('is_validation: %r             # Whether this is a validation run, if true will print which points failed and how many reached' % self.is_validation)
        print('normalise_reward: %r          # Perform reward normalisation, this happens before reward bonus and penalties' % self.normalise_reward)
        print(f'-------------------------------------------------------------------------------------')

        assert self.accepted_dist_to_bounds >= 0.0, 'Allowable distance to joint limits should be positive'
        assert self.accepted_error >= 0.0, 'Accepted error to end coordinates should be positive'
        assert self.reach_target_bonus_reward >= 0.0, 'Reach target bonus reward should be positive'
        assert self.contact_penalty >= 0.0, 'Contact penalty should be positive'
        assert self.reach_bounds_penalty >= 0.0, 'Reach bounds penalty should be positive'
        assert isinstance(self.episodes_per_goal, int), f'Episodes per goal should be an integer, current type: {type(self.episodes_per_goal)}'
        assert self.episodes_per_goal >= 1, 'Episodes per goal be greather than or equal to 1, i.e. episodes_per_goal >= 1'
        assert isinstance(self.goal_buffer_size, int), f'Goal buffer size should be an integer, current type: {type(self.goal_buffer_size)}'
        assert self.goal_buffer_size > 0, 'Goal buffer size should be greather than or equal to 1, i.e. episodes_per_goal >= 1'
        assert 0 <= self.goal_from_buffer_prob <= 1, 'Probability of selecting goal from buffer should be between 0 and 1'
        assert isinstance(self.num_adjacent_goals, int), f'Number of adjacent goals should be an integer, current type: {type(self.num_adjacent_goals)}'
        assert self.num_adjacent_goals >= 0, f'Number of adjacent goals should be positive, current value: {self.num_adjacent_goals}'
        if self.random_goal_seed is not None:
            assert isinstance(self.random_goal_seed, int), f'Random goal seed should be an, current type: {type(self.random_goal_seed)}'

        self._max_time_step = max_time_step
        lobot_desc_share_path = get_package_share_directory('lobot_description')
        arm_urdf_path = os.path.join(lobot_desc_share_path, 'robots/arm_standalone.urdf')
        self._fk = fk.ForwardKinematics(arm_urdf_path)

        self.coords_buffer = deque(maxlen=self.goal_buffer_size)
        self.target_coords_ik, self.target_coords = self.__get_target_coords()
        target_x = self.target_coords[0]
        target_y = self.target_coords[1]
        target_z = self.target_coords[2]
        if isinstance(robot, LobotArmSim):  # Check if is gazebo
            # Spawn the target marker if it is gazebo
            print(f'Spawning to: {(target_x, target_y, target_z)}')
            spawn_success = ut_gazebo.create_marker(node, target_x, target_y, target_z, diameter=0.004)
        self.previous_coords = numpy.array([0.0, 0.0, 0.0])
        self.__reset_count: int = 0
        self.fail_points = []
        self.__reach_count: int = 0

    def __del__(self):
        print('Arm random goal destructor called')
        if self.is_validation:
            # If fail_points is not defined yet, do nothing
            if not hasattr(self, 'fail_points'):
                return

            def print_list(list_):
                for item in list_:
                    print(item)
            print('Failed goals, target coords: ')
            print_list(enumerate([x for x, y in self.fail_points]))
            k = 100
            ut_gazebo.create_marker(self.node, 0.0, 0.0, 0.0, diameter=0.004)
            for x, y in self.fail_points:
                ut_gazebo.create_marker(self.node, x[0], x[1], x[2], diameter=0.001, id=k)
                k += 1

    def is_done(self, joint_states: numpy.ndarray, contact_count: int, observation_space: Box, time_step: int = -1) -> Tuple[bool, Dict]:
        failed, arm_state = self.__is_failed(joint_states, contact_count, observation_space, time_step)
        info_dict = {'arm_state': arm_state}
        if failed:
            self.fail_points.append((self.target_coords, self.target_coords_ik))
            if self.is_validation:
                print(f'Failed to reach {self.target_coords}')
            return True, info_dict

        current_coords = self.__get_coords(joint_states)
        # If time step still within limits, as long as any coordinate is out of acceptance range, we are not done
        for i in range(3):
            if abs(self.target_coords[i] - current_coords[i]) > self.accepted_error:
                info_dict['arm_state'] = ArmState.InProgress
                return False, info_dict
        # If all coordinates within acceptance range AND time step within limits, we are done
        info_dict['arm_state'] = ArmState.Reached
        print(f'Reached destination, target coords: {self.target_coords}, current coords: {current_coords}')
        self.__reach_count += 1
        if self.is_validation:
            print(f'Reach count: {self.__reach_count}')
        return True, info_dict

    def compute_reward(self, joint_states: numpy.ndarray, arm_state: ArmState) -> Tuple[float, Dict]:
        assert len(joint_states) == 3, f'Expected 3 values for joint states, but got {len(joint_states)} values instead'
        coords_get_result = self.__get_coords(joint_states)
        assert len(coords_get_result) == 3, f'Expected 3 values after getting coordinates, but got {len(coords_get_result)} values instead'
        current_coords = coords_get_result

        assert arm_state != ArmState.Undefined, f'Arm state cannot be undefined, please check logic'

        # Give 0 reward on initial state
        if numpy.array_equal(self.previous_coords, numpy.array([0.0, 0.0, 0.0])):
            # print('Initial state detected, giving 0 reward')
            reward = 0.0
        else:
            reward = self.__calc_dist_change(self.previous_coords, current_coords)

        # normalise rewards
        mag_target = numpy.linalg.norm(self.target_coords)
        normalised_reward = reward / mag_target

        # Scale up normalised reward slightly such that the total reward is between 0 and 10 instead of between 0 and 1
        normalised_reward *= 10

        # Scale up reward so that it is not so small if not normalised
        normal_scaled_reward = reward * 100

        # Calculate current distance to goal (for information purposes only)
        dist = numpy.linalg.norm(current_coords - self.target_coords)

        reward_info = {'normalised_reward': normalised_reward,
                       'normal_reward': normal_scaled_reward,
                       'distance_to_goal': dist,
                       'current_goal': self.target_coords}

        if self.normalise_reward:
            reward = normalised_reward
        else:
            reward = normal_scaled_reward

        self.previous_coords = current_coords

        # Reward shaping logic

        # Check if it has reached target destination
        if arm_state == ArmState.Reached:
            reward += self.reach_target_bonus_reward

        # Check if it has approached any joint limits
        if arm_state == ArmState.ApproachJointLimits:
            reward -= self.reach_bounds_penalty

        # Check for collision
        if arm_state == ArmState.Collision:
            reward -= self.contact_penalty

        return reward, reward_info

    def reset(self):
        self.previous_coords = numpy.array([0.0, 0.0, 0.0])
        self.__reset_count += 1
        if self.__reset_count % self.episodes_per_goal == 0:
            self.target_coords_ik, self.target_coords = self.__get_target_coords()
            if isinstance(self.robot, LobotArmSim):  # Check if is simulator or real
                # Move the target marker if it is gazebo
                print(f'Moving to [{self.target_coords[0]:.6f}, {self.target_coords[1]:.6f}, {self.target_coords[2]:.6f}], '
                      f'Given by joint values [{self.target_coords_ik[0]:.6f}, {self.target_coords_ik[1]:.6f}, {self.target_coords_ik[2]:.6f}]')
                spawn_success = ut_gazebo.create_marker(self.node, self.target_coords[0],
                                                        self.target_coords[1], self.target_coords[2], diameter=0.004)

    def __is_failed(self, joint_states: numpy.ndarray, contact_count: int, observation_space: Box, time_step: int = -1) -> Tuple[bool, ArmState]:
        info_dict = {'arm_state': ArmState.Undefined}
        # If there is any contact (collision), we consider the episode done
        if contact_count > 0:
            return True, ArmState.Collision

        # Check if time step exceeds limits, i.e. timed out
        if time_step > self._max_time_step:
            return True, ArmState.Timeout

        # Check that joint values are not approaching limits
        upper_bound = observation_space.high[:3]  # First 3 values are the joint states
        lower_bound = observation_space.low[:3]
        min_dist_to_upper_bound = min(abs(joint_states - upper_bound))
        min_dist_to_lower_bound = min(abs(joint_states - lower_bound))
        # self.accepted_dist_to_bounds is basically how close to the joint limits can the joints go,
        # i.e. limit of 1.57 with accepted dist of 0.1, then the joint can only go until 1.47
        if min_dist_to_lower_bound < self.accepted_dist_to_bounds:
            joint_index = abs(joint_states - lower_bound).argmin()
            print(f'Joint {joint_index} approach joint limits, '
                  f'current joint value: {joint_states[joint_index]}, '
                  f'minimum joint value: {lower_bound[joint_index]}')
            return True, ArmState.ApproachJointLimits
        if min_dist_to_upper_bound < self.accepted_dist_to_bounds:
            joint_index = abs(joint_states - upper_bound).argmin()
            print(f'Joint {joint_index} approach joint limits, '
                  f'current joint value: {joint_states[joint_index]}, '
                  f'maximum joint value: {upper_bound[joint_index]}')
            info_dict['arm_state'] = ArmState.ApproachJointLimits
            return True, ArmState.ApproachJointLimits
        # Didn't fail
        return False, ArmState.Undefined

    def __calc_dist_change(self, coords_init: numpy.ndarray,
                           coords_next: numpy.ndarray) -> float:
        # Efficient euclidean distance calculation by numpy, most likely uses vector instructions
        diff_abs_init = numpy.linalg.norm(coords_init - self.target_coords)
        diff_abs_next = numpy.linalg.norm(coords_next - self.target_coords)

        return diff_abs_init - diff_abs_next

    def __get_coords(self, joint_states: numpy.ndarray) -> numpy.ndarray:
        if len(joint_states) != 3:
            print(f'Expected 3 values for joint states, but got {len(joint_states)} values instead')
            return numpy.array([0.0, 0.0, 0.0])

        res = self._fk.calculate('world', 'grip_end_point', joint_states)
        return numpy.array([res.translation.x, res.translation.y, res.translation.z])

    def __generate_target_coords(self) -> Tuple[numpy.ndarray, numpy.ndarray]:
        if self.random_goal_seed is not None:
            self.__seed_numpy()

        while True:
            random_joint_values = numpy.random.uniform([-2.3562, -1.5708, -1.5708], [2.3562, 0.5, 1.5708])
            res = self._fk.calculate('world', 'grip_end_point', random_joint_values)
            if res.translation.z > 0.0:
                break
        target_coords = numpy.array([res.translation.x, res.translation.y, res.translation.z])
        self.np_random_state = numpy.random.get_state()
        return random_joint_values, target_coords

    def __generate_adjacent_coords(self, joint_values: numpy.ndarray):
        if self.random_goal_seed is not None:
            self.__seed_numpy()

        while True:
            rand_range = numpy.array([1.0, 1.0, 1.0]) * 0.1
            random_addition = numpy.random.uniform(-rand_range, rand_range)
            joint_values_adjacent = joint_values + random_addition
            joint_values_adjacent = joint_values_adjacent.clip([-2.356194, -1.570796, -1.570796], [2.356194, 0.5, 1.570796])
            res = self._fk.calculate('world', 'grip_end_point', joint_values_adjacent)
            if res.translation.z > 0.0:
                break
        target_coords = numpy.array([res.translation.x, res.translation.y, res.translation.z])
        self.np_random_state = numpy.random.get_state()
        return joint_values_adjacent, target_coords

    def __get_target_coords(self) -> Tuple[numpy.ndarray, numpy.ndarray]:
        rand_num = numpy.random.rand()
        # if probability to choose from buffer is very high, i.e. p > 0.99, we make sure buffer is filled first before choosing from buffer
        high_prob_but_buffer_unfilled = self.goal_from_buffer_prob > 0.99 and len(self.coords_buffer) < self.coords_buffer.maxlen
        select_from_buffer = rand_num < self.goal_from_buffer_prob
        buffer_is_empty = len(self.coords_buffer) == 0

        if buffer_is_empty or high_prob_but_buffer_unfilled or not select_from_buffer:
            # Generate random coords and store
            random_joint_values, target_coords = self.__generate_target_coords()
            # Store into buffer, since it is a deque with maxlen configured, it will auto pop, no need to manual pop
            self.coords_buffer.append((random_joint_values, target_coords))
            for _ in range(self.num_adjacent_goals):
                joint_vals, coords = self.__generate_adjacent_coords(random_joint_values)
                self.coords_buffer.append((joint_vals, coords))
            return random_joint_values, target_coords

        if len(self.coords_buffer) == self.coords_buffer.maxlen and not hasattr(self, 'first_time_printing_coords'):
            # Use this attribute to determine if we have printed coords or not, if no such attribute means first time printing
            self.first_time_printing_coords = True
            def print_list(list_):
                for item in list_:
                    print(item)
            print('Using target coords: ')
            print_list(enumerate([y for x, y in self.coords_buffer]))

        random_joint_values, target_coords = random.choice(self.coords_buffer)
        return random_joint_values, target_coords

    def __seed_numpy(self):
        # Properly seed the numpy RNG if a random seed is given
        # The set state and get_state is such that this generator function always returns the same set of values given the same seed
        # This is regardless of how many random calls are used in between
        if hasattr(self, 'np_random_state'):
            numpy.random.set_state(self.np_random_state)
        else:
            numpy.random.seed(self.random_goal_seed)
