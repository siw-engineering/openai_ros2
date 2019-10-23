#!/usr/bin/env python3
""" NOT A REAL TRAINING SCRIPT
Please check the README.md in located in this same folder
for an explanation of this script"""

import gym
import time
import openai_ros2
from openai_ros2.envs.task_envs.lobot_arm.arm_move_simple import LobotArmMoveSimpleEnv
import rclpy

env: LobotArmMoveSimpleEnv = gym.make('LobotArmMoveSimple-v0')


# def sleep(sec_input: float):
#     current_time_a = time.time()
#     while time.time() - sec_input < current_time_a:
#         rclpy.spin_once(env.node)


rclpy.spin_once(env.node)
env.reset()
while True:
    print("-------------Starting----------------")
    for x in range(300):
        observation, reward, done, info = env.step(env.action_space.sample())
        current_time = time.time()
        # sleep(0.1)
    # sleep(1.0)
    print("-------------Resetting environment---------------")
    env.reset()
    print("-------------Reset finished----------------")
    time.sleep(2.0)
