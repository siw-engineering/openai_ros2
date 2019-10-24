#!/usr/bin/env python3
""" NOT A REAL TRAINING SCRIPT
Please check the README.md in located in this same folder
for an explanation of this script"""

import gym
import time
import numpy
import openai_ros2
from openai_ros2.envs.lobot_arm_move_simple import LobotArmMoveSimpleEnv
from openai_ros2.robots.lobot.lobot_arm_sim import LobotArmSim
import rclpy
import random

env: LobotArmMoveSimpleEnv = gym.make('LobotArmMoveSimple-v0')


# def sleep(sec_input: float):
#     current_time_a = time.time()
#     while time.time() - sec_input < current_time_a:
#         rclpy.spin_once(env.node)


rclpy.spin_once(env.node)
env.reset()
while True:
    print("-------------Starting----------------")
    for x in range(3000):
        # action = numpy.array([1.00, -1.01, 1.01])
        action = [random.choice(list(LobotArmSim.Action)) for _ in range(3)]
        observation, reward, done, info = env.step(action)
        current_time = time.time()
        # sleep(0.1)
    # sleep(1.0)
    time.sleep(5.0)
    print("-------------Resetting environment---------------")
    env.reset()
    print("-------------Reset finished----------------")
