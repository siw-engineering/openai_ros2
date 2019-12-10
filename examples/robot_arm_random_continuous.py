#!/usr/bin/env python3
""" NOT A REAL TRAINING SCRIPT
Please check the README.md in located in this same folder
for an explanation of this script"""

import gym
import time
from openai_ros2.envs import LobotArmContinuousEnv
from gym.spaces import Box
from typing import Type

env: LobotArmContinuousEnv = gym.make('LobotArmContinuous-v0')
action_space: Type[Box] = env.action_space
env.reset()
while True:
    print("-------------Starting----------------")
    for x in range(500):
        # action = numpy.array([1.00, -1.01, 1.01])
        action = action_space.sample()[:,0]
        observation, reward, done, info = env.step(action)
        # Type hints
        observation: LobotArmContinuousEnv.ObservationData
        reward: float
        done: bool
        info: str
        if done:
            break
    time.sleep(1.0)
    print("-------------Resetting environment---------------")
    env.reset()
    print("-------------Reset finished----------------")
