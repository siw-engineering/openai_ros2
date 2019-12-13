#!/usr/bin/env python3

""" NOT A REAL TRAINING SCRIPT
Please check the README.md in located in this same folder
for an explanation of this script"""

import gym
import numpy
import time
from openai_ros2.envs import LobotArmEnv
from gym.spaces import Box
from typing import Type


def main(args=None):
    env: LobotArmEnv = gym.make('LobotArmContinuous-v0')
    action_space: Type[Box] = env.action_space

    while True:
        print("-------------Starting----------------")
        for x in range(500):
            # action = numpy.array([1.00, -1.01, 1.01])
            action = action_space.sample()
            observation, reward, done, info = env.step(action)
            # Type hints
            observation: numpy.ndarray
            reward: float
            done: bool
            info: dict
            if done:
                break
        time.sleep(1.0)
        print("-------------Resetting environment---------------")
        env.reset()
        print("-------------Reset finished----------------")


if __name__ == '__main__':
    main()
