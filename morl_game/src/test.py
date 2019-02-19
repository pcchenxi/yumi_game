#!/usr/bin/env python3
import sys
import rospy
# from morl_game.srv import *
import gym
import gym_ipadgame 

env = gym.make('ipadgame-v0')
env.reset()

# env.step([0.02, -0.02])

