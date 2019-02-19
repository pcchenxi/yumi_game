import gym
from gym import error, spaces, utils
from gym.utils import seeding

import rospy
from morl_game.srv import *

def interact_game(cmd_type, cmd_data=[]):
    try:
        game_control = rospy.ServiceProxy('cmd_service', GameCmd)
        resp1 = game_control(cmd_type, cmd_data)
        return resp1.obs
    except (rospy.ServiceException, e):
        print ("Service call failed: %s"%e)


class GameEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    rospy.wait_for_service('cmd_service')
    print('init game')

  def step(self, action):
    obs = interact_game(2, action)
    print('in step', obs)

  def reset(self):
    obs = interact_game(1)
    print('in reset', obs)

  def render(self, mode='human', close=False):
    ...
