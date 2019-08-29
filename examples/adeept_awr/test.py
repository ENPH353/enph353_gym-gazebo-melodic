#!/usr/bin/env python
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import qlearn
import liveplot

import os.path
from os import path

import matplotlib.pyplot as plt
import numpy as np
import pickle
import math
import matplotlib.colors as colors

class StateActionReward:
    def __init__(self, state, action, reward):
        self.x0 = state[0]
        self.x1 = state[1]
        self.x2 = state[2]
        self.x3 = state[3]
        self.x4 = state[4]
        if len(state) > 5:
            self.x5 = state[5]
        else:
            self.x5 = None

        self.action = action
        self.reward = reward

def parseStateActionRewardData(state_action, reward):
    # Parse out state and action
    state_str, action = state_action
    print(state_str)
    state = stateStrToState(state_str)
    if state is None:
        return None

    retVal = StateActionReward(state, action, reward)
    return retVal

def stateStrToState(state_str):
    retVal = []
    for s in state_str:
        retVal.append(s)
    return retVal

if __name__ == '__main__':

    # Load parameters, move file before running if not wanted
    filename = '29-08-2019=11-14-04.pkl'
    if path.exists(filename):
        with open(filename) as f:
            data = pickle.load(f)
        print("Loading params from {}".format(filename))
    else:
        print("{} not found. Starting fresh.".format(filename))

    num_none = 0
    state_action_rewards = []
    for state_action, reward in data.items():
        print("State action {}. Reward {}".format(state_action, reward))
        state_action_reward = parseStateActionRewardData(state_action, reward)
        if not state_action_reward is None:
            print("Parsed! State {} {} {} {} {} {}. Action {}. Reward {}".format(state_action_reward.x0, state_action_reward.x1, state_action_reward.x2, state_action_reward.x3, state_action_reward.x4, state_action_reward.x5, state_action_reward.action, state_action_reward.reward))
            print("***")
            state_action_rewards.append(state_action_reward)
        else:
            num_none += 1

    print("Num none: {}".format(num_none))

    print("**********************")
    for sar in state_action_rewards:
        print(sar.x5)
        if sar.x0 == '1' and sar.x1 == '1' and sar.x2 == '1' and sar.x3 == '1' and sar.x4 == '1' and sar.x5 == '0':
            continue
            #print("State {} {} {} {} {}. Action {}. Reward {}".format(sar.x0, sar.x1, sar.x2, sar.x3, sar.x4, sar.action, sar.reward))
