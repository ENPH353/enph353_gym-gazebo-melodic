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
        self.x = state[0]
        self.x_dot = state[1]
        self.a = state[2]
        self.a_dot = state[3]
        self.action = action
        self.reward = reward

def parseStateActionRewardData(state_action, reward):
    # Parse out state and action
    state_str, action = state_action
    state_str = state_str[0] + '.00.' + state_str[1:] ## HARDCODE
    state = stateStrToState(state_str)
    if state is None:
        return None

    retVal = StateActionReward(state, action, reward)
    return retVal

def stateStrToState(state_str):
    # Get indices of periods and dashes
    import re
    old_period_indices = [m.start() for m in re.finditer('\.', state_str)]
    dash_indices = [m.start() for m in re.finditer('-', state_str)]

    # Remove dashes for parsing, will add back after
    new_str = state_str
    new_str = new_str.replace('-', '')
    new_period_indices = [m.start() for m in re.finditer('\.', new_str)]

    # Get x,y,Y values
    retVal = []
    if len(new_period_indices) == 4:
        retVal.append(float(new_str[0:new_period_indices[1]-1]))
        retVal.append(float(new_str[new_period_indices[1]-1:new_period_indices[2]-1]))
        retVal.append(float(new_str[new_period_indices[2]-1:new_period_indices[3]-1]))
        retVal.append(float(new_str[new_period_indices[3]-1:]))
    else:
        print("Not 4 periods")
        return None

    # Add negatives
    for index in dash_indices:
        if index < old_period_indices[0]:
            retVal[0] = -1 * retVal[0]
        elif index < old_period_indices[1]:
            retVal[1] = -1 * retVal[1]
        elif index < old_period_indices[2]:
            retVal[2] = -1 * retVal[2]
        elif index < old_period_indices[3]:
            retVal[3] = -1 * retVal[3]

    return retVal

if __name__ == '__main__':

    # Load parameters, move file before running if not wanted
    filename = 'objs.pkl'
    filename = '15-08-2019=16-28-09.pkl'
    if path.exists(filename):
        with open(filename) as f:
            data = pickle.load(f)
        print("Loading params from {}".format(filename))
    else:
        print("{} not found. Starting fresh.".format(filename))

    
    # X = np.arange(-3, 3, 1)
    # Y = np.arange(-3, 3, 1)
    # # U, V = np.meshgrid(X, Y)
    # # print(V)
    # U = np.zeros((6, 6))
    # U[0, 1] = 1
    # V = np.zeros((6, 6)) 
    X, Y = np.meshgrid(np.arange(0, 300, 1), np.arange(0, 300, 1))
    U = np.zeros(X.shape)
    V = np.zeros(Y.shape)
    # U[150, 150] = math.cos(0.78)
    # V[150, 150] = math.sin(0.78)

    fig, ax = plt.subplots()
    #ax.quiverkey(q, X=0.3, Y=1.1, U=10,
    #             label='Quiver key, length = 10', labelpos='E')


    num_none = 0
    state_action_rewards = []
    for state_action, reward in data.items():
        print("State action {}. Reward {}".format(state_action, reward))
        state_action_reward = parseStateActionRewardData(state_action, reward)
        if not state_action_reward is None:
            print("Parsed! State {} {} {} {}. Action {}. Reward {}".format(state_action_reward.x, state_action_reward.x_dot, state_action_reward.a, state_action_reward.a_dot, state_action_reward.action, state_action_reward.reward))
            print("***")
            state_action_rewards.append(state_action_reward)
        else:
            num_none += 1
    print(num_none)
    print(U.shape)
    for sar in state_action_rewards:
        if sar.action == 0:
            print("Action: 0! State {} {} {} {}. Reward {}".format(sar.x, sar.x_dot, sar.a, sar.a_dot, sar.reward))
#            print("Parsed! State {} {} {}. Action {}. Reward {}".format(sar.x, sar.y, sar.Y, sar.action, sar.reward))
#            print(int(150 + 150*sar.x))
#            print(int(100*sar.y))
#            U[int(150 + 150*sar.x), int(100*sar.y)] = sar.reward * math.cos(sar.Y)
#            V[int(150 + 150*sar.x), int(100*sar.y)] = sar.reward * math.sin(sar.Y)
#    EE = np.sqrt(U**2 + V**2)
#    q = ax.quiver(X, Y, U, V, scale=1 / 0.005, cmap='autumn')
#    plt.show()
