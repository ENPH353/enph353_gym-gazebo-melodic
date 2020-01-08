import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gym.utils import seeding
import copy
import math
import os

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState


class GazeboCartPolev0Env(gazebo_env.GazeboEnv):
    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboCartPole_v0.launch")

        # Define end conditions
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 15

        # Setup pub/sub for state/action
        self._pub = rospy.Publisher('/cart_pole_controller/command', Float64, queue_size=1)
        rospy.Subscriber("/cart_pole/joint_states", JointState, self.callback)

        # Gazebo specific services to start/stop its behavior and
        # facilitate the overall RL environment
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.set_link = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        rospy.wait_for_service('/gazebo/set_link_state')

        # Setup the environment
        self._seed()
        self.action_space = spaces.Discrete(2)
        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians * 2,
            np.finfo(np.float32).max])
        self.observation_space = spaces.Box(-high, high)

        # State
        self.current_vel = 0
        self.data = None

        # Round state to decrease state space size
        self.num_dec_places = 2

    def callback(self, data):
        self.data = data

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        # Unpause simulation to make observations
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # Wait for data
        data = self.data
        while data is None:
            data = self.data

        # Pause
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Take action
        if action > 0.5:
            self.current_vel += 0.2
        else:
            self.current_vel += -0.2

        action_msg = Float64()
        action_msg.data = self.current_vel
        self._pub.publish(action_msg)

        # Define state
        x = self.data.position[1]
        x_dot = self.data.velocity[1]
        theta = math.atan(math.tan(self.data.position[0]))
        theta_dot = self.data.velocity[0]
        state = [round(x, 2), round(x_dot, 1), round(theta, 2), round(theta_dot, 0)]

        # Limit state space
        state[0] = 0
        state[1] = 0

        # Check for end condition
        done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians
        done = bool(done)

        if not done:
            reward = 1.0
        else:
            reward = 0

        # Reset data
        self.data = None
        return state, reward, done, {}

    def reset(self):
        # Reset world
        rospy.wait_for_service('/gazebo/set_link_state')
        self.set_link(LinkState(link_name='pole'))
        self.set_link(LinkState(link_name='cart'))

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Wait for data
        data = self.data
        while data is None:
            data = self.data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # Process state
        x = self.data.position[1]
        x_dot = self.data.velocity[1]
        theta = math.atan(math.tan(self.data.position[0]))
        theta_dot = self.data.velocity[0]
        state = [round(x, 2), round(x_dot, 1), round(theta, 2), round(theta_dot, 0)]

        # Limit state space
        state[0] = 0
        state[1] = 0

        self.current_vel = 0

        # Reset data
        self.data = None
        return state
