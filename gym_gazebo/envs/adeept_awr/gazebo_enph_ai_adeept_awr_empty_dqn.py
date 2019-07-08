import gym
import rospy
import roslaunch
import time
import numpy as np
import math

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool

from gym.utils import seeding

class Gazebo_ENPH_Ai_Adeept_Awr_Empty_NN_Env(gazebo_env.GazeboEnv):

    def pose_callback(self, data):
        self.pose_data = data

    def collision_callback(self, data):
        self.collision_data = data

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/tylerlum/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/enph_ai/launch/sim.launch")

        # Setup publisher for velocity
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Setup simulation services
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # Setup subcription to position and collision
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        self.collision_sub = rospy.Subscriber('/isHit', Bool, self.collision_callback)
        self.pose_data = None
        self.collision_data = None

        # Setup simulation parameters
        self.reward_range = (-np.inf, np.inf)
        self._seed()

    def process_pose_and_collision(self,p_data,num_decimal_places,c_data):
        # Find index of robot
        index = -1
        for i in range(0, len(p_data.name)):
            if p_data.name[i] == 'robot':
                index = i
        if index == -1:
            print("ERROR: Can't find robot")
            return [0, 0, 0], False, False

        # Get robot pose
        robot_pose = p_data.pose[index]

        # Setup position state return value
        position = []
        position.append(round(robot_pose.position.x, num_decimal_places))
        position.append(round(robot_pose.position.y, num_decimal_places))
        # Use yaw angle rather than quaternion
        q_w = robot_pose.orientation.w
        q_x = robot_pose.orientation.x
        q_y = robot_pose.orientation.y
        q_z = robot_pose.orientation.z
        yaw = math.atan2(2*(q_w*q_z+q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z))
        position.append(round(yaw, num_decimal_places))

        # Check for end cases
        success = False
        fail = False
        if (robot_pose.position.y > 2):
            # Succeeds if reaches the end
            success = True
        elif c_data.data:
            fail = True

        return position, success, fail

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Setup velocity, with const forward speed and varying angular speed
        max_ang_speed = 0.5
        ang_vel = (action-10)*max_ang_speed*0.1 #from (-0.33 to + 0.33)

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.2
        vel_cmd.angular.z = ang_vel
        self.vel_pub.publish(vel_cmd)

        # Read pose and collision data
        p_data = self.pose_data
        while p_data is None:
            p_data = self.pose_data

        c_data = self.collision_data
        while c_data is None:
            c_data = self.collision_data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, succeeded, failed = self.process_pose_and_collision(p_data, 1, c_data)
        done = (succeeded or failed)

        # Want robot to move to the end of the track at the center (y = 2, x = 0)
        if succeeded:
            # Reward based on how far it got and how close it was to the center
            reward = 100 * state[1] + (100 - abs(state[0]) * 250)
        elif failed:
            # Punish based on how far it is from goal
            reward = -200 + state[1]*100
        elif state[1] > 1:
            # Reward more for moving forward and staying in center
            reward = 5 * state[1] - 5 * abs(state[0])
        else:
            # Reward for moving forward and staying in center
            reward = state[1] - abs(state[0])
        print ("Action : "+str(action)+" Ang_vel : "+str(ang_vel)+" reward="+str(reward))


        return np.asarray(state), reward, done, {}

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_world service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # Read pose and collision data
        p_data = self.pose_data
        while p_data is None:
            p_data = self.pose_data

        c_data = self.collision_data
        while c_data is None:
            c_data = self.collision_data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, succeeded, failed = self.process_pose_and_collision(p_data, 1, c_data)

        return np.asarray(state)
