import gym
import rospy
import roslaunch
import time
import numpy as np
import math
import cv2

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool

from gym.utils import seeding

class Gazebo_ENPH_Ai_Adeept_Awr_Empty_Env(gazebo_env.GazeboEnv):

    def image_callback(self, data):
        self.image_data = data

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "/home/tylerlum/gym-gazebo/gym_gazebo/envs/installation/catkin_ws/src/enph_ai/launch/sim.launch")

        self.robot_name = "/R1"

        # Setup publisher for velocity
        self.vel_pub = rospy.Publisher('{}/cmd_vel'.format(self.robot_name), Twist, queue_size=5)

        # Setup simulation services
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # Setup subscription to position and collision
        self.image_sub = rospy.Subscriber('{}/pi_camera/image_raw'.format(self.robot_name), Image, self.image_callback)
        self.image_data = None
        self.bridge = CvBridge()

        # Setup simulation parameters
        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        self._seed()

    def process_image(self, image_data):

        try:
          cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
          print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
          cv2.circle(cv_image, (50,50), 10, 255)

        print("{}{}{}".format(rows, cols, channels))
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        state = []
        success = False
        fail = False

        return state, success, fail

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

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 3
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 1
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = -1
            self.vel_pub.publish(vel_cmd)

        # Read image data
        image_data = self.image_data
        while image_data is None:
            image_data = self.image_data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, succeeded, failed = self.process_image(image_data)

        # Reward function
        reward = 1
        return state, reward, (succeeded or failed), {}

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

        # Read image data
        image_data = self.image_data
        while image_data is None:
            image_data = self.image_data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, succeeded, failed = self.process_image(image_data)

        return state
