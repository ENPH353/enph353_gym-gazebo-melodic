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
          image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
          print(e)

        import numpy as np
        import cv2
        def region_of_interest(img, vertices):
            mask = np.zeros_like(img)
            match_mask_color = 255 # <-- This line altered for grayscale.
            
            cv2.fillPoly(mask, vertices, match_mask_color)
            masked_image = cv2.bitwise_and(img, mask)
            return masked_image

        (height, width, channels) = image.shape
        import matplotlib.pyplot as plt
        import matplotlib.image as mpimg
        region_of_interest_vertices = [
            (0, height),
            (width / 2, height / 2),
            (width, height),
        ]

        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        cannyed_image = cv2.Canny(gray_image, 100, 200)
        # Moved the cropping operation to the end of the pipeline.
        cropped_image = region_of_interest(
            cannyed_image,
            np.array([region_of_interest_vertices], np.int32)
        )
        #plt.figure()
        #plt.imshow(cropped_image)
        #plt.show()

        #cv2.imshow("Image window", cropped_image)
        #cv2.waitKey(3)
        lines = cv2.HoughLinesP(
            cropped_image,
            rho=6,
            theta=np.pi / 60,
            threshold=160,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=25
        )

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []
        if not lines is None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
                    if math.fabs(slope) < 0.5: # <-- Only consider extreme slope
                        continue
                    if slope <= 0: # <-- If the slope is negative, left group.
                        left_line_x.extend([x1, x2])
                        left_line_y.extend([y1, y2])
                    else: # <-- Otherwise, right group.
                        right_line_x.extend([x1, x2])
                        right_line_y.extend([y1, y2])
        min_y = int(float(image.shape[0]) * float(3) / float(5)) # <-- Just below the horizon
        max_y = image.shape[0] # <-- The bottom of the image
        if len(left_line_x) > 0 and len(left_line_y) > 0 and len(right_line_x) > 0 and len(right_line_y) > 0:
            poly_left = np.poly1d(np.polyfit(
                left_line_y,
                left_line_x,
                deg=1
            ))
            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
            poly_right = np.poly1d(np.polyfit(
                right_line_y,
                right_line_x,
                deg=1
            ))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))

            lines = [
                        (left_x_start, max_y, left_x_end, min_y),
                        (right_x_start, max_y, right_x_end, min_y),
                    ]
        else:
            lines = None

        if not lines is None:
            for line in lines:
                print(line)
                x1, y1, x2, y2 = line
                cv2.line(cropped_image,(x1,y1),(x2,y2),(255,0,0),5)
        cv2.imshow("Image window", cropped_image)
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
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
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
