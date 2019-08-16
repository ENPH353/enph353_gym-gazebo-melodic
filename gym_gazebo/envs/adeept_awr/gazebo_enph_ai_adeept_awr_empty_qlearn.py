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
            # Define a blank matrix that matches the image height/width.
            mask = np.zeros_like(img)
            # Retrieve the number of color channels of the image.
            channel_count = img.shape[2]
            # Create a match color with the same color channel counts.
            match_mask_color = (255,) * channel_count
        
            # Fill inside the polygon
            cv2.fillPoly(mask, vertices, match_mask_color)
        
            # Returning the image only where mask pixels match
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
        cropped_image = region_of_interest(
            image,
            np.array([region_of_interest_vertices], np.int32),
        )
        #cv2.imshow("Image window", cropped_image)
        #cv2.waitKey(3)
        state = []

        # Get gray image and process GaussianBlur
        gray = cv2.cvtColor(cropped_image,cv2.COLOR_BGR2GRAY)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size),0)

        # Process edge detection with Canny
        low_threshold = 50
        high_threshold = 150
        edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

        # HoughLinesP to get the lines, adjust params for performance
        rho = 1  # distance resolution in pixels of the Hough grid
        theta = np.pi / 180  # angular resolution in radians of the Hough grid
        threshold = 5  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 100  # minimum number of pixels making up a line
        max_line_gap = 25  # maximum gap in pixels between connectable line segments
        line_image = np.copy(cropped_image) * 0  # creating a blank to draw lines on

        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

        if not lines is None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)

        # Check if any lines have close slopes
        slopes = []
        if not lines is None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if not x2 - x1 == 0:
                        slopes.append(float(y2-y1) / float(x2-x1))
        print(slopes)

        indices = []
        for i, slope_i in enumerate(slopes):
            for j, slope_j in enumerate(slopes):
                if not i == j and i < j:
                    if abs(slope_i - slope_j) < 0.02 and not slope_i == 0.0:
                        indices.append((i, j))
        print(indices)




        print("__________________________________")

        # Draw the lines on the image
        lines_edges = cv2.addWeighted(cropped_image, 0.8, line_image, 1, 0)

        cv2.imshow("Image window", lines_edges)
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
