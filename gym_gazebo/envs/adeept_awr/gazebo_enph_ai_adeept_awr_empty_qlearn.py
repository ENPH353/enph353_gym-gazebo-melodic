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
import os

class Gazebo_ENPH_Ai_Adeept_Awr_Empty_Env(gazebo_env.GazeboEnv):

    def image_callback(self, data):
        self.image_data = data

    def collision_callback(self, data):
        self.c_data = data

    def __init__(self):
        # Launch the simulation with the given launchfile name
        scriptpath = os.path.realpath(__file__)
        this_dir = os.path.dirname(os.path.abspath(scriptpath))
        print(this_dir)
        gazebo_env.GazeboEnv.__init__(self, this_dir + "/../enph353/src/enph353/enph353_utils/launch/sim.launch")

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
        self.c_sub = rospy.Subscriber('{}_Hit'.format(self.robot_name), Bool, self.collision_callback)
        self.c_data = None
        self.bridge = CvBridge()

        # Setup simulation parameters
        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        self._seed()

        self.prev_lines = None
        import time
        self.last_time = time.time()

    def four_point_transform(self, image, rect):
      # obtain a consistent order of the points and unpack them
      # individually
      (tl, tr, br, bl) = rect
     
      # compute the width of the new image, which will be the
      # maximum distance between bottom-right and bottom-left
      # x-coordiates or the top-right and top-left x-coordinates
      widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
      widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
      maxWidth = max(int(widthA), int(widthB))
     
      # compute the height of the new image, which will be the
      # maximum distance between the top-right and bottom-right
      # y-coordinates or the top-left and bottom-left y-coordinates
      heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
      heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
      maxHeight = max(int(heightA), int(heightB))
     
      # now that we have the dimensions of the new image, construct
      # the set of destination points to obtain a "birds eye view",
      # (i.e. top-down view) of the image, again specifying points
      # in the top-left, top-right, bottom-right, and bottom-left
      # order
      dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
     
      # compute the perspective transform matrix and then apply it
      M = cv2.getPerspectiveTransform(rect, dst)
      warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
     
      # return the warped image
      return warped

    def process_image(self, image_data):

        # Get image
        try:
          image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
          print(e)

        import numpy as np
        import cv2

        # Manually found these numbers for perspective->ortho
        rect = np.zeros((4, 2), dtype = "float32")
        rect[0][0] = 585
        rect[0][1] = 392
        rect[1][0] = 700
        rect[1][1] = 392
        rect[2][0] = 1224
        rect[2][1] = 720
        rect[3][0] = 172
        rect[3][1] = 720

        # Get ortho image
        warped = self.four_point_transform(image, rect)

        # Get reward based on number of gray pixels in roi
        region_of_interest = warped[warped.shape[0]*4//5:warped.shape[0],0:warped.shape[1]]
        cv2.imshow("Image window", region_of_interest)
        cv2.waitKey(3)
        reward = np.sum( ((region_of_interest > 35) & (region_of_interest < 45)) | ((region_of_interest > 80) & (region_of_interest < 90)) ) / (255*100)
        reward -= 7

        ### Get histogram for state
        num_cols = 5
        w = region_of_interest.shape[1] / num_cols
        num_gray = []
        for i in range(num_cols):
            cropped_img = region_of_interest[0:region_of_interest.shape[0], i*w:(i+1)*w]
            num_gray.append(np.sum( ((cropped_img > 35) & (cropped_img < 45)) | ((cropped_img > 80) & (cropped_img < 90)) ))

        output = []
        for num in num_gray:
            if num > 27000:  # threshold
                output.append(1)
            else:
                output.append(0)

        state = output
        success = False
        fail = False


        return state, reward, success, fail

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
            vel_cmd.linear.x = 1
            vel_cmd.angular.z = 0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 10
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = -10
            self.vel_pub.publish(vel_cmd)

        # Read image data
        image_data = self.image_data
        while image_data is None:
            image_data = self.image_data

        # Read collision data
        c_data = self.c_data
        while c_data is None:
            c_data = self.c_data

        # Pause simulation
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, reward, succeeded, failed = self.process_image(image_data)

        # use collision to affect state and reward
        if c_data.data:
            reward -= abs(2*reward)
            state.append(1)
        else:
            state.append(0)

        if action == 0:
            reward *= 2

        import time
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
        # Reward function
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

        state, reward, succeeded, failed = self.process_image(image_data)

        return state
