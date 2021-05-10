#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
import time
import math

import keras_ocr
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import moveit_commander


class RobotAction(object):
    def __init__(self):
        rospy.init_node('perception')

        # fetch action matrix and actions
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        # Load trained q-matrix stored in csv file
        self.q_matrix = []
        self.q_matrix_path =  os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        with open(self.q_matrix_path, newline='') as q_matrix_csv:
            reader = csv.reader(q_matrix_csv)
            for row in reader:
                self.q_matrix.append(list(map(lambda x: int(x), row)))
        
        # initialize cv bridge
        self.bridge = cv_bridge.CvBridge()

        # get ocr pipeline model
        self.ocr_pipeline = keras_ocr.pipeline.Pipeline()

        # initialize data arrays to be added to in the subscriber msgs
        self.image_data = []
        self.scan_data = []
        self.actions = []

        # define distances between robot/dumbbell and robot/block. TODO change these numbers
        self.min_dist_db = 0.25
        self.min_dist_block = 0.5

        # initialize turtlebot arm and gripper, and set them to initial positions.
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.init_arm_gripper_pos()

        # Initialize robot mode (to figure out what functions to run)
        self.mode = "get_dumbbell"

        # initialize publishers and subscribers
        time.sleep(2)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_received)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_received)

    
    def init_arm_gripper_pos(self):
        """ Initialize turtlebot's arm and gripper positions to be able to grab the dumbbell. """
        # TODO: change up the numbers a bit
        arm_goal = [0.0, 0.65, 0.15, -0.9]
        gripper_goal = [0.01, 0.01]
        self.arm.go(arm_goal, wait=True)
        self.gripper.go(gripper_goal, wait=True)
        self.arm.stop()
        self.gripper.stop()

    def image_received(self, data):
        """ Handle incoming image data from turtlebot. We want to pass this through to cv2. """
        self.image_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # convert cv image data to HSV
        self.image_data = cv2.cvtColor(self.image_data, cv2.COLOR_BGR2HSV)


    def scan_received(self, data):
        pass


    def get_robot_actions(self):
        pass


    def get_dumbbell(self, color):
        """Find a particular color dumbbell, and move robot towards that dumbbell."""
        # Ensure image data and scan data was received from robot sensors
        if len(self.image_data) == 0 or len(self.scan_data) == 0:
            return
        
        # Define dumbbell color bounds
        lower, upper = None, None
        if color == "red":
            lower = np.array([0,40,45])
            upper = np.array([18,255,255])
        elif color == "green":
            lower = np.array([40, 40,40])
            upper = np.array([70,255,255])
        elif color == "blue":
            lower = np.array([100,45,0])
            upper = np.array([180,255,255])

        # define mask and get moment of dumbbell color
        mask = cv2.inRange(self.image_data, lower, upper)
        M = cv2.moments(mask)

        #TODO: left off here




    def run(self):
        """run the rospy node"""
        # set rate and continuously run program
        r = rospy.Rate(5)
        while not rospy.is_shutdown:
            # TODO: fix this logic
            if self.mode == "get_dumbbell":
                self.get_dumbbell()
            # elif ....
            # elif ....

            r.sleep()



        
if __name__ == "__main__":
    node = RobotAction()
    node.run()



