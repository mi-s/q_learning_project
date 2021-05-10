#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import os
import csv
import time
# import math

import keras_ocr.pipeline
from sensor_msgs.msg import Image

class Perception(object):
    def __init__(self):
        rospy.init_node('perception')

        # initialize cv bridge
        self.bridge = cv_bridge.CvBridge()

        # initialize pre-trained ocr model
        self.pipeline = keras_ocr.pipeline.Pipeline()

        # initialize data array for image subscriber 
        self.image_rgb = []
        self.image_hsv = None 

        # initialize image subscriber
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_received)
        time.sleep(1)
 

    def image_received(self, data):
        """ Handle incoming image data from turtlebot. We want to pass this through to cv2. """
        self.image_rgb = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # convert cv image data to HSV
        self.image_hsv = cv2.cvtColor(self.image_rgb, cv2.COLOR_BGR2HSV)


    def find_color(self, color):
        lb, ub = None, None
        if color == "red":
            lb = np.array([0,40,45])
            ub = np.array([18,255,255])
        elif color == "green":
            lb = np.array([40,40,40])
            ub = np.array([70,255,255])
        elif color == "blue":
            lb = np.array([100,45,0])
            ub = np.array([180,255,255])
        else:
            print("error: Perception color not one of red, green, blue")
            return None

        M = cv2.moments(cv2.inRange(self.image_hsv, lb, ub))
        
        if M['m00'] > 0:
            x,y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            print("find_color(" + color + "): " + color + " found")
            return (x,y)
        else:
            print("find_color(" + color + "): N/A")

        return None


    def find_number(self, num):
        print("find_number start")
        prediction_group = self.pipeline.recognize([self.image_rgb])[0]
        print("pipeline called")

        if len(prediction_group) == 0:
            print("length 0")
            return None

        # test with just one [0]
        print("Perception: " + str(prediction_group[0][0]) + " detected")

        if num == prediction_group[0][0]:
            lb = np.array([0, 0, 0])
            ub = np.array([255, 255, 0]) 
            M = cv2.moments(cv2.inRange(self.image_hsv, lb, ub))

            x,y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            print("find_number(" + num + "): " + num + " found")
            return (x,y) 
        else:
            print("find_number(" + num + "): N/A")
            
        return None

    def run(self):
        r = rospy.Rate(3)
        self.find_color("red")
        self.find_color("green")
        self.find_color("blue")
        self.find_number("1")
        self.find_number("2")
        self.find_number("3")


if __name__ == "__main__":
    node = Perception()
    node.run()
