#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import os
import csv
import time

import keras_ocr.pipeline
from sensor_msgs.msg import Image

class Perception(object):
    def __init__(self):
        # rospy.init_node('perception')

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
        """ Given color red, green or blue, return centerpoint of that colored dumbbell, or None if not found. """
        # Define color ranges for our target color
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

        # Find pixels in image matching that color range
        M = cv2.moments(cv2.inRange(self.image_hsv, lb, ub))
        
        # If pixels were matched, return the centerpoint of the "mask" covering those pixels.
        if M['m00'] > 0:
            x,y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            h, w, d = self.image_rgb.shape
            return (x,y,h,w,d)

        return None


    def find_number(self, num):
        """ Given a number 1, 2 or 3, return the center point of that number in image frame, or None if not found. """
        # run keras ocr pretrained model on image data
        prediction_groups = self.pipeline.recognize([self.image_rgb])
        print(prediction_groups)
    
        # Handle cases where number is misidentified: "l" instead of "1", etc.
        misidentified_nums = {
            '1': ['l','L','I','i', 'ii'],
            '2': ['Q', 'z', 'Z', 'm', 'd'],
            '3': ['5', '8', 'B', 'b', 'S', 's', '31']
        }

        lb = np.array([0, 0, 0])
        ub = np.array([255, 255, 0]) 
        
        M = cv2.moments(cv2.inRange(self.image_hsv, lb, ub))

        x,y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        h, w, d = self.image_rgb.shape

        if prediction_groups == [[]]:
            return (x,y,h,w,-1)

        for i,_ in prediction_groups[0]:
            if i == num or i in misidentified_nums[num]:
                print("matched to ", i)
                return (x,y,h,w,d)

        return (x,y,h,w,-1)
                

    def run(self):
        """ run node function. Keep this node consistently running as action.py calls its functions. """
        rospy.spin()

if __name__ == "__main__":
    node = Perception()
    node.run()
