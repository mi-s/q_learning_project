#!/usr/bin/env python3

import rospy, rospkg
import os
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import moveit_commander

from perception import Perception

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class Action(object):
    def __init__(self):
        rospy.init_node('action')
        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
   
        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))

        # Load converged q_matrix from output file in 2-D array format
        self.q_matrix_path = os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        self.q_matrix = np.loadtxt(self.q_matrix_path, delimiter=',')

        # Use converged q_matrix to find optimal sequence of actions 
        self.action_seq = []
        self.get_actions()

        # Identifies current action from 0-2, transition occurs when dumbbell is placed 
        self.curr_action = 0

        # Identifies distance of closest object within a 10 degree arc in front of the robot
        self.distance = 100 
        # Bring in tools from other libraries/files
        self.p = Perception()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_distance)
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

    def scan_distance(self, data):
        # Updates self.distance to contain distance of nearest object in front of robot
        dist = data.range_max 
        front = [355,356,357,358,359,0,1,2,3,4]

        for i in front:
            if data.ranges[i] < dist:
                dist = data.ranges[i]

        self.distance = dist 


    def get_actions(self):
        # Reads converged q_matrix to write optimal sequence of actions into actions[]
        curr_state = 0

        # Find 3 optimal actions to place 3 dumbbells
        for i in range(3):
            options = self.q_matrix[curr_state]
            optimal_reward = 0
            optimal_action = 0

            # Find optimal action given the current state
            for i in range(len(options)):
                if options[i] > optimal_reward:
                    optimal_reward = options[i]
                    optimal_action = i

            db = self.actions[optimal_action]["dumbbell"]
            num = self.actions[optimal_action]["block"]
            self.action_seq.append((db, num)) 

            # Update current state by searching through action matrix
            curr_state = np.where(self.action_matrix[curr_state] == optimal_action)[0][0]

    def lift(self):
        # Grasps dumbbell and raises it
        self.move_group_gripper.go([0.0, 0.0], wait=True)
        self.move_group_arm.go([0, -0.8, 0, 0], wait=True)

    def drop(self):
        # Lowers dumbbell and releases it
        self.move_group_arm.go([0, 0.7, 0, -0.85], wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

    def set_velocity(self, linear_vel, angular_vel):
        # Helper to set robot velocity
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_pub.publish(msg)

    def do_db_action(self, db):
        # Performs the given action by picking up dumbbell then placing
        # it at the correct block number
        print("start do_db_action " + db)
        res = self.p.find_color(db)
        
        if res == None:
            self.set_velocity(0, 0.3)
            rospy.sleep(0.5)
            self.set_velocity(0, 0)
            rospy.sleep(0.5)
            self.do_db_action(db)
            print("1")
            return

        (cx, cy, h, w, d) = res
        linear_vel = 0
        angular_vel = 0
        
        if self.distance <= .22:
            self.set_velocity(linear_vel, angular_vel)
            self.lift()
        elif self.distance <= .4:
            self.set_velocity(linear_vel, angular_vel)
            self.drop()
            rospy.sleep(0.5)
            linear_vel = .25 * (self.distance - .22)
            err = w/2 - cx
            angular_vel = min(.25, err * .003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db)
        elif self.distance < 1:
            print("2")
            err = w/2 - cx
            if (err < 5):
                linear_vel = .25 * (self.distance - .22)

            angular_vel = min(.25, err * 0.003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db)
        else:
            print("3")
            err = w/2 - cx
            if (err < 15):
                linear_vel = .25 * (self.distance - .22)

            angular_vel = min(.25, err * 0.003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db) 

        return


    def do_block_action(self, num):
        return

    def run(self):
        self.lift()
        self.do_db_action("red")
#        for i in range(len(action_seq)):
#            (db, num) = action_seq[i]
#            self.do_db_action(db)
#            self.do_block_action(num)

if __name__ == "__main__":
    node = Action()
    node.run()
