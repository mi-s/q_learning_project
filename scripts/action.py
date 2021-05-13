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

        # Record state of arm (lowered or raised)
        self.arm_dropped = True

    def scan_distance(self, data):
        # Updates self.distance to contain distance of nearest object in front of robot
        dist = data.range_max 
        front = [355,356,357,358,359,0,1,2,3,4]

        # Find closest distance measurement from LiDAR sensor's `ranges` arg
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
            self.action_seq.append((db, str(num))) 

            # Update current state by searching through action matrix
            curr_state = np.where(self.action_matrix[curr_state] == optimal_action)[0][0]

    def lift(self):
        # Grasps dumbbell and raises it
        self.move_group_gripper.go([0.004, 0.004], wait=True)
        self.move_group_arm.go([0, -0.35, 0, 0], wait=True)
        self.arm_dropped = False

    def drop(self):
        # Lowers dumbbell and releases it
        self.move_group_arm.go([0, 0.7, 0, -0.85], wait=True)
        self.move_group_gripper.go([0.015, 0.015], wait=True)
        self.arm_dropped = True

    def set_velocity(self, linear_vel, angular_vel):
        # Helper to set robot velocity
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_pub.publish(msg)

    def turn(self, dir):
        # Helper to turn robot around after dumbbell picked up.
        if dir == "right":
            vel = -0.175
        elif dir == "left":
            vel = 0.25
        else:
            print("error: pass 'right' or 'left' direction to turn()")
            return
        self.set_velocity(0,vel)
        rospy.sleep(8)
        self.set_velocity(0,0)

    def do_db_action(self, db):
        # Performs the given action by picking up dumbbell then placing
        # it at the correct block number
        
        # Grab image sensor data from function in `perception.py`
        res = self.p.find_color(db)
        
        # If dumbbell not in robot's field of view, rotate robot in-place and scan again.
        if res == None:
            self.set_velocity(0, 0.3)
            rospy.sleep(0.5)
            self.set_velocity(0, 0)
            rospy.sleep(0.5)
            self.do_db_action(db)
            return

        (cx, cy, h, w, d) = res
        linear_vel = 0
        angular_vel = 0
        
        # If robot is within grabbing distance of target dumbbell, 
        # close gripper and lift up arm, then end function loop.
        if self.distance <= .2:
            self.set_velocity(linear_vel, angular_vel)
            self.lift()
        # If robot is almost within grabbing distance, lower arm and open grabber,
        # slowly inch towards the dumbbell handle, and re-call this function.
        elif self.distance <= .4:
            if not self.arm_dropped:
                self.set_velocity(linear_vel, angular_vel)
                self.drop()
                rospy.sleep(0.5)
            # Adjust linear/angular velocities with proportional control
            linear_vel = .25 * (self.distance - .2)
            err = w/2 - cx
            angular_vel = min(.25, err * .003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db)
        # If robot is getting close to grabbing distance, move towards the dumbbell with
        # proportional control, and re-call this function.
        elif self.distance < 1:
            err = w/2 - cx
            if (err < 5):
                linear_vel = .25 * (self.distance - .2)
            angular_vel = min(.25, err * 0.003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db)
        # If robot is far from grabbing distance, move towards the dumbbell with proportional
        # control again, but it can travel a bit faster. Then re-call this function.
        else:
            err = w/2 - cx
            if (err < 15):
                linear_vel = .25 * (self.distance - .2)
            angular_vel = min(.25, err * 0.003)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(0.5)
            self.do_db_action(db) 

        return


    def do_block_action(self, num):
        """To be performed after dumbbell is picked up. Find the numbered block
           for the respective dumbbell and navigate to it, then drop dumbbell."""
        # Grab image sensor data from function in `perception.py`
        res = self.p.find_number(num)
        
        (cx, cy, h, w, d) = res

        # Handle cases when target number was not found in robot's field of view.
        # (this is encoded with a d = -1 that comes from `find_number` in `perception.py`)
        if d == -1 and any([self.distance > 2, cx == 0]):
            # Rotate robot in-place, and re-call this function to scan for number again.
            if num == "1":
                self.set_velocity(0,-0.3)
                rospy.sleep(1)
            else:
                self.set_velocity(0,0.3)
                rospy.sleep(1)
            self.set_velocity(0, 0)
            rospy.sleep(0.5)
            self.do_block_action(num)
            return

        linear_vel = 0
        angular_vel = 0
        
        # If robot is sufficiently close to target numbered block, drop dumbbell
        # and end this function loop.
        if self.distance <= .75:
            self.set_velocity(linear_vel, angular_vel)
            self.drop()
            self.set_velocity(-0.25,0)
            rospy.sleep(2)
            self.set_velocity(0,0)
        # When robot is getting close to target numbered block, the number will no
        # longer be visible in the robot's field of view (it's too far above). In this case,
        # we assume the robot is already pointed towards the correct numbered block. So just
        # move the robot slowly forward using proportional control, and re-call this function.
        elif self.distance <= 1.5:
            err = w/2 - cx
            angular_vel = min(.25, err * .003)
            linear_vel = 0.25 * (self.distance - 0.6)
            self.set_velocity(linear_vel, angular_vel)
            rospy.sleep(1.5)
            self.set_velocity(0, 0)
            rospy.sleep(0.5)
            self.do_block_action(num)
        # When the robot is far from target numbered block but it's within the image sensor data,
        # move the robot forward at a constant velocity. Pause after two seconds and re-call this
        # function. 
        # (Note that it's possible for the number to disappear from the robot's field
        # of view as it moves forward. In this case, the robot will turn in-place until it catches
        # the number again. This was handled in the conditional above.)
        else:
            self.set_velocity(0.2, angular_vel)
            rospy.sleep(2)
            self.set_velocity(0,0)
            rospy.sleep(0.5)
            self.do_block_action(num)

        return


    def run(self):
        """ Run rospy node."""
        # Lift up arm/gripper upon init. This ensures that arm will not get in the way of
        # robot's camera/LiDAR sensor.
        self.lift()

        # For each optimal dumbbell/numbered block pair, call the action sequences to pick up
        # the specified dumbbell and drop it off at its respective numbered block.
        for db,num in self.action_seq:
            self.do_db_action(db)
            if num == "1":
                self.turn("right")
            else:
                self.turn("left")
            self.do_block_action(num)

        

if __name__ == "__main__":
    node = Action()
    node.run()
