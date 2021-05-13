#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv
import time

from std_msgs.msg import Header
from q_learning_project.msg import QLearningReward, QMatrix, QMatrixRow, RobotMoveDBToBlock

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

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

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # init publishers and subscribers
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10) 
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        time.sleep(3)

        # Init q matrix as an empty 2D array of zeros
        self.q_matrix = np.zeros([64, 9]) 
        self.q_matrix_path =  os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')

        # Init parameters for q learning algorithn
        self.alpha = 1
        self.gamma = .65

        # Init parameters for detecting q learning convergence
        self.convergence_count = 0
        self.convergence_max = 1000

        # init q learning states/action at 0
        self.curr_state = 0
        self.next_state = 0
        self.curr_action = 0

        # Take first action to begin q learning
        self.do_action()

    def save_q_matrix(self):
        """Save converged q matrix to csv file"""
        print('saving matrix...')
        with open(self.q_matrix_path, 'w+', newline='') as q_matrix_csv:
            writer = csv.writer(q_matrix_csv)
            for row in self.q_matrix:
                writer.writerow(row)
        print('matrix saved, press Ctrl+C to exit...')

    def do_action(self):
        """
        Take a random action out of all the possible actions at a given state, and publish
        that action and at what state it was taken at in order for reward to get calculated.
        """
        # Get "action space" - possible actions that robot can take at its current state.
        options = self.action_matrix[self.curr_state]
        allowed_actions = []
        allowed_action_nums = []
        for i in range(len(options)):
            action_num = int(options[i])
            if action_num != -1:
               allowed_actions.append(i) 
               allowed_action_nums.append(action_num)

        # If we have run out of actions possible to take at current state, reset state to 0,
        # and re-call this function.
        if len(allowed_actions) == 0:
            self.curr_state = 0
            return self.do_action()

        # Select a random action to take within our action space. Specify the next state to occur
        # as a result of taking this action.
        index = np.random.choice(len(allowed_actions))
        self.next_state = allowed_actions[index]
        self.curr_action = allowed_action_nums[index]

        # Take the random action selected. Publish the action taken in order for reward to get calculated
        action = RobotMoveDBToBlock()
        action_info = self.actions[self.curr_action]
        action.robot_db = action_info['dumbbell']
        action.block_id = action_info['block']
        self.action_pub.publish(action)

    def get_reward(self, data):
        """
        Calculate reward according to q learning algorithm. Check if q matrix has converged,
        and handle accordingly if so.
        """
        # Calculate the max reward for next state
        next_reward = max(self.q_matrix[self.next_state])
        
        # Calculate reward currently written in q matrix for our current state/action pair.
        current_reward = self.q_matrix[self.curr_state][self.curr_action]
        
        # Calculate a new reward using the q learning algorithm
        new_reward = current_reward + self.alpha * (data.reward + self.gamma * next_reward - current_reward) 
        
        # If newly calculated reward is the same as what's already written in the q matrix,
        # we count one instance of convergence. If the convergence count reaches 1000 instances,
        # we consider that our q matrix has converged and we save it to CSV.
        if current_reward == new_reward:
            self.convergence_count += 1
            if self.convergence_count == self.convergence_max:
                self.save_q_matrix()
                return
        # If not converged, write the new reward in the q matrix and publish the updated q matrix.
        else:
            self.convergence_count = 0
            self.q_matrix[self.curr_state][self.curr_action] = new_reward
            
            qm = QMatrix()
            temp = []
            # potentially can cast rows as QMatrixRow more simply temp.append(row)
            for row in self.q_matrix:
                r = QMatrixRow()
                r.q_matrix_row = row
                temp.append(r)

            qm.header = Header(stamp=rospy.Time.now())
            qm.q_matrix = temp
            self.matrix_pub.publish(qm)

        # Move to next state, and call another action to be taken.
        self.curr_state = self.next_state
        self.do_action()

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = QLearning()
    node.run()
