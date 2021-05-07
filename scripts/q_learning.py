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

        rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10) 
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        time.sleep(3)
        self.alpha = 1
        self.gamma = .5
        self.discount_factor = .8
        self.convergence_count = 0

        self.has_saved_matrix = False
        self.q_matrix = [] 
        self.q_matrix_path = 'output/q_matrix.csv'
        self.init_q_matrix()

        self.curr_state = 0
        self.next_state = 0
        self.curr_action = 0

        self.do_action()

    def init_q_matrix(self):
        if self.has_saved_matrix:
            with open(self.q_matrix_path, newline='') as q_matrix_csv:
                reader = csv.reader(q_matrix_csv)
                for row in reader:
                     #TODO
                     self.q_matrix.append(list(map(lambda x: int(x), row)))
        else:
            self.q_matrix = np.zeros([64, 9])

    def save_q_matrix(self):
        with open(self.q_matrix_path, 'w', newline='') as q_matrix_csv:
            writer = csv.writer(q_matrix_csv)
            for row in self.q_matrix:
                writer.writerow(row)

    def do_action(self):
        print("do_action start")
        options = self.action_matrix[self.curr_state]
        allowed_actions = []
        allowed_action_nums = []

        for i in range(len(options)):
            action_num = int(options[i])
            if action_num != -1:
               allowed_actions.append(i) 
               allowed_action_nums.append(action_num)

        if len(allowed_actions) == 0:
            # if self.use_saved_matrix: return
            self.curr_state = 0
            return self.do_action()

        # if self.use_saved_matrix etc.
        index = np.random.choice(len(allowed_actions))
        self.next_state = allowed_actions[index]
        print(len(self.q_matrix))
        print(self.next_state)
        self.curr_action = allowed_action_nums[index]

        action_info = self.actions[self.curr_action]
        action = RobotMoveDBToBlock()
        action.robot_db = action_info['dumbbell']
        action.block_id = action_info['block']
        self.action_pub.publish(action)
        print("do_action end")

    def get_reward(self, data):
        print("get_reward start")
        reward = data.reward
        next_reward = max(self.q_matrix[self.next_state])
        
        current_reward = self.q_matrix[self.curr_state][self.curr_action]
        new_reward = current_reward + self.alpha * (data.reward + self.gamma * next_reward - current_reward) 
        print("printing reward")
        print(data.reward)
        print("printing con")
        print(self.convergence_count)
        if current_reward == new_reward:
            self.convergence_count += 1
        elif self.convergence_count == 100:
            ## self.save_q_matrix()
            print("@@@@@@@@matrix converged")
            return
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

        self.current_state = self.next_state
        self.do_action()
        print("get_reward end")

    def run(self):
        print("run")
        rospy.spin()

if __name__ == "__main__":
    node = QLearning()
    node.run()
