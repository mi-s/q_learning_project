# q_learning_project

# Implementation Plan

## Team Members
Michael Su, Arjun Voruganti

## Component Plan
### Q-Learning Algorithm
#### Executing the Q-Learning Algorithm
We will implement the Q-Learning algorithm described in class meeting 10. At random we will select from the action matrix in order to compute Q. Accordingly, we will call the phantom movement code to simulate new states without actually moving the robot. 

We will test it by printing to console the Q matrix after each update to ensure that the matrix is updating. We can also cross-check by computing an answer on paper and checking it with the code.

#### Determining when the Q-matrix has Converged
We will calculate the sum of differences in the matrix for every iteration of the algorithm.  Once the sum of differences is below a certain threshold, we will consider the Q-matrix to be converged.

To test the difference calculations, we will manually calculate the differences and compare them to the output of the program.  To test what will be an appropriate threshold, we will run the program with multiple different thresholds until we find a value that is accurate and runs at an acceptable speed.

#### Q-matrix Post-Convergence Actions
After the Q-matrix has converged, at any given state the robot will take the action with the highest reward (which we can look up in the matrix).  By taking the action with the highest reward at each state, the robot will achieve the goal state in the fewest number of steps.

To test this, we will print out a converged matrix and check that taking the action with highest reward at each state achieves a near-optimal solution.

### Robot Perception
#### Determining the Identities and Locations of the Three Colored Dumbbells
For each of the three colors, we will define a RGB range that uniquely identifies that color.  Then, we will use the /scan topic to identify the direction (with respect to the front of the robot) that has an object with the color in the desired color range.

To test this, we will run the Gazebo world and print out the direction of the three dumbbells with the robot in various positions.  It should be visibly clear that the dumbbells are in the printed directions.

#### Determining the Identities and Locations of the Three Numbered Blocks
We will use the /scan topic to identify the directions of the blocks.  Then, we will rotate the robot to face each of the blocks and use the keras_ocr image recognition package to identify if the block has the desired number.

We will test this in the same manner as the dumbbell detection.

### Robot Manipulation and Movement
#### Picking Up and Putting Down the Dumbbells with the Arm
First, we will test different joint angles until we find the angles which place the hand at the level of the dumbbell handle.  We will adjust the base joint angle to lift the dumbbell.  To drop it, we will revert the base joint angle.  

To test this, we will observe this process in Gazebo multiple times with and without movement to ensure that it consistently performs.

#### Navigating to the Appropriate Locations to Pick Up/Put Down Dumbbells
We will take the data provided by the components described in the Robot Perception section and use proportional control to navigate the robot to the correct location regardless of its starting point.

We will test this by having the robot drive to all possible dumbbells and numbered blocks from various locations.

## Timeline
By May 5th, we want to have the Q-Learning algorithm implemented and tested.  By May 8th, we want to have the Robot Perception components implemented.  By May 11th, we want to have the Robot Manipulation and Movement components implemented.  We will do the write-up as we go along.  By May 12th, we will have finished the final sections of the write-up.

# Final result

![gif](https://github.com/mi-s/q_learning_project/blob/master/bags/q_learning.gif)

(Note: GIF is at 4x speed)

# Writeup

## Objectives description:

The first main task of the project was to design a Q-Learning algorithm that gives us a Q-matrix. When the robot is in a particular state in the environment, the Q-matrix dictates the best action(s) for the robot to take next in order to maximize its reward and reach the desired end-state in an efficient way. After that, our task was to get the robot to actually take those actions informed by the Q-Matrix in order to place the colored dumbbells at their corresponding blocks.

## High level description:

We used Q-Learning to generate a Q-Matrix which provides the robot a sequence of actions to efficiently place the dumbbells in front of the blocks. The position of dumbbells in the Gazebo environment represents the different robot states. Additionally, there is a particular action space that contains all of the possible actions that the robot can take to manipulate the environment (i.e. move the dumbbells and enter a new state in environment) at any possible state. In the Q-Matrix, we contain reward values for all possible actions in all possible states. In order to write these rewards in the Q-Matrix, we begin by selecting random actions for the robot to take in the environment. Everytime a random action is selected, the algorithm looks at the state that is being moved to by the random action and calculates the average reward of all possible actions from the future state. With this average reward, the algorithm can update the reward value of the current state before the action is performed.  Initially, rewards are only given when all three dumbbells are uniquely placed, so the Q-Matrix is updated from back-to-front.  Whenever this end-state is reached, the state is reset to 0 and the process is repeated until performing the algorithm creates no change over many iterations and the Q-Matrix is determined to be converged.

## Q-Learning algorithm description:

#### Selecting and executing actions for the robot (or phantom robot) to take

Essentially, we instructed the robot to randomly take all available (or possible) actions in the action space. If there were no more actions for the robot to take, we reset the robot's state back to 0 and have the robot continue to take actions. In order to make the phantom robot take the action, we published a `RobotMoveDBToBlock()` message. A reward gets calculated between each of the actions performed (see next section). 

See `do_action()` (line 88) in `q_learning.py`.

#### Updating the Q-matrix

When the robot chooses a random possible action at a particular state, we update the reward of taking that action in that state using the averaged reward of all possible actions at the state being moved to.

See line 145 of `get_reward()` in `q_learning.py`.

#### Determining when to stop iterating through the Q-learning algorithm

We want to stop iterating through the Q learning algorithm when the algorithm no longer is updating the Q-Matrix. After each action taken, a reward gets updated in our Q matrix--but if the code tries to write in the *same* reward that was already written in the cell, and if it tries to do this >1000 times in a row, then we consider the Q matrix to have converged.

See line 139 of `get_reward()` in `q_learning.py`.

#### Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot

The simulated robot starts at state 0, so we first instruct the robot to take any action that yields the highest reward at state 0. Then, whatever state that action leads us to, we instruct the robot to take the action that yields the highest reward for that state. And so on.

See `get_actions()` (line 78) in `action.py`.

## Robot perception description:

#### Identifying the locations and identities of each of the colored dumbbells

To figure out which colored dumbbell was where, we defined red/blue/green color ranges, and we used OpenCV to interpret the image data from the robot's sensor and determine which pixels matched those color ranges. Then we either return the center pixel of the mask that covers the colored pixels that got matched, or we return None if the dumbbell is not detected. This is handled in our `get_dumbbell()` function in `perception.py` and gets called by `action.py`. If it returns None, `action.py` turns the robot until the number is in frame.

See `find_color()` (line 38) in `perception.py`.

#### Identifying the locations and identities of each of the numbered blocks

We used the Keras OCR package and its pretrained models for number recognition. We run this model on the robot's image sensor to look for the specified number in the image frame (we also define a black color range like above.) Either our `get_block` function in `perception.py` returns the center point of the specified number on the block, or it returns "-1" if the object is not detected. If it returns -1, `action.py` turns the robot until the number is in frame.

We also handled some cases were digits got misidentified as numbers: https://www.ismp.org/resources/misidentification-alphanumeric-symbols (we also used trial and error/print statements to record the misidentifications.)

See `find_number()` (line 67) of `perception.py`.

## Robot manipulation/movement description:

#### Moving to the right spot in order to pick up a dumbbell

We used proportional control in conjunction with the robot's Lidar data to navigate to a position in front of the desired dumbbell. When the dumbbell is in the robot's camera, proportional control allows the robot to slowly drive towards the dumbbell.  When the dumbbell is out of the camera, the robot will rotate in-place until it is back in the camera.

See `do_db_action()` (line 133) of `action.py`.

#### Picking up the dumbbell

We used the moveit_commander interface to easily manipulate the robot arm into positions to pick-up the dumbbell and hold the dumbbell in the air.

See `do_db_action()` (line 133) and helper function `lift()` (line 101) of `action.py`.

#### Moving to the desired destination (numbered block) with the dumbbell

We used proportional control in conjunction with the keras_ocr image recognition library to navigate the robot from the dumbbells to the numbered blocks.  After picking up the dumbbell, the robot rotates in place until it can the keras_ocr image recognition identifies that the chosen block is within the camera.  Afterwards, the robot drives a short distance towards the block before checking the camera again for the chosen block.  This process of rotating and driving forward continues until the robot is too close to see the full number in the camera.  At this point, the robot slowly drives up to the block until it is in front of it.

See `do_block_action()` (line 196) of `action.py`.

#### Putting the dumbbell back down at the desired destination

We used the moveit_commander interface to easily manipulate the robot arm into lowering the arm and releasing the gripper.  This is performed once it is determined that the robot is close enough to the desired block.

See `drop()` (line 107) of `action.py`.

## Challenges:

Our biggest challenges came from navigating the robot to the numbered blocks.  Due to the unique colors of the dumbbells, it was fairly easy to use proportional control to have the robot move precisely in front of the dumbbells.  However, because we had to utilize the relatively slow keras_ocr image recognition library for the numbered blocks, we had to approach the numbered blocks differently from the dumbbells.  The fact that the numbered blocks all share the same color also meant that we could not use color to guide the robot to the desired block.  We overcame these challenges by considering the fixed properties of the map to design an approach which would most likely not work with a different map but consistently works with the given map.

## Future work:

If we had more time, we would definitely want to improve how our robot navigates to the numbered blocks.  We are content with the implementations of navigation to the dumbbells and the Q-Learning algorithm.  However, we had to exploit fixed characteristics of the environment to solve navigation to numbered blocks due to the challenges listed in the previous section.  Ideally, the navigation to blocks should work even if the dumbbell and block positions were changed.  This is not currently the case, as the navigation only works with the given environment.

## Takeaways:

* Chipping away at the project day-by-day is the best way to ensure the work is done well.  This is a general tip with larger coding projects but it worked well for our group.

* Try to streamline the debugging process as much as possible.  Due to the robot simulation environment being rather slow, as well as the need to debug many times, it saves a lot of time in the long run to optimize the debugging as much as possible.
