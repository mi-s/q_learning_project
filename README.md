# q_learning_project

### Team Members
Michael Su, Arjun Voruganti

### Component Plan
#### Q-Learning Algorithm
##### Executing the Q-Learning Algorithm
We will implement the Q-Learning algorithm described in class meeting 10. At random we will select from the action matrix in order to compute Q. Accordingly, we will call the phantom movement code to simulate new states without actually moving the robot. 

We will test it by printing to console the Q matrix after each update to ensure that the matrix is updating. We can also cross-check by computing an answer on paper and checking it with the code.

##### Determining when the Q-matrix has Converged
We will calculate the sum of differences in the matrix for every iteration of the algorithm.  Once the sum of differences is below a certain threshold, we will consider the Q-matrix to be converged.

To test the difference calculations, we will manually calculate the differences and compare them to the output of the program.  To test what will be an appropriate threshold, we will run the program with multiple different thresholds until we find a value that is accurate and runs at an acceptable speed.

##### Q-matrix Post-Convergence Actions
After the Q-matrix has converged, at any given state the robot will take the action with the highest reward (which we can look up in the matrix).  By taking the action with the highest reward at each state, the robot will achieve the goal state in the fewest number of steps.

To test this, we will print out a converged matrix and check that taking the action with highest reward at each state achieves a near-optimal solution.

#### Robot Perception
##### Determining the Identities and Locations of the Three Colored Dumbbells
For each of the three colors, we will define a RGB range that uniquely identifies that color.  Then, we will use the /scan topic to identify the direction (with respect to the front of the robot) that has an object with the color in the desired color range.

To test this, we will run the Gazebo world and print out the direction of the three dumbbells with the robot in various positions.  It should be visibly clear that the dumbbells are in the printed directions.

##### Determining the Identities and Locations of the Three Numbered Blocks
We will use the /scan topic to identify the directions of the blocks.  Then, we will rotate the robot to face each of the blocks and use the keras_ocr image recognition package to identify if the block has the desired number.

We will test this in the same manner as the dumbbell detection.

#### Robot Manipulation and Movement
##### Picking Up and Putting Down the Dumbbells with the Arm
First, we will test different joint angles until we find the angles which place the hand at the level of the dumbbell handle.  We will adjust the base joint angle to lift the dumbbell.  To drop it, we will revert the base joint angle.  

To test this, we will observe this process in Gazebo multiple times with and without movement to ensure that it consistently performs.

##### Navigating to the Appropriate Locations to Pick Up/Put Down Dumbbells
We will take the data provided by the components described in the Robot Perception section and use proportional control to navigate the robot to the correct location regardless of its starting point.

We will test this by having the robot drive to all possible dumbbells and numbered blocks from various locations.

### Timeline
By May 5th, we want to have the Q-Learning algorithm implemented and tested.  By May 8th, we want to have the Robot Perception components implemented.  By May 11th, we want to have the Robot Manipulation and Movement components implemented.  We will do the write-up as we go along.  By May 12th, we will have finished the final sections of the write-up.
