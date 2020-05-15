# ECE-470-Final-Project
Team Name: Book Worm

Team Members: Ali Albazroun - aia

The goal of this project is to simulate a robot that could sort books by size and color using CoppeliaSim and Python.

Project YouTube playlist: https://www.youtube.com/playlist?list=PLpN81Jj_B-xPhZFfH4yKj61GJNcMmq5qz

## Project Update 1:
Got UR3 robot arm to move to a specified position and back, and got a Spherical Vision Sensor to output depth and color data.

- Used ```simxSetJointTargetPosition``` to move the arm.

- Used ```simxGetVisionSensorDepthBuffer``` and ```simxGetVisionSensorImage``` to get depth and color data respectively.
Thanks to
"Am I two?"\n"No, UR3." Team
for their python code which helped me understand how to use the commands to interface with CoppeliaSim.

## Project Update 2:
Got the UR3 robot to respond to a book passing by on a conveyor belt and created forward kinematic model for UR3 robot arm.

- Defined a function to calculate the pose of the end effector from given joint angles using forward kinematics.

- Proximity sensor detects a book on conveyor belt and sends a signal using ```simxsetFloatSignal```.

- Signal is received in python using ```simxgetFloatSignal``` then the robot arm does a 180º base rotation.

## Project Update 3:
Got the Color Sorter UR3 arm to color sort book passing by the conveyor belt using inverse kinematics.

- Defined a function to calculate the joint angles of a robot given the world coordinates of the gripper.

- Created an algorithm to sort the books using vision sensor data from ```simxGetVisionSensorImage```.

- Set up communication between proximity sensors, conveyor belts, and UR3 arms using ```simxgetFloatSignal``` and ```simxsetFloatSignal```.

## Final Project Update:
Got the Size Sorter UR3 arm to sort books by size using inverse kinematics.

- Created an algorithm to sort the books using data from two proximity sensors.

- Redefined the ```moveJoints``` function to allow the robots to pick and place the books without knocking the rest down.

- Refined the communication between proximity sensors, conveyor belts, and UR3 arms.

- Fine tuned inverse kinematics for better stacking.







