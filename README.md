# ECE-470-Final-Project
Team Name: Book Worm

Team Members: Ali Albazroun - aia

The goal of this project is to simulate a robot that could sort books by size and color using CoppeliaSim and Python.

## Project Update 1:
Got UR3 robot arm to move to a specified position and back, and got a Spherical Vision Sensor to output depth and color data.

- Used ```simxSetJointTargetPosition``` to move the arm.

- Used ```simxGetVisionSensorDepthBuffer``` and ```simxGetVisionSensorImage``` to get depth and color data respectivly.
Thanks to
"Am I two?"\n"No, UR3." Team
for their python code which helped me understand how to use the commands to interface with CoppeliaSim.

## Project Update 2:
Got the UR3 robot to respond to a book passing by on a conveyor belt and created forward kinematic model for UR3 robot arm.

- Defined a function to calculate the pose of the end effector from given joint angles using forward kinematics.

- Robot detects a book on conveyor belt using ```simxGetFloatSignal``` and the robot arm does a 180º base rotation after.
