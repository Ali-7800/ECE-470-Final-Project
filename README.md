# ECE-470-Final-Project
Team Name: Book Worm

Team Members: Ali Albazroun - aia

The goal of this project is to simulate a robot that could sort books by size and color using CoppeliaSim and Python.

# Project Update 1:
Got UR3 robot arm to move to a specified position and back, and got a Spherical Vision Sensor to output depth and color data.

*Used ```simxSetJointTargetPosition``` to move the arm.

*Used ```simxGetVisionSensorDepthBuffer``` and ```simxGetVisionSensorImage``` to get depth and color data respectivly.

Thanks to the @github/ur3-robot for there python code which helped me understand how to use the commands to interface with CoppeliaSim.
