import sim
import time
import math 
import numpy as np

print ('Program started')

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
position1 = np.array([0,0,0,0,0,0])
position2 = np.array([180*math.pi/180,0*math.pi/180,45*math.pi/180,90*math.pi/180,0*math.pi/180,0*math.pi/180])
# Get "handle" to the base of robot
result, base_handle = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_blocking)

	
# Get handles to the all joints of robot
result, joint_one_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
result, joint_two_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
result, joint_three_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
result, joint_four_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
result, joint_five_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
result, joint_six_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)

# Get Sensor Handle
result, sensorRGB_handle = sim.simxGetObjectHandle(clientID, 'sphericalVisionRGBAndDepth_sensorRGB', sim.simx_opmode_blocking)
result, sensorDepth_handle = sim.simxGetObjectHandle(clientID, 'sphericalVisionRGBAndDepth_sensorDepth', sim.simx_opmode_blocking)


# Move to position 1  
sim.simxSetJointTargetPosition(clientID, joint_one_handle, position1[0], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_two_handle, position1[1], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_three_handle, position1[2], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_four_handle, position1[3], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_five_handle, position1[4], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_six_handle, position1[5], sim.simx_opmode_oneshot)
time.sleep(2)

# Move to position 2  

sim.simxSetJointTargetPosition(clientID, joint_one_handle, position2[0], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_two_handle, position2[1], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_three_handle, position2[2], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_four_handle, position2[3], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_five_handle, position2[4], sim.simx_opmode_oneshot)
time.sleep(0)
sim.simxSetJointTargetPosition(clientID, joint_six_handle, position2[5], sim.simx_opmode_oneshot)
time.sleep(2)

# Access Sensor Measurement
returnCode,res,DepthData = sim.simxGetVisionSensorDepthBuffer(clientID,sensorDepth_handle,sim.simx_opmode_blocking)
returnCode,res,RGBData = sim.simxGetVisionSensorImage(clientID,sensorRGB_handle,1,sim.simx_opmode_blocking)
print (DepthData,RGBData,res)