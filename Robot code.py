########################################## Setup ##########################################

import sim
import time
import math 
import numpy as np
from scipy.linalg import expm
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID == -1:
	raise Exception('Failed connecting to remote API server')
print("Simulation Started")

position0 = np.array([0,0,0,0,0,0])


####################################################
################# Joint Functions ##################

#Function to find handle 2 position with resepct to handle 1 (world frame by default )
#Format: position = [x,y,z]
def getPosition(handle2,handle1 = -1):
    result,position = sim.simxGetObjectPosition(clientID, handle2, handle1,sim.simx_opmode_blocking)
    return position

#Function to find handle 2 oreintation with resepct to handle 1 (world frame by default )
#Format: Rotation Matrix R   
def getOrientation(handle2,handle1 = -1):
    result, Quaternions = sim.simxGetObjectQuaternion(clientID, handle2 , handle1,sim.simx_opmode_blocking)
    x = Quaternions[0]
    y = Quaternions[1]
    z = Quaternions[2]
    w = Quaternions[3]
    R = np.array([[1-2*(y**2)-2*(z**2),2*x*y-2*w*z,2*x*z+2*w*y],[2*x*y+2*w*z,1-2*(x**2)-2*(z**2),2*y*z-2*w*x],[2*x*z-2*w*y,2*y*z+2*w*x,1-2*(x**2)-2*(y**2)]])
    return R

#Joint Move Function
def moveJoints(robot,angles):
    for i in range(1,7):
        sim.simxSetJointTargetPosition(clientID, robot[1][6-i], angles[6-i], sim.simx_opmode_oneshot)
        time.sleep(0)
    time.sleep(5)


# Get distances measurements from each joint center to base frame for forward kinematics
def getJointDistances(robot):
    X = []
    Y = []
    Z = []
    for joint in robot[1]:
        vector = getPosition(joint)
        X.append(vector[0])
        Y.append(vector[1])
        Z.append(vector[2])
    return X,Y,Z


# Function that used to read joint angles
def getJointAngles(robot):
    theta = []   
    for joint in robot[1]:
        result, angle = sim.simxGetJointPosition(clientID, joint, sim.simx_opmode_blocking)
        if result != sim.simx_return_ok:
            raise Exception('could not get' + str(joint+1) +  'joint variable')
        theta.append(angle)
    return theta


  
####################################################
########### Forward Kinematics Functions ###########

#get endPose using coppelieaSim functions used to confirm forward kinematics 
def getEndPose(robot):
    T = np.eye(4)
    x,y,z = getPosition(robot[3])
    T[0:3,3] = np.array([x,y,z])
    T[0:3,0:3] = getOrientation(robot[3])
    return T

#get pose M and screws S
def get_MS(robot):
    #Intialize the robot to position0
    moveJoints(robot,position0)
    X,Y,Z = getJointDistances(robot)
    M = np.eye(4)
    S = np.zeros((6,6))
    
    #Get position and orientation of end effector
    M[0:3,0:3] = getOrientation(robot[3])
    M[0:3,3] = np.array(getPosition(robot[3]))
    
    #Joint 1 screw
    S[0:3,0] = np.array([0,0,1])
    S[3:6,0] = -np.cross(S[0:3,0],[X[0],Y[0],Z[0]])
    
    #Joint 2 screw
    S[0:3,1] = np.array([-1,0,0])
    S[3:6,1] = -np.cross(S[0:3,1],[X[1],Y[1],Z[1]])
    
    #Joint 3 screw
    S[0:3,2] = np.array([-1,0,0])
    S[3:6,2] = -np.cross(S[0:3,2],[X[2],Y[2],Z[2]])
    
    #Joint 4 screw
    S[0:3,3] = np.array([-1,0,0])
    S[3:6,3] = -np.cross(S[0:3,3],[X[3],Y[3],Z[3]])
    
    #Joint 5 screw
    S[0:3,4] = np.array([0,0,1])
    S[3:6,4] = -np.cross(S[0:3,4],[X[4],Y[4],Z[4]])
    
    #Joint 6 screw
    S[0:3,5] = np.array([-1,0,0])
    S[3:6,5] = -np.cross(S[0:3,5],[X[5],Y[5],Z[5]])
    return M,S

#to get matrix [w] from
def skew(S):
    S_MAT = np.zeros((3,3))
    S_MAT[0,1] = -S[2]
    S_MAT[0,2] = S[1]
    S_MAT[1,0] = S[2]
    S_MAT[1,2] = -S[0]
    S_MAT[2,0] = -S[1]
    S_MAT[2,1] = S[0]
    return S_MAT


# get matrix [s] from vector s
def S2MAT(S):
    S_MAT = np.zeros((4,4))
    S_MAT[0,3] = S[3]
    S_MAT[1,3] = S[4]
    S_MAT[2,3] = S[5]
    S_MAT[0,1] = -S[2]
    S_MAT[0,2] = S[1]
    S_MAT[1,0] = S[2]
    S_MAT[1,2] = -S[0]
    S_MAT[2,0] = -S[1]
    S_MAT[2,1] = S[0]
    return S_MAT


#Forward Kinematics Function given the angles theta, the screw matrix S, and the base pose M
def fk(theta, M, S):
    T = M
    for i in range(0,6):
        T = np.dot(expm(S2MAT(S[:,5-i])*theta[5-i]),T)
    return T



####################################################
########### Inverse Kinematics Functions ###########

#Matrix Adjoint for finding the Jacobian
def adj(T):
    adj = np.zeros((6,6))
    adj[0:3,0:3] = T[0:3,0:3]
    adj[3:6,0:3] = np.dot(T[0:3,0:3],skew(T[0:3,3]))
    adj[3:6,3:6] = T[0:3,0:3]
    return adj



#inverse kinematics function returns the six joint angles
def invk(robot, x_w, y_w, z_w, yaw=0):
	#World frame to base frame
	l = [0,0.152,0.12,0.243,0.093,0.213,0.083,0.082,0.082,0,0.1]
	basePosition = getPosition(robot[0])
	x = x_w - basePosition[0]
	y = y_w - basePosition[1]
	z = z_w - basePosition[2]    
    
    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    #(x_cen,y_cen,z_cen)
	xcen = x-l[9]*np.cos(yaw)
	ycen = y-l[9]*np.sin(yaw)  
	zcen = z
    
    #theta 1
	theta11 = np.arctan2(ycen, xcen)
	theta12 = np.arcsin((l[2]-l[4]+l[6])/(xcen**2+ycen**2)**0.5) 
	thetas[0] = theta11 - theta12
    
    #(x_3end,y_3end,z_3end)
	x3end = xcen-l[7]*np.cos(thetas[0])+(l[6]-l[4]+l[2])*np.sin(thetas[0])
	y3end = ycen-l[7]*np.sin(thetas[0])-(l[6]-l[4]+l[2])*np.cos(thetas[0]) 
	z3end = zcen+l[10]+l[8] 
    
	R = (x3end**2+y3end**2+(z3end-l[1])**2)**0.5
    
    #theta2
	theta21 = np.arcsin((z3end-l[1])/R)
	theta22 = np.arccos((R**2+l[3]**2-l[5]**2)/(2*l[3]*R))
	thetas[1]= -(theta21+theta22)
    
    #theta3
	thetas[2]= np.arccos((R**2-l[3]**2-l[5]**2)/(2*l[3]*l[5]))
    
    #theta4
	thetas[3]= -(thetas[2]+thetas[1])
    
    #theta5
	thetas[4]= -np.pi/2
    
    #theta6
	thetas[5] = np.pi/2 - yaw + thetas[0]
    
    #theta adjustments
	thetas[0] += 3*np.pi/2
	thetas[1] += np.pi/2
    
	return thetas



####################################################
################## Sensors Setup ###################


# Get Proximity Sensors Handles
result,Proximity_sensor_left = sim.simxGetObjectHandle(clientID, "Proximty_sensor_left", sim.simx_opmode_blocking)
result,Proximity_sensor = sim.simxGetObjectHandle(clientID, "Proximty_sensor", sim.simx_opmode_buffer)




####################################################
################ UR3 Robots Setup ##################
########### color sorter (cs) robot setup ##########

# Get "handle" for the base of color sorter (cs) robot
result, cs_base = sim.simxGetObjectHandle(clientID, 'cs_link1_visible', sim.simx_opmode_blocking)

	
# Get handles for the all joints of the color sorter (cs) robot
result, cs_joint1 = sim.simxGetObjectHandle(clientID, 'cs_joint1', sim.simx_opmode_blocking)
result, cs_joint2 = sim.simxGetObjectHandle(clientID, 'cs_joint2', sim.simx_opmode_blocking)
result, cs_joint3 = sim.simxGetObjectHandle(clientID, 'cs_joint3', sim.simx_opmode_blocking)
result, cs_joint4 = sim.simxGetObjectHandle(clientID, 'cs_joint4', sim.simx_opmode_blocking)
result, cs_joint5 = sim.simxGetObjectHandle(clientID, 'cs_joint5', sim.simx_opmode_blocking)
result, cs_joint6 = sim.simxGetObjectHandle(clientID, 'cs_joint6', sim.simx_opmode_blocking)

cs_joints = [cs_joint1,cs_joint2,cs_joint3,cs_joint4,cs_joint5,cs_joint6]

# Get handle for suction cup
result, cs_succ = sim.simxGetObjectHandle(clientID, 'cs_succ', sim.simx_opmode_blocking)


# Get handle for suction cup connector
result, cs_connection = sim.simxGetObjectHandle(clientID, 'cs_connection', sim.simx_opmode_blocking)

# Makes calling functions easier
cs_robot = [cs_base,cs_joints,cs_connection,cs_succ]

#cs Table and Conveyors Setup
cs_left_conveyor = invk(cs_robot,-2.0439,-0.6504,0.1658)
cs_table = invk(cs_robot,-1.65,-0.95,0.22)

#cs vision sensor
result, cs_vis = sim.simxGetObjectHandle(clientID, 'cs_Vision_sensor', sim.simx_opmode_blocking)


#####################################################
############ size sorter (ss) robot setup ###########

# Get "handle" for the base of size sorter (ss) robot
result, ss_base = sim.simxGetObjectHandle(clientID, 'ss_link1_visible', sim.simx_opmode_blocking)

	
# Get handles for the all joints of the color sorter (cs) robot
result, ss_joint1 = sim.simxGetObjectHandle(clientID, 'ss_joint1', sim.simx_opmode_blocking)
result, ss_joint2 = sim.simxGetObjectHandle(clientID, 'ss_joint2', sim.simx_opmode_blocking)
result, ss_joint3 = sim.simxGetObjectHandle(clientID, 'ss_joint3', sim.simx_opmode_blocking)
result, ss_joint4 = sim.simxGetObjectHandle(clientID, 'ss_joint4', sim.simx_opmode_blocking)
result, ss_joint5 = sim.simxGetObjectHandle(clientID, 'ss_joint5', sim.simx_opmode_blocking)
result, ss_joint6 = sim.simxGetObjectHandle(clientID, 'ss_joint6', sim.simx_opmode_blocking)

ss_joints = [ss_joint1,ss_joint2,ss_joint3,ss_joint4,ss_joint5,ss_joint6]

# Get handle for suction cup
result, ss_succ = sim.simxGetObjectHandle(clientID, 'ss_succ', sim.simx_opmode_blocking)


# Get handle for suction cup connector
result, ss_connection = sim.simxGetObjectHandle(clientID, 'ss_connection', sim.simx_opmode_blocking)

#makes calling functions easier
ss_robot = [ss_base,ss_joints,ss_connection,ss_succ]

#ss table and coneyvor setup
ss_table = invk(ss_robot,-0.8,0.175,0.22)
ss_right_conveyor = invk(ss_robot,-0.425,0.6025,0.3)



########################################## Simulation ##########################################



####################################### cs robot routine ########################################

color_list = []
result,cs_bookline = sim.simxGetFloatSignal(clientID,"cs_bookline",sim.simx_opmode_streaming)
result,book = sim.simxGetFloatSignal(clientID,"book",sim.simx_opmode_streaming)

#set the robot as busy
sim.simxSetFloatSignal(clientID, 'cs_robotBusy', 1, sim.simx_opmode_oneshot)

while(cs_bookline == 0):
    #move to intial position
    moveJoints(cs_robot,position0)
    
    #wait for book to be detected
    while(book == 0):
        result,book = sim.simxGetFloatSignal(clientID,"book",sim.simx_opmode_buffer)
    
    #move book to cs table
    moveJoints(cs_robot,cs_left_conveyor)
    sim.simxSetFloatSignal(clientID, 'cs_succ', 1, sim.simx_opmode_oneshot) # pick up the book
    moveJoints(cs_robot,cs_table)
    sim.simxSetFloatSignal(clientID, 'cs_succ' , 0, sim.simx_opmode_oneshot) # drop the book off
    
    #check book color
    result,res,image = sim.simxGetVisionSensorImage(clientID, cs_vis, 0 ,sim.simx_opmode_blocking)
    book_color = [image[0],image[1],image[2]] #RGB color value
    book_order = len(color_list) #assumes color is new initially
    rng = 10 #range to determine wheter to colors are the same or not
    
    #loop that check RGB values of every color list with book_color to determine is the same as a color in color_list
    for color in enumerate(color_list):
            if (abs(color[1][0]-book_color[0])<rng) and (abs(color[1][1]-book_color[1])<rng) and (abs(color[1][2]-book_color[2])<rng):
                book_order = color[0]
    
    #if color is new append it to the color_list
    if (book_order == len(color_list)):
        color_list.append(book_color)
    
    #set the book position depending on its order
    book_position = invk(cs_robot,-1.2402,-0.6504+0.15*(book_order),0.3)
    
    #move and place the book in the proper position
    sim.simxSetFloatSignal(clientID, 'cs_succ', 1, sim.simx_opmode_oneshot) # pick up the book
    moveJoints(cs_robot,book_position)
    sim.simxSetFloatSignal(clientID, 'cs_succ', 0, sim.simx_opmode_oneshot) # drop the book off
    result,cs_bookline = sim.simxGetFloatSignal(clientID,"cs_bookline",sim.simx_opmode_buffer)
    time.sleep(1)

#set the robot as free
sim.simxSetFloatSignal(clientID, 'cs_robotBusy', 0 , sim.simx_opmode_oneshot)
moveJoints(cs_robot,position0)

#################################################################################################
####################################### ss robot routine (WIP) ########################################

result,ss_bookline = sim.simxGetFloatSignal(clientID,"ss_bookline",sim.simx_opmode_streaming)
result,stack = sim.simxGetFloatSignal(clientID,"stack",sim.simx_opmode_streaming)
result,length = sim.simxGetFloatSignal(clientID,"length",sim.simx_opmode_streaming)

#set the robot as busy
sim.simxSetFloatSignal(clientID, 'ss_robotBusy', 1, sim.simx_opmode_oneshot)

while(ss_bookline == 0):
    #move to intial position
    moveJoints(ss_robot,position0)
    
    #wait for stack to be detected
    while(stack == 0):
        result,stack = sim.simxGetFloatSignal(clientID,"stack",sim.simx_opmode_buffer)
    
    #get stack length
    result,length = sim.simxGetFloatSignal(clientID,"length",sim.simx_opmode_buffer)
    
    #set book position depending on stack length
    book_position = invk(ss_robot,-1.2402,0.6025,0.15+length)

    #move book to ss table
    moveJoints(ss_robot,book_position)
    sim.simxSetFloatSignal(clientID, 'ss_succ', 1, sim.simx_opmode_oneshot) # pick up the book
    moveJoints(ss_robot,position0)
    moveJoints(ss_robot,ss_table)
    sim.simxSetFloatSignal(clientID, 'ss_succ' , 0, sim.simx_opmode_oneshot) # drop the book off
    
    sim.simxSetFloatSignal(clientID, 'ss_succ', 1, sim.simx_opmode_oneshot) # pick up the book
    moveJoints(ss_robot,ss_right_conveyor)
    sim.simxSetFloatSignal(clientID, 'ss_succ', 0, sim.simx_opmode_oneshot) # drop the book off
    time.sleep(1)
    result,ss_bookline = sim.simxGetFloatSignal(clientID,"ss_bookline",sim.simx_opmode_buffer)

#set the robot as free
sim.simxSetFloatSignal(clientID, 'ss_robotBusy', 0 , sim.simx_opmode_oneshot)
moveJoints(ss_robot,position0)