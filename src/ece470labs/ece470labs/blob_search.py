#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0,-1,0,392],\
        [0,0,-1,432],\
        [1,0,0,215.5],\
        [0,0,0,1]])

	S1 = np.array([[0,-1,0,150],\
        [1,0,0,150],\
        [0,0,0,0],\
        [0,0,0,0]])

	S2 = np.array([[0,0,1,-162],\
        [0,0,0,0],\
        [-1,0,0,-150],\
        [0,0,0,0]])

	S3 = np.array([[0,0,1,-162],\
        [0,0,0,0],\
        [-1,0,0,94],\
        [0,0,0,0]])

	S4 = np.array([[0,0,1,-162],\
        [0,0,0,0],\
        [-1,0,0,307],\
        [0,0,0,0]])

	S5 = np.array([[0,0,0,0],\
        [0,0,-1,162],\
        [0,1,0,-281],\
        [0,0,0,0]])

	S6 = np.array([[0,0,1,-162],\
        [0,0,0,0],\
        [-1,0,0,392],\
        [0,0,0,0]])       
 

	# ==============================================================#
	return M, S1, S2, S3, S4, S5, S6


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	M, S1, S2, S3, S4, S5, S6 = Get_MS()

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = expm(S1*theta1)@expm(S2*theta2)@expm(S3*theta3)@expm(S4*theta4)@expm(S5*theta5)@expm(S6*theta6)@M

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # =================== Your code starts here ====================#
    
	L1 = 152
	L2 = 120
	L3 = 244
	L4 = 93
	L5 = 213
	L6 = 104
	L7 = 85
	L8 = 92
	L9 = 53.5
	L10 = 59
	
	yaw = np.radians(yaw_WgripDegree)

	xgrip = xWgrip*1000 + 150
	ygrip = yWgrip*1000 - 150
	zgrip = zWgrip*1000 - 10

	xcen = xgrip - L9*np.cos(yaw)
	ycen = ygrip - L9*np.sin(yaw)
	zcen = zgrip


	offset = L2-L4+L6 
	theta1 = np.arctan2(ycen, xcen) - np.arcsin(offset/np.sqrt(xcen**2 + ycen**2))
	theta6 = theta1 + pi/2 - yaw

	x3end = xcen - L7*np.cos(theta1) + offset*np.sin(theta1) 
	y3end = ycen - L7*np.sin(theta1) - offset*np.cos(theta1)
	z3end = zcen + L8 + L10 


	d = x3end*np.cos(theta1)+ y3end*np.sin(theta1)
	h = z3end - L1
	r = np.linalg.norm(np.array([x3end, y3end, z3end])-np.array([0,0,L1]))
	alpha = np.arccos((r**2 + L3**2 - L5**2)/(2*L3*r))
	beta = np.arccos((-r**2 + L3**2 + L5**2)/(2*L3*L5))
	gamma = np.arctan2(h, d)

	theta2 = -(alpha + gamma)
	theta3 = pi - beta
	theta4 = -(theta2 + theta3)
	theta5 = -pi/2

	print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3), np.degrees(theta4), np.degrees(theta5), np.degrees(theta6))

    # ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
