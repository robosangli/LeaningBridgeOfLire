#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab6_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and v1~6, as well as the M matrix
	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])
	q1 = np.array([[-150], [150], [162]])					# in mm
	q2 = q1 + np.array([[0], [120], [0]])
	q3 = q2 + np.array([[244], [0], [0]])
	q4 = q3 + np.array([[213], [-93], [0]])
	q5 = q4 + np.array([[0], [83], [0]])
	q6 = q5 + np.array([[83], [0], [0]])
	S1 = np.block([w1, np.cross(-w1, q1.T)]).T
	S2 = np.block([w2, np.cross(-w2, q2.T)]).T
	S3 = np.block([w3, np.cross(-w3, q3.T)]).T
	S4 = np.block([w4, np.cross(-w4, q4.T)]).T
	S5 = np.block([w5, np.cross(-w5, q5.T)]).T
	S6 = np.block([w6, np.cross(-w6, q6.T)]).T
	P0 = q6 + np.array([[0], [82 + 59], [53.5]])
	R0 = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]])
	M = np.block([R0, P0])
	M = np.block([[M], [0,0,0,1]])
	S = np.block([S1, S2, S3, S4, S5, S6])


	#print("q1:\n", q1 , "\n", "q2:\n", q2 , "\n", "q3:\n", q3 , "\n", "q4:\n", q4 , "\n", "q5:\n", q5 , "\n", "q6:\n", q6 , "\n")

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	# T = np.eye(4)

	M, S = Get_MS()

	zero = np.array([[0,0,0,0]])
	w1_bracket = skew_symm(S[0:3,0])
	S1_bracket = np.hstack((w1_bracket,np.column_stack([S[3:,0]])))
	S1_bracket = np.vstack((S1_bracket,zero))

	w2_bracket = skew_symm(S[0:3,1])
	S2_bracket = np.hstack((w2_bracket,np.column_stack([S[3:,1]])))
	S2_bracket = np.vstack((S2_bracket,zero))

	w3_bracket = skew_symm(S[0:3,2])
	S3_bracket = np.hstack((w3_bracket,np.column_stack([S[3:,2]])))
	S3_bracket = np.vstack((S3_bracket,zero))
	
	w4_bracket = skew_symm(S[0:3,3])
	S4_bracket = np.hstack((w4_bracket,np.column_stack([S[3:,3]])))
	S4_bracket = np.vstack((S4_bracket,zero))

	w5_bracket = skew_symm(S[0:3,4])
	S5_bracket = np.hstack((w5_bracket,np.column_stack([S[3:,4]])))
	S5_bracket = np.vstack((S5_bracket,zero))

	w6_bracket = skew_symm(S[0:3,5])
	S6_bracket = np.hstack((w6_bracket,np.column_stack([S[3:,5]])))
	S6_bracket = np.vstack((S6_bracket,zero))

	T = expm(S1_bracket * theta1) @ expm(S2_bracket * theta2) @ expm(S3_bracket * theta3) @ expm(S4_bracket * theta4) @ expm(S5_bracket * theta5) @ expm(S6_bracket * theta6) @ M
	T[:3,3] = T[:3,3] * 0.001

	# print(str(T) + "\n")

	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	yaw = math.radians(yaw_WgripDegree)
	xBase = 0.15
	yBase = 0.15
	zBase = 0.01

	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059
	

	xGrip = xWgrip + xBase
	yGrip = yWgrip - yBase
	zGrip = zWgrip - zBase

	
	zCen = zGrip
	yCen = yGrip - L9*np.sin(yaw)
	xCen = xGrip - L9*np.cos(yaw)

	b = L2 - L4 + L6

	theta_big = np.arcsin(yCen/np.sqrt((xCen**2) + (yCen**2)))
	theta_small = np.arcsin(b/np.sqrt((xCen**2) + (yCen**2)))
	theta1 = theta_big - theta_small

	theta6 = np.pi/2 - yaw + theta1			# TODO: check validity

	x3end = xCen - L7*np.cos(theta1) + (L6+0.027)*np.sin(theta1)
	y3end = yCen - L7*np.sin(theta1) - (L6+0.027)*np.cos(theta1)
	z3end = zCen + L8 + L10


	L = np.sqrt(x3end**2 + y3end**2 + (z3end - L1)**2)
	#check the following value if receiving NaN error
	dummy = np.clip((L**2 + L3**2 - L5**2)/(2*L*L3),-1,1)
	theta2_a = np.arccos(dummy)
	theta2_b = np.arcsin((z3end - L1)/L)
	theta2 = -(theta2_a + theta2_b)

	#causing NAN errors
	dummy2 = np.clip((L3**2 + L5**2 - L**2)/(2*L3*L5),-1,1)
	theta3_a = np.arccos(dummy2)
	theta3 = np.pi - theta3_a

	theta4 = -(np.pi - theta2_a - theta2_b - theta3_a)

	# theta1 = float(math.degrees(theta1))
	# theta2 = float(math.degrees(theta2))
	# theta3 = float(math.degrees(theta3))
	# theta4 = float(math.degrees(theta4))
	# # theta5 float(= math.degrees(theta)5)
	# theta6 = float(math.degrees(theta6))
	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	theta5 = -np.pi/2
	# theta6 = 0.0
	# ==============================================================#
	# print("1: ", theta1, "2: ", theta2, "3: ", theta3, "4: ", theta4, "5: ", theta5, "6: ", theta6)
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


# skew-symmetric helper function
def skew_symm(input):
	return np.array([[0, -input[2], input[1]],
					[input[2], 0, -input[0]],
					[-input[1], input[0], 0]])

# helper function for calculating distances between centroid for each layer of the bridge
# returns an array of distances between block centroids starting from the lowermost layer
def centroid_dist_for_layer(max_layers):
	array_returned = np.array([0])
	for j in range(max_layers):
		distance_returned = 0
		for i in range(max_layers):
			distance_returned += (1/(2*(max_layers - i - 1)))
		array_returned.append(distance_returned)
	return array_returned

# function that creates the bridge Q matrix based on 