#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of red and yellow blocks
xw_yw_R = []
xw_yw_Y = []
xw_yw_G = []

stick_length = 7.6/100
block_width = 2.35
yaw = 0
board_z_offset = 0.2
block_z_offset = 2.5        #TODO: double check
block_height = [board_z_offset + block_z_offset, board_z_offset + (2*block_z_offset), board_z_offset + (3*block_z_offset),board_z_offset + (4*block_z_offset),board_z_offset + (5*block_z_offset),board_z_offset + (6*block_z_offset),board_z_offset + (7*block_z_offset),board_z_offset + (8*block_z_offset),board_z_offset + (9*block_z_offset)]                       # in cm
# block_1_height = board_z_offset + block_z_offset
# block_2_height = board_z_offset + (2*block_z_offset)
# coordinates of block end locations in the world frame (in m)
# TODO: update to bridge end locations:
Q = [[],[],[]]
bottom_left = [22/100, -18/100, block_height[0]/100-0.005]
for i in range (12): #up to 6 layers
    if i < 4:
        Q[0].append([bottom_left[0],bottom_left[1]+i*3.5/100,bottom_left[2]])
    elif i < 8:
        Q[1].append([bottom_left[0] - 8.5/100,bottom_left[1]+(i-4)*3.5/100,bottom_left[2]])
    elif i < 12:
        Q[2].append([bottom_left[0] - 2*8.5/100,bottom_left[1]+(i-8)*3.5/100,bottom_left[2]])


# Any other global variable you want to define
# Hints: where to put the blocks?
# matrix storing the destination locations for the blocks in the world coordinate

# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


    # ========================= Student's code starts here =========================
# TODO: create a helper function that returns an array of the lire fractions based on the number of levels (command-line argument passed to the function in main())
# TODO: block height 2.4 cm, bridge_brick length 7.2 cm, R&Y centroid distance 4.8 cm

#TODO: refine blob search or somehow improve accuracy of centroid
def turn(pub_cmd, loop_rate, start_xw_yw_zw, dest_xw_yw_zw, angle, vel, accel):
    global Q
    # level = count//4
    # num = count % 4
    cur_angle = math.degrees(angle)
    print("initil: ", cur_angle)
    error = 0
    midpoint = lab_invk(30/100,30/100,start_xw_yw_zw[2]+0.15,yaw)
           #not alligned
    # test_yaw = 75
    # test_yaw1 = 75
    #arbitary value
    # while test_yaw == 75 or abs(start_theta6)> 85:      #if start_theta6 can't manage a +-90 degree turn, might have to change to smaller increments
    #     test_yaw += 15                                  #try another yaw
    #     start_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw)[5]
    if cur_angle < -70 and cur_angle > -180:
        before_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]-0.003, cur_angle+150)
        before_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1, cur_angle+150)
        # after_turn = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2],150)
        # after_turn_above = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2]+0.1, 150)
        after_turn = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2],150)
        after_turn_above = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2]+0.1, 150)
    else:
        before_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]-0.003, cur_angle)
        before_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1, cur_angle)
        # after_turn = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2], 0)
        # after_turn_above = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2]+0.1, 0)
        after_turn = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2],0)
        after_turn_above = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2]+0.1, 0)
    # print("update: ", cur_angle)
    # print("test_yaw", test_yaw)
        

    cur_angle = 0

        # if abs(90-cur_angle) < 90:     #i.e. in 1st or second quad
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(90-cur_angle) or abs(end_theta6-start_theta6) < 0.95*(90-cur_angle):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle = 90 

 
        # elif cur_angle <= 0 and cur_angle >= -90:            #4th quad, turn 90 into 1st quad
           
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(90) or abs(end_theta6-start_theta6) < 0.95*(90):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle -= (360-90)
            
        # elif cur_angle <=-90 and cur_angle >= -180:          #3rd quad, turn -90 into 2nd quad  
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(-90) or abs(end_theta6-start_theta6) < 0.95*(-90):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle -= 90      
        
        # else:
        #     print("Error with turning angle")
        #     sys.exit()


    move_arm(pub_cmd, loop_rate, before_turn_above, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, before_turn, 2.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_on) 
    time.sleep(.5)
    if(analog_in_0 > 1.75):
        move_arm(pub_cmd, loop_rate, before_turn_above, 3.0, 3.0)
        move_arm(pub_cmd, loop_rate, midpoint, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, after_turn_above, 3.0, 3.0)
        move_arm(pub_cmd, loop_rate, after_turn, 1.0, 1.0)
    else:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(.5)
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        print("Block not at provided coord ")
        sys.exit()
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.5)
    #TODO: check why almost never go diretcy to after above
    move_arm(pub_cmd, loop_rate, after_turn_above, 4.0, 4.0)
    time.sleep(1)
    return error   
    
def turn_high(pub_cmd, loop_rate, start_xw_yw_zw, dest_xw_yw_zw, angle, vel, accel):
    global Q
    # level = count//4
    # num = count % 4
    cur_angle = math.degrees(angle)
    print("initil: ", cur_angle)
    error = 0
    midpoint = lab_invk(30/100,30/100,start_xw_yw_zw[2]+0.22,yaw)
           #not alligned
    # test_yaw = 75
    # test_yaw1 = 75
    #arbitary value
    # while test_yaw == 75 or abs(start_theta6)> 85:      #if start_theta6 can't manage a +-90 degree turn, might have to change to smaller increments
    #     test_yaw += 15                                  #try another yaw
    #     start_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw)[5]
    if cur_angle < -70 and cur_angle > -180:
        before_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]-0.003, cur_angle+150)
        before_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1, cur_angle+150)
        # after_turn = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2],150)
        # after_turn_above = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2]+0.1, 150)
        after_turn = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2],150)
        after_turn_above = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2]+0.04, 150)
    else:
        before_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]-0.003, cur_angle)
        before_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1, cur_angle)
        # after_turn = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2], 0)
        # after_turn_above = lab_invk(Q[level][num][0],Q[level][num][1],Q[level][num][2]+0.1, 0)
        after_turn = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2],0)
        after_turn_above = lab_invk(dest_xw_yw_zw[0],dest_xw_yw_zw[1],dest_xw_yw_zw[2]+0.04, 0)
    # print("update: ", cur_angle)
    # print("test_yaw", test_yaw)
        

    cur_angle = 0

        # if abs(90-cur_angle) < 90:     #i.e. in 1st or second quad
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(90-cur_angle) or abs(end_theta6-start_theta6) < 0.95*(90-cur_angle):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle = 90 

 
        # elif cur_angle <= 0 and cur_angle >= -90:            #4th quad, turn 90 into 1st quad
           
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(90) or abs(end_theta6-start_theta6) < 0.95*(90):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle -= (360-90)
            
        # elif cur_angle <=-90 and cur_angle >= -180:          #3rd quad, turn -90 into 2nd quad  
        #     while test_yaw1 == 75 or abs(end_theta6-start_theta6) > 1.05*(-90) or abs(end_theta6-start_theta6) < 0.95*(-90):
        #         test_yaw1 += 1
        #         end_theta6 = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)[5]
        #     after_turn = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     after_turn_above = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], test_yaw1)
        #     cur_angle -= 90      
        
        # else:
        #     print("Error with turning angle")
        #     sys.exit()


    move_arm(pub_cmd, loop_rate, before_turn_above, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, before_turn, 2.0, 2.0)
    gripper(pub_cmd, loop_rate, suction_on) 
    time.sleep(.5)
    if(analog_in_0 > 1.75):
        move_arm(pub_cmd, loop_rate, before_turn_above, 3.0, 3.0)
        move_arm(pub_cmd, loop_rate, midpoint, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, after_turn_above, 3.0, 3.0)
        move_arm(pub_cmd, loop_rate, after_turn, 1.0, 1.0)
    else:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(.5)
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        print("Block not at provided coord ")
        sys.exit()
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.5)
    #TODO: check why almost never go diretcy to after above
    move_arm(pub_cmd, loop_rate, after_turn_above, 4.0, 4.0)
    time.sleep(1)
    return error   

def pairing(xw_yw_R_elems,xw_yw_G_elems,xw_yw_B_elems):
    global block_width
    #sort based on x coord for each color
    R_coord = sorted(xw_yw_R_elems)
    G_coord = sorted(xw_yw_G_elems)
    B_coord = sorted(xw_yw_B_elems)
    print(G_coord)
    R_coord_paired = []
    G_coord_paired = []
    B_coord_paired = []
    angles = []
    #TODO: check these values
    left_bounds = 2/100
    lefter_bounds = -2/100
    right_bounds = 23/100
    bottom_bounds = 30/100
    found = 0
    for i in range(len(G_coord)):                                       #looping through each Red coord
        found = 0
        for j in range(len(R_coord)):
            if found == 1:
                break                                   #loop through green coord
            elif math.dist(G_coord[i],R_coord[j]) < 1.5*block_width/100:      #if the green coord is close to red
                for k in range(len(B_coord)):
                    if found == 1:
                        break
                    else:                           #loop through yellow
                        GR_angle = np.arctan2(R_coord[j][1] - G_coord[i][1],R_coord[j][0] - G_coord[i][0])
                        GB_angle = np.arctan2(B_coord[k][1] - G_coord[i][1],B_coord[k][0] - G_coord[i][0])
                        RB_angle = np.arctan2(B_coord[k][1] - R_coord[j][1],B_coord[k][0] - R_coord[j][0])
                        
                        print(math.dist(R_coord[j],B_coord[k]),abs(GR_angle-GB_angle))
                        #if yellow is close to green and has approximatly same angle
                        if math.dist(R_coord[j],B_coord[k]) < 1.5*block_width/100 and abs(GR_angle-GB_angle) < math.radians(8):
                            angle_mean = np.mean((RB_angle,GR_angle,GB_angle))
                            if R_coord[j][1] < lefter_bounds:                 #if y coord is out of left bounds
                                if (angle_mean > -5 and angle_mean < 5) or (angle_mean < -175) or (angle_mean > 175):     #if close to a vertical stick, y coord shifted to right
                                    R_center = (R_coord[j][0],R_coord[j][1] - 0.0008, R_coord[j][2])                        #shift back to left by 0.5mm
                                elif (angle_mean > -85 and angle_mean <= -5) or (angle_mean > 95 and angle_mean <= 175):  # if close to a tilted stick, TODO: check if center is shifted to right and bottom
                                    R_center = (R_coord[j][0] - 0.001,R_coord[j][1] - 0.0008, R_coord[j][2])
                                elif (angle_mean >-95 and angle_mean <= -85) or (angle_mean > 85 and angle_mean <= 95):   #if close to horizontal, should be accurate
                                    R_center = (R_coord[j][0],R_coord[j][1], R_coord[j][2])  
                                elif (angle_mean > -175 and angle_mean <= -95) or (angle_mean > 5 and angle_mean <= 85):  # if close to a tilted stick, TODO: check if center is shifted to right and up
                                    R_center = (R_coord[j][0] + 0.001,R_coord[j][1] - 0.001, R_coord[j][2])
                            elif R_coord[j][1] < left_bounds:                 #if y coord is out of left bounds
                                if (angle_mean > -5 and angle_mean < 5) or (angle_mean < -175) or (angle_mean > 175):     #if close to a vertical stick, y coord shifted to right
                                    R_center = (R_coord[j][0],R_coord[j][1] - 0.0005, R_coord[j][2])                        #shift back to left by 0.5mm
                                elif (angle_mean > -85 and angle_mean <= -5) or (angle_mean > 95 and angle_mean <= 175):  # if close to a tilted stick, TODO: check if center is shifted to right and bottom
                                    R_center = (R_coord[j][0] - 0.0005,R_coord[j][1] - 0.0005, R_coord[j][2])
                                elif (angle_mean >-95 and angle_mean <= -85) or (angle_mean > 85 and angle_mean <= 95):   #if close to horizontal, should be accurate
                                    R_center = (R_coord[j][0],R_coord[j][1], R_coord[j][2])  
                                elif (angle_mean > -175 and angle_mean <= -95) or (angle_mean > 5 and angle_mean <= 85):  # if close to a tilted stick, TODO: check if center is shifted to right and up
                                    R_center = (R_coord[j][0] + 0.0005,R_coord[j][1] - 0.0005, R_coord[j][2])   
                            elif R_coord[j][1] > right_bounds:              #if y coord is out of right bounds
                                if (angle_mean > -5 and angle_mean < 5) or (angle_mean < -175) or (angle_mean > 175):     #if close to a vertical stick, y coord shifted to left
                                    R_center = (R_coord[j][0],R_coord[j][1] -0.0023, R_coord[j][2])                        #shift back to right by 0.5mm
                                elif (angle_mean > -85 and angle_mean <= -5) or (angle_mean > 95 and angle_mean <= 175):  # if close to a tilted stick, TODO: check if center is shifted to left and up
                                    R_center = (R_coord[j][0] + 0.0008,R_coord[j][1] - 0.0023, R_coord[j][2])
                                elif (angle_mean >-95 and angle_mean <= -85) or (angle_mean > 85 and angle_mean <= 95):   #if close to horizontal, should be accurate
                                    R_center = (R_coord[j][0],R_coord[j][1], R_coord[j][2])  
                                elif (angle_mean > -175 and angle_mean <= -95) or (angle_mean > 5 and angle_mean <= 85):  # if close to a tilted stick, TODO: check if center is shifted to left and bottom
                                    R_center = (R_coord[j][0] - 0.0008,R_coord[j][1] - 0.0023, R_coord[j][2]) 
                            elif R_coord[j][0] > bottom_bounds:              #if y coord is out of right bounds
                                if (angle_mean > -15 and angle_mean < 15) or (angle_mean < -165) or (angle_mean > 165):     #if close to a vertical stick, should be accurate
                                    R_center = (R_coord[j][0],R_coord[j][1] , R_coord[j][2])                        #shift back to right by 0.5mm
                                elif (angle_mean > -75 and angle_mean <= -15) or (angle_mean > 105 and angle_mean <= 165):  # if close to a tilted stick, TODO: check if center is shifted to left and up
                                    R_center = (R_coord[j][0] + 0.005,R_coord[j][1] + 0.0005, R_coord[j][2])
                                elif (angle_mean >-105 and angle_mean <= -75) or (angle_mean > 75 and angle_mean <= 105):   #if close to horizontal, x is shifted to up
                                    R_center = (R_coord[j][0] + 0.005,R_coord[j][1], R_coord[j][2])  
                                elif (angle_mean > -165 and angle_mean <= -105) or (angle_mean > 15 and angle_mean <= 75):  # if close to a tilted stick, TODO: check if center is shifted to right and up
                                    R_center = (R_coord[j][0] + 0.005,R_coord[j][1] - 0.0005, R_coord[j][2]) 
                            else:
                                R_center = (R_coord[j][0],R_coord[j][1], R_coord[j][2])  

                            G_coord_paired.append(G_coord[i])
                            R_coord_paired.append(R_center)
                            # (np.mean((R_coord[j][0],G_coord[i][0],B_coord[k][0])),np.mean(((R_coord[j][1],G_coord[i][1],B_coord[k][1]))),R_coord[j][2])
                            B_coord_paired.append(B_coord[k])
                            angles.append(angle_mean)
                            found = 1
                            # remove might break the for as index changed
                            # R_coord.remove(R_coord[i])
                            # G_coord.remove(G_coord[j])
                            # Y_coord.remove(Y_coord[k])
                        
    if len(R_coord) != len(R_coord_paired) or len(B_coord)!= len(B_coord_paired) or len(G_coord)!=len(G_coord_paired):
        print("Not able to pair blocks")
    return R_coord_paired, angles


    # ========================= Student's code ends here ===========================

"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    global analog_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
    analog_in_0 = msg.AIN0
"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global Q
    global yaw
    print("start: ", *start_xw_yw_zw)
    start_loc = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], yaw)
    end_loc = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], yaw)
    start_loc_above = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2]+0.08,yaw)
    end_loc_above = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.08, yaw)

    # copied from lab2:  
    error = 0
    move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, start_loc, 3.0, 3.0)
    gripper(pub_cmd, loop_rate, suction_on)
    # loop_rate.sleep()
    time.sleep(.5)

    if(digital_in_0):
        move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
        # move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc, 1.0, 1.0)

    else:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(.5)
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        print("Quitting with error... ")
        sys.exit()

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.5)
    move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
    time.sleep(.5)

def move_block_high(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global Q
    global yaw
    print("start: ", *start_xw_yw_zw)
    start_loc = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], yaw)
    end_loc = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], yaw)
    start_loc_above = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2]+0.03,yaw)
    end_loc_above = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.03, yaw)

    # copied from lab2:  
    error = 0
    move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, start_loc, 3.0, 3.0)
    gripper(pub_cmd, loop_rate, suction_on)
    # loop_rate.sleep()
    time.sleep(.5)

    if(digital_in_0):
        move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
        # move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc, 2.0, 2.0)

    else:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(.5)
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        print("Quitting with error... ")
        sys.exit()

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.5)
    move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
    time.sleep(.5)

   

    # ========================= Student's code ends here ===========================

    return error

def move_block_far(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global Q
    global yaw
    print("start: ", *start_xw_yw_zw)
    start_loc = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], yaw)
    end_loc = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], yaw)
    start_loc_above = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2]+0.15,yaw)
    #TODO: test midpoint value, 
    midpoint = lab_invk(30/100,30/100,start_xw_yw_zw[2]+0.15,yaw)
    end_loc_above = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.15, yaw)

    # copied from lab2:  
    error = 0
    move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, start_loc, 4.0, 4.0)
    gripper(pub_cmd, loop_rate, suction_on)
    # loop_rate.sleep()
    time.sleep(.5)

    if(digital_in_0):
        move_arm(pub_cmd, loop_rate, start_loc_above, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, midpoint, 4.0, 4.0)
        # move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
        move_arm(pub_cmd, loop_rate, end_loc, 4.0, 4.0)

    else:
        error = 1
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(.5)
        move_arm(pub_cmd, loop_rate, go_away, 4.0, 4.0)
        print("Quitting with error... ")
        sys.exit()

    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(.5)
    move_arm(pub_cmd, loop_rate, end_loc_above, 4.0, 4.0)
    time.sleep(.5)


   

    # ========================= Student's code ends here ===========================

    return error

class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):
        global xw_yw_B # store found blue blocks in this list
        global xw_yw_R # store found red blocks in this list
        global xw_yw_G

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of red blocks
        # and yellow blocks, and store the centers in xw_yw_R & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_R or xw_yw_Y.

        # Remember, xw_yw_R & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_R = blob_search(cv_image, "red")
        xw_yw_B = blob_search(cv_image, "blue")
        xw_yw_G = blob_search(cv_image,"green")


"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_R
    global xw_yw_B
    global xw_yw_G
    
    global Q
    global block_height
    global stick_length

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(0.5)


    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_R, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    centroids = []
    while(len(centroids) != 17):
        time.sleep(5)
        xw_yw_R_temp = xw_yw_R
        xw_yw_G_temp = xw_yw_G
        xw_yw_B_temp = xw_yw_B
    

        # if (len(xw_yw_R_temp) != len(xw_yw_G_temp) or len(xw_yw_G_temp) != len(xw_yw_Y_temp) ):                                                    # if the correct number of blocks are not present at the beginning
        #     print("Error: Incorrect number of blocks")
            # sys.exit()
            
        # else:
        if (len(xw_yw_R_temp) == len(xw_yw_G_temp) and len(xw_yw_G_temp) == len(xw_yw_B_temp) ):
            xw_yw_zw_R_elems = []                                                                                    # list include z coordinate fo red block: xw_yw_R_elems = [[block1_x, block1_y, block2_z], [[block2_x, block2_y, block2_z]]
            xw_yw_zw_G_elems = []  
            xw_yw_zw_B_elems = []                                                                                    # list include z coordinate fo yellow block: xw_yw_Y_elems = [[block1_x, block1_y, block2_z], [[block2_x, block2_y, block2_z]]
        # angles = []
            for i in range(len(xw_yw_R)):
                xw_yw_zw_R_elems.append((xw_yw_R_temp[i][0], xw_yw_R_temp[i][1], block_height[0]/100))
                xw_yw_zw_G_elems.append((xw_yw_G_temp[i][0], xw_yw_G_temp[i][1], block_height[0]/100))   
                xw_yw_zw_B_elems.append((xw_yw_B_temp[i][0], xw_yw_B_temp[i][1], block_height[0]/100))
            centroids, angles = pairing(xw_yw_zw_R_elems,xw_yw_zw_G_elems,xw_yw_zw_B_elems)
            print("centroids: ", centroids)
            print("angles: ", angles)
            
        
    # for i in range(len(centroids)):
    #     print(i)
    #     turn(pub_command, loop_rate, centroids[i], angles[i], vel, accel,i)
    #TODO; make more error space
    tower_level = int((len(centroids)+1)/2)
    center = [10/100, 40/100, (block_height[tower_level-1]+0.2)/100]
    for i in range(tower_level):
        if i == tower_level-1:
           
            dest = [center[0],center[1],center[2],90]
            print(i,dest)
            turn_high(pub_command, loop_rate, centroids[2*i], dest, angles[2*i], vel, accel)
        elif i == 7:
            dest_r = [center[0]+1/2*stick_length+0.0010,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-1/2*stick_length-0.0010,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block_far(pub_command, loop_rate,Q[2][0],dest_l, vel/2, accel/2)
            turn_high(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[2][1],dest_r, vel/2, accel/2)
            turn_high(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
        elif i == 6:
            dest_r = [center[0]+(1/2+1/2/2)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block_far(pub_command, loop_rate,Q[1][2],dest_l, vel/2, accel/2)
            turn_high(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[1][3],dest_r, vel/2, accel/2)
            turn_high(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
            
        elif i == 5:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block_far(pub_command, loop_rate,Q[1][0],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[1][1],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
            
        elif i == 4:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3+1/2/4)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3+1/2/4)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block_far(pub_command, loop_rate,Q[0][2],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[0][3],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
            
        elif i == 3:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3+1/2/4+1/2/5)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3+1/2/4+1/2/5)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block(pub_command, loop_rate,Q[0][0],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[0][1],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
        elif i == 2:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block(pub_command, loop_rate,Q[0][0],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[0][1],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
        elif i == 1:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6+1/2/7)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6+1/2/7)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block(pub_command, loop_rate,Q[0][0],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[0][1],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
        elif i == 0:
            dest_r = [center[0]+(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6+1/2/7+1/2/8)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            dest_l = [center[0]-(1/2+1/2/2+1/2/3+1/2/4+1/2/5+1/2/6+1/2/7+1/2/8)*stick_length,center[1],(block_height[i]+0.2)/100,90]
            print(i,dest_l)
            # move_block(pub_command, loop_rate,Q[0][0],dest_l, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i], dest_l, angles[2*i], vel, accel)
            print(i,dest_r)
            # move_block(pub_command, loop_rate,Q[0][1],dest_r, vel/2, accel/2)
            turn(pub_command, loop_rate, centroids[2*i+1], dest_r, angles[2*i+1], vel, accel)
               


    #     angles.append(np.arctan2(xw_yw_Y_elems[i][1] - xw_yw_R_elems[i][1],xw_yw_Y_elems[i][0] - xw_yw_R_elems[i][0]))  # finding the angles of each block wrt the x-axis
        # TODO: solve for different quadrants; 1 -> pos change (90-change); 2-> neg change (90-change); 3-> 90deg change into quad 2 -> same method as quad 2; 4-> -90deg change into quad 1 -> same method as quad 1
        # TODO: pairing, Kai's thoughts:    1. arrange red in order of small to big xcoord 
        #                                   2. for each red, find green where dist < block width, might have multiple 
        #                                   3. for that green, find yellow where dist < block width, might have multiple
        #                                   4. parse through possible yellow to find yellow that has approx same x and y diff with green
        #                                   5. if no such yellow, find next possible green
        # Note that atan returns angles in radians
        # move_block(pub_command, loop_rate, xw_yw_R_elems[i], Q[0][i], vel, accel)
    # for i in range(2):
        # move_block(pub_command, loop_rate, xw_yw_Y_elems[i], Q[1][i], vel, accel)
    
    move_arm(pub_command, loop_rate, go_away, vel, accel)
    print("Sticks aligned!")
    # TODO: add bridge building
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()



    # ========================= Student's code ends here ===========================

    # move_arm(pub_command, loop_rate, go_away, vel, accel)
    # rospy.loginfo("Task Completed!")
    # print("Use Ctrl+C to exit program")
    # rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass