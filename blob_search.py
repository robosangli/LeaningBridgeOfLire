#!/usr/bin/env python

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

###COPIED###
theta = 0
world_dist = 100
pix_dist = ((315.6544494628906 - 390.0198974609375)**2 + (224.7423553466797 - 224.14085388183594)**2)**0.5
beta = world_dist/pix_dist

dist_x = 0
dist_y = 250
tx = 271.9239807128906- dist_x / beta
ty = 205.71560668945312- dist_y / beta






# Function that converts image coord to world coord
def IMG2W(col, row):
    fx = row
    fy = col
    real_pixel_x = (fx - ty) *beta
    real_pixel_y = (fy - tx) *beta
    x = real_pixel_x * np.cos(theta) + real_pixel_y * np.sin(theta)
    y = real_pixel_y * np.cos(theta) + real_pixel_x * np.sin(theta)
    return x/1000,y/1000

###ORIGINAL CODE###
# # Params for camera calibration
# theta = math.atan2(1.928,72.747)                                    # -0.09 , check sign
# beta = 1/0.0013                                                     # 730
# O_r = 240
# O_c = 320
# tx = (14.29 - O_r - 2)/beta                                             # 640
# ty = (235.68  - O_c - 8.5)/beta                                    # 7.69 is the offset amount calculated while testing the robot

# # Function that converts image coord to world coord
# def IMG2W(col, row):
#     global theta
#     global beta
#     global tx
#     global ty

#     x_c = (row-O_r)/beta - tx
#     y_c = (col-O_c)/beta - ty

#     xw = np.cos(theta)*x_c + np.sin(theta)*y_c
#     yw = -np.sin(theta)*x_c + np.cos(theta)*y_c

#     return xw,yw


# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.2

    # Filter by Inerita
    params.filterByInertia = True
    params.minInertiaRatio = 0.3

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    #TODO: use if statements to set different values of lower and upper for each color (red, yellow)
    # lower = (110,50,50)     # blue lower
    # upper = (130,255,255)   # blue upper
    # lower = (0, 100, 100)        # orange lower
    # upper = (40, 255, 255)      # orange upper
    # if (color == "red"):
    #     lower = (-10, 50, 20)
    #     upper = (10, 255, 255)
    # elif (color == "green"):
    #     lower = (25,100,20)
    #     upper = (100,255,255)
    # elif (color == "blue"):                                               
    #     lower = (100,80,30)
    #     upper = (130,255,255) 
    if (color == "red"):
        lower = (-6.8, 218, 85)        
        upper = (6.8, 255, 255)
    elif (color == "green"):
        lower = (40,132,28)
        upper = (82,255,255)
    elif (color == "blue"):                                               
        lower = (110,215,28)
        #(93,57,20)
        upper = (120,255,255) 
    


    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0)
    # print(color, cv2.KeyPoint_convert(keypoints), "\n")
    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
