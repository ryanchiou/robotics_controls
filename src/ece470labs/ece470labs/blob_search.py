#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 750
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    Or = 240
    Oc = 320

    xc = (row - Or) / beta
    yc = (col - Oc) / beta

    xw = np.cos(theta) * (xc-tx) * np.sin(theta) * (yc-ty)
    yw = -np.sin(theta) * (xc-tx) * np.cos(theta) * (yc-ty)

    return xw, yw

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = True
    params.blobColor = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 2500

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower_green = (45,100,150)     # green lower
    upper_green = (85,255,255)   # green upper

    lower_orange = (0,100,90)     # orange lower
    upper_orange = (20,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    green_mask_image = cv2.inRange(hsv_image, lower_green, upper_green)
    orange_mask_image = cv2.inRange(hsv_image, lower_orange, upper_orange)

    combined_mask = cv2.bitwise_or(green_mask_image, orange_mask_image)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(combined_mask)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255))

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", combined_mask)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
