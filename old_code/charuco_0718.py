import numpy as np
import cv2
from cv2 import aruco
import pickle
import glob
import math
import os 

ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_250)
CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=9,
        squaresY=12,
        squareLength=0.02,
        markerLength=0.015,
        dictionary=ARUCO_DICT)
type(CHARUCO_BOARD)

# Create the arrays and variables we'll use to store info like corners and IDs from images processed
corners_all = [] # Corners discovered in all images processed
ids_all = [] # Aruco ids corresponding to corners discovered
image_size = None # Determined at runtime
imgs=[]

# This requires a set of images or a video taken with the camera you want to calibrate
# I'm using a set of images taken with the camera with the naming convention:
# 'camera-pic-of-charucoboard-<NUMBER>.jpg'
# All images used should be the same size, which if taken with the same camera shouldn't be a problem
images = glob.glob('picture/8_3_high2/*_color.jpg')
# images = glob.glob('./pic3/*_color.jpg')
print(images[:1])


# Loop through images glob'ed

for i, iname in enumerate(images[:1]):
    
    base_name = os.path.basename(iname)
    print(base_name)
    
    # Open the image
    img = cv2.imread(iname)
    # Grayscale the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # print(gray.shape)

    # Find aruco markers in the query image
    corners, ids, _ = aruco.detectMarkers(
            image=gray,
            dictionary=ARUCO_DICT)
    print("len(corners)", len(corners))
    print("--")
    
    if len(corners) > 10:
    # Outline the aruco markers found in our query image
        img = aruco.drawDetectedMarkers(
                image=img, 
                corners=corners)
        


        # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=CHARUCO_BOARD)
        print("len(charuco_corners)", len(charuco_corners))
        
               


        # Add these corners and ids to our calibration arrays
        corners_all.append(charuco_corners)
        ids_all.append(charuco_ids)
        
        # Draw the Charuco board we've detected to show our calibrator the board was properly detected
        # img = aruco.drawDetectedCornersCharuco(
        #         image=img,
        #         charucoCorners=charuco_corners,
        #         charucoIds=charuco_ids)
        
        # plt.imshow(img)
        # plt.show()
        # save_img(base_name, img)
        
        # If our image size is unknown, set it now
        if not image_size:
            image_size = gray.shape[::-1]