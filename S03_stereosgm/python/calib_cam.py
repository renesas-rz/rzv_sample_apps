"""
/***********************************************************************************************************************
 * Copyright 2026 Renesas Electronics Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2024-2026 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name    : calib_cam.py
 * Version      : v2.00
 * Description  : RZ/V2H AI SDK Sample Application for StereoSGM
 ***********************************************************************************************************************/
"""

import sys
import numpy as np
import cv2
import os
import select
import datetime
import time
from time import sleep
import subprocess

SQUARE_SIZE=24.0     # Size of one square on a chessboard (mm)
BOARD_SIZE=(10,7)    # Number of squares on a chessboard
 
cam_type   = ['', 'EC22', 'EC25', 'USB']
cam_side   = ['', 'left', 'right']
cam_format = ['', 'YUYV', 'MJPG']
cam_width  = [0,640,1280,1920]
cam_height = [0,480,720,1080]

CAMERA_FILE=""   # output camera file
DIST_FILE=""     # output dist file

SRC=""
HEIGHT=""
WIDTH=""
 
MAX_COUNT=25                  # Number of images capture

def main():
    # Information of hessboard
    pattern_points = np.zeros( (np.prod(BOARD_SIZE), 3), np.float32 ) 
    pattern_points[:,:2] = np.indices(BOARD_SIZE).T.reshape(-1, 2)
    pattern_points *= SQUARE_SIZE
    obj_points = []
    img_points = []
    
    #cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture(1)
    cap = cv2.VideoCapture(SRC, cv2.CAP_GSTREAMER)

    print("Capture {0} images. Press c key to capture a image. Press q key to end.".format(MAX_COUNT))
    cnt=0
 
    while(cnt<MAX_COUNT):
        # Capture image from camera
        ret, img = cap.read()
        cv2.imshow('frame0', img)
        
        key = cv2.waitKey(1)
        if key == ord('c'):
            # Convert BGR to GRAY
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
            # Judge if a chessboard is in the image
            found, corner = cv2.findChessboardCorners(img_gray, BOARD_SIZE)
            if found:
                cnt=cnt+1
                print("Detect chessboard = {0}/{1}".format(cnt,MAX_COUNT))
                # Calculate chessboard
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
                r = cv2.cornerSubPix(img_gray, corner, (5,5), (-1,-1), term)
                img2 = cv2.drawChessboardCorners(img_gray, BOARD_SIZE, r, found)
                cv2.imshow('chess', img2)
                # Save information of chessboard
                obj_points.append(pattern_points)
                img_points.append(corner.reshape(-1, 2))
                img_ok=img_gray
                
        elif key == ord('q'): # "q" --> quit                                                          
            break
        
    # Calculate camera distortion parameters
    rms, k, d, r, t = cv2.calibrateCamera(obj_points, img_points, (img_ok.shape[1],img_ok.shape[0]), None, None)
    # Display the calculation results
    print ("RMS = ", rms)
    print ("K = \n", k)
    print ("d = ", d.ravel())
 
    # Save the result to file
    print("Output Camera file", CAMERA_FILE)
    print("Output Distortion file", DIST_FILE)
    np.savetxt(CAMERA_FILE, k, delimiter =',',fmt="%0.14f")
    np.savetxt(DIST_FILE, d, delimiter =',',fmt="%0.14f")
            
    # Cloase camera and windows
    cap.release()
    cv2.destroyAllWindows()

    
if __name__ == '__main__':
    args = sys.argv
    if 5 > len(args):
        print('Arguments are too short')
        print('  For example of command')
        print('    python3 calib_cam.py 2 1 1 2')
        print('      1st argument : Camera type   1:EC22    2:EC25      3:USB')
        print('      2nd argument : Camera Side   1:left    2:right')
        print('      3rd argument : Camera format 1:YUYV    2:MJPG                   <EC22,EC25 cannot select MJPG>')       
        print('      4th argument : Camera Size   1:640x480 2:1280x1080 3:1920x1080  <EC25 cannot select 640x480>')        
        sys.exit()

    WIDTH=cam_width[int(args[4])]
    HEIGHT=cam_height[int(args[4])]

    CAMERA_FILE='camera_{}_{}_{}_{}_{}.csv'.format(cam_type[int(args[1])], cam_side[int(args[2])], cam_format[int(args[3])], cam_width[int(args[4])], cam_height[int(args[4])]) # output camera file
    DIST_FILE  =  'dist_{}_{}_{}_{}_{}.csv'.format(cam_type[int(args[1])], cam_side[int(args[2])], cam_format[int(args[3])], cam_width[int(args[4])], cam_height[int(args[4])]) # output camera file

    if(args[1] == "1"):
        cmd ='./set_e-CAM22.sh {} {} {}'.format(cam_side[int(args[2])], cam_width[int(args[4])], cam_height[int(args[4])])
        print(cmd)
        subprocess.run(cmd, shell=True)
        if(args[2] == "1" and args[3] == "1" ):
            SRC = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
        elif(args[2] == "2" and args[3] == "1" ):
            SRC = "v4l2src device=/dev/video1 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
        else:
            print("Error")
    elif(args[1] == "2"):
        cmd ='./set_e-CAM25.sh {} {} {}'.format(cam_side[int(args[2])], cam_width[int(args[4])], cam_height[int(args[4])])
        print(cmd)
        subprocess.run(cmd, shell=True)
        if(args[2] == "1" and args[3] == "1" ):
            SRC = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
        elif(args[2] == "2" and args[3] == "1" ):
            SRC = "v4l2src device=/dev/video1 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
        else:
            print("Error")
    else:
        print('Select USB')
        if(args[2] == "1"):
            if(args[3] == "1"):
                SRC = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
            else:
                SRC = "v4l2src device=/dev/video0 ! image/jpeg,  format=MJPG, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! jpegdec ! videoconvert ! appsink";
        elif(args[2] == "2"):
            if(args[3] == "1"):
                SRC = "v4l2src device=/dev/video2 ! video/x-raw, format=YUY2, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! videoconvert ! appsink";
            else:
                SRC = "v4l2src device=/dev/video2 ! image/jpeg,  format=MJPG, width=" + str(WIDTH) + ", height=" + str(HEIGHT) + ", framerate=(fraction)30/1 ! jpegdec ! videoconvert ! appsink";
        else:
            print("Error")

    main()
