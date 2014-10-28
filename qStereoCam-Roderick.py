#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

#Added Camera Constants
baseline = .4
focal_length = .085 


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)

    height, width, depth = imgL.shape    #both camera image height/width known to be same
    imgLHSV = cv2.cvtColor(imgL,cv2.COLOR_BGR2HSV)
    imgRHSV = cv2.cvtColor(imgR,cv2.COLOR_BGR2HSV)

    # define range of green color in HSV
    lower_green = np.array([50,50,50],dtype=np.uint8)
    upper_green = np.array([70,255,255],dtype=np.uint8)

    # blue
    upper_blue = np.array([130,255,255], dtype=np.uint8)
    lower_blue = np.array([110,0,0], dtype=np.uint8)

    # Threshold the HSV image to get only green colors
    maskL = cv2.inRange(imgLHSV, lower_green, upper_green)
    maskR = cv2.inRange(imgRHSV, lower_green, upper_green)

    # Threshold the HSV image to get only blue colors
    #mask = cv2.inRange(imgHSV, lower_blue, upper_blue)
    
    #erode and dilate both left and right images, find contours for Moments calc
    kernelL = np.ones((5,5),np.uint8)
    kernelR = np.ones((5,5),np.uint8)
    erosionL = cv2.erode(maskL,kernelL,iterations = 3)
    erosionR = cv2.erode(maskR,kernelR,iterations = 3)
    dilationL = cv2.dilate(erosionL,kernelL,iterations = 3)    
    dilationR = cv2.dilate(erosionR,kernelR,iterations = 3)    
    contoursL, hierarchyL = cv2.findContours(dilationL,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contoursR, hierarchy = cv2.findContours(dilationR,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    #calculate centroids of left image
    if (len(contoursL) > 0):
	cntL = contoursL[0]
	#Finding centroids of best_cnt and draw a circle there
	ML = cv2.moments(cntL)
	cxL = int(ML['m10']/ML['m00'])
	cyL = int(ML['m01']/ML['m00'])
	cv2.circle(imgL,(cxL,cyL),5,(0,0,255),-1)
    
    else: #will equate negative CoG values to correspond with no color seen
	cxL = -1
	cyL = -1

    #calculate centroids of right image
    if (len(contoursR) > 0):
	cntR = contoursR[0]
	#Finding centroids of best_cnt and draw a circle there
	MR = cv2.moments(cntR)
	cxR = int(MR['m10']/MR['m00'])
	cyR = int(MR['m01']/MR['m00'])
	cv2.circle(imgR,(cxR,cyR),5,(0,0,255),-1)
    
    else: #will equate negative CoG values to correspond with no color seen
	cxR = -1
	cyR = -1

    #ignore values when no color seen. Often occuring when camera reads blank for a frame and
    #causes errors in distance calc. When both return blank it causes a divide by zero error.
    #to my best estimation this is an issue with my laptop not being able to process the images
    #fast enough. Following if statement ignores the false data.
    if (not cxL == -1) and (not cxR == -1):
        #calculate distance from each camera axis to determine disparity between cams
        xL = cxL - width/2
        xR = cxR - width/2
        disparity_pixels = xL - xR

        #convert disparity in pixels to meters
        disparity = disparity_pixels * .00028

        #Using simplified stereo vision model found at http://www.techbriefs.com/component/content/article/23-ntb/features/feature-articles/14925
        distance = focal_length*(baseline/disparity)
 
        print "distance = " , distance , " meters"

    # Sleeps

    time.sleep(.1)
    '''
    simtime = tim.sim[0]
    timeSleep = simtime + .1
    while(simtime < timeSleep):
        [status, framesize] = t.get(tim, wait=False, last=True)
        simtime = tim.sim[0]
    '''
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
