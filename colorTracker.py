#!/usr/bin/env python

import controller_include as ci
import csv


import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

import actuator_sim as ser


CONTROLLER_REF_NAME  = 'controller-ref-chan'
e = ach.Channel(ci.CONTROLLER_REF_NAME)
e.flush()
controller = ci.CONTROLLER_REF()


cap = cv2.VideoCapture(0)
ret, frame = cap.read()
height, width, depth = frame.shape
print 'H = ', height, ' W = ', width

while True:
    
    ret, frame = cap.read()
    img = frame
    
    # No color blub ditected = no error:
    x = 0
    y = 0
    
    # Convert RGB to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Define upper and lower range of green color in HSV
    lower_blue = np.array([0,100,100], dtype=np.uint8)
    upper_blue = np.array([20,255,255], dtype=np.uint8)

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    kernel = np.ones((5,5), np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 7)
    dilation = cv2.dilate(erosion, kernel, iterations = 7)

    # Use findContours to get the boundry of the green blob
    contours,hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Look through all the seperate contours and highlight the boundry and centroid
    for cnt in contours:
		# Calculate moments
        moments = cv2.moments(cnt)           
        if moments['m00']!=0:
            x = int(moments['m10']/moments['m00'])
            y = int(moments['m01']/moments['m00'])
            
            # draw contours 
            cv2.drawContours(img,[cnt],0,(0,0,255),1)   
            # draw centroids in red
            cv2.circle(img,(x,y),10,(0,0,255),-1)
            
            # get the errors from the center:
            x = int(x - (width/2))
            y = int(y - (height/2))
            break; # recognize only the first object    

    cv2.imshow('wctrl',img)
    
    cv2.waitKey(10)
    
    print '\nError in x & y: ', x, '\t', y
    
    # send the error to controller
    controller.x = x
    controller.y = y
    e.put(controller)
    time.sleep(.1)

   
   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
