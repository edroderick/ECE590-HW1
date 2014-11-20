#!/usr/bin/env python

import controller_include as ci
import controller_include2 as ci2
import collections

import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math


e = ach.Channel(ci.CONTROLLER_REF_NAME)
e.flush()
controller = ci.CONTROLLER_REF()

e2 = ach.Channel(ci2.DYNO_REF_NAME)
e2.flush()
controller2 = ci2.DYNO_REF()

'''
errorWindow = collections.deque(maxlen = 25)
errorWindow.append(0)
Kp = 1
Ki = 1
oldError = 0
Kd = .2

width = 640
hight = 480
'''
print '\nWaiting for the tracking error ...'

near = 7	# tolerance near the center
while True:
	[statuss, framesizes] = e.get(controller, wait=True, last=True)
	x = controller.x
	y = controller.y
	print '\nErrors in X & Y is:\t', x, ',\t', y
	
	k = 0.012
	dThetaX = math.radians(-k * x)
	dThetaY = math.radians(k * y)
	#if (y < 0):
	#	dThetaY = -dThetaY
	
	#if (x > 0):
	#	dThetaX = -dThetaX
	
	'''
	if abs(x) < near:
		dThetaX = 0
	else:
		if abs(x) < 40: 	# Slow down when near the center
			dThetaX = math.radians(.5)
		else:
			dThetaX = math.radians(2)
		if (x > 0):
			dThetaX = -dThetaX
	'''
	
	'''
	if abs(y) < near:
		dThetaY = 0
	else:
		if abs(y) < 40: 	# Slow down when near the center
			dThetaY = math.radians(.5)
		else:
			dThetaY = math.radians(2)
		if (y < 0):
			dThetaY = -dThetaY
	'''
		
	# Command dyn only if there is an error:
	if ( (dThetaX != 0) | (dThetaY !=0) ):
		controller2.dThetaX = dThetaX
		controller2.dThetaY = dThetaY
		e2.put(controller2)
		print 'dTheta X & Y is: \t', dThetaX, ',\t', dThetaY
	else:
		print 'no Error detected!'
		
	
	'''
	# normalize error so that +/- 320 error is +/-1
	errorNorm = abs(error) / maxError
	
	# Proportional Controller:
	Sp = errorNorm * Kp
	# Integral Controller:
	errorWindow.append(errorNorm)
	eSum = sum(errorWindow)
	Si = (eSum * .1) * Ki
	# Derivative Controller:
	Sd = ( (errorNorm - oldError) / .1 ) * Kd
	oldError = errorNorm
	
	# sum them up and if more than 1, set to 1
	Speed = Sp + Si + Sd
	if (Speed > 1):
		Speed = 1
		
	print 'Speed = ', Speed
	
	if ( -maxError <= error <= 0):		# Object is left of the Center
		print 'Object is left of the Center'
		ref.ref[0] = Speed
		ref.ref[1] = -Speed
	elif ( 0 <= error <= maxError ):	# Object is right of the Center
		print 'Object is right of the Center'		
		ref.ref[0] = -Speed
		ref.ref[1] = Speed
	else:								# No Object, Searching for it
		print 'No Object, Searching for it'
		ref.ref[0] = -.5
		ref.ref[1] = .5
	
	# control the robot	
	r.put(ref);
	'''
