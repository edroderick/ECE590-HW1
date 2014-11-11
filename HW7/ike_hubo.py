#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
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



import hubo_ach as ha
import ach
import sys
import time
import math
import numpy as np
from numpy.linalg import inv
from ctypes import *
import ike_include as ike

threshold = .075
l1 = .2145
l2 = .17914
l3 = .18159
alpha = .01
dT1 = .001		#might need to make negative to correspond with arm directions
dT2 = .01
dT3 = .01
dE = np.array([[.01], [.01], [.01]])	# eX, eY, eZ

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
s.flush()
r.flush()

#open ach channel for coodinates being sent from file
k = ach.Channel(ike.CONTROLLER_REF_NAME)
coordinates = ike.CONTROLLER_REF()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

#test code

ref.ref[ha.RSR] = -math.pi/2*0	#raising shoulder
ref.ref[ha.REB] = -math.pi/4	#curling elbow
ref.ref[ha.RSP] = -math.pi/6*0
r.put(ref)
time.sleep(5)

#end test code
'''
eX = .02
eY = .02
eZ = .02	

dirX = 1
dirY = 1
dirZ = 1
'''
e = 1

while True:
	[statuss, framesizes] = k.get(coordinates, wait=False, last=True)
	while (e > threshold):

		# Get the current feed-forward (state) 
		[statuss, framesizes] = k.get(coordinates, wait=False, last=True)

		#rotate/translate position vector to match IKE equations reference
		T = np.array([[0, 1, 0, 0],[0, 0, 1, 0],[1, 0, 0, .2145],[0, 0, 0, 1]])
		#X = np.array([[coordinates.x], [coordinates.y], [coordinates.z], [1]])
		X = np.array([[-.45],[-.2],[.2], [1]])
		pos = T.dot(X)

		[statuss, framesizes] = s.get(state, wait=False, last =True)
		#Get current joint angles
		t1 = state.joint[ha.RSP].pos 
		t2 = state.joint[ha.RSR].pos
		t3 = state.joint[ha.REB].pos 		

		print 'angles' , t1, t2, t3

		#compute current position
		x = l3*(math.sin(t3)*math.sin(t1) - math.cos(t1)*math.cos(t2)*math.cos(t3)) - l2*math.cos(t1)*math.cos(t2)
		y = -l3*(math.sin(t1)*math.cos(t2)*math.cos(t3) + math.cos(t1)*math.sin(t3)) - l2*math.sin(t1)*math.cos(t2)
		z = l3*math.sin(t2)*math.cos(t3) + l2*math.sin(t2)		

		#find direction to move
		
		if (pos[0,0] - x) > 0:
			dirX = 1
		else:
			dirX = -1
		if (pos[1,0] - y) > 0:
			dirY = 1
		else:
			dirY = -1
		if (pos[2,0] - z) > 0:
			dirZ = 1
		else: 
			dirZ = -1

		#compute dE values from dTheta and desired position
		
		eX = (l3*(math.sin(t3+dT3)*math.sin(t1+dT1) - math.cos(t1+dT1)*math.cos(t2+dT2)*math.cos(t3+dT3)) - l2*math.cos(t1+dT1)*math.cos(t2+dT2)) - pos[0,0]
		eY = (-l3*(math.sin(t1+dT1)*math.cos(t2+dT2)*math.cos(t3+dT3) + math.cos(t1+dT1)*math.sin(t3+dT3)) - l2*math.sin(t1+dT1)*math.cos(t2+dT2)) - pos[1,0]
		eZ = (l3*math.sin(t2+dT2)*math.cos(t3+dT3) + l2*math.sin(t2+dT2)) - pos[2,0]
		'''
		eX = dirX*(l3*(math.sin(dT3)*math.sin(dT1) - math.cos(dT1)*math.cos(dT2)*math.cos(dT3)) - l2*math.cos(dT1)*math.cos(dT2))
		eY = dirY*(-l3*(math.sin(dT1)*math.cos(dT2)*math.cos(dT3) + math.cos(dT1)*math.sin(dT3)) - l2*math.sin(dT1)*math.cos(dT2))
		eZ = dirZ*(l3*math.sin(dT2)*math.cos(dT3) + l2*math.sin(dT2))
		'''
		dE = np.array([[eX],[eY],[eZ]])	
		
		print 'target', pos[0,0], pos[1,0], pos[2,0]
		print 'current pos' , x, y, z
		print 'dE', eX, eY, eZ

		#compute jacobian
		dxt1 = (l3*(math.sin(t3)*math.sin(t1+dT1) - math.cos(t1+dT1)*math.cos(t2)*math.cos(t3)) - l2*math.cos(t1+dT1)*math.cos(t2)) - x
		dyt1 = (-l3*(math.sin(t1+dT1)*math.cos(t2)*math.cos(t3) + math.cos(t1+dT1)*math.sin(t3)) - l2*math.sin(t1+dT1)*math.cos(t2)) - y
		dzt1 = (l3*math.sin(t2)*math.cos(t3) + l2*math.sin(t2)) - z
			
		dxt2 = (l3*(math.sin(t3)*math.sin(t1) - math.cos(t1)*math.cos(t2+dT2)*math.cos(t3)) - l2*math.cos(t1)*math.cos(t2+dT2)) - x
		dyt2 = (-l3*(math.sin(t1)*math.cos(t2+dT2)*math.cos(t3) + math.cos(t1)*math.sin(t3)) - l2*math.sin(t1)*math.cos(t2+dT2)) - y
		dzt2 = (l3*math.sin(t2+dT2)*math.cos(t3) + l2*math.sin(t2+dT2)) - z

		dxt3 = (l3*(math.sin(t3+dT3)*math.sin(t1) - math.cos(t1)*math.cos(t2)*math.cos(t3+dT3)) - l2*math.cos(t1)*math.cos(t2)) - x
		dyt3 = (-l3*(math.sin(t1)*math.cos(t2)*math.cos(t3+dT3) + math.cos(t1)*math.sin(t3+dT3)) - l2*math.sin(t1)*math.cos(t2)) - y
		dzt3 = (l3*math.sin(t2)*math.cos(t3+dT3) + l2*math.sin(t2)) - z

		J = np.array([[dxt1/dT1, dyt1/dT1, dzt1/dT1],[dxt2/dT2, dyt2/dT2, dzt2/dT2],[dxt3/dT3, dyt3/dT3, dzt3/dT3]])
		print J
		#computing pseudo inverse
		Jplus = (inv((J.T).dot(J))).dot(J.T)
		#print Jplus

		#computing dTheta
		dTheta = Jplus.dot(dE)
		#dTheta = J.T.dot(dE)
		print dTheta*alpha
		
		#send to robot
		print dirX, dirY, dirZ
		ref.ref[ha.RSP] = state.joint[ha.RSP].pos + dTheta[0,0]*alpha
		ref.ref[ha.RSR] = state.joint[ha.RSR].pos + dTheta[1,0]*alpha
		ref.ref[ha.REB] = state.joint[ha.REB].pos + dTheta[2,0]*alpha
		

		
		r.put(ref)
		time.sleep(.1)

		#update e
		[statuss, framesizes] = s.get(state, wait=False, last =False)
		#Get current joint angles after move //update for set angles?
		t1 = state.joint[ha.RSP].pos 
		t2 = state.joint[ha.RSR].pos
		t3 = state.joint[ha.REB].pos 		

		#compute current position
		x = l3*(math.sin(t3)*math.sin(t1) - math.cos(t1)*math.cos(t2)*math.cos(t3)) - l2*math.cos(t1)*math.cos(t2)
		y = -l3*(math.sin(t1)*math.cos(t2)*math.cos(t3) + math.cos(t1)*math.sin(t3)) - l2*math.sin(t1)*math.cos(t2)
		z = l3*math.sin(t2)*math.cos(t3) + l2*math.sin(t2)
	

		print 'new' , x, y, z
		e = math.sqrt((pos[0,0] - x)**2 + (pos[1,0] - y)**2 + (pos[2,0] - z)**2)
		print e	, threshold

	
# Close the connection to the channels
r.close()
s.close()
ike.close()
