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

threshold = .05
l1 = .2145
l2 = .17914
l3 = .18159
alpha = .0001

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

ref.ref[ha.REB] = -.5
r.put(ref)
time.sleep(2)

while True:
	[statuss, framesizes] = k.get(coordinates, wait=True, last=True)
	e = 1
	while (e > threshold):

		# Get the current feed-forward (state) 
		[statuss, framesizes] = k.get(coordinates, wait=False, last=True)
		#rotate position vector to match IKE equations reference
		rot = np.array([[0,0,-1],[0, 1, 0],[1, 0, 0]])
		X = np.array([[coordinates.x-l1], [coordinates.y], [coordinates.z]])
		pos = rot.dot(X)

		[statuss, framesizes] = s.get(state, wait=False, last =False)
		#Get current joint angles and compute compatible rotation offsets for position and jacbian calcs
		t3 = state.joint[ha.RSP].pos - math.pi/2
		t4 = state.joint[ha.RSR].pos + math.pi/2
		t5 = state.joint[ha.RSY].pos + math.pi/2
		t6 = state.joint[ha.REB].pos

		#compute current position
		x = l2*math.cos(t3)*math.sin(t4) - l3*(math.sin(t6)*(math.sin(t3)*math.sin(t5) - math.cos(t3)*math.cos(t4)*math.cos(t5)) - math.cos(t3)*math.cos(t6)*math.sin(t4))
		y = l3*(math.sin(t6)*(math.cos(t3)*math.sin(t5) + math.cos(t4)*math.cos(t5)*math.sin(t3)) + math.cos(t6)*math.sin(t3)*math.sin(t4)) + l2*math.sin(t3)*math.sin(t4)
		z = l1 - l3*(math.cos(t4)*math.cos(t6) - math.cos(t5)*math.sin(t4)*math.sin(t6)) - l2*math.cos(t4)
		'''
		#compute next position
		x1 = l2*math.cos(t3+dTheta)*math.sin(t4+dTheta) - l3*(math.sin(t6+dTheta)*(math.sin(t3+dTheta)*math.sin(t5+dTheta) - math.cos(t3+dTheta)*math.cos(t4)*math.cos(t5+dTheta)) - math.cos(t3+dTheta)*math.cos(t6+dTheta)*math.sin(t4+dTheta))
		y1 = l3*(math.sin(t6+dTheta)*(math.cos(t3+dTheta)*math.sin(t5+dTheta) + math.cos(t4+dTheta)*math.cos(t5+dTheta)*math.sin(t3+dTheta)) + math.cos(t6+dTheta)*math.sin(t3+dTheta)*math.sin(t4+dTheta)) + l2*math.sin(t3+dTheta)*math.sin(t4+dTheta)
		z1 = l1 - l3*(math.cos(t4+dTheta)*math.cos(t6+dTheta) - math.cos(t5+dTheta)*math.sin(t4+dTheta)*math.sin(t6+dTheta)) - l2*math.cos(t4+dTheta)
		'''
		dE = [pos[0]-x, pos[1]-y, pos[2]-z]

		#compute jacobian
	
		z0 = np.array([0,0,1])
		z1 = np.array([math.sin(t3),-1*math.cos(t3),0])	
		z2 = np.array([-1*math.cos(t3)*math.sin(t4),-1*math.sin(t3)*math.sin(t4),math.cos(t4)])	
		z3 = np.array([math.cos(t5)*math.sin(t3)+math.cos(t3)*math.cos(t4)*math.sin(t5),math.cos(t4)*math.sin(t3)*math.sin(t5)-math.cos(t3)*math.cos(t5),math.sin(t4)*math.sin(t5)])
		z4 = np.array([math.cos(t3)*math.cos(t6)*math.sin(t4) - math.sin(t6)*(math.sin(t3)*math.sin(t5) - math.cos(t3)*math.cos(t4)*math.cos(t5)),math.sin(t6)*(math.cos(t3)*math.sin(t5) + math.cos(t4)*math.cos(t5)*math.sin(t3)) + math.cos(t6)*math.sin(t3)*math.sin(t4),math.cos(t5)*math.sin(t4)*math.sin(t6) - math.cos(t4)*math.cos(t6)])

		o3 = np.array([l2*math.cos(t3)*math.sin(t4),l2*math.sin(t3)*math.sin(t4),-l2*math.cos(t4)])
		o4 = np.array([l2*math.cos(t3)*math.sin(t4),l2*math.sin(t3)*math.sin(t4),-l2*math.cos(t4)])
		split1 = l2*math.cos(t3)*math.sin(t4)-l3*(math.sin(t6)*(math.sin(t3)*math.sin(t5)-math.cos(t3)*math.cos(t4)*math.cos(t5))-math.cos(t3)*math.cos(t6)*math.sin(t4))
		split2 = l3*(math.sin(t6)*(math.cos(t3)*math.sin(t5)+math.cos(t4)*math.cos(t5)*math.sin(t3)) + math.cos(t6)*math.sin(t3)*math.sin(t4))+l2*math.sin(t3)*math.sin(t4)
		split3 = -l3*(math.cos(t4)*math.cos(t6)-math.cos(t5)*math.sin(t4)*math.sin(t6))-l2*math.cos(t4)
		o5 = np.array([split1,split2,split3])

		A = np.cross(z0, o5)
		B = np.cross(z1, o5)
		C = np.cross(z2, o5)
		D = np.cross(z3, (o5-o3))
		E = np.cross(z4, (o5-o4))
		
		J = np.array([[A[0], B[0], C[0], D[0], E[0]], [A[1], B[1], C[1], D[1], E[1]], [A[2], B[2], C[2], D[2], E[2]]])
		JT = J.T
		G = np.dot(JT, J)
		Jplus = np.dot(inv(G), JT)
		#computing jacobian, partial derivatives found analytically from DH equations
		


		#computing pseudo inverse
		#J = np.array([[dxd1, dxd2, dxd3, dxd4],[dyd1, dyd2, dyd3, dyd4],[dzd1, dzd2, dzd3, dzd4]])
		#Jstep = J.T.dot(J)
		#print Jstep
		#Jplus = inv(J.T.dot(J)).dot(J.T)
		#print Jplus
		#computing dTheta
		dTheta = Jplus.dot(dE)

		#send to robot
		ref.ref[ha.RSP] = state.joint[ha.RSP].pos + dTheta[0,0]*alpha
		ref.ref[ha.RSR] = state.joint[ha.RSR].pos + dTheta[1,0]*alpha
		ref.ref[ha.RSY] = state.joint[ha.RSY].pos + dTheta[2,0]*alpha
		ref.ref[ha.REB] = state.joint[ha.REB].pos + dTheta[3,0]*alpha
		
		print dTheta[0,0]*alpha
		print dTheta[1,0]*alpha
		print dTheta[2,0]*alpha
		print dTheta[3,0]*alpha

		r.put(ref)
		time.sleep(.5)
	
# Close the connection to the channels
r.close()
s.close()
ike.close()
