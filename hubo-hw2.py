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

#increased maxtorque of left ankle roll, left knee pitch left hip roll to 100

import hubo_ach as ha
import ach
import sys
import time
import math
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
s.flush()
r.flush()

#Set constants
maxRAR = (math.pi/2)-1.423
maxRHR = -(math.pi/2)+1.423
maxLAR = (math.pi/2)-1.423
maxLHR = -(math.pi/2)+1.423
maxRKN = 2.4
maxRHP = -2.4/2
maxRAP = -2.4/2
steps = 5

def simSleep(tsim, Ts, state):
	time = tsim
	t = tsim + Ts	
	while(time < t):		
		[statuss, framesizes] = s.get(state, wait=False, last=False)
		time = state.time


# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

#Move CG over left foot 
for x in range(0,steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos + maxRAR/steps
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos - maxRAR/steps
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos + maxRAR/steps
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos - maxRAR/steps
	r.put(ref)
	simSleep(state.time, .5, state)

#raise right leg
for x in range(0,steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	print state.joint[ha.LHR].pos
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos + maxRKN/steps
	ref.ref[ha.RHP] = state.joint[ha.RHP].pos + maxRHP/steps
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos + maxRAP/steps
	r.put(ref)
	simSleep(state.time, .5, state)

legstate = 0 #centered over left foot, right leg raised, standing up straight

while True:
	[statuss, framesizes] = s.get(state, wait=False, last=False)

	if 0==legstate:
		for x in range(0,steps):
			tstart = time.time()
			ref.ref[ha.LAR] = state.joint[ha.LAR].pos + .075/steps
			ref.ref[ha.LHR] = state.joint[ha.LHR].pos - .075/steps
			ref.ref[ha.LKN] = state.joint[ha.LHR].pos + 1.68/steps
			ref.ref[ha.LHP] = state.joint[ha.LHR].pos - .84168/steps
			ref.ref[ha.LAP] =  state.joint[ha.LHR].pos - .84168/steps
			r.put(ref)	
			simSleep(state.time, (.5-(tstart-time.time()))/steps, state)
		legstate = 1
	

	if 1==legstate:
		for x in range(0,steps):
			tstart = time.time()
			ref.ref[ha.LAR] = state.joint[ha.LAR].pos - .075/steps
			ref.ref[ha.LHR] = state.joint[ha.LHR].pos + .075/steps
			ref.ref[ha.LKN] = state.joint[ha.LHR].pos - 1.68/steps
			ref.ref[ha.LHP] = state.joint[ha.LHR].pos + .84168/steps
			ref.ref[ha.LAP] = state.joint[ha.LHR].pos + .84168/steps
			r.put(ref)
			simSleep(state.time, (.5-(tstart-time.time()))/steps, state)	
		legstate = 0	


# Close the connection to the channels
r.close()
s.close()
