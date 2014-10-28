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
maxRAR = (math.pi/2)-1.415
maxRHR = -(math.pi/2)+1.415
maxLAR = (math.pi/2)-1.415
maxLHR = -(math.pi/2)+1.415
startRKN = 1.0
startRHP = -.5
startRAP = -.5
startLKN =1.0
startLHP = -.5
startLAP = -.5

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

#move cg to left
for x in range(0,steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos + maxRAR/steps
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos - maxRAR/steps
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos + maxRAR/steps
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos - maxRAR/steps
	r.put(ref)
	simSleep(state.time, .5, state)
[statuss, framesizes] = s.get(state, wait=False, last=False)
simSleep(state.time, 2, state)
#Squat
for x in range(1,(steps+1)):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = (startRKN/steps)*x
	ref.ref[ha.LKN] = (startLKN/steps)*x
	ref.ref[ha.RHP] = (startRHP/steps)*x
	ref.ref[ha.LHP] = (startLHP/steps)*x
	ref.ref[ha.RAP] = (startRAP/steps)*x
	ref.ref[ha.LAP] = (startLAP/steps)*x
	r.put(ref)
	simSleep(state.time, .5, state)

for x in range(1,(steps+1)):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos + 1.0/steps
	ref.ref[ha.RHP] = state.joint[ha.RHP].pos - .5/steps
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos - .5/steps
	r.put(ref)
	simSleep(state.time, .5, state)

for x in range(1,(steps+1)):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos - 1.0/steps
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos + 1.0/steps
	r.put(ref)
	simSleep(state.time,.5,state)

#put foot down
ref.ref[ha.LKN] = 1.3
ref.ref[ha.LHP] = -.65
ref.ref[ha.LAP] = -.65
r.put(ref)
simSleep(state.time,.5,state)


#move cg to center from right
for x in range(1,steps*2):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.LKN] = state.joint[ha.LKN].pos - .5/(2*steps)
	ref.ref[ha.LHP] = state.joint[ha.LHP].pos - (-.5/(2*steps))
	ref.ref[ha.RHP] = state.joint[ha.RHP].pos - (-.4/(2*steps))
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos - .4/(2*steps)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos - maxRAR/(2*steps)
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos - maxRHR/(2*steps)
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos - maxLAR/(2*steps)
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos - maxLHR/(2*steps)
	r.put(ref)
	simSleep(state.time, .5, state)

[statuss, framesizes] = s.get(state, wait=False, last=False)
ref.ref[ha.RKN] = 1.0
ref.ref[ha.RHP] = -.5
ref.ref[ha.RAP] = -.5
r.put(ref)


#Zero incase of drift
ref.ref[ha.RAR] = 0
ref.ref[ha.RHR] = 0
ref.ref[ha.LAR] = 0
ref.ref[ha.LHR] = 0
r.put(ref)

#move cg to right
for x in range(0,steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos - (.01+maxRAR)/(steps)
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos - (.01+maxRHR)/(steps)
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos - (.01+maxLAR)/(steps)
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos - (.01+maxLHR)/(steps)
	r.put(ref)
	simSleep(state.time, 1, state)	

for x in range(0, 2*steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.LKN] = state.joint[ha.LKN].pos + 1.0/(2*steps)
	ref.ref[ha.LHP] = state.joint[ha.LHP].pos - .5/(2*steps)
	ref.ref[ha.LAP] = state.joint[ha.LAP].pos - .5/(2*steps)
	r.put(ref)
	simSleep(state.time, .5, state)

for x in range(1,(steps+1)):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.LKN] = state.joint[ha.LKN].pos - 1.0/steps
	ref.ref[ha.LAP] = state.joint[ha.LAP].pos + 1.0/steps
	r.put(ref)
	simSleep(state.time,.5,state)

#move cg to center from left
for x in range(1,steps*2):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos - .35/(2*steps)
	ref.ref[ha.RHP] = state.joint[ha.RHP].pos - (-.35/(2*steps))
	ref.ref[ha.LHP] = state.joint[ha.LHP].pos - (-.25/(2*steps))
	ref.ref[ha.LAP] = state.joint[ha.LAP].pos - .25/(2*steps)
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos + maxLAR/(2*steps)
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos + maxLHR/(2*steps)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos + maxRAR/(2*steps)
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos + maxRHR/(2*steps)
	r.put(ref)
	simSleep(state.time, .5, state)

#zeroing to compensate for potential drift
ref.ref[ha.RAR] = 0
ref.ref[ha.RHR] = 0
ref.ref[ha.LAR] = 0
ref.ref[ha.LHR] = 0
r.put(ref)

#move cg to right
for x in range(0,steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RAR] = state.joint[ha.RAR].pos + (.01+maxRAR)/(steps)
	ref.ref[ha.RHR] = state.joint[ha.RHR].pos + (.01+maxRHR)/(steps)
	ref.ref[ha.LAR] = state.joint[ha.LAR].pos + (.01+maxLAR)/(steps)
	ref.ref[ha.LHR] = state.joint[ha.LHR].pos + (.01+maxLHR)/(steps)
	r.put(ref)
	simSleep(state.time, 1, state)	

for x in range(0, 2*steps):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos + 1.0/(2*steps)
	ref.ref[ha.RHP] = state.joint[ha.RHP].pos - .5/(2*steps)
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos - .5/(2*steps)
	r.put(ref)
	simSleep(state.time, .5, state)

for x in range(1,(steps+1)):
	[statuss, framesizes] = s.get(state, wait=False, last=False)
	ref.ref[ha.RKN] = state.joint[ha.RKN].pos - 1.0/steps
	ref.ref[ha.RAP] = state.joint[ha.RAP].pos + 1.0/steps
	r.put(ref)
	simSleep(state.time,.5,state)

# Close the connection to the channels
r.close()
s.close()
