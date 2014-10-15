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




import ach
import sys
import time
import math
from ctypes import *
import ike_include as ike

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
k = ach.Channel(ike.CONTROLLER_REF_NAME)
coordinates = ike.CONTROLLER_REF()

with open("hw7-ik.txt", "r") as coords:
	for i in range(0,6):
		for line in coords:
			x = float(line.rstrip('\n').rsplit(" ", 3)[0])
			y = float(line.rstrip('\n').rsplit(" ", 3)[1])
			z = float(line.rstrip('\n').rsplit(" ", 3)[2])
			coordinates.x = x
			coordinates.y = y
			coordinates.z = z
			k.put(coordinates)		
			print x, y, z
		coords.seek(0)
		print i
		simtime = tim.sim[0]
		timeSleep = simtime + 3
		while(simtime < timeSleep):
			[status, framesize] = t.get(tim, wait=False, last=True)
			simtime = tim.sim[0]
	coords.close()
