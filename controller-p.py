#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro
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



import controller_include as ci
import ach
import sys
import time
import numpy as np
from ctypes import *
import curses



# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
c = ach.Channel(ci.CONTROLLER_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
controller = ci.CONTROLLER_REF()

#intialize curses module
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(True)
stdscr.nodelay(True)

# Get the current feed-forward (state) 
while (True):
	inputKey = stdscr.getch()
	if inputKey == ord('x'): #exit on X key and close curses module and ach channel
		curses.nocbreak()
		stdscr.keypad(False)
		curses.echo()
		curses.endwin()
		c.close()
		break
	if inputKey == curses.KEY_RIGHT:
		controller.mot1 = .325
		controller.mot2 = 1
		c.put(controller)
	if inputKey == curses.KEY_LEFT:
		controller.mot1 = 1
		controller.mot2 = .325
		c.put(controller)
	if inputKey == curses.KEY_UP:
		controller.mot1 = .5
		controller.mot2 = .5
		c.put(controller)
	if inputKey == curses.KEY_DOWN:
		controller.mot1 = -.5
		controller.mot2 = -.5
		c.put(controller)

        if inputKey == ord('s'):
		controller.mot1 = 0
		controller.mot2 = 0
		c.put(controller)

	time.sleep(.25)


