#!/usr/bin/env python

import numpy as np
import math
target_R = np.array([[-.25],[-.2], [.2]])
target_L = np.array([[.25],[-.2], [.2]])

origin_R = np.array([[-.2145],[0],[0]])
origin_L = np.array([[.2145],[0],[0]])

def rot():
	A1_R = np.array([[1,0,0,origin_R[0,0]],[0,1,0,origin_R[1,0]],[0,0,1,origin_R[2,0]],[0,0,0,1]])
	A1_L = np.array([[1,0,0,origin_L[0,0]],[0,1,0,origin_L[1,0]],[0,0,1,origin_L[2,0]],[0,0,0,1]])
	A2 = np.array([[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]])
	A3 = np.array([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]])

	T_R = A3.dot(A2.dot(A1_R))
	T_L = A3.dot(A2.dot(A1_L))
	return T_R, T_L

def test(test):
	if (test == 'R'):
		print 'right'
	else:
		print 'wrong'

T_R, T_L = rot()
test('R')
print T_R
print T_L

