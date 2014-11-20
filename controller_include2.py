#!/usr/bin/env python

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

DYNO_REF_NAME = 'dyno-ref-chan'

class DYNO_REF(Structure):
    _pack_ = 1
    _fields_ = [("dThetaX",    c_double),
                ("dThetaY",    c_double)]
                
