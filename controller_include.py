#!/usr/bin/env python

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

CONTROLLER_REF_NAME              = 'controller-ref-chan'

class CONTROLLER_REF(Structure):
    _pack_ = 1
    _fields_ = [("x",    c_int16),
                ("y",    c_int16)]
                
