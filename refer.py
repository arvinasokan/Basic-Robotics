from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

class my_msg(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_double*9)]
