#!/usr/bin/env python
#-*- coding:utf-8 -*-

def InverUint8(dat):
    temp = 0
    for i in range(8):
        if dat & (1 << i) > 0:
            shift_bit = 7 - i
            temp |= 1 << shift_bit
    return temp

def InverUint16(dat):
    temp = 0 
    for i in range(16):
        if dat & (1 << i) > 0:
            shift_bit = 15 - i 
            temp |= 1 << shift_bit
    return temp

for i in range(256):
    print hex(InverUint16(i))
