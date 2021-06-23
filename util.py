#!/usr/bin/env python3

def clamp(v, limit):
    if v > limit:
        return limit
    elif v < -limit:
        return -limit
    else:
        return v
