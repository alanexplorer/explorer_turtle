#!/usr/bin/env python
import numpy as np
import math

# Calculate difference between 2 angles
# wrap the result into the interval [-pi pi]
def angdiff(th1, th2):
    d = th1 - th2
    d = np.mod(d+np.pi, 2*np.pi) - np.pi
    return d

# Calculate distance between 2 points
def calcDistance(P1, P2):
    x = math.sqrt((P1[0] - P2[0])**2 + (P1[1] - P2[1])**2)
    return x

def normangle(th):
    if th > np.pi:
        th = 2*np.pi - th
    if th < -np.pi:
        th = - 2*np.pi - th
    return th