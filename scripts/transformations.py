#!/usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point

def point2grid(x0, y0, x1, y1, width, height, resolution):

    gridX = int(round((x0-x1)/resolution))
    gridY = int(round((y0-y1)/resolution))

    if x1 == (width + x0):
        gridX -=1
    if y1 == (height + y0):
        gridY -=1

    return(gridX, gridY)

def scan2world(x, y, w, dx, dy):

    rotation = np.array([[math.cos(w), -math.sin(w), 0.0], 
                        [math.sin(w), math.cos(w), 0.0],
                        [0.0, 0.0, 1.0]])

    translation = np.array([[1.0, 0.0, dx], 
                            [0.0, 1.0, dy], 
                            [0.0, 0.0, 1.0]])
    
    prod = np.dot(translation, rotation)

    vector = np.array([x, y, 1])

    result = np.dot(prod, vector)

    return result

