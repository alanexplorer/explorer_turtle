#!/usr/bin/env python

import math

def bresenham(x0, y0, x1, y1):

	# Setup initial conditions
	dx = x1 - x0
	dy = y1 - y0

	# Determine how steep the line is
	is_steep = abs(dy) > abs(dx)

	# Rotate line if it is steep
	if is_steep:
		x0, y0 = y0, x0
		x1, y1 = y1, x1

	# Swap the start and end points if necessary and stor swap state
	swapped = False
	if x0 > x1:
		x0, x1 = x1, x0
		y0, y1 = y1, y0
		swapped = True

	# Recalculate differentials
	dx = x1 - x0
	dy = y1 - y0

	# Calculate error
	error = int(dx/2.0)
	ystep = 1 if y0 < y1 else -1

	# Iterate over bounding box generation points between start and end
	y = y0
	points = []
	for x in range(x0, x1 + 1):
		coord = (y, x) if is_steep else (x, y)
		points.append(coord)
		error -= abs(dy)
		if error < 0:
			y += ystep
			error += dx

	# Reverse the list if the coordinates were swapped
	if swapped:
		points.reverse()

	return points