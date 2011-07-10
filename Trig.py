# Basic trig functions

import math

def direction(x1, y1, x2, y2):
	""" Returns the angle of a line defined by two points
	    relative to the horizontal access. """

	return math.atan2(y2-y1, x2-x1)

def unitVectorFromAngle(angle):
	""" Returns the unit vector for a given angle """

	return (math.cos(angle), math.sin(angle))

def endOfVector(x, y, angle, dist):
	""" Returns the point at the tip of the vector described
	    by point (x,y) at angle relative to horizontal and distance """

	x2, y2 = unitVectorFromAngle(angle)
	x2 *= dist
	y2 *= dist
	return (x+x2,y+y2)

def distance(x1, y1, x2, y2):
	""" distance between two points """

	return math.sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) )

def __dot__(u,v):
	return u[0]*v[0] + u[1]*v[1]

def pointRayDistance(px, py, rx0, ry0, rx1, ry1):

	v = (rx1 - rx0, ry1 - ry0)
	w = (px - rx0, py - ry0)
	
	c1 = __dot__(w,v)
	if (c1 <= 0):
		return distance(px, py, rx0, ry0)
	
	c2 = __dot__(v,v)
	b = float(c1) / float(c2)
	bx, by = (rx0 + b*v[0]), (ry0 + b*v[1])
	return distance(px, py, bx, by)
