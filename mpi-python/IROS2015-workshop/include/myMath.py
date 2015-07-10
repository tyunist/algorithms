import math
import numpy as np
RE = 1e-6
def addVectors((angle1, length1), (angle2, length2)):
	x  = math.sin(math.radians(angle1)) * length1 + math.sin(math.radians(angle2)) * length2
	y  = math.cos(math.radians(angle1)) * length1 + math.cos(math.radians(angle2)) * length2
	length = math.hypot(x, y)
	angle = math.degrees(0.5 * math.pi - math.atan2(y, x))
	return (angle, length)

def cosinLaw(a,c, angle_c):
	"""This function imploying cosin law to find the side b of a triangle given one angle and two sides"""
	"""Turn it to a quadric equation of b based on cosine law: c^2 = a^2 + b^2 - 2*a*b*cosC"""
	if a < 0 or c < 0:
		print "cosinLaw is not applied to a triangle with a negative side"
		return -1
	deltt = np.power(a,2)*np.power(math.cos(math.radians(angle_c)),2) + np.power(c,2) - np.power(a,2)
	b = a*math.cos(math.radians(angle_c)) + math.sqrt(deltt)
	if b >=0 and a+b-c >=0 and a+c-b >=0 and b+c-a >=0:
		return b
	else: 
		return a*math.cos(math.radians(angle_c)) - math.sqrt(deltt)

def findSegment(wheelTrajectory, x):
	"""This function returns the segment index of which the front wheel is currently on"""
	for i in range(len(wheelTrajectory)):
		if abs(wheelTrajectory[i].ex - x) < 1e-9:
			return i 
		elif  wheelTrajectory[i].ex > x and wheelTrajectory[i].rx < x:
			return i 
	print "out of road. Exit...ing"
	return -1
		
def eulerDistance((x1,y1), (x2,y2)):
	"""This function returns the Euler distance of two points given their coordinates"""
	return math.sqrt(np.power(x2- x1,2) + np.power(y2-y1,2))

def pointOnLine((x1,y1), (x2,y2), b):
	"""This function returns the position of a point on a two-point-bounded line segment given its distance to the end point"""
	l = eulerDistance((x1,y1), (x2,y2))
	if l < 1e-6: 
		return (x1,y1)
	# using the property: (x-x2)/(x1-x2) = (y-y2)/(y1-y2) = b/l
	if abs(x1-x2) < 1e-9:
		x = x2 
		y = y2 + (y1-y2)*b/l
	elif abs(y1-y2) < 1e-9:
		y = y2 
		x = x2 + (x1-x2)*b/l
	else:
		x = x2 + (x1-x2)*b/l
		y = y2 + (y1-y2)*b/l
	return (x,y)

def twoLineCollision((x1,y1),(x2,y2), angle1,(x3,y3),(x4,y4), angle2 ):
	"""This function returns the intersection point between two lines and return (-1000,-1000) if the two lines are parallel"""
	if abs(angle1-angle2) < 1e-3:
		return (-1000,-1000)
	if abs(x2-x1) < 1e-6:
		x = x2
		y = y3 + (y4-y3)*(x2-x3)/(x4-x3)
		print "case 2"
		return (x,y)
	if abs(x4-x3) < 1e-6:
		x = x3
		y = y1 + (y2-y1)*(x3-x1)/(x2-x1)
		print "case 3"
		return (x,y)
	A1 = (y2-y1)*1.0/(x2-x1)
	B1 = ((x2-x1)*y1 - (y2-y1)*x1)*1.0/(x2-x1)
	A2 = (y4-y3)*1.0/(x4-x3)
	B2 = ((x4-x3)*y3 - (y4-y3)*x3)*1.0/(x4-x3)
	x  = (B2-B1)*1.0/(A1-A2)
	y  = (A1*B2 - A2*B1)*1.0/(A1-A2)
	return (x,y)

def valueWithSign(speed,angle):
	"""This function returns a positive value if the vehicle moves to the right and vice versa"""
	if angle >=0 and  angle <= 180:
		return speed 
	return -speed 
def inSegment((m1,n1), (x1,y1),(x2,y2)):
	"""This function checks whether a given point (m1,n1) is in between two points"""
	if -1e-4 < eulerDistance((m1,n1), (x1,y1))+ eulerDistance((m1,n1), (x2,y2)) - eulerDistance((x1,y1),(x2,y2)) < 1e-4:
		return 1 
	return 0 

def circleSegmentCollision((a,b),Radius,(x1,y1),(x2,y2)):
	"""This function detects collision if has between a circle and a line segment"""
	# distance from center to two points A (x1,y1) and B (x2,y2)
	AC = eulerDistance((a,b),(x1,y1))
	BC = eulerDistance((a,b),(x2,y2))
	# let consider local coordinates system based on circle'center 
	localP1 = (x1-a, y1-b)
	localP2 = (x2-a, y2-b)
	P2MinusP1 = (localP2[0] - localP1[0], localP2[1] - localP1[1])
	
	# supply parameters 
	a = (P2MinusP1[0]) * (P2MinusP1[0]) + (P2MinusP1[1]) * (P2MinusP1[1])
	b = 2 * ((P2MinusP1[0] * localP1[0]) + (P2MinusP1[1] * localP1[1]))
	c = np.power(localP1[0],2) + np.power(localP1[1],2) - np.power(Radius,2)
	delta = np.power(b,2) - 4.*a*c
	
	if AC <= Radius and BC <= Radius: # segment is inside the circle 
		print "segment inside circle"
		return 1 

	if (delta < 0): # No intersection
		print "no cut at all"
		return 0;
	elif abs(delta) < 1e-6: # One intersection
		u = -b / (2 * a)
		(m1,n1) = (x1 + u*P2MinusP1[0], y1 + u*P2MinusP1[1])
		print "delta is zero"
		return inSegment((m1,n1), (x1,y1),(x2,y2))
		# Use (x1,y1) instead of localP1 because we want our answer in global
		#  space, not the circle's local space 
	elif (delta > 0): # Two intersections
		print "delta:", delta
		SquareRootDelta = math.sqrt(delta)
		u1 = (-b + SquareRootDelta) / (2 * a)
		u2 = (-b - SquareRootDelta) / (2 * a)
		(m1,n1) = (x1 + u1*P2MinusP1[0], y1 + u1*P2MinusP1[1])
		(m2,n2) = (x1 + u2*P2MinusP1[0], y1 + u2*P2MinusP1[1])
		print "there are two cuts but not sure in segment or not"
		return inSegment((m1,n1), (x1,y1),(x2,y2)) or inSegment((m2,n2), (x1,y1),(x2,y2)), (m1,n1),(m2,n2)
 # def circleLineCollision(p1, p2, angle, nextAngle, c, r):
	# """ circleLineCollision(p1, p2, angle, nextAngle c, r)
	# p1: the first line point
	# p2: the end line point
	# angle: angle of the line segment
	# nextAngle: angle of the next line segment
	# c,r: circle' center and radius"""
	# # check whether center point is on the line segment
	# if c[0] - p1[0] >= RE and p2[0] - c[0] >= RE:
		# print "center point is on the line segment"
		# # return two intersection points 
		# return c[0] - r.math.sin(math.radians(180-angle)), c[1] - r.math.cos(math.radians(180-angle)))
	# # check whether center point is in the range to cut the line segment 
	# elif c[0] - p2[0] >= RE + r.math.sin(math.radians(180-nextAngle)):
		# print "circle does not cut the line segment cuz its too far from the end point"
		# return ()
	# # final case: center on the next line segment but still cut the segment line 
	# else: 
		# print "circle cuts the line segment though its on the next segment"
		