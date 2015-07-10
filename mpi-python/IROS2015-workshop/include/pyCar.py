import pygame,sys
import math
import numpy as np
from myMath import *
from env import *
from __init__ import *
import PID  
class Car:   
	"""This class defines a car with two wheels and one chasis. Each object consists of coordinates of the center of the 
	right wheel, its angle to the y axis and size and colour of each component.
	(x,y) here is the coordinates of the right wheel. """
	def __init__(self,(x,y) = (0,0), force = (0,0), speed = 0, angle= 90, mass = 1000, d = 40, wheelSize = 20, chasisSize = (40,30), wheelcolour = (0,0,0), chasisColour = (0,0,128) ):
		self.rightWheel = Particle(YELLOW,(x,y),wheelSize,0)
		self.x = x
		self.y = y
		self.d = d 
		self.mass = mass 
		self.force = force
		self.drag = 0.999
		self.speed = speed
		self.angle = angle
		beta = 180 - angle
		# shift a distance from (rx,ry) to have leftwheel's center position
 		delta_x = d*math.sin(math.radians(beta))
		delta_y = d*math.cos(math.radians(beta))
		lx = self.x - delta_x  
		ly = self.y - delta_y
		chaCenx = (lx + self.x)/2
		chaCeny = (ly + self.y)/2
		self.leftWheel = Particle(wheelcolour,(lx,ly),wheelSize, 0)
		self.chasis = Chasis((chaCenx, chaCeny), chasisSize, angle)
		self.wheelSize = wheelSize
		self.wheelMass = 0
		self.chasisColour = chasisColour
		self.chasisSize = chasisSize	
		self.segment_index = 0
		# controller
		self.pid = PID.PID(KP, KI) 
		# self.braker = PID.PID(kp_brake, ki_brake)
		
	def findLeftWheel(self, wheelTrajectory,(x,y)):
		"""This function find the position of the left(back) wheel of the vehicle given the right wheel position"""
		segment_index = self.segment_index
		# have to check after update 
		if segment_index == 0:
			if self.angle > 0 and self.angle < 180:
				(lx,ly) = (x - self.d*math.sin(math.radians(180-self.angle)), y - self.d*math.cos(math.radians(180-self.angle)))
			else: 
				(lx,ly) = (x + self.d*math.sin(math.radians(180-self.angle)), y + self.d*math.cos(math.radians(180-self.angle)))
			if lx > x:
				print 'hell in case 1'
			return (lx,ly)
		# use distance between current's segment starting point and rightWheel center to determine cases 
		(x2,y2) = (wheelTrajectory[segment_index].rx, wheelTrajectory[segment_index].ry)
		a = eulerDistance((x,y), (x2,y2))
		
		# there is case 2 in which the road segment length is shorter than the chasis so that two wheels lie on two jointed or disjointed segments
        # this depends on the length of road segment which the front wheel is lying on. 
		left_wheel_segment_index = segment_index-1
		if a < self.d and x >= x2: # the rightWheel is on current segment while the leftWheel is still on one of previous segments
			while(True):
				(x1,y1) = (wheelTrajectory[left_wheel_segment_index].rx, wheelTrajectory[left_wheel_segment_index].ry)
				beta = 180 + wheelTrajectory[segment_index].angle - wheelTrajectory[left_wheel_segment_index].angle
				# the triangle used to calculate leftWheel position changed to another one 
				# first, (x2,y2) now turn to intersection between the two segment on which the two wheels lie 
				(x12, y12) = (wheelTrajectory[left_wheel_segment_index].ex, wheelTrajectory[left_wheel_segment_index].ey)
				(x3,y3) = (wheelTrajectory[segment_index].rx, wheelTrajectory[segment_index].ry)
				(x4,y4) = (wheelTrajectory[segment_index].ex, wheelTrajectory[segment_index].ey)
				(x2,y2) = twoLineCollision((x1,y1), (x12,y12), wheelTrajectory[left_wheel_segment_index].angle, (x3,y3),(x4,y4), wheelTrajectory[segment_index].angle)
				if (x2,y2) == (-1000, -1000): 
					print "Two segment are parallel"
					break 
				a = eulerDistance((x,y), (x2,y2))
				# distance between leftWheel and end point of previous segment:
				b = cosinLaw(a, self.d, beta)
				#print ['angle1: ',wheelTrajectory[left_wheel_segment_index].angle , 'angle2: ', wheelTrajectory[segment_index].angle," beta: ", beta, " b: ", b, " x: ", a]
				# now, knowing that leftWheel center is on a line with its distance from the endpoint is given. we can calculate it. 
				(lx,ly) = pointOnLine((x1,y1), (x2,y2), b)
				if lx >= x1: # the leftWheel is right on its current road segment 
					#print "okie","segment_index:", segment_index, "left_wheel_segment_index:", left_wheel_segment_index
					return (lx,ly)
				left_wheel_segment_index-= 1
				#print "okie","segment_index:", segment_index, "left_wheel_segment_index:", left_wheel_segment_index
		
		# case 3
		# the both two wheels are on the current segment 
		# print [segment_index, wheelTrajectory[segment_index].rx, wheelTrajectory[segment_index].ry, wheelTrajectory[segment_index].ex, wheelTrajectory[segment_index].ey, x, y]
		(x1,y1) = (wheelTrajectory[segment_index].rx, wheelTrajectory[segment_index].ry)
		(x2,y2) = (x,y)
		(lx,ly) = pointOnLine((x1,y1), (x,y), self.d)
		# if lx > x:
			# print 'hell in case 3'
			# print [a, self.d, x1, y1, x, y]
		return (lx,ly)
	
	def estimatePos(self, wheelTrajectory, norm):
		# static force. Enable or not. 
		# if self.speed < 0.01:
			# static_force = 1 
			# min_force = abs(GRAVITY[1]*math.sin(math.radians(self.angle)))
			# if self.force[1] <= min_force:
				# return 
		(self.angle, self.speed) = addVectors((self.angle, self.speed), self.force)
		if self.angle > 180 or self.angle < 0:
			self.speed = 0

		delta_x = math.sin(math.radians(self.angle))*self.speed*norm*TIME_STAMP
		delta_y = math.cos(math.radians(self.angle))*self.speed*norm*TIME_STAMP
		self.speed *= self.drag

		self.x+= delta_x
		self.y-= delta_y	
	
	def move(self, wheelTrajectory,norm):
		self.rightWheel.move(self.force, (self.x, self.y), self.angle, self.speed)
		# find out leftWheel position 
		(lx,ly) = self.findLeftWheel(wheelTrajectory, (self.x, self.y))
		self.leftWheel.move(self.force, (lx, ly), self.angle, self.speed)
		# find out the center of the lower line of the chasis:
		chaCenx = (lx + self.x)/2
		chaCeny = (ly + self.y)/2
		self.chasis.move((chaCenx,chaCeny), math.degrees(-math.atan2(self.y - ly, self.x - lx)))
	
	def PID(self, delta_v, ticks):
		#print 'ticks:', ticks
		cmd = self.pid.update(delta_v, ticks) # ticks here must be in second, otherwise, divide it by 1000. 
		# print "pid controller: delta_v:", delta_v, "ticks:",ticks, "cmd:", cmd
		return cmd
		# return delta_v*kp
	
	# def brake(self,delta_v,ticks)
		# cmd = cmd = self.braker.update(delta_v, ticks) 
		# return cmd 
	
class Chasis:
	"""This class defines a chasis of a car with given width, height, coordinates of the center of the lower side"""
	def __init__(self,(x,y), (width,height), angle, colour = (0,0,128), thickness = 4):
		self.x = x
		self.y = y
		self.angle = angle
		self.width = width
		self.height = height
		self.points = []

		delta_y = width*math.sin(math.radians(angle))/2
		delta_x = width*math.cos(math.radians(angle))/2
		llpoint = (self.x - delta_x, self.y + delta_y)
		rlpoint = (self.x + delta_x, self.y - delta_y)
		
		delta_x = height*math.sin(math.radians(angle))
		delta_y = height*math.cos(math.radians(angle))
		lhpoint = (llpoint[0] - delta_x, llpoint[1] - delta_y)
		rhpoint = (rlpoint[0] - delta_x, rlpoint[1] - delta_y)
		self.points.append(llpoint)
		self.points.append(lhpoint)
		self.points.append(rhpoint)
		self.points.append(rlpoint)
	
	def move(self, (x,y), angle):
		"""x,y here are coordinates of the center of the lower side of the chasis, angle is the angle of the chasis"""
		self.x = x
		self.y = y
		self.angle = angle


		delta_y = self.width*math.sin(math.radians(angle))/2
		delta_x = self.width*math.cos(math.radians(angle))/2
		llpoint = (self.x - delta_x, self.y + delta_y)
		rlpoint = (self.x + delta_x, self.y - delta_y)
		
		delta_x = self.height*math.sin(math.radians(angle))
		delta_y = self.height*math.cos(math.radians(angle))
		lhpoint = (llpoint[0] - delta_x, llpoint[1] - delta_y)
		rhpoint = (rlpoint[0] - delta_x, rlpoint[1] - delta_y)
		
		self.points[0] = llpoint
		self.points[1] = lhpoint
		self.points[2] = rhpoint
		self.points[3] = rlpoint

		
class Particle:
	def __init__(self,colour,(x,y), size, mass = 1):
		self.x = x
		self.y = y
		self.size = size
		self.colour = colour
		self.thickness = size
		self.mass = mass
		self.force =  (180, 0.05)
		self.drag = 0.999
		self.speed = 0
		self.angle = 0
		
	def move(self,force,(x, y), angle, speed):
		self.x = x
		self.y = y
		self.force =  force
		self.speed = speed
		self.angle = angle
			
class RoadSegment:
	def __init__(self, colour,(rx, ry), angle, length, thickness = 4 ):
		self.length = length
		self.angle = angle
		self.rx = rx
		self.ry = ry
		self.ex = self.rx + math.cos(math.radians(self.angle - 90))*self.length
		self.ey = self.ry + math.sin(math.radians(self.angle - 90))*self.length		
		self.colour = colour
		self.thickness = thickness
	def roadSegment(self, (rx, ry), (ex,ey), angle):
		self.rx = rx
		self.ry = ry
		self.ex = ex
		self.ey = ey
		self.length = math.sqrt(np.power(ry-rx,2)+ np.power(ey-ex,2))
		# self.angle = math.degrees(math.atan2(ey-ry,ex-rx))
		self.angle = angle