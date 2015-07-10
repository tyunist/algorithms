import pygame,sys
import math
import numpy as np
from myMath import *
from pyCar import *
from __init__ import *


class Constant:
	def __init__(self):
		self.YELLOW = YELLOW

class Environment:
	def __init__(self,(width,height)):
		self.width = width
		self.height = height
		
		self.norm = 1 # to compute effect of a velocity to position, multi velocity with norm. 
		self.drag = DRAG # airforce 
		self.elasticity = 1 # declining velocity after hitting bounce
		self.FPS = FPS 
		self.gravity = GRAVITY
		self.background_colour = (255,255,255)
		self.road = []
		self.carWheelSize = WHEEL_SIZE
		self.wheelTrajectory = []
		self.car = Car((0,0), GRAVITY, V0, 90, MASS, WHEEL_AXIS, WHEEL_SIZE, CHASIS_SIZE)
		self.simTime = 0
		self.realTime = 0
		self.configIndex = 0
		self.vref = VREF 
		self.timeSeries = []
		self.velSeries = []
		self.vrefSeries = []
		self.distance = 0
	
	def modCar(self, (x,y) = (5,10),v0 = 0, angle = 90, kp = KP, ki = KI,  force = GRAVITY, mass = MASS, d = WHEEL_AXIS, size = WHEEL_SIZE, chasisSize = CHASIS_SIZE, wheelcolour = WHEEL_COLOUR, chasisColour = CHASIS_COLOUR  ):
		"""x,y here are coordinates of the right wheel's central point"""
		self.car = Car((x,y), force, v0, angle, mass, d, size, chasisSize, wheelcolour, chasisColour)
		self.car.pid.setPGain(kp)
		self.car.pid.setIGain(ki)
		# print 'asigned vel:',speed,'v0:',self.car.speed
		self.timeSeries.append(0.)
		self.velSeries.append(v0)
	 
	def update(self,ticks): # ticks: time (in milisecond) of each iteration

		alpha = self.road[self.car.segment_index].angle		
		exForce = addVectors(self.gravity, (alpha - 90, self.gravity[1]*math.sin(math.radians(alpha))))
	
		if self.car.angle >=0 and self.car.angle <= 180:
			vNow = self.car.speed
		else:
			vNow = -self.car.speed
		delta_v = self.vref - vNow 
		enForce = (alpha, POW*self.car.PID(delta_v, ticks))
		self.car.force = addVectors(exForce, enForce)
		self.car.estimatePos(self.wheelTrajectory,self.norm)
		# print "g:",self.gravity,"N:",[alpha - 90, self.gravity[1]*math.sin(math.radians(alpha))]
		# print "suppF:", [alpha,self.gravity[1]*math.cos(math.radians(alpha))], "PulF:", [alpha, POW*self.car.PID(delta_v, ticks)]
		# print "total force:", self.car.force, "speed:", [self.car.angle,self.car.speed]
		if self.car.x > self.wheelTrajectory[self.car.segment_index].ex:
			#print "fuk. segment_index:",self.car.segment_index,'x:',self.car.x, 'ex:',self.wheelTrajectory[self.car.segment_index].ex
			deltaMirror =  eulerDistance((self.car.x, self.car.y), (self.wheelTrajectory[self.car.segment_index].ex,self.wheelTrajectory[self.car.segment_index].ey) )
			self.car.segment_index+=1
			if self.car.segment_index < 0 or self.car.segment_index >= len(self.road):
				print 'No next road segment: Out of road. Stopping!!!'
				return 0
			self.car.angle = self.road[self.car.segment_index].angle
			self.car.x = self.wheelTrajectory[self.car.segment_index-1].ex + deltaMirror*math.cos(math.radians(self.car.angle - 90))
			self.car.y = self.wheelTrajectory[self.car.segment_index-1].ey + deltaMirror*math.sin(math.radians(self.car.angle - 90))
			# reset pid controller to 
			# reset pid controller to 
			if self.car.angle != self.road[self.car.segment_index-1].angle:
				self.car.pid.reset() 

		elif self.car.x < self.wheelTrajectory[self.car.segment_index].rx:
			#print "fu. segment_index:",self.car.segment_index,'x:',self.car.x, 'rx:',self.wheelTrajectory[self.car.segment_index].rx
			deltaMirror =  eulerDistance((self.car.x, self.car.y),(self.wheelTrajectory[self.car.segment_index].rx,self.wheelTrajectory[self.car.segment_index].ry ))
			self.car.segment_index-=1
			if self.car.segment_index  < 0 or self.car.segment_index  >= len(self.road):
				print 'No next road segment: Out of road. Stopping!!!'
				return 0
			self.car.angle = self.road[self.car.segment_index].angle - 180
			self.car.x = self.wheelTrajectory[self.car.segment_index+1].rx + deltaMirror*math.cos(math.radians(self.car.angle - 90))
			self.car.y = self.wheelTrajectory[self.car.segment_index+1].ry + deltaMirror*math.sin(math.radians(self.car.angle - 90))
			# reset pid controller to 
			self.car.pid.reset()
		
		# calculate direction and value of the force applied on the car
		# depending of the current road condition, force applied to car change in both direction and magnitude.
		# print ['segment:', self.car.segment_index, 'x:', self.car.x, ' angle: ', self.car.angle]
		self.car.move(self.wheelTrajectory,self.norm) 
		# update time
		self.simTime+= ticks
		# print 'ticks:',ticks, 'simTime:',self.simTime
		self.realTime+= 1*TIME_STAMP
		# update velocity and time series 
		self.timeSeries.append(self.realTime)
		if self.car.angle >=0 and self.car.angle <= 180:
			vNow = self.car.speed
		else:
			vNow = -self.car.speed
		self.velSeries.append(vNow)
		self.vrefSeries.append(self.vref)
		self.distance += vNow*TIME_STAMP
		
	
	def addRoadSegment(self, angle = 0, length = 0, type = 0, (x,y) = (0,0), colour = RED):
		if type == 0: # the original segment
			self.road.append(RoadSegment(RED, (x, y), angle, length))
			# print "first road"
		else:
			nextSegment = len(self.road)
			(rx,ry) = (self.road[nextSegment-1].ex, self.road[nextSegment-1].ey) 
			self.road.append(RoadSegment(RED, (rx,ry), angle, length))
		# print['Segment ', len(self.road)+1, ' angle: ', self.road[len(self.road) - 1].angle, ' length: ', self.road[len(self.road) - 1].length]
	
	def genTrajectory(self):
		""""This function generate imaginary trajectory for the wheel's center """
		if len(self.road) < 1:
			print "ROAD LENGTH = 0. Exitting"
			return 0
		for i in range(len(self.road)):
			# calculate starting point of a segment
			if i == 0: # first segment
				rx = self.road[0].rx
				ry = self.road[0].ry - self.carWheelSize/math.sin(math.radians(self.road[0].angle))
			else:
				rx = self.wheelTrajectory[i-1].ex
				ry = self.wheelTrajectory[i-1].ey
			# calculate delta x which is the distance that the road corner shifts 
			if i == len(self.road) - 1: # final segment 
				ex = self.road[i].ex 
				ey = self.road[i].ey - self.carWheelSize 
			else:
				angle1 = self.road[i].angle
				angle2 = self.road[i+1].angle
				beta = (180 + angle2 - angle1)/2
				gamma = (angle1 + angle2)/2 -90
				R = self.carWheelSize/math.sin(math.radians(beta))
				delta_x = math.sin(math.radians(gamma))*R
				delta_y = math.cos(math.radians(gamma))*R
				ex = self.road[i].ex + delta_x 
				ey = self.road[i].ey - delta_y 
			wheelSegment = RoadSegment(GREEN, (0,0), 0, 0)
			wheelSegment.roadSegment((rx,ry),(ex,ey), self.road[i].angle)
			self.wheelTrajectory.append(wheelSegment)
	def addTrajectorySegment(self):
		""""This function adds an imaginary trajectory segment for the wheel's center """
		if len(self.road) < 1:
			print "ROAD LENGTH = 0. Exitting"
			return 0
		# up-to-date wheelTrajectory index:
		nextSegment = len(self.wheelTrajectory) 
		print "Next wheelTrajectory segment:", nextSegment
		print "road segment number is:", len(self.road)
		for i in range(nextSegment,len(self.road)):
			# calculate starting point of a segment
			if i == 0: # first segment
				rx = self.road[0].rx
				ry = self.road[0].ry - self.carWheelSize/math.sin(math.radians(self.road[0].angle))
			else:
				rx = self.wheelTrajectory[i-1].ex
				ry = self.wheelTrajectory[i-1].ey
				print "starting points:", rx, ry 
			# calculate delta x which is the distance that the road corner shifts 
			if i == len(self.road) - 1: # final segment 
				ex = self.road[i].ex 
				ey = self.road[i].ey - self.carWheelSize 
			else:
				angle1 = self.road[i].angle
				angle2 = self.road[i+1].angle
				beta = (180 + angle2 - angle1)/2
				gamma = (angle1 + angle2)/2 -90
				R = self.carWheelSize/math.sin(math.radians(beta))
				delta_x = math.sin(math.radians(gamma))*R
				delta_y = math.cos(math.radians(gamma))*R
				ex = self.road[i].ex + delta_x 
				ey = self.road[i].ey - delta_y 
			wheelSegment = RoadSegment(GREEN, (0,0), 0, 0)
			wheelSegment.roadSegment((rx,ry),(ex,ey), self.road[i].angle)
			self.wheelTrajectory.append(wheelSegment)
	# def vPlot()
		# """This function plots online velocity over x-value of the vehicle"""
	
	def planner(self,planner = [(50,10), (10000,20)], targetDistance = TARGET_DISTANCE, mode=1 ): # mode =1 : test running time;  mode= 0: accProfile.py 
		"""this function define a plan for the vehicle to follow"""
		'''Modified at 5:28 AM 19/04/2015: multiply VEL_SCALE and divide some by 10 for convenient with planGenerator.py's output.'''
		# print "planner:",planner 
		# add initial value to vref series 
		if len(self.vrefSeries) == 0:
			self.vrefSeries.append(planner[0][1]) 
		# check, if distance > targetDistance -> stop
		if self.distance >= targetDistance:
			print 'finish target distance:', self.distance
			return 1 
		# else:
			# print '\t current distance:', self.distance,' vs targetDistance:',targetDistance
		print 'plan number', self.configIndex
		if  self.realTime - planner[self.configIndex][0] > 0.01:
			self.vref = planner[self.configIndex][1]
			# move on to the next configuration 
			self.configIndex+=1
			if mode == 1:
				if self.configIndex >= len(planner):
					if self.vref > 0: # vref > 0, otherwise, it will run forever in case vref = 0 and 
				# current distance < targetDistance
						print "end of planner. Continue with vref = vel of the ending element of the planner"
						self.configIndex -= 1
						return 0
					else:
						print 'end of planner but not end of targetDistance.'
						return 1 
			else:
				if self.configIndex >= len(planner):
					print "end of planner"
					return 1
			return 0
		
		self.vref = planner[self.configIndex-1][1]
		return 0

		
		
		
