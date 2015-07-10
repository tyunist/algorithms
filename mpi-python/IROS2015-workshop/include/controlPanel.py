# controlPanel
# last editted by Ty Nguyen 1:00 AM 4/02/2015
# this function set up all fundamental aspects of simulation. 
# to use it, just import and use function simulation(GUI_ON = 0) 
import pygame
import random
import math
import time 
import numpy as np
from env import *
from pyCar import *
import myTime
import matplotlib.pyplot as plt
import helpers 

def envLaunch(road_angles = ROAD_ANGLES, road_lengths = ROAD_LENGTHS, v0 = 0, kp = KP, ki = KI):
	env = Environment((width, height))
	env.addRoadSegment(road_angles[0], road_lengths[0], 0, (rx_orig, ry_orig))
	# print ['Segment 1:  angle ', road_angles[0],' length:', road_lengths[0]]
	number_of_road_segments = len(road_angles)
	for i in range(1,number_of_road_segments):
		# angle = ROAD_ANGLES[i]
		# length = ROAD_LENGTHS[i]
		angle = road_angles[i]
		length = road_lengths[i]
		env.addRoadSegment(angle, length, 1) # 1 means not the first segment
		# print['Segment ', i+1, ' angle: ', env.road[i].angle, ' length: ', env.road[i].length]

	env.genTrajectory()
	# for segment in env.wheelTrajectory:
		# print 'wheelTrajectory segment: ', segment.rx, segment.ry, segment.ex, segment.ey
	x = env.wheelTrajectory[0].rx + math.sin(math.radians(180-env.road[0].angle))*WHEEL_AXIS*2
 	y = env.wheelTrajectory[0].ry + math.cos(math.radians(180-env.road[0].angle))*WHEEL_AXIS*2
	# print "x, y are: "
	# print [x,y, env.road[0].rx, env.road[0]]
	env.modCar((x,y), v0, env.road[0].angle, kp, ki)
	return env

class GUI:
	def __init__(self, origin = (0,0)): 
		self.vArray = [origin]
		
	
	# display car animation 
	def animation(self, screen, env):
		p = env.car.rightWheel
		screen.fill(env.background_colour)
		for segment in env.road:
			pygame.draw.line(screen, segment.colour, (segment.rx, segment.ry), (segment.ex, segment.ey), segment.thickness)
		for segment in env.wheelTrajectory:
			pygame.draw.line(screen, segment.colour, (segment.rx, segment.ry), (segment.ex, segment.ey), segment.thickness)
		pygame.draw.polygon(screen,env.car.chasisColour, (env.car.chasis.points) )
		pygame.draw.circle(screen, p.colour, (int(p.x), int(p.y)), p.size, p.thickness)
		pygame.draw.circle(screen, env.car.leftWheel.colour, (int(env.car.leftWheel.x), int(env.car.leftWheel.y)), p.size, p.thickness)
		
	
	def velPlot(self, screen, env):
		p = env.car.rightWheel
		yValue = height - p.speed/100*height 	
		self.vArray.append((p.x, yValue))
		# draw velocity function
		pygame.draw.lines(screen, AQUA, False, tuple(self.vArray), 4)

	
	def update(self):
		pygame.display.update()
	
	def figure(self,env, kp, ki , VEL_ON = 0):
		"""This function plot velocity value over time on a figure"""
		if VEL_ON == 0:
			return 
		plt.figure()
		plt.plot(env.timeSeries, env.velSeries, 'r', env.timeSeries, env.vrefSeries,'b')
		plt.title('Velocity Response with kp:' + str(kp) + ',ki:' + str(ki))
		plt.xlabel('time(second)')
		plt.ylabel('velocity(m/s)')
		plt.grid(True)
		plt.show()
		return
	
	def writeData(self,env, fileName = 'velResponse.csv'):
		for i in range(len(env.timeSeries)):
			# print "writing to file"
			list = (env.timeSeries[i], env.vrefSeries[i], env.velSeries[i])
			helpers.writeRow(list, fileName)

class Simulation:
	"""# simple mode is 0"""
	def __init__(self, MODE = 0,GUI_ON = 0, VEL_ON = 0, road_angles = ROAD_ANGLES, road_lengths = ROAD_LENGTHS, v0 = 0, (tEnd,vEnd) = (0,0), kp = KP, ki = KI, planner = [(50,10),(10000,20)], fileName = "velResponse"):
		self.gFlag = GUI_ON
		self.vFlag = VEL_ON
		self.road_angles = road_angles
		self.road_lengths = road_lengths
		self.v0 = v0
		self.kp = kp
		self.ki = ki
		self.fileName = fileName
		self.planner = planner 
		self.arrConfig = (tEnd,vEnd)
		if MODE == 1: # test kp, ki mode 
			print "Test mode"
			self.env = envLaunch(self.road_angles, self.road_lengths, self.v0, self.kp, self.ki)
		else: 
			print "Simple mode"
			self.env = envLaunch(self.road_angles, self.road_lengths, self.v0)
	
	
 	def launch(self): 
		GUI_ON = self.gFlag
		VEL_ON = self.vFlag
		gui = GUI((rx_orig, height))
		if GUI_ON == 1:
			screen = pygame.display.set_mode((width, height))
			screen.fill(self.env.background_colour)
			pygame.display.set_caption('Gravity Test by Ty Nguyen')
		
		ticks = 0
		fpsClock = pygame.time.Clock()
		running = True
		timer = myTime.Time()
		while running:
			circleTime = timer.elapse()
			if GUI_ON==1:
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						gui.writeData(self.env, self.fileName)
						gui.figure(self.env,self.kp, self.ki,VEL_ON)
						running = False
			# make a plan for the car
			self.env.planner(self.planner)
			# execute plan
			if self.env.update(circleTime) == 0:
				gui.writeData(self.env, self.fileName)
				# print "time series:", self.env.timeSeries
				# print "vel series:", self.env.velSeries
				gui.figure(self.env,self.kp, self.ki,VEL_ON)
				running = False
			
			print 'simTime:', self.env.simTime, 'realTime:',self.env.realTime, 'vref:', self.env.vref,'vel:', self.env.car.speed
			# Gui display or not 
			if GUI_ON==1:
				gui.animation(screen, self.env)
				gui.velPlot(screen, self.env)
				gui.update()
			ticks=fpsClock.tick(self.env.FPS)
			print circleTime
