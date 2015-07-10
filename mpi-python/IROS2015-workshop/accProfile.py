# accProfile.py 
# this function do experiments to build up acceleration profile of the vehicle. 
# velocity range are 0:2:50 (m/2)
import os
path = os.getcwd() + '/include/'
import sys
sys.path.insert(0, path)
from controlPanel import *
import random
from controlPanel import *
from myMath import *
import helpers
from __init__ import * 
# running in default is without GUI
class CheckStable:
	def __init__(self):
		self.hold = 0
		self.holdTime = POS
		self.stableTime = 0

	def checkStable(self,env,v1):
		if abs(valueWithSign(env.car.speed,env.car.angle) - v1) < 0.1:
			if self.hold == 0:
				# print "start to hold"
				self.hold = 1
				self.stableTime = env.realTime
				return 0
			else:
				self.holdTime -= 1*TIME_STAMP
				# print "reducing time, holdTime:", self.holdTime
				if self.holdTime <= 0 :
					# print "success stable time" 
					
					return 1 
				return 0
		# print "still no hold", "vel: ", valueWithSign(env.car.speed,env.car.angle) 
		self.hold = 0
		self.holdTime = POS
		self.stableTime = 0
		return 0



class TTrainer:
	def __init__(self, v0 = 0, v1 = 0, road_angles = ROAD_ANGLES, road_lengths = ROAD_LENGTHS, fileName = "velResponse.csv",GUI_ON = 0, VEL_ON = 0):
		self.gFlag = GUI_ON
		self.vFlag = VEL_ON
		self.road_angles = road_angles
		self.road_lengths = road_lengths
		self.v0 = v0
		self.v1 = v1
		self.kp = KP
		self.ki = KI
		self.fileName = fileName
		self.planner = [(0,v1),(1000,v1)]
		self.env = envLaunch(self.road_angles, self.road_lengths, self.v0)
		self.stableTime = 0
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
		tChecker = CheckStable()
		while running:
			circleTime = timer.elapse()
			if GUI_ON==1:
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						# gui.writeData(self.env, self.fileName)
						# gui.figure(self.env,self.kp, self.ki,VEL_ON)
						running = False
						return 0 
			# make a plan for the car
			self.env.planner(self.planner, TARGET_DISTANCE, mode = 0)
			# execute plan
			if self.env.update(circleTime) == 0:
				# gui.writeData(self.env, self.fileName)
				# gui.figure(self.env,self.kp, self.ki,VEL_ON)
				return 0
			#print 'simTime:', self.env.simTime, 'realTime:',self.env.realTime, 'vref:', self.env.vref,'vel:', valueWithSign(self.env.car.speed,self.env.car.angle)
			
			# check stop condition 
			if tChecker.checkStable(self.env, self.v1) == 1:
				self.stableTime = tChecker.stableTime
				# gui.figure(self.env,self.kp, self.ki,VEL_ON)
				running = False
				return 1
			# Gui display or not 
			if GUI_ON==1:
				gui.animation(screen, self.env)
				gui.velPlot(screen, self.env)
				gui.update()
			ticks=fpsClock.tick(self.env.FPS)

class DTrainer:
	"""this class creates a base for an object to calculate stable distance using stable time"""
	def __init__(self, v0 = 0, v1 = 0, stableTime = 0,road_angles = ROAD_ANGLES, road_lengths = ROAD_LENGTHS, fileName = "velResponse.csv",GUI_ON = 0, VEL_ON = 0):
		self.gFlag = GUI_ON
		self.vFlag = VEL_ON
		self.road_angles = road_angles
		self.road_lengths = road_lengths
		self.v0 = v0
		self.v1 = v1
		self.kp = KP
		self.ki = KI
		self.fileName = fileName
		self.env = envLaunch(self.road_angles, self.road_lengths, self.v0)
		self.stableTime = stableTime
		self.planner = [(0,v1),(self.stableTime,v1)]
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
		tChecker = CheckStable()
		while running:
			circleTime = timer.elapse()
			if GUI_ON==1:
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						# gui.writeData(self.env, self.fileName)
						# gui.figure(self.env,self.kp, self.ki,VEL_ON)
						running = False
						return 0 
			# make a plan for the car and check whether end of planner
			if self.env.planner(self.planner, mode = 0) == 1:
				# gui.writeData(self.env, self.fileName)
				# gui.figure(self.env,self.kp, self.ki,VEL_ON)
				return self.env.distance
			
			# execute plan
			if self.env.update(circleTime) == 0:
				# gui.writeData(self.env, self.fileName)
				# gui.figure(self.env,self.kp, self.ki,VEL_ON)
				return self.env.distance
			# print 'simTime:', self.env.simTime, 'realTime:',self.env.realTime, 'vel:', valueWithSign(self.env.car.speed,self.env.car.angle), "d:",self.env.distance
			# Gui display or not 
			if GUI_ON==1:
				gui.animation(screen, self.env)
				gui.velPlot(screen, self.env)
				gui.update()
			ticks=fpsClock.tick(self.env.FPS)
			
def sTimeCal(v0, v1, road_angles, road_lengths):
	"""given start velocity and end velocity, this function will return stable time"""
	trials = 1
	tArray = []
	while trials > 0: 
		trials -=1
		episo = TTrainer(v0, v1, road_angles, road_lengths, "velResponse.csv",GUI_ON = 0, VEL_ON = 1)
		if episo.launch() == 1:
			# print "stable time of this trial:", episo.stableTime
			tArray.append(episo.stableTime)
		else:
			print "stable time trial fails."
	# print "stable Time:", tArray
	return sum(tArray,0.0)/len(tArray) 

def sDisCal(v0, v1, stableTime,road_angles, road_lengths):
	# stableTime = sTimeCal(v0, v1, road_angles, road_lengths)
	episo = DTrainer(v0, v1, stableTime, road_angles, road_lengths, "velResponse.csv",GUI_ON = 0, VEL_ON = 1)
	stableDistance = episo.launch()
	# print "stableDistance-","v0",v0,"-v1",v1,":",stableDistance 
	return stableDistance
	
def roadModel(angle, velInterval):
	"""this calculates stable time and stable distance of each pair of velocities on a road specified by the given angle value, and then 
	store the result in [angle]TStable.csv and [angle]DStable.csv files"""
	tFile = "files/" + str(angle) + "TStable.csv";
	dFile = "files/" + str(angle) + "DStable.csv"
	# check whether stable files existing. If yes, remove it 
	try:
		os.remove(tFile)
		os.remove(dFile)
	except OSError:
		pass 

	for v0 in range(0,velInterval,VEL_SCALE):
		tArray = []
		dArray = []
		for v1 in range(0,velInterval,VEL_SCALE):
			if abs(v0- v1) < 0.1:
				print "Two value is equal", v0, v1
				stableTime = TIME_SCALE
				stableDistance = TIME_SCALE*v0 
			else:
				stableTime = sTimeCal(v0, v1, [angle], [1000])
				stableDistance = sDisCal(v0, v1, stableTime, [angle], [1000])
			print "v0:", v0, "v1:",v1, "sT:", stableTime, "sD:",stableDistance
			tArray.append(stableTime)
			dArray.append(stableDistance)
		# write to files
		helpers.writeRow(tArray, tFile )
		helpers.writeRow(dArray, dFile)

def accProfile(angle_range):
	"""This functions creates profile for a number of roads with the same velocity ranges"""
	for angle in angle_range:
		roadModel(angle,VEL_MAX+1)


def normalize():
	"""this functions normalizes stable distance and time using unit values declared in __init__."""
	
	# generate profiles 
	# angle_range = range(40,131,5) # from 40 degree to 131 degree with 5 degree step 
	angle_range = range(90,91) # only 90 degree 
	accProfile(angle_range)
	
	# now, normalize values into integers of unit values 
	for angle in angle_range:
		tInputFile = "files/" + str(angle) + "TStable.csv";
		dInputFile = "files/" + str(angle) + "DStable.csv";
		tOutputFile = "profiles/" + str(angle) + "STNORMED.csv";
		dOutputFile = "profiles/" + str(angle) + "SDNORMED.csv";
		helpers.normSTD(tInputFile,dInputFile, tOutputFile, dOutputFile, TIME_SCALE)
		removeFile(tInputFile)
		removeFile(dInputFile)
	
def removeFile(fileName):
	os.remove(fileName)
		
if __name__ == "__main__":
	print "main function running"
	# roadModel(60, 51)
	# sTimeCal(10,10,[120],[10000])
	# sDisCal(20,10,[60],[1000])
	# accProfile()
	normalize()
