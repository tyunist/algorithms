#	This program evaluates how the car runs on a known road with a given distance and report ending time, ending
#	velocity.  

import os
path = os.getcwd() + '/include/'
import sys
sys.path.insert(0, path)
from controlPanel import *
import random
import subprocess
import numpy as np 
import time 
from myMath import *
from helpers import *
from __init__ import *
from dynamic_generator import dynamicGenerator
from bio_generator import bioGenerator
from accProfile import normalize

# parallel simulation 
from mpi4py import MPI
from mpi4py.MPI import ANY_SOURCE 

class accuMove:
	"""This class provides platform of car running with on-fly updated road"""

	def __init__(self, v0 = 0, road_angles = ROAD_ANGLES, road_lengths = ROAD_LENGTHS, planner = PLANNER, targetDistance = TARGET_DISTANCE, vel_on = 0, gui_on = 0):
		self.gFlag = gui_on
		self.vFlag = vel_on
		self.road_angles = road_angles
		self.road_lengths = road_lengths
		self.v0 = v0
		if gui_on == 1: 
			print "GUI ON"
			self.kp = KP_GUI_ON
			self.ki = KI_GUI_ON
		else:
			print "GUI OFF due to gui_on =", gui_on  
			self.kp = KP
			self.ki = KI

		self.planner = planner
		self.env = envLaunch(self.road_angles, self.road_lengths, self.v0, self.kp, self.ki)
		self.targetDistance = targetDistance

		# initialize the screen 
		self.gui = GUI((rx_orig, height))
		if gui_on == 1:
			self.screen = pygame.display.set_mode((width, height))
			self.screen.fill(self.env.background_colour) 
			pygame.display.set_caption('Car Test by Ty Nguyen')
		
		self.ticks = 0
		self.fpsClock = pygame.time.Clock()
		self.running = True 
		self.timer = myTime.Time()
		
 	def launch(self,comm, planner_buffer, indicator_buffer): # add commm for parallel messaging  
		while self.running:
			# communicate with planner process 
			print 'On simulator', 'before recv:   data = ', planner_buffer
			req=comm.Irecv(planner_buffer,source=1,tag=11)
			re = False
			while re == False :
				re=MPI.Request.Test(req)
			# print 'test result',re
			re=MPI.Request.Wait(req)
			# print 'wait result',re
			print 'On simulator','after recv:    sent= ',planner_buffer
			indicator_buffer[0] = 10
			print 'On simulator, before sending:    recv = ',indicator_buffer
			req=comm.Isend(indicator_buffer,1,11)
		
		
			circleTime = self.timer.elapse() 
			if self.gFlag==1:
				for event in pygame.event.get():
					if event.type == pygame.QUIT:
						# gui.writeData(self.env, self.fileName)
						# draw velocity response
						self.gui.figure(self.env,self.kp, self.ki,self.vFlag)
						self.running = False
						return 0 
			# make a plan for the car
			if self.env.planner(self.planner, self.targetDistance) == 1:
				# print "End of planner."
				self.running = False
				print "arrival time and velocity:",self.env.realTime, self.env.car.speed
				self.gui.figure(self.env,self.kp, self.ki,self.vFlag)
				return [self.env.realTime, self.env.car.speed]
			# update road: 
			self.updateRoad(self.env.car.x, self.env)				
			# execute plan
			if self.env.update(circleTime) == 0:
				# gui.writeData(self.env, self.fileName)
				self.gui.figure(self.env,self.kp, self.ki,self.vFlag)
				return 0
			print 'simTime:', self.env.simTime, 'realTime:',self.env.realTime, 'segment:',self.env.car.segment_index, 'pos:',self.env.car.x,'vref:', self.env.vref,'vel:', valueWithSign(self.env.car.speed,self.env.car.angle)
			
			
			# Gui display or not 
			if self.gFlag == 1:
				self.gui.animation(self.screen, self.env)
				self.gui.velPlot(self.screen, self.env)
				self.gui.update()
			self.ticks=self.fpsClock.tick(self.env.FPS)
			
	def updateRoad(self,x, env, road_angles=ROAD_ANGLES, road_lengths=ROAD_LENGTHS):
		"""This functions updates road on the way"""
		if len(env.road) == len(road_lengths): 
			print "road update finishes"
			return 1 
		# if car is still far from the current road's final point, do nothing
		if env.road[len(env.road)-1].ex - x > 20:
			# print "still far from the end point. "
			return 0
		# update
		i = env.car.segment_index
		print "current segment:", i 
		env.addRoadSegment(road_angles[i+1], road_lengths[i+1], 1)
		env.addTrajectorySegment()
		return 0


		
##############################################################################################
# Main function 
def main(argv):
	'''This program takes the plans_File as input and then run the simulation before saving running results to result_file.txt'''
	# Turn on or off visualization 
	if len(sys.argv) == 3:
		print 'User entered gui_on and vel_on:{0} and {1}'.format(sys.argv[1], sys.argv[2])
		gui_on = int(sys.argv[1]) 
		vel_on = int(sys.argv[2])
	else:
		gui_on = 0
		vel_on = 0 
		# parallel configuration 
	comm = MPI.COMM_WORLD
	rank = comm.Get_rank()
	size = comm.Get_size()
	
	# buffers 
	planner_buffer = np.zeros(2,dtype=np.int)
	indicator_buffer = np.zeros(1,dtype=np.int)
	# Initialize simulator 
	if(rank == 0):
		print "simulator process...Initializing"
		timer = myTime.Time()
		logFile = 'log_test_finish_road.txt';
		result_test_finish_road = 'result_test_finish_road.csv' 	
		try: 
			os.remove(logFile)
			os.remove(result_test_finish_road)
		except:
			print 'cannot delete all two files...passing'
			pass 

		# Declare road distance/ road profile
		# Declare plan configuration 
		plan_config = [[125,21,12,75], [(0,21),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(9,0),(10,0),(11,0),(12,0),(13,0),(14,0),(15,0),(16,0),(17,0),(18,0),(19,0),(20,0),(21,0),(22,0),(23,0),(24,0),(25,0),(26,0),(27,0),(28,0),(29,0),(30,0),(31,0),(32,0),(33,0),(34,0),(35,0),(36,0),(37,0),(38,0),(39,0),(40,0),(41,0),(42,0),(43,0),(44,0),(45,0),(46,0),(47,0),(48,0),(49,0),(50,0),(51,0),(52,0),(53,0),(54,0),(55,0),(56,0),(57,0),(58,0),(59,0),(60,0),(61,0),(62,0),(63,0),(64,0),(65,0),(66,0),(67,0),(68,0),(69,0),(70,0),(71,10),(73,15),(74,17),(75,12)]]
		print 'plan_config[0]: ', plan_config[0]

	
		# Extract point schedule, road configuration from plan_config 
		configuration = plan_config[0]
		planner = plan_config[1]
		planner_scaled = [i*0 for i in range(len(planner))]
		# change configuration and format to real values with respect to scales:
		for i in range(len(planner)):
			planner_scaled[i] = (planner[i][0]*TIME_SCALE , planner[i][1]*VEL_SCALE)
		configuration[0] = configuration[0]*DISTANCE_SCALE 
		configuration[1] = configuration[1]*VEL_SCALE
		configuration[2] = configuration[2]*VEL_SCALE
		configuration[3] = configuration[3]*TIME_SCALE 

		road_angles = [90, 90, 90, 90]
		road_lengths = [100, 8000, 10000, 400]
		v0 = planner_scaled[0][1] 
		targetDistance = configuration[0]
		# Initialize simulator 
		simulator = accuMove(v0, road_angles, road_lengths, planner_scaled,targetDistance, vel_on, gui_on)
	else: 
		print "plan generator process. First step...do nothing.."
		
	# Now, its time for simulation and plan generation 
	
	if rank == 1: 
		print "This is the planner process" # debug2 
		count = 1 
		timer =  myTime.Time()
		while True: 
			# send message to the simulator
			planner_buffer[0] = count
			req=comm.Isend(planner_buffer,0,11)
			print 'Now:', timer.stop(),  '- count: ', count, '- on task',rank,'before sending:    data = ',planner_buffer
			print 'on task',rank,'before recv:   data = ',indicator_buffer
			response=comm.Irecv(indicator_buffer,source=0,tag=11)
			re = False
			while re == False :
				re=MPI.Request.Test(response)
			# print 'test result',re
			re=MPI.Request.Wait(response)
			# print 'wait result',re
			print 'on task',rank,'after recv:    data = ',indicator_buffer
			count = count + 1 


	else: 
		print "This is simulator's thread"  # debug2
		running_result = simulator.launch(comm, planner_buffer, indicator_buffer)
		if len(running_result) != 2:
			print 'Error! no returned arrival time and velocity'
			return -1  
		else: 
			trial_result = configuration + running_result
			print 'result: ', trial_result
			# write down to file 
			helpers.writeRow(trial_result, result_test_finish_road)
			

	
	# Terminate when car reaches to the end of the road 
	# Report ending time and velocity 
	
	writeRow(["*********",timer.now(),': END OF PROGRAM : **********', str(timer.stop())], logFile)
	


if __name__ == "__main__":
	print "main function running"
	main(sys.argv)
