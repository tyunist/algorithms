#PID.py this is where a PID class definition is asserted
#the problem with this PID controller is that it is subject to circleTime which depends on 
#whether we turn of the simulation GUI
class PID:
	def __init__(self,kp = 0.0, ki = 0.0, kd = 0.0, cmdMax = 100, cmdMin = -100):
		self.pGain = kp
		self.iGain = ki
		self.dGain = kd
		self.pErrLast = 0.0;
		self.pErr = 0.0;
		self.iErr = 0.0;
		self.dErr = 0.0;
		self.cmd = 0.0;
		self.cmdMax = 0.0;
		self.cmdMin = 0.0;
	
	def setPGain(self,kp):
		self.pGain = kp 
	def setIGain(self,ki):
		self.iGain = ki 
	def setDGain(self,kd):
		self.dGain = kd 
	def setCmdMax(self,cmdMax):
		self.cmdMax = cmdMax
	def setCmdMin(self,cmdMin):
		self.cmdMin = cmdMin	
	
	def reset(self):
		self.pErrLast = 0.0;
		self.pErr = 0.0;
		self.iErr = 0.0;
		self.dErr = 0.0;
		self.cmd = 0.0;
		self.cmdMax = 0.0;
		self.cmdMin = 0.0;
	
	def update(self,error, circleTime):
		"""this function update the command using circleTime which is in second unit"""
		self.pErr = error
		if circleTime == 0 or circleTime > 200 or circleTime < - 200:
			return 0.0

		# Calculate proportional contribution to command
		pTerm = self.pGain * self.pErr

		# Calculate the integral error
		self.iErr = self.iErr + circleTime * self.pErr

		# Calculate integral contribution to command
		iTerm = self.iGain * self.iErr

		# Calculate the derivative error
		if circleTime != 0:
			self.dErr = (self.pErr - self.pErrLast)*1.0 / circleTime
			self.pErrLast = self.pErr


		# Calculate derivative contribution to command
		dTerm = self.dGain * self.dErr
		self.cmd = pTerm + iTerm  + dTerm

		# Check the command limits
		if self.cmdMax != 0.0 and self.cmd > self.cmdMax:
			self.cmd = self.cmdMax
		if self.cmdMax != 0.0 and self.cmd < self.cmdMin:
			self.cmd = self.cmdMin

		return self.cmd

	def setCmd(self,cmd):
		self.cmd = cmd 
	
	def getCmd(self):
		return self.cmd 
	
	def getErrors(self):
		return (self.pErr, self.iErr, self.dErr)