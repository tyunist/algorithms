import time 
class Time:
	def __init__(self):
		self.getTime = time.time()
		self.elapsedTime = 0
		self.duration = 0
	def elapse(self):
		self.elapsedTime = time.time() - self.getTime
		self.getTime = time.time()
		self.duration += self.elapsedTime
		return self.elapsedTime
		
	def now(self):
		return time.strftime("%c")
	
	def stop(self):
		self.elapsedTime = time.time() - self.getTime
		self.getTime = time.time()
		self.duration += self.elapsedTime

		return self.duration 
