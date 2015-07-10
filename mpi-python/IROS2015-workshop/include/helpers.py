# helpers.py 
# contains useful functions
import csv 
import sys,os
from __init__ import *
def writeRow(list, fileName):
    with open(fileName, "a+b") as f:
		try:
			writer = csv.writer(f,delimiter ='\t', quotechar=''  ,quoting=csv.QUOTE_NONE)
			writer.writerow(list)  
		except IOError:
			raise IOError

def readMatrixFile(inputFile, mode = 1): # mode = 1: read integers; mode = 0: read float numbers. 
	"""This function return a matrix read from a given file"""
	try:
		f = open (inputFile, 'r+');
		if mode == 0:
			T = [map(float, line.split('\t') )for line in f];
		else:
			T = [map(int, line.split('\t') )for line in f];
	except IOError as e:
		print "I/O error({0}): {1}:".format(e.errno, e.strerror), inputFile
	except:
		print "Unexpected error:", sys.exc_info()[0]
		raise
	return T 

def writeMatrixFile(T, outputFile):
	"""This functions write a matrix to a file and return 1 if okie, -1 if no"""
	try:
		os.remove(outputFile)
	except OSError:
		pass 
	try:
		for list in T:
			writeRow(list,outputFile)
	except IOError as e:
		print "I/O error({0}): {1}".format(e.errno, e.strerror)
		return -1 
	except:
		print "Unexpected error:", sys.exc_info()[0]
		return -1 
		raise
	return 1
		
	
def normSTD(t_orig_file, d_orig_file, t_out_file, d_out_file, unitValue):
	"""This functions normalizes the stable time and distance values and then store to new files"""
	T = readMatrixFile(t_orig_file, mode = 0)
	D = readMatrixFile(d_orig_file, mode = 0)
	if unitValue <= 0:
		print "Unitvalue should not <= 0. "
		return -1
	for i in range(len(D)):
		for j in range(len(D[0])):
			if i == j:
				# print "bang nhau kia",i,j, D[i][j]
				T[i][j] = 1
				D[i][j] = i*VEL_SCALE
			else:
				T[i][j] = int(round(T[i][j]/unitValue))
				D[i][j] = int(round(D[i][j]/unitValue))
				if T[i][j] == 0:
					T[i][j] = T[i][j] + 1 # so that it will be 1 if it's rounded off to 0
					D[i][j] = int(round((i+j)/2*VEL_SCALE))
	# write to file 
	writeMatrixFile(T, t_out_file)
	writeMatrixFile(D, d_out_file)
