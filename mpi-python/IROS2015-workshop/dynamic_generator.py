#	this program use manualy input of plans generated by dynamic programming method
# 	then save them to a .pkl file for later use of test-arrival-configuration.py 
import os 
import re
import sys
import subprocess
import matplotlib.pyplot as plt 
import math
from __init__ import *
# import pickle as pickle
def print_usage():
	print("Usage:")
	print '--------------------'
	print "\tpython", sys.argv[0], 'plans_by_dynamic_file_name', '\n'
	print 'where plans_by_dynamic_file_name is where we store the obtained plans a'
	print '\tby default, plans_by_dynamic_file_name = plans_by_dynamic.pkl'
	print '\n....continue running...'
	print '--------------------'

def map2Plan(numbers_array):
	'''this function maps the given list of separate numbers into a plan in form of [(t1,v1),(t2,v2)....]'''
	return [(numbers_array[i], numbers_array[i+1]) for i in range(0,len(numbers_array),2)]
	
	
# def dynamicGenerator(argv):
def dynamicGenerator():
	# if len(sys.argv) != 2:
		# print_usage()
		# plans_by_dynamic_file_name = 'plans_by_dynamic.pkl'
	# else:
		# plans_by_dynamic_file_name = sys.argv[1]
		# configurations_file_name = sys.argv[2]
	plans_by_dynamic_file_name = 'plans_by_dynamic.pkl'
	# get the directory of T and D files to put to dynamic-c process 
	Tfile = os.getcwd() + '/profiles/90STNORMED.csv'
	Dfile = os.getcwd() + '/profiles/90SDNORMED.csv'
	dynamic_c_process = './dynamic-generator/bin/dynamic-c' 
	player = subprocess.Popen([dynamic_c_process, Tfile, Dfile, str(MAX_TIME)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	#######################################
	# to print out subprocess's output
	stdout_value = player.stdout  # read all at once, returns an arrays 
	
	# read stdout and store feasible points 
	# a list of feasible points for problem (i, j) start by <$i,j> as beginning marker and shows up line by line e.g: [3, 4] before ending up by an ending marker <i,j&>
	plans_array = []
	for line in stdout_value:
		# print 'line:', line
		line = line.strip()
		if len(line) == 0:
			continue 
		if line[0:2] == '<$':
			plan_prefix = map(int, re.findall(r'\d+', line))
			# if previous line show the arrival-configuration, next line will be a plan 
			line = next(stdout_value).strip()
			numbers_array = map(int, re.findall(r'\d+', line))
			# now, we need to maps those numbers to a plan 
			plan_tail = map2Plan(numbers_array)
			plan = [plan_prefix, plan_tail]
			plans_array.append(plan)
	
	print 'successfully saving plan data to plans_array.' 
	
	# testing by printing plans 
	print 'a sample plan obtained by dynamic:', plans_array[0]
	return plans_array

if __name__ == '__main__': 
	# dynamicGenerator(sys.argv)
	dynamicGenerator()
	
