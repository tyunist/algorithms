This is a version of carSimulation which contains a car model, pid controller, dynamic programming with and without GUI.
It is for IROS2015 -  workshop.
********************************************************************

The proper steps to test dynamic programming versus bisection method are as following:
1.	Requirements: 
	*	python 2.7 or above with packages: pygame, matplotlib, numpy 
	* 	cmake 2.8 or above 
********************************************************************	
2.	Installation: 
	*	Copy the whole folder test_arrival_configurations 
	*	Open terminal, move to subfolder dynamic-generator/build 
		*	Remove the whole files and folder there 
		*	..../build/$ cmake ..
		*	..../build/$ make 
********************************************************************		
3.	Running:
	*	Open terminal, move to subfolder include
		*	edit file /include/__init__.py as following: 
			# NOTE: to test arrival configuration with unit = 0.1, set 
			#		TIME_SCALE = 0.1
			#		DISTANCE_SCALE = 0.1 and 
			# 		MAX_TIME = 4000

			#		to test arrival configuration with unit = 0.2, set 
			#		TIME_SCALE = 0.2
			#		DISTANCE_SCALE = 0.2 and 
			# 		MAX_TIME = 2000

			#		to test arrival configuration with unit = 0.4, set 
			#		TIME_SCALE = 0.4
			#		DISTANCE_SCALE = 0.4 and 
			# 		MAX_TIME = 1000
			
	*	Open terminal, move to the main folder, run:
		*	.../test_arrival_configurations$ python test_arrival_configurations.py
	
	*	Result for each running is stored in the files bio_result.csv and dynamic_result.csv
	with the format: 
	trial_number	distance	V0(plan)	Vend(plan)	Tend(plan)	Tend(response)	Vend(response)
	e.g:
	1				100.0			18			2			48.2	48.200000000000415	1.9999999999999984	

	********************************************************************		
3.	Structure of the program:
	*	test_arrival_configurations.py: main function that calls:
	*	accProfile.py: to generate stable distance and stable time tables 
	*	dynamic_generator.py: that call a C++ progress (named: dynamic-c) to 
		generate plans using dynamic programming
	*	bio_generator.py: generate plans using bisection method with the inputs
		are plans generated from dynamic_generator.py
		
	*	/include/* : files that used for simulation. Specially:
		/include/__init__.py: stores constants used for the whole program. 
	*	/dynamic-generator/*: dynamic programming 
	*	/profiles/* : stable time and stable distance files 
	
4. Note: The forementioned above procedure takes alot of time (maybe) due to the subprocess of python. 
In order to solve this problem, i changed a little bit: 
	*	Run dynamic-c manually to generate plans then put them into separate files based on the unit case
	(3 files: 500_TEST_RUNNING.py, 1000...., 2000... just run them). 
	*	The only task to do now is just run each file once to obtain the result. Note that we still need to change 
	constants such as TIME_SCALE, DISTANCE_SCALE and MAX_TIME to meet the corresponding scales. 


