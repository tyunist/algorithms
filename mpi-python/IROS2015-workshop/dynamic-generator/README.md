This is an implementation of dynamic programming to compute some plans of of some road segments. It is by far faster than using python lonely (about 100 times faster). 
This program is run as a hidden process that is called by a python process to save all plans to a .pkl file . 

Usage:
	* All source file are located in src folder. 
	* C-based program is built to /bin subfolder. To build it, move to ./build folder and run $ cmake ..
						   $ make
	
