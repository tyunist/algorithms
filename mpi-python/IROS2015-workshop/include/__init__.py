# constants that are used for dynamic programming and finding plans 
# NOTE: to test arrival configuration with unit = 0.1, set 
#		TIME_SCALE = 0.1
#		DISTANCE_SCALE = 0.1 and 
# 		MAX_TIME = 2000

#		to test arrival configuration with unit = 0.2, set 
#		TIME_SCALE = 0.2
#		DISTANCE_SCALE = 0.2 and 
# 		MAX_TIME = 1000

#		to test arrival configuration with unit = 0.4, set 
#		TIME_SCALE = 0.4
#		DISTANCE_SCALE = 0.4 and 
# 		MAX_TIME = 500

MAX_TIME = 500  # maximum running time (in units) considered when finding plans
TIME_SCALE = 0.4 # e.g TIME_SCALE = 0.1, meaning that each time unit used in dynamic programming is equal to 0.1 second.  
DISTANCE_SCALE = 0.4 # e.g TIME_SCALE = 0.1, meaning that each distance unit used in dynamic programming is equal to 0.1 m. 


VEL_SCALE = 2 # e.g VEL_SCALE = 2, meaning that a velocity of a has index of a/2 
M  = 26 # number of discrete velocity values

#*****************************************#
# constants that are used for simulation
#*****************************************#
# constants which are not changed 
MAROON	=	(128,128,0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)
YELLOW	=	(255,255,0)
NAVY_BLUE = (0,0,128)
AQUA=(0,255,255)
BLUE	=	(0,0,255)
FUCHSIA	=	(255,0,255)
GRAY	=	(128,128,128)
LIME	=	(0, 255,0)
OLIVE	=	(128,128,0)
PURPLE	=	(128,0,128)
SILVER	=	(192,192,192)
TEAL	=	(0,128,128)
WHITE	=	(255,255,255)
#*****************************************#
# constants which are not changable on purpose 
# in this program we normalize 1 pixel/second = 2 m/s 
FPS = 1000
GRAVITY = (180, 9.8)
FRICTION = 0.3
DRAG = 1  # air force factor which is 1 if unable and < 1 if enable 
# car's characteristic parameters 
WHEEL_SIZE = 20
MASS = 1000
WHEEL_AXIS = 60
CHASIS_SIZE = (110, 40)
WHEEL_COLOUR = (200,200,200)
CHASIS_COLOUR = GRAY
# car initial parameters
V0 = 0
VREF = 10
POW = 10 # power capability of the vehicle's engine
TIME_STAMP = 0.1 # time interval to update car's controller state 
KP = 0.08;
KI = 5;
VEL_MAX = 50; # maximum velocity (m/s)
KP_GUI_ON = 0.1;
KI_GUI_ON = 0.08;
KD = 0;


# road environment 
(width, height) = (2000, 1200)
rx_orig = 50.
ry_orig = 400.
d = 40 # distance between two wheels 
number_of_road_segments = 1
ROAD_ANGLES = [50,90,60]
ROAD_LENGTHS = [200,200,300]
PLANNER = [(10000,10)]
TARGET_DISTANCE = 100 
# number of trials to decide stable time
NOST = 4
# period of time in stable state to decide a stable time
POS = 4 #(seconds)
# discrete time value used for simulation. 
TUnit = 0.1 #(second)
DUnit = 0.1 #(m/s)


