// mountain car simulator in C++

// based on Sutton's and Boyan and Moore's NIPS papers



#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include "krandom.h"  // link it with Knuth's random number generator
#include <math.h> 
#include <time.h>

#define GRAVITY -0.0025  // acceleration due to gravity 

#define GOAL 0.5  // above this value means goal reached 

// car position is limited to the following range

static double POS_RANGE[2] = {-1.2, 0.5}; 


// car velocity is limited to the following range

static double VEL_RANGE[2] =  {-0.07,0.07}; 


// allowable actions

enum ACTION {coast, forward, backward};

int run_trial(int trial); // function prototype 

ostream& operator << (ostream& out, ACTION a); 


// define the main class

class mcar {

	public: 
	
		mcar();  // constructor function defines a random starting point 
				 // within the allowable position and velocity 
				 
		mcar(double pos, double vel);  // second constructor function with specified pos and vel
				 
		friend double random_pos();  // generate random starting position 
		
		friend double random_vel();  // generate random starting velocity 
		
		double reward();  // return -1 if goal not reached, else 0		
		
		double curr_pos(){ return(p); };  // retrieve current position
		
		double	curr_vel(){ return(v); };   // retrieve current velocity
		
		void set_curr_pos(double pos) { p = pos;};  // set position to pos
		
		void set_curr_vel(double vel) {v = vel;};  // set velocity to vel
		
		ACTION int_to_act(int action);  // return named act 
		
		ACTION choose_random_act();  // choose an action 
		
		void  update_position_velocity(ACTION a); // update velocity and velocity 
		
		int reached_goal();  // 1 if reached, else 0
		
		friend ostream& operator << (ostream& out, mcar c); // display position and velocity 
		
		
	private: 
	
		double v, p; // current state = velocity and position 
		
}; 

