#include "mc.h"   

// choose a random starting position and velocity for the car

mcar::mcar(void)
{
	double vrand, prand; // random values for velocity and position 
	
	prand = choose_random_value();  
	
	p = (POS_RANGE[1] - POS_RANGE[0]) * prand + POS_RANGE[0]; // scale position into legal range 
	
	vrand = choose_random_value();  
	
	v =  (VEL_RANGE[1] - VEL_RANGE[0]) * vrand + VEL_RANGE[0]; // scale velocity into legal range
	
//	cout << "Starting car at " << p << "  with velocity " << v << endl; 
	
}

// second constructor function where pos and vel are given 

mcar::mcar(double pos, double vel)
{
	p = pos; 
	v = vel; 
	
}

// reward function for mountain car problem 

double mcar::reward()
{
	// if (reached_goal())
// 		return(0);  
//	else 
	return(-1); 
}

// generate random starting position 

double random_pos()
{
	double  prand = choose_random_value();  
	
	return((POS_RANGE[1] - POS_RANGE[0]) * prand + POS_RANGE[0]); // scale position into legal range 
}

// generate random starting velocity 
double random_vel()
{
	double vrand = choose_random_value();  
	
	return((VEL_RANGE[1] - VEL_RANGE[0]) * vrand + VEL_RANGE[0]); // scale velocity into legal range
}

// overload the << operator for the enum data type

ostream& operator << (ostream& out, ACTION a)
{
	if (a == coast)
	 out << " Coast"; 
	else if (a == forward)
	 out << " Forward"; 
	else if (a == backward)
	 out << " Backward"; 

	return(out); 
	
}
	
ACTION mcar::int_to_act(int value)
{
	ACTION act; 
	
	if (value == 0)
		act = coast; 
	else if (value == 1)
		act = forward; 
	else if (value == 2)
		act = backward; 	
	
	return(act); 
}

// for now, choose a random action 

ACTION mcar::choose_random_act()
{
	int rvalue = choose_random_int_value(2); // return a value between 0 and 2
	
	return(int_to_act(rvalue)); 
}


// update velocity and position  -- range is clipped if it is out of bounds 


void mcar::update_position_velocity(ACTION a)
{
	
	double oldv = v; // preserve old values
	double oldp = p; 
	
	double newv, newp;  // new values of velocity and position
	
	int aval;
	
	if (a == backward)
	 aval = -1; 
	else aval = (int) a;  // coast = 0, forward = +1, backward = -1; 
	
	newv = oldv + (0.001 * aval) + (GRAVITY * cos(3 * oldp)); // update equation for velocity 
	
	if (newv < VEL_RANGE[0])  // clip velocity if necessary to keep it within range
	  {
//	    cout << "clipping velocity to " << VEL_RANGE[0] << endl; 
		newv = VEL_RANGE[0]; 
	  }
	else if (newv > VEL_RANGE[1])
	  {
//	    cout << "clipping velocity to " << VEL_RANGE[1] << endl; 
	    newv = VEL_RANGE[1];

	  }
			
	newp = p + newv;  // update equation for position 
	
	if (newp < POS_RANGE[0])  // clip position and velocity if necessary to keep it within range
		{
			newp = POS_RANGE[0]; 
			newv = 0;  // reduce velocity to 0 if position was out of bounds 
		}
	else if (newv > POS_RANGE[1])
		{
			newp = POS_RANGE[1]; 
			newv = 0;  
		}
		
	p = newp; 
	v = newv;   // update state to new values 
	
}
	
// see if car is up the hill 

int mcar::reached_goal(void)
{
	if (mcar::p > GOAL)  // over the hill!
		return(1); 
	else return(0); 
	
}


// overload the << operator for printing the current position and velocity 

ostream& operator << (ostream& out, mcar mc)
{
	out << "Position: " << mc.p << "   Velocity:   " << mc.v << "  "; 
	
	return(out); 
	
}

