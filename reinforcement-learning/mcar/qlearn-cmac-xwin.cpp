// Q-learning for mountain car problem 
/// Q-learning for mountain car problem 
// uses CMAC as a function approximator
// follows approach described in Sutton/Singh papers

// implements TD(lambda) sarsa

// Code written by Sridhar Mahadevan
// Department of Computer Science and Engineering
// University of South Florida
// 4202 East Fowler Avenue, ENG 118
// Tampa, Florida 33620-5399
// mahadeva@csee.usf.edu
// http://www.csee.usf.edu/~mahadeva/mypage.html

#include "cmac.h" // definition of CMAC class and functions 
#include "display.h" // Xwindows graphics 
#include "string.h"
#include <unistd.h>

#define RUNS 1

#define MAX_TRIALS 1000

#define VALUE_PLOT_STEP_SIZE 100 // output value function once in N trials

#define Q0 0 // for initializing weights to Q0/TILES 

#define GRID_RES 50

double POS_STEP = (POS_RANGE[1] - POS_RANGE[0])/GRID_RES; 

double VEL_STEP = (VEL_RANGE[1] - VEL_RANGE[0])/GRID_RES; 

double weight[POS_BINS+1][VEL_BINS+1][TILES][ACTIONS];  // qvalue representation over tiles 

double eligibility[POS_BINS+1][VEL_BINS+1][TILES][ACTIONS]; // eligibility of a tile 

static int trial_data[RUNS][MAX_TRIALS]; // keep track of solution time for each trial and run 

// main routine for running trials

void run_trials();  

void output_trial_std_dev_data(); 


// define Q-learning with CMAC as a derived class of cmac

class mcar_qlearn_cmac : public cmac {   // inherit from cmac and mcar 

public: 
	
  mcar_qlearn_cmac(double pos, double vel); // constructor initializes state and cmac tiles 
		
		ACTION choose_action(); // choose highest Q-value action, but explore sometimes 
		
		void initialize_weights_eligibilities(); 
		
		void update_eligibilities(ACTION a); // mark all active tiles for a particular action 
		
		double qvalue(ACTION a); //  qvalue  computed as weighted sum over active tiles 
		
		double qvalue(ACTION a, double pos, double vel); // at any desired point
		
		double best_qvalue(double pos, double vel); 
		
		void generate_qvalue_plot(int run, int trial); 
		
		// Given previously active tiles and newly active tiles, update Q values 
		void update_weights(double reward, double old_qval, double new_qval); 
		
	private:  // data structures for Q-learning
	
		double exploration_rate; 
		
		double GAMMA;  // discount factor 
		double BETA;   // learning rate  
		double LAMBDA; // recency parameter
		
}; 


// constructor function calls cmac and mcar constructor functions 
mcar_qlearn_cmac::mcar_qlearn_cmac(double pos, double vel) : cmac(pos, vel) 
{
// initialize Q-values  to 0

  for (int pbin=0; pbin <= POS_BINS; pbin++)
    for (int vbin=0; vbin <= VEL_BINS; vbin++)
      for (int tile=0; tile<TILES; tile++)
	for (int act=0; act<ACTIONS; act++)
	  {
	    weight[pbin][vbin][tile][act] = Q0/TILES; 
	    eligibility[pbin][vbin][tile][act] = 0.0; 
	  } 
									
				
GAMMA = 1.0; // discount factor

BETA = 0.5;  // learning rate 

LAMBDA = 0.9;  // recency parameter 

exploration_rate = 0.0;  // percentage of randomness

}

// initialize weight and eligibilities

void mcar_qlearn_cmac::initialize_weights_eligibilities()
{

  for (int pbin=0; pbin <= POS_BINS; pbin++)
    for (int vbin=0; vbin <= VEL_BINS; vbin++)
      for (int tile=0; tile<TILES; tile++)
		for (int act=0; act<ACTIONS; act++)
		  {
		    weight[pbin][vbin][tile][act] = Q0/TILES; 
		    eligibility[pbin][vbin][tile][act] = 0.0; 
		  } 
}		

// compute Q value of current state as weighted sum over active tiles 
double mcar_qlearn_cmac::qvalue(ACTION a)
{
	double value = 0; 
	
	for (int tile=0; tile<TILES; tile++)
		value += weight[atiles[tile].pos_bin][atiles[tile].vel_bin][tile][a]; 
		
	return(value);
 	
}

// compute qvalue at any desired state
double mcar_qlearn_cmac::qvalue(ACTION a, double pos, double vel)
{
	double value =0; 
	
	// compute active tiles 
  for (int tile =0; tile<TILES; tile++)
    {
      int pos_tile = (pos - POS_RANGE[0] + offset[0][tile])/pos_interval; // add offset!!
      int vel_tile  = (vel - VEL_RANGE[0] + offset[1][tile])/vel_interval; 
      
      value += weight[pos_tile][vel_tile][tile][a];
    }
    
    return(value); 
    
}

double mcar_qlearn_cmac::best_qvalue(double pos, double vel)
{
	double bvalue = qvalue(coast,pos,vel); 
	
	for (int a=0; a<ACTIONS; a++)
      if (qvalue(int_to_act(a), pos, vel) > bvalue)
		bvalue = qvalue(int_to_act(a),pos,vel); 
	
	return(bvalue);
}

// given a position and velocity bin, pick the highest Q-value action with high probability 

ACTION mcar_qlearn_cmac::choose_action()
{
  double rvalue; 
  int bact = choose_random_int_value(2); 
  
  rvalue = choose_random_value(); 
	
  if (rvalue < exploration_rate)  // do a random action 
    return(choose_random_act()); 
  else 
    for (int a=0; a<ACTIONS; a++)
      if (qvalue(int_to_act(a)) > qvalue(int_to_act(bact)))
		bact = a; 

  return(int_to_act(bact)); 
}

// update eligibilities 
void mcar_qlearn_cmac::update_eligibilities(ACTION a)
{
	for (int pbin=0; pbin <= POS_BINS; pbin++)
		for (int vbin=0; vbin <= VEL_BINS; vbin++)
			for (int tile=0; tile<TILES; tile++)
				for (int act=0; act<ACTIONS; act++)
					eligibility[pbin][vbin][tile][act] *= LAMBDA; // decay eligibilites
						
					
	for (int tile=0; tile<TILES; tile++)
		for (int act = 0; act<ACTIONS; act++)
			if (act == a)
				eligibility[atiles[tile].pos_bin][atiles[tile].vel_bin][tile][act] = 1;
			else 
				eligibility[atiles[tile].pos_bin][atiles[tile].vel_bin][tile][act] = 0; 
		
}

// update weights 

void mcar_qlearn_cmac::update_weights(double reward, double old_qval, double new_qval)
{

// TD(lambda) sarsa update rule 

for (int pbin=0; pbin <= POS_BINS; pbin++)
		for (int vbin=0; vbin <= VEL_BINS; vbin++)
			for (int tile=0; tile<TILES; tile++)
				for (int act=0; act<ACTIONS; act++)
					weight[pbin][vbin][tile][act] +=  
		 			 (BETA/TILES)*(reward + new_qval - old_qval) * eligibility[pbin][vbin][tile][act]; 
}


// collect statistics on each trial over runs 

void output_trial_std_dev_data()
{

  ofstream out("mcar-sarsa-soln-data");  

  double sum,sum_sq,std_dev,mean;
  
  for (int j=0; j<MAX_TRIALS; j++)
      {
   	   sum=0.0;
   	   sum_sq=0.0;

	   for (int i=0; i<RUNS; i++)
	   {
	     sum += trial_data[i][j];
	     sum_sq += trial_data[i][j]*trial_data[i][j];
	    }

	   mean = sum/RUNS;

//	   std_dev = sqrt((sum_sq - (sum*sum/RUNS))/(RUNS-1));
	   
	   out <<  j <<  " " << mean << endl; 

//	     <<  " " << mean - std_dev << " " << mean + std_dev << endl; 
     }

  out.close(); 
} 


void mcar_qlearn_cmac::generate_qvalue_plot(int run, int trial)
{
  char name[20];
	
  sprintf(name,"qvalue-%d-%d",run, trial);
	
  ofstream output(name); 
	
  for (int pos=0; pos<= GRID_RES; pos++)
		{
		  double pvalue = POS_RANGE[0] + POS_STEP*pos; 
//  + POS_STEP/2; // midpoint of bin
			
		  output << endl; 
		
		  for (int vel=0; vel <= GRID_RES; vel++)
		    {
		      double vvalue = VEL_RANGE[0] + VEL_STEP*vel;  
// + VEL_STEP/2; // midpoint of bin
					
		      double value = best_qvalue(pvalue,vvalue); 
					
		      if (value < 0) value = -value; 
					
		      output << value << "\t"; 
					
		    }
		}
  output.close(); 
	
}	


void run_trials()
{
  int count = 0; 
  char data[30]; // data string to print out
  
  mcar_qlearn_cmac mcq(-0.5,0.0); // initialize state to bottom rest and cmac tiles

  for (int run = 0; run<RUNS; run++)
    {

      int best_changed = 1; 
      int best_so_far=10000;  // shortest distance to goal so far 

      mcq.initialize_weights_eligibilities(); 
      // initialize weights and eligibilities to 0. 



      if (run > 0)  // on subsequent runs, keep the same CMAC tiling 
	{

	  mcq.set_curr_pos(-0.5);  // restart at bottom resting 
	  mcq.set_curr_vel(0.0); 

	}
	
      for (int trial=0; trial< MAX_TRIALS; trial++)
	{

	  if ((trial+1)%VALUE_PLOT_STEP_SIZE==0)
	    mcq.generate_qvalue_plot(run, trial+1); 
       
	  best_changed = 1; 

	  double old_qvalue, new_qvalue;  // for TD(lambda) sarsa update rule 
     
	  int done  = 0, i = 0; 
	  double r; 
	  ACTION a, na; 

	  mcq.active_tiles();  // recompute set of active CMAC tiles 
		
	  a = mcq.choose_action(); // pick highest Q-value action with high probability 
	  
	  old_qvalue = mcq.qvalue(a); // compute q value as weighted sum over active tiles 
	
	  while (!done) // not yet reached goal
		{
		
		  i++; 

		  sprintf(data, "Run: %d Trial: %d Shortest solution: %d", run, trial, best_so_far); 

		  if (best_changed)
		    {
		      display_data(data,1); 
		      best_changed=0;
		    }
		  else display_data(data,0); 

		  //		  usleep(1000);  // microseconds 

		  display_mcar(mcq.curr_pos(),mcq.curr_vel()); 
		 
		  mcq.update_eligibilities(a); // decay and update eligibilities 
		 
		  mcq.update_position_velocity(a);  // move the car 
	 	 
		  if (!mcq.reached_goal())
		    {
	 	 
		      mcq.active_tiles();  // recompute set of active tiles 
	 		 
		      na = mcq.choose_action();  // choose highest Q-value action in new state
		 	 
		      new_qvalue = mcq.qvalue(na);
		 	 
		      r = mcq.reward();  // -1 always 

		      mcq.update_weights(r,old_qvalue, new_qvalue);  // TD(0) sarsa update 
	  
// RECOMPUTE QVALUE AFTER WEIGHTS CHANGED! (rich's correction)
		      old_qvalue = mcq.qvalue(na); 

		      a = na; 
		    }
		  else 
		    {
		      // TD(lambda) sarsa update at goal
		      mcq.update_weights(-1,old_qvalue, 0);  

		      if (i<best_so_far) 
			{
			  best_so_far = i; 
			  best_changed = 1; 
			}
		      trial_data[run][trial] = i; // record number of steps 
		      done=1;
		    }
		}
		
	  mcq.set_curr_pos(-0.5);  // restart at bottom resting 
	  mcq.set_curr_vel(0.0); 
	
	}
    }
	
}


// do a bunch of learning runs

int main(void)  
{

  int rinit = -time(NULL)%100; 
  int rseed = -1000 + rinit*rinit*rinit; 

  WINDOW = start_display(); 

  WINDOW2 = start_display2(); 

  initialize_random_number_generator(rseed); // set up large negative number as random seed; 
	
  cout << "Learning run started at " << __TIME__ << " with seed " << rseed << endl; 

  run_trials(); 

  output_trial_std_dev_data(); 

}
			
		
		
				

