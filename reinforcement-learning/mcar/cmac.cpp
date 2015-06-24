// based on Rich and Satinder's  papers

// Code written by Sridhar Mahadevan
// Department of Computer Science and Engineering
// University of South Florida
// 4202 East Fowler Avenue, ENG 118
// Tampa, Florida 33620-5399
// mahadeva@csee.usf.edu
// http://www.csee.usf.edu/~mahadeva/mypage.html

#include "cmac.h"

// create the tiles for the CMAC


cmac::cmac(double pos, double vel) : mcar(pos, vel)
{
  // for each grid, compute an offset from center
	 
  cout << "setting up CMAC tiles..." << endl; 
	 
  pos_interval = (POS_RANGE[1] - POS_RANGE[0])/POS_BINS; 
  vel_interval = (VEL_RANGE[1] - VEL_RANGE[0])/VEL_BINS; 
	
  cout << "Position interval is " << pos_interval << endl; 
  cout << "Velocity interval is " << vel_interval << endl; 
	
  for (int tile = 0; tile<TILES; tile++)
    {
      // generate random offset as a fraction of an interval 
			
//      offset[0][tile]  = (choose_random_value() * 2  * pos_interval) - pos_interval; 
//      offset[1][tile]  = (choose_random_value() * 2 * vel_interval) - vel_interval; 

// CMAC offsets are positive fractions of an interval (Rich's correction!)
      offset[0][tile]  = choose_random_value() * pos_interval;  // offsets = pos fraction of int
      offset[1][tile]  = choose_random_value() * vel_interval;
    }
		
  active_tiles();  // set up active tiles 
}

// display the offsets 

void cmac::display_cmac(void)
{

  for (int tile = 0; tile < TILES; tile++)
    {
      cout << endl; 
			
      cout << "Tile Number " << tile << " Position offset " << offset[0][tile] << endl; 
      
      cout << "Position intervals: " ;
      
      for (int pos=0; pos<=POS_BINS; pos++)
      	cout << POS_RANGE[0] - offset[0][tile] + pos*(POS_RANGE[1] - POS_RANGE[0])/POS_BINS << "  "; 
      
      cout << endl; 
      	
			
      cout << "Tile Number " << tile << " Velocity offset " << offset[1][tile] << endl; 
      
      cout << "Velocity intervals: " ;
      
      for (int vel=0;vel <= VEL_BINS ; vel++)
      	cout << VEL_RANGE[0] - offset[1][tile] + vel*(VEL_RANGE[1] - VEL_RANGE[0])/VEL_BINS << "  "; 
      
      cout << endl; 
    }
		
}

// given a state of the mountain car, return the set of CMAC tiles corresponding to it

void cmac::active_tiles()
{
  double pos = curr_pos(); 
  double vel = curr_vel();  // get state of car
  
//  cout << "Active tiles in pos " << pos << " and vel " << vel << endl; 
	
  for (int tile =0; tile<TILES; tile++)
    {
      int pos_tile = (pos - POS_RANGE[0] + offset[0][tile])/pos_interval; // add offset!!
      int vel_tile  = (vel - VEL_RANGE[0] + offset[1][tile])/vel_interval; 
      
   //   cout << "(" << pos_tile << "," << vel_tile << ")" << " "; 
			
      atiles[tile].pos_bin = pos_tile; 
      atiles[tile].vel_bin = vel_tile; 
    }
//    cout << endl; 
}

// show which tiles are active 	
void cmac::display_active_tiles()
{

  for (int tile = 0; tile < TILES; tile++)
    {
     // cout << "Tile " << tile << " "; 
      cout << "(" << atiles[tile].pos_bin << "," << atiles[tile].vel_bin << ")  ";
  //    cout << endl; 
    }
    
    cout << endl << endl; 
}


