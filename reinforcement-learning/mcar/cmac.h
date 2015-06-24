// CMAC representation for mountain car problem 

#include "mc.h"   // definition of mountain car problem 

// define a tiling for the mcar problem 

#define POS_BINS 8
#define VEL_BINS 8
#define TILES 10

#define ACTIONS 3  // coast, forward, backward 

struct cell {int pos_bin; int vel_bin;}; 

// make a class for a CMAC

class cmac : public mcar {
	public: 
	
		cmac(void);   // create the tiles for a new position
		
		cmac(double pos, double vel); // call base constructor and set up tiles
		
		void display_cmac(void);  // print out the offsets 
		
		void active_tiles();  // map the state of the car into the tiles 
		
		void display_active_tiles(); // show which tiles are active currently
		
	protected: 
	
		cell atiles[TILES];  // keep list of active tiles 
		
		double offset[2][TILES];  // 0 is for position, 1 is for velocity 
		
		double pos_interval, vel_interval; // bin size for each dimension 
	
	private:
		// put in the data structures here
			
};
