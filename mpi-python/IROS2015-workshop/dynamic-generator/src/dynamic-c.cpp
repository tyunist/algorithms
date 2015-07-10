/* This program implements dynamic programming to find feasible set on one road segment
 */
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm> // count()
#include <array>
#include <cstdlib>
#include "helpers.h"
#include "Timer.h"
#include <stdlib.h>
#include <time.h>       /* time */
#include <deque>
#include <string> 

#ifndef NUM_DISCRETE_VEL
#define NUM_DISCRETE_VEL 26
#endif

#ifndef NUM_PLANS
#define NUM_PLANS 10
#endif
#ifndef NUM_RANDOM_TRIALS
#define NUM_RANDOM_TRIALS 100 
#endif

#ifndef M
#define M 26 
#endif

typedef std::vector<classPoint2T> fSet;

/* This functions add valid feasible points of F_sub to F_orig after shifting an amount of delta_t time*/
int addPoints(std::vector<classPoint2T>&, std::vector<classPoint2T> &, int, int);

/* This function find out the trajectory of a feasible point */
int findTrajectory(std::vector<std::vector<fSet> >& , int &, int, classPoint2T &, std::vector<std::vector<int> >  &, std::vector<std::vector<int> > &);

/* This functions traveres the table and add vint and time to the trajectory. */
int	recursiveMove(std::vector<classPoint2T> &, int, int, int, int, std::vector<std::vector<fSet> >& , std::vector<std::vector<int> >  &, std::vector<std::vector<int> > & );

//*******************************************************//
int main(int argc, char* argv[]){

	int max_time; 
	string Tfile, Dfile;
	std::cout<<"start dynamic-c"<<std::endl;
	//std::cout<<"start dynamic-c process, number of argv:"<<argc<<std::endl;
	if(argc != 3){
		std::cout<<"Usage:"<<"\n \t"<<argv[0]<<" Tstable-file"<<" Dstable-file"<<" max_time"<<std::endl;
		std::cout<<"where	Tstable-file and Dstable-file are stable time and stable distance profiles of the car on corresponding road segment"<<std::endl;
		std::cout<<"\tmax_time: this set the limit of arrival time"<<std::endl;
		std::cout<<"	E.g:"<<argv[0]<<" T.csv D.csv 4000"<<std::endl;
		std::cout<<"-------end--------"<<std::endl;
		std::cout<<"Type Y/N to run the default:"<<argv[0]<<" ../../profiles/90STNORMED.csv ../../profiles/90SDNORMED.csv 500:"<<std::endl;
	/* 	std::string c;
		std::cin >>c;
		std::cout<<"You have typed: "<<c;
		if(c == "y" || c == "Y"){
			std::cout<<" .Continue Running...";
			Tfile = "../../profiles/90STNORMED.csv";
			Dfile = "../../profiles/90SDNORMED.csv";
			max_time = 500; 
			std::cout<<" with: Tfile= " << Tfile <<" Dfile = "<< Dfile <<" and max_time = "<<max_time<<std::endl;
		}
		else{
			std::cout<<" .Stop Running!"<<std::endl;
			return 0; 
		}   */
 		std::cout<<" .Continue Running...";
		Tfile = "../../profiles/90STNORMED.csv";
		Dfile = "../../profiles/90SDNORMED.csv";
		max_time = 500; 
		std::cout<<" with: Tfile= " << Tfile <<" Dfile = "<< Dfile <<" and max_time = "<<max_time; 
	}
	else {
		Tfile = argv[1];
		Dfile = argv[2];
		max_time = atoi(argv[3]);
	}

	int N = max_time/2 + 1;
		
	/* open log file to write down running time */
	std::string log_file_name = string("running_time.csv");
	std::ofstream logFile(log_file_name.c_str() );
	std::vector<std::vector<int> > T, D; 
	Timer timer;
	timer.start();
	T = readFile(Tfile, NUM_DISCRETE_VEL);
	D = readFile(Dfile, NUM_DISCRETE_VEL);
	// T = readFile("/bigdrive/pygame/src/carSimulation3/profiles/T.csv", M);
	// D = readFile("/bigdrive/pygame/src/carSimulation3/profiles/D.csv", M);
	// T = readFile("./profiles/90STNORMED.csv", M);
	// D = readFile("./profiles/90SDNORMED.csv", M);
 	/* std::cout<<"T table:"<<std::endl;
	for(int i=0; i<11; i++){
		for(int j = 0; j<11; j++)
			std::cout<<T[i][j]<<"\t";
		std::cout<<"\n";
	}  */


	std::cout<<"Now, begin dynamic programming."<<std::endl;
	std::vector<std::vector<fSet> > G(M, std::vector<fSet>(N));
	
	/* initialize a table's elements that have distance value equal 0 */
	for(int i=0;i<M;i++){

		classPoint2T nPoint(0,i);
		fSet x;
		x.push_back(nPoint);
		G[i][0] = x;
	}

	// print the original
/* 	for(int i=0;i<M;i++){
		for(int j=0;j<N;j++){
			std::cout<<"i,j:"<<i<<" "<<j<<" have size ="<<G[i][j].size()<<std::endl;
			for(unsigned int k=0;k<G[i][j].size();k++)
				G[i][j][k].print();
		}
	}   */
	
	/* dynamic program */
	for(int k=0;k<N;k++){
		for(int i=0;i<M;i++){
			// std::cout<<"problem with i,k:"<<i<<" "<<k<<std::endl;
			for(int j=0;j<M;j++)
				if(k>= D[i][j]){
					// std::cout<<"	subproblem: j:"<<j<<" k:"<<k<<" vs D[i][j]:"<<D[i][j]<<std::endl;
					if(addPoints(G[i][k], G[j][k-D[i][j]], T[i][j], max_time)){};
				}
		} 
	} 
	/* print final result */
/* 	std::cout<<"\nFinal data is:"<<std::endl;
	for(int i=0;i<M;i++){
		for(int j=0;j<N;j++){
			std::cout<<"<$"<<i<<","<<j<<">"<<std::endl;
			for(unsigned int k=0;k<G[i][j].size();k++)
				G[i][j][k].print();
			std::cout<<"<"<<i<<","<<j<<"&>"<<std::endl;
		}
	}   */
	
	std::cout<<"Acquiring feasible points took: running time:"<< " seconds" <<std::endl;
	
	// write down running time: 
	logFile << "*******start dynamic********" <<"\n";
	logFile << timer.now() << ":" <<" max_time\t"<< max_time <<"\n";
	logFile << timer.now() <<": table creation time:\t" << timer.elapse()<<"\n"; 
	
	
	/* Find trajectory corresponding different arrival configurations and road segment's distances */
	// vector of road segments 
	std::vector<int> distance_array;
	
	distance_array.push_back(max_time/4);
	distance_array.push_back(max_time*3/8);
	distance_array.push_back(max_time/2);

	
	// choose feasible points for each road segment 
	/* initialize random seed: */
	srand (time(NULL));
	for(unsigned int i=0; i<distance_array.size(); i++){
		for(int j=0; j<NUM_PLANS; j++){
			int v0_index; 
			int feasible_size;
			while(true){
				v0_index = rand()%M; 
				feasible_size = G[v0_index][distance_array[i]].size();
				if(feasible_size >= NUM_PLANS){
					// std::cout<<"subproblem "<<v0_index<<"-"<<distance_array[i]<<" has" << feasible_size <<" plans, >= NUM_PLANS. Continue"<<std::endl; 
					break; 
				}
				// std::cout<<"subproblem "<<v0_index<<"-"<<distance_array[i]<<" has" << feasible_size <<" plans, >= NUM_PLANS. Move to next v0_index."<<std::endl;  
			}
			std::cout<<"\n**********************"<<std::endl;
			// std::cout<<"\n** Problem v0 = "<< v0_index<< "distance = "<<distance_array[i]<<std::endl;
		
			classPoint2T point(0,0);
			int rand_index = rand()%feasible_size; 
			point = G[v0_index][distance_array[i]][rand_index];
			// std::cout<<"\t* Chosen feasible_point: "<<point.time<<" "<<point.vel<<"\n"<<std::endl;
			if(findTrajectory(G, distance_array[i], v0_index, point, T, D) > 0) {}; 
			// std::cout<<"One more plan is extracted!"<<std::endl;
			
		}
		std::cout<<"\n**********************"<<std::endl;
	}
	

	// write down running time: 
	logFile << "************************" <<"\n";
	logFile << timer.now() << ":" <<" time to extract plans:\t"<< timer.elapse() <<"\n";
	logFile << "************************" <<"\n";
	logFile << timer.now() << ":" <<" End! Total running time:\t"<< timer.stop() <<"\n";
	logFile << "************End dynamic programming **********************" <<"\n";
	return 0; 
}

//*******************************************************//
int addPoints(std::vector<classPoint2T>& F_orig, std::vector<classPoint2T> &F_sub, int delta_t, int max_time){
	/* This functions add valid feasible points of F_sub to F_orig after shifting an amount of delta_t time*/
	// std::cout<<"adding point"<<std::endl;
	
	/* iterate over the both sorted vectors and merge them together to a temporary  */
	unsigned int sub_index = 0, orig_index = 0; 
	std::vector<classPoint2T> F_aux, F_shifted;
	
	/* add time values to feasible points of F_sub and copy to F_shifted */
	for(unsigned int i=0;i<F_sub.size();i++){
		if(F_sub[i].time + delta_t > max_time)
			break;
		int temp_time = F_sub[i].time; 
		int temp_vel =  F_sub[i].vel;
		classPoint2T point(temp_time + delta_t, temp_vel);
		F_shifted.push_back(point);
	}
/* 	std::cout<<"\tF_shifted is:"<<std::endl;
	for(unsigned int i=0;i<F_shifted.size();i++)
		F_shifted[i].print(); */
	
	while(orig_index < F_orig.size() || sub_index < F_shifted.size()){
		// std::cout<<"orig_index:"<<orig_index<<" sub_index:"<<sub_index<<std::endl;
		/* case 1: done iteration with F_orig */
		if( orig_index == F_orig.size()){
			// std::cout<<"case 1"<<std::endl;
			// append the entire elements of F_shifted to F_aux
			F_aux.reserve(F_aux.size() + F_shifted.size() - sub_index-1);
			F_aux.insert(F_aux.end(), F_shifted.begin() + sub_index, F_shifted.end());
			F_orig = F_aux;
			return 1; 
		}
		
		/* case 2: done iteration with F_shifted */
		if(sub_index == F_shifted.size()){
			// std::cout<<"case 2 and F_orig sofar:"<<std::endl;
			/* for(unsigned int i=0;i<F_orig.size();i++)
				F_orig[i].print(); */
			// append the entire elements of F_orig to F_aux
			F_aux.reserve(F_aux.size() + F_orig.size() - orig_index-1);
			F_aux.insert(F_aux.end(), F_orig.begin() + orig_index, F_orig.end());
			F_orig = F_aux;
			return 1;
		}

		/* case 3: if not done iteration both F_orig and F_mod:  */
		// add smaller element to F_aux and continue iterate the corresponding vector of which that smaller element comes from 
		// std::cout<<"case 3"<<std::endl;
		int leq = F_orig[orig_index].leqTuple2(F_shifted[sub_index]);
		if(leq==1){
			// F_shifted[sub_index].print();
			F_aux.push_back(F_shifted[sub_index]);
			sub_index +=1;
		}
		if(leq==-1){
			// F_orig[orig_index].print();
			F_aux.push_back(F_orig[orig_index]);
			orig_index +=1;
		}
		if(leq==0){
			// F_orig[orig_index].print();
			F_aux.push_back(F_orig[orig_index]);
			orig_index +=1;
			sub_index +=1;
		}
	}
	
	return 1;
}

//*******************************************************//
int findTrajectory(std::vector<std::vector<fSet> >& G, int &targetDistance, int v0_index, classPoint2T &point, std::vector<std::vector<int> >  &T, std::vector<std::vector<int> > &D){
	/* This function find out the trajectory of a feasible point */
	// Initialize a trajectory vector 
	std::vector<classPoint2T> traj;


	// std::cout<<"Size of G table:"<<M<<" x "<<N<<std::endl;
	int t = point.time;
	int d = targetDistance;
	int vend_index = point.vel;
	
  	// print T 
	std::cout<<"T table in findTrajectory:"<<std::endl;
	for(int i=0; i<M; i++){
		for(int j = 0; j<M; j++)
			std::cout<<T[i][j]<<"\t";
		std::cout<<"\n";
	}  
	
	// Add starting point to the trajectory
	classPoint2T startingPoint(0,v0_index);
	traj.push_back(startingPoint);
	
	/* Now travere the table and add vint and time to the trajectory. */
	if(recursiveMove(traj, t, d, vend_index, v0_index, G, T, D) ) {
		// std::cout<<"\t Successfully retrieve a plan"<<std::endl;
	}
	
/* 	// if starting time is not 0, donot return the obtained plan 
	if(traj[0].time >= 1) {
		std::cout<<"\t But starting time is not zero."<<std::endl; 
		return -1; 
	} */
	
	std::cout<<"\tplan obtained with arrival point ";
	point.print();
	std::cout<<" is:" <<std::endl;
	std::cout<<"[["<<targetDistance<<","<<v0_index<<","<<point.vel<<","<<point.time<<"],"<<std::endl;
	std::cout<<"[";
	for(unsigned int i =0;i<traj.size();++i){
		traj[i].print();
		if(i<traj.size() - 1)
			std::cout<<",";
	}
	std::cout<<"]\n"<<std::endl;
	
	return 1;
}

/* This fuction traveres the table and add vint and time to the trajectory. */
int	recursiveMove(std::vector<classPoint2T> &traj, int t, int d, int vend_index, int v0_index, std::vector<std::vector<fSet> >& G, std::vector<std::vector<int> >  &T, std::vector<std::vector<int> > &D  ){
	// std::cout<<"t and d: "<<t<< " - "<<d<<std::endl; 
	if(t <= 0.1 && d <= 0.1){
		// std::cout<<" End recursive move!"<<std::endl;
		return 1; 
	} 
	  	// print T 
	std::cout<<"T table in recursive:"<<std::endl;
	for(int i=0; i<26; i++){
		for(int j = 0; j<26; j++)
			std::cout<<T[i][j]<<"\t";
		std::cout<<"\n";
	} 
	for(int vint_index=0; vint_index<M; vint_index++){
		int delta_d = d - D[v0_index][vint_index];
		if(delta_d >=0){
			int delta_t = t - T[v0_index][vint_index];
			classPoint2T point(delta_t, vend_index); 
			if(delta_t>=0 && point.inSet(G[vint_index][delta_d]) >=0){
				/* add point to the trajectory */
				std::cout<<"v0_index: "<<v0_index<<" vint_index: "<<vint_index<<" T[][]: "<<T[v0_index][vint_index]<<" D[][]: "<<D[v0_index][vint_index]<< " t:" <<t<<" d:"<<d<<std::endl;
				classPoint2T intPoint(T[v0_index][vint_index] + traj.back().time, vint_index); 
				traj.push_back(intPoint);
				// std::cout<<"\t recursiveMove: obtained an intermediate point:"<< intPoint.time <<"-"<<intPoint.vel<<std::endl; 
				// continue recursiveMove() with new ending time, ending distance and starting velocity
				recursiveMove(traj, delta_t, delta_d, vend_index, vint_index, G, T, D);
				break; 
			}
		}
	}
}



