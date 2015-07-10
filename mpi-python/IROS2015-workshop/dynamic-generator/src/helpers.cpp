/* helpers.cpp
This stores handy functions which are utilized by the main function*/


#include "helpers.h"



std::vector<std::vector<int> > readFile(std::string fileName, int line_num){
	/* This functions read a file which has line_num number of lines and return a matrix storing data */
	
	// input file 
	std::ifstream inFile(fileName.c_str());
	if(inFile.fail()){
		std::cout<<"cannot open file.!"<<std::endl;
	}
	
	// std::cout<<"number of lines in file "<<line_num<<std::endl;
	// matrix to store data 
	std::vector<std::vector<int> > data; 
	
	for(int i=0; i< line_num; i++){
		std::vector<int> temp;
		for(int j=0; j< line_num; j++){
			// std::cout<<"begin with i="<<i<<" j="<<j<<std::endl;
			int x; 
			inFile >> x;
			temp.push_back(x);
			// std::cout<<"x="<<x<<std::endl;
		}
		data.push_back(temp);
	}
	inFile.close();
	// std::cout<<"size of data:"<< data.size();
	return data; 
} 
/////////////////////////////////////////////////////////////////////////
/* Functions of Class Point2T that define feasible points and their handy functions such as to compare two points*/
classPoint2T::classPoint2T(int t, int v){
	this->time = t;
	this->vel = v;
}

int classPoint2T::leqTuple2(classPoint2T point2){
	if(this->time > point2.time)
		return 1;
	else if(this->time < point2.time)
		return -1;
	else{
		if(this->vel == point2.vel)
			return 0;
		else if(this->vel > point2.vel)
			return 1;
		else
			return -1;
	}
}

void classPoint2T::shiftTime(int delta_t){
	this->time += delta_t;
}

void classPoint2T::print(){
	std::cout<<"("<<this->time<<","<<this->vel<<")";
}

int midPoint(int imin, int imax){
	return (imin + ((imax - imin) / 2) );
}

/* find whether the point exist in set  */
int classPoint2T::inSet(std::vector<classPoint2T> & set_of_point){
	/* Use binary search algorithm to find out whether the point is in set_of_point */
	
	if(set_of_point.size() <= 0){
		std::cout<<"classPoint2T::inSet(): set_of_point is empty. Return -1"<<std::endl;
		return KEY_NOT_FOUND;
	}
	
	int imax = set_of_point.size();
	int imin = 0; 
	
	// continue searching while [imin,imax] is not empty
	while (imax >= imin){
		// calculate the midpoint for roughly equal partition
		int imid = midPoint(imin, imax);
		if(this->leqTuple2(set_of_point[imid]) == 0)
			// key found at index imid
			return KEY_FOUND; 
		// determine which subarray to search
		else if (this->leqTuple2(set_of_point[imid]) > 0)
			// change min index to search upper subarray
			imin = imid + 1;
		else         
			// change max index to search lower subarray
			imax = imid - 1;
	}
	// key was not found
	return KEY_NOT_FOUND;
	
}