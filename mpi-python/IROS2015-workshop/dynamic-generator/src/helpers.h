/* helpers.h
This stores handy functions which are utilized by the main function*/
#ifndef _helpers_h_
#define _helpers_h_

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#ifndef KEY_NOT_FOUND
#define KEY_NOT_FOUND -1
#endif

#ifndef KEY_FOUND
#define KEY_FOUND 1
#endif
/* This functions read a file which has line_num number of lines and return a matrix storing data */
std::vector<std::vector<int> > readFile(std::string fileName, int line_num);
//////////////////////////////////////////////
/* Class that define feasible points and their handy functions such as to compare two points. The class is written in form of a template to use any kind of data type of points.*/
class classPoint2T{
	public:
		int time;
		int vel; 
		classPoint2T(int, int);
		/* compare with other point to see which whether it is larger, equal or smaller */
		int leqTuple2(classPoint2T);
		/* shift an amount of delta_t in time */
		void shiftTime(int);
		/* expand trajectory to the velocity of subproblem */
		void print();
		/* find whether the point exist in set  */
		int inSet(std::vector<classPoint2T> &);
};


////////////////////////////////////////////////////////////
/* typedef struct to store a feasible points  */


#endif 