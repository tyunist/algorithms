/*
 * File: Timer.h
 * Last modified on Wed July 31  02:10 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides timer counting with accuracy of milisecond
 * Usage: First, create an timer object
 									Timer timer;
 					then, start counting
 									timer.start();
 * 				At each elapse, call 
 									double duration = Timer::elapse(); 
 *				At the end, call 
 									double runningTime = Timer::stop();
 					Print time duration: 
 									Timer::timer.printTime(duration);
 */
#ifndef _Timer_h_
#define _Timer_h_
 
#include <iostream>
#include <ctime>
#include <sys/time.h> /// gettimeofday()
#include <string> 
using namespace std;
 
class Timer {
private:

    struct timeval startTime;
		struct timeval elapsedTime;
		
public:

    void start();
	/* /// Get current time in second
	double now(); */
    /// Get total time in second
    double stop();
    /// Get elapsed time in second
    double elapse();
    /// Print time to std
    void printTime(double duration);
	// display current time in d-m-y format 
	std::string now();
};

#endif
