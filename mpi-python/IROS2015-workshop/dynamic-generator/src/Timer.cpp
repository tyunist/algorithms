 /*
 * File: Timer.cpp
 * Last modified on Wed July 31  02:10 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides timer counting with accuracy of milisecond
 * Usage: Timer::start(); to start counting
 * 			At each elapse, call 
 									double duration = Timer::elapse(); 
 *			At the end, call 
 									double runningTime = Timer::stop();
 */
 
#include <iostream>
#include <ctime>
#include <sys/time.h> /// gettimeofday()
#include "Timer.h"
#include <iomanip> /// Format cout
using namespace std;

void Timer::start(){
    gettimeofday(&startTime, NULL);
    elapsedTime = startTime; /// First, there is no elapse. 
}

/* /// Get current time in seconds
double Timer::now(){
    struct timeval currentTime;
    long seconds, useconds;
	double now;
    gettimeofday(&currentTime, NULL);

    seconds  = currentTime.tv_sec;
    useconds = currentTime.tv_usec;
    now = seconds + useconds/1000000.0;
   
	return now;
} */

double Timer::stop(){
    struct timeval endTime;
    long seconds, useconds;
    double duration;

    gettimeofday(&endTime, NULL);

    seconds  = endTime.tv_sec  - startTime.tv_sec;
    useconds = endTime.tv_usec - startTime.tv_usec;

    duration = seconds + useconds/1000000.0;

    return duration;
}

double Timer::elapse(){
    struct timeval currentTime;
    long seconds, useconds;
    double duration;

    gettimeofday(&currentTime, NULL);

    seconds  = currentTime.tv_sec  - elapsedTime.tv_sec;
    useconds = currentTime.tv_usec - elapsedTime.tv_usec;

    duration = seconds + useconds/1000000.0;
		
	elapsedTime = currentTime; /// Update elapsedTime upon each calling
    
    return duration;
}
    
void Timer::printTime(double duration){
    cout<<fixed<<setprecision(3)<<duration<<" seconds\n"<<endl;
}

std::string Timer::now(){
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
	std::string str(buffer);
	return str; 
}