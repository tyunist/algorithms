
#ifndef _decl_rand_h_
#define _decl_rand_h_

#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define MBIG 1000000000
#define MSEED 161803398
#define MZ 0
#define FAC (1.0/ MBIG)

long double ran3(int *idum);

int choose_random_int_value(int range);

long double choose_random_value();

void initialize_random_number_generator(int s);

#endif
