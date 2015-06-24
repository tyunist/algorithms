#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream.h>
#include <fstream.h>

#include "modified-graphics.h" // modified Jon's graphics file 

#define MAX_COLORS 5  

#define CAR_RADIUS 10


#define WINDOW_WIDTH 850
#define WINDOW_HEIGHT 600


#define X_OFFSET 10
#define Y_OFFSET 590

static jwin *WINDOW = NULL; // main window
static jwin *WINDOW2 = NULL; // steps window

//  define an array for storing colors 
static unsigned int COLORS[MAX_COLORS];


static char *COLOR_NAMES[MAX_COLORS] = {"OrangeRed","SlateBlue1", 
					  "DarkSeaGreen1",  "LightYellow1", 
					  "DarkOrange1"}; 

void display_mcar(double pos, double vel); 
void display_data(char *data, int refresh); 

void init_win(int wd, int ht);
void init_win2(int wd, int ht);

void start_up_window();

jwin *start_display();
jwin *start_display2();


