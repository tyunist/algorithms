// header file for modified graphics file 

#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream> // Ty removed .h

#include <X11/Xlib.h>
//#include <X11/Xlibint.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
// Ty modified iostream.h and add following line
using namespace std;


#if !defined(_BASIC_GRAPHICS_)

//  remember server, actual window, and graphics defaults 

#define FALSE 0

typedef struct
{
  Display *disp;
  Window win;
  GC gcon;
  unsigned long fg;
} jwin;


#endif
#define _BASIC_GRAPHICS_

//  function prototypes 

jwin *j_make_window (int width, int height, char *color,
		     char *host, char *font, int expose);
unsigned long find_color_val (char *name, jwin *w);

void j_kill_window (jwin *w);
void j_clear_window (jwin *w);
void j_refresh_window (jwin *w);

void j_name_window (jwin *w, char *title);

void j_draw_line (int x1, int y1, int x2, int y2,
		  unsigned long color, int thickness, int dashed, int _xor,
		  jwin *w);
void j_draw_rectangle (int xlo, int ylo, int width, int height,
		       unsigned long color, int thickness, int _xor,
		       jwin *w);
void j_draw_circle (int xcent, int ycent, int radius,
		    unsigned long color, int thickness, int _xor,
		    jwin *w);

void j_draw_string (int xleft, int ybase, char *text,
		    unsigned long color, jwin *w);


int mouse_coords (int *x, int *y, int *button, jwin *w);
int mouse_click (int *x, int *y, jwin *w);




