// display file for mountain car problem 

#include "display.h" 

#include "cmac.h"

int old_ppixel=0, old_hpixel=0; 

void  init_win (int wd, int ht) // Make window     
{

  cout << "Creating window ..." << endl; 

  cout.flush(); 

  WINDOW = j_make_window(wd, ht, NULL, NULL, "12x24", 1);

  j_name_window(WINDOW, "Mountain Car Problem using TD/Sarsa and CMAC"); 

}

void  init_win2 (int wd, int ht) // Make second window     
{

  cout << "Creating data window ..." << endl; 

  cout.flush(); 

  WINDOW2 = j_make_window(wd, ht, NULL, NULL, "12x24", 1);

  j_name_window(WINDOW2, "Data Window for Mountain Car Problem"); 

}

//  allocate some colors:   red,blue,green,yellow,orange 

void allocate_colors()
{

  int c; 

  cout << "\nAllocating colors ..." << endl; 

  cout.flush(); 

  for (c=0; c<=4; c++)
    COLORS[c] = (int) find_color_val(COLOR_NAMES[c], WINDOW);

}

// scale pos into window width 

int scale_pos(double pos)
{

  double step = POS_RANGE[1] - POS_RANGE[0]; 

  return((pos - POS_RANGE[0])*WINDOW_WIDTH/step + X_OFFSET); 
}

// scale height into window height 

int scale_height(double height)
{
  double step = 2; 

  return(Y_OFFSET - (height + 1)*(WINDOW_HEIGHT-10)/step); 

}


// show mountain car given position

void display_mcar(double pos, double vel)
{

  double height = sin(3*pos); // height of mcar 

  int height_pixel = scale_height(height); // convert height into pixel value

  int pos_pixel = scale_pos(pos); // convert pos into pixel value 

   XFlush(WINDOW->disp);

  if (old_ppixel != 0 && old_hpixel !=0)
    j_draw_circle(old_ppixel, old_hpixel, CAR_RADIUS, COLORS[0],0,1,WINDOW);

  j_draw_circle(pos_pixel, height_pixel, CAR_RADIUS, COLORS[0],0,0,WINDOW);

  old_ppixel = pos_pixel;
  old_hpixel = height_pixel; 

}

void display_data(char data[], int refresh)
{

  XFlush(WINDOW2->disp);

  if (refresh)  // new value 
    {
      j_clear_window(WINDOW2); 
      j_clear_window(WINDOW2); 
    }
      

//  sprintf(name, "Shortest Solution:  %d steps ", steps); 

  j_draw_string(10, 50, data, COLORS[1], WINDOW2); 

}


  
jwin *start_display()
{

// pop up a window to draw world 

  init_win(WINDOW_WIDTH,WINDOW_HEIGHT); 

  allocate_colors();

  XFlush(WINDOW->disp);

//  set up event filter 

  XSelectInput(WINDOW->disp,WINDOW->win,  
	      ButtonPressMask);

//  ButtonPressMask|ButtonMotionMask|ButtonReleaseMask|ExposureMask); 

  return(WINDOW);

}

jwin *start_display2()
{

// pop up a window to draw steps

  init_win2(500,100); 

  XFlush(WINDOW2->disp);

//  set up event filter 

  XSelectInput(WINDOW2->disp,WINDOW2->win,  
	      ButtonPressMask);

//  ButtonPressMask|ButtonMotionMask|ButtonReleaseMask|ExposureMask); 

  return(WINDOW2);

}

