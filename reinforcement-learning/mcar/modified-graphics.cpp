// graphics routines modified for C++

#include "modified-graphics.h"


// Make a window on the specified host and then initialize    */
// graphical attributes to standard values. If host if NULL,  */
// value of Unix DISPLAY variable is used. Type "xcolors" at  */
// shell to see possible names (NULL = "paleturquoise"). Type */
// "xfontsel" to see possible type fonts (NULL = Times-12).   */

jwin *j_make_window (int width, int height, char *color,
		     char *host, char *font, int expose)
{
  XSetWindowAttributes attr;
  char* display_name = "whateveryouneed";
  Display *disp; 
 
  Window win;
  GC gcon;
  XGCValues vals;
  jwin *ans;
  unsigned long amask, vmask, bg;
  char wcol[80] = "paleturquoise";
  char style[80] = "-*-times-medium-r-*-12-*";

  if ((ans = (jwin *) malloc(sizeof(jwin))) == NULL)
  {
    cout <<  "\nJ_MAKE_WINDOW: Unable to allocate structure!\n"; 
    exit(-1);
  }

  if ((disp = XOpenDisplay(host)) == NULL)
  {
    cout <<   "\nJ_MAKE_WINDOW: Cannot connect to host !\n" << 
      XDisplayName(host); 
    exit(-2);
  }
	
  attr.bit_gravity = NorthWestGravity;

  attr.backing_store = Always;
  attr.event_mask = ButtonPressMask | ExposureMask;
  attr.do_not_propagate_mask = ButtonPressMask;
  amask = CWBitGravity | CWEventMask | CWDontPropagate | CWBackingStore;
  win = XCreateWindow(disp, (disp->screens)->root,
		      2, 2, width, height, 0,
		      CopyFromParent, InputOutput, CopyFromParent,
		      amask, &attr);

  if (font != NULL) strcpy(style, font);
  vals.font = XLoadFont(disp, style);
  vals.cap_style = CapRound;
  vals.join_style = JoinRound;
  vmask = GCFont | GCCapStyle | GCJoinStyle;
  gcon = XCreateGC(disp, win, vmask, &vals);

  ans->disp = disp;
  ans->win = win;
  ans->gcon = gcon;
  ans->fg = find_color_val("black", ans);

  if (color != NULL) strcpy(wcol, color);
  bg = find_color_val(wcol, ans);
  XSetWindowBackground(disp, win, bg);
  XSetBackground(disp, gcon, bg);
  XStoreName(disp, win, "random");
  XMapWindow(disp, win);
  XClearWindow(disp, win);
  if (expose != 0) XFlush(disp);

  return(ans);
}


//  Find number of color closest to the one requested.   
//  To see color value strings, type "xcolors" at shell.

unsigned long find_color_val (char *name, jwin *w)
{
  XColor cell, exact;
  Display *disp;
  unsigned short er, eg, eb, cr, cg, cb;

  disp = w->disp;
  if (XAllocNamedColor(disp, (disp->screens)->cmap, name, &cell, &exact)
      == 0)
    cout << "\nFIND_COLOR_VAL: Could not allocate color \n" <<  name; 

  return(cell.pixel);
}


// kill window 
void j_kill_window (jwin *w)
{
  XDestroyWindow(w->disp, w->win);
  XFlush(w->disp);
  free(w);
}


// clear 

void j_clear_window (jwin *w)
{
  XClearWindow(w->disp, w->win);
  XFlush(w->disp);
}


// refresh 

void j_refresh_window (jwin *w)
{
  XFlush(w->disp);
}


// assign a title to a window after it has been created

void j_name_window (jwin *w, char *title)
{
  XStoreName(w->disp, w->win, title);
  XFlush(w->disp);
}


//  draw a line with some more variations    
//  a negative color value means _xor drawing  

void j_draw_line (int x1, int y1, int x2, int y2,
		  unsigned long color, int thickness, int dashed, int _xor,
		  jwin *w)
{
  XGCValues vals;
  unsigned long vmask;

  if (_xor != 0)
    vals.function = GXinvert;
  else
    vals.function = GXcopy;
  if (dashed != 0)
    vals.line_style = LineOnOffDash;
  else
    vals.line_style = LineSolid;
  vals.foreground = color;
  vals.line_width = thickness;
  vmask = GCForeground | GCLineWidth | GCLineStyle | GCFunction;
  XChangeGC(w->disp, w->gcon, vmask, &vals);

  XDrawLine(w->disp, w->win, w->gcon, x1, y1, x2, y2);

}


// draw a box with some more parameters     
//  thickness 0 means fill it                

void j_draw_rectangle (int xlo, int ylo, int width, int height,
		       unsigned long color, int thickness, int _xor,
		       jwin *w)
{
  XGCValues vals;
  unsigned long vmask;

  if (_xor != 0)
    vals.function = GXinvert;
  else
    vals.function = GXcopy;
  vals.foreground = color;
  vals.line_width = thickness;
  vmask = GCForeground | GCLineWidth | GCFunction;
  XChangeGC(w->disp, w->gcon, vmask, &vals);

  if (thickness == 0)
    XFillRectangle(w->disp, w->win, w->gcon,
		   xlo, ylo, width+1, height+1);
  else
    XDrawRectangle(w->disp, w->win, w->gcon,
		   xlo, ylo, width, height);
}


//  draw a circle with some more parameters  
//  thickness 0 means fill it                

void j_draw_circle (int xcent, int ycent, int radius,
		    unsigned long color, int thickness, int _xor,
		    jwin *w)
{
  XGCValues vals;
  unsigned long vmask;

  if (_xor != 0)
    vals.function = GXinvert;
  else
    vals.function = GXcopy;
  vals.foreground = color;
  vals.line_width = thickness;
  vmask = GCForeground | GCLineWidth | GCFunction;
  XChangeGC(w->disp, w->gcon, vmask, &vals);

  if (thickness == 0)
    XFillArc(w->disp, w->win, w->gcon,
	     xcent-radius, ycent-radius, 2*radius, 2*radius,
	     0, 360*64);
  else
    XDrawArc(w->disp, w->win, w->gcon,
	     xcent-radius, ycent-radius, 2*radius, 2*radius,
	     0, 360*64);
}


// draw string 

void j_draw_string (int xleft, int ybase, char *text,
		    unsigned long color, jwin *w)
{
  XGCValues vals;

  vals.foreground = color;
  XChangeGC(w->disp, w->gcon, GCForeground, &vals);
  XDrawString(w->disp, w->win, w->gcon, xleft, ybase,
	      text, strlen(text));
}


//  tell where in the window the mouse currently is 

int mouse_coords (int *x, int *y, int *button, jwin *w)
{
  Window root, child;
  int rx, ry;
  unsigned int ww, wh, b, d;
  Bool valid;

  XGetGeometry(w->disp, w->win, &root,
	       &rx, &ry, &ww, &wh, &b, &d);
  valid = XQueryPointer(w->disp, w->win, &root, &child,
			&rx, &ry, x, y, &b);
  switch (b & (Button1Mask | Button2Mask | Button3Mask))
  {
    case Button1Mask:
      *button = 1;
      break;
    case Button2Mask:
      *button = 2;
      break;
    case Button3Mask:
      *button = 3;
      break;
    default:
      *button = 0;
      break;
    }
  if (valid && (*x >= 0) && (*y >= 0)
            && (*x < ww) && (*y < wh))
    return(1);
  else
    return(0);
}


//  Waits for mouse to be clicked in window then tells  
// where the click occured. Returns button number.     
// events did not work so this uses polling instead    

int mouse_click (int *x, int *y, jwin *w)
{
  int button, valid, clear = 0;

  while (1)
  {
    valid = mouse_coords(x, y, &button, w);
    if ((valid == 1) && (clear == 1) && (button != 0))
      break;
    if ((valid == 1) && (button == 0))
      clear = 1;
    else
      clear = 0;
  }
  return(button);
}










