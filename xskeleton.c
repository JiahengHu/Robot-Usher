/*
  xskeleton.c

  This is a skeleton interface for creating a simple
  xwindow in which to draw.  This sample program 
  draws a line on the screen.

  To compile, use
  cc xskeleton.c -lX11 -o xbox

*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

/*
  Global Variables determining window and display
*/
Display *curDisplay;
Window   curWindow;
GC       curGC; 
/* graphics context */

void initWindow() {
   int defScreen;
   XSetWindowAttributes wAttr;
   XGCValues gcValues;
   
   /*
    * connect to the X server.  uses the server specified in the
    * DISPLAY environment variable
    */
   curDisplay = XOpenDisplay((char *) NULL);
   if ((Display *) NULL == curDisplay) {
      fprintf(stderr, "xskeleton:  could not open display.\n");
      exit(-1);
   }
   
   /*
    * begin to create a window
    */
   defScreen = DefaultScreen(curDisplay);
   curWindow = XCreateSimpleWindow(curDisplay, DefaultRootWindow(curDisplay),
			     100, 100, 8000, 8000, 1,
			     BlackPixel(curDisplay, defScreen),
			     WhitePixel(curDisplay, defScreen));
   /*
    * request mouse button and keypress events
    */
   wAttr.event_mask = ButtonPressMask | KeyPressMask | ExposureMask;
   XChangeWindowAttributes(curDisplay, curWindow, CWEventMask, &wAttr);

   /*
    * force it to appear on the screen
    */
   XMapWindow(curDisplay, curWindow);

   /*
    * create a graphics context.  this stores a drawing state; stuff like
    * current color and line width.  this gc is for drawing into our
    * window. 
    */
   gcValues.foreground = BlackPixel(curDisplay, defScreen);
   curGC = XCreateGC(curDisplay, curWindow, GCForeground, &gcValues);

   printf("setting color map to default\n");
   XSetWindowColormap(curDisplay, 
		      curWindow,
		      DefaultColormapOfScreen(DefaultScreenOfDisplay(curDisplay)));   

   return;
}

void refreshWindow() {
  return;
}

void closeWindow() {
   XCloseDisplay(curDisplay);

   return;
}

int main(int argc, char *argv[]) {
   Bool   done = False;
   XEvent curEvent;
   long event_mask = 0xFFFFFFFF;
   char   c;
   int x, y;

   initWindow();

   // draw something
   

   
   while(!done) {
       usleep(50000);

       // x = rand() % 500;
       // y = rand() % 500;
       XDrawPoint( curDisplay, curWindow, curGC, x, y );

       if(XCheckWindowEvent( curDisplay, curWindow, event_mask, &curEvent)) {
	   
     // switch (curEvent.type) {

	  //  case ButtonPress:
	  //      /*
		 // access button by curEvent.xbutton.button (1,2,3)
		 // access position by curEvent.xbutton.x and .y
	  //      */
	  //      printf("button press: %d %d\n", curEvent.xbutton.x, curEvent.xbutton.y);

	  //      XDrawPoint( curDisplay, curWindow, curGC, curEvent.xbutton.x, curEvent.xbutton.y );
	  //      break;

	  //  case KeyPress:
	       
		 // access string using XLookupString(*event,*char,numChars,NULL,NULL)
	       
	  //      if(XLookupString((XKeyEvent *)&curEvent, &c, 1, NULL, NULL) == 1) {
		 //   switch (c) {
		 //   case 'q':
		 //       done = True;
		 //       break;
		 //   case 'c':
		 //       // clear the window
		 //       XClearWindow(curDisplay, curWindow);
		 //       break;
		 //   default:
		 //       break;
		 //   }
	  //      }
	  //      break;

	  //  case Expose:
	  //      refreshWindow();

	  //  default:
	  //      break;
	  //  }
   //     }
   }
   
   closeWindow();
}
