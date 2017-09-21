#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <math.h>
#include "../../include/Mage.h"
#include "../../include/ptzClient.h"
#include "videoRoutines.h"
#include "ppmIO.h"

#define VISION_DEFAULT_IMAGE_ROWS     120
#define VISION_DEFAULT_IMAGE_COLUMNS  160
#define VISION_DEFAULT_IMAGE_DEPTH    3
#define VISION_DEFAULT_NUM_FRAMES     2

unsigned char* findface(unsigned char *raw_image,long rows,long cols);

unsigned long timeNow(){
  struct timeb t;
  ftime(&t);
  return ((t.time<<10)|t.millitm);
}


int main(int argc, char *argv[])
{
  unsigned long tN;
  long th;
  long State[NUM_STATE]; // State is where Mage dumps all of its info
  Pixel *imageBuffer[2] = {NULL,NULL};
  unsigned char *rawImage;
  unsigned char *faceImage;
  int CurFrame, frame;
  long imageHeight = VISION_DEFAULT_IMAGE_ROWS;
  long imageWidth = VISION_DEFAULT_IMAGE_COLUMNS;
  long imageDepth = VISION_DEFAULT_IMAGE_DEPTH;
  static long offset[VISION_DEFAULT_NUM_FRAMES];
  char filename[256];
  long x;
  long y;
  long t;
  long cx =-500;
  long cy =1700;
  long cpos[6] = {-1700, -1200, -600, 600, 1200, 1700}; 
  
  if (imageBuffer[0] != NULL) {
    CloseVideoDevice((unsigned char *)imageBuffer[0], imageHeight, imageWidth, imageDepth, 2);
  }
  
  rawImage = (unsigned char *)OpenVideoDevice(&imageHeight, 
					      &imageWidth, 
					      imageDepth, 
					      VISION_DEFAULT_NUM_FRAMES, 
					      0,
					      offset);
  if(!rawImage) {
    printf("Vision: Unable to open camera.\n");
    exit(-1);
  }

  //  gblImageBuffer = rawImage;
  // set up a pointer to the second image buffer
  imageBuffer[0] = (Pixel *)rawImage;
  imageBuffer[1] = (Pixel *)&(rawImage[offset[1]]);

  CaptureVideo(0);
  CaptureVideo(1);
  CurFrame = 0;
  frame = 0;

  SyncVideo(CurFrame);
  CurFrame = !CurFrame;
  SyncVideo(CurFrame);

  connectRobot(State);   // Connect (serial line) with the robot 
  irOn(); // Start up IRs
  sonarOn(); // Start up Sonars
  
  vm(0,0);
  startPTZ();
  usleep(1000000);

  tN = timeNow();

  CaptureVideo(0);
  CaptureVideo(1);


  for (frame=0;frame<54;frame++){
    if (!(frame%6)){
      vm(250, 0);
      usleep(1000000);
      vm(0,0);
      usleep(1000000);
    }
    
    cx = cpos[frame%6];
    setPanTilt(cx,cy);
    while (!donePTZ()) usleep(10000);
    
    x = State[STATE_X];
    y = State[STATE_Y];
    t = State[STATE_T];
   
    CurFrame = !CurFrame;
    CaptureVideo(CurFrame); 
    SyncVideo(CurFrame);
    flipRedBlue(imageBuffer[CurFrame], imageHeight, imageWidth, imageBuffer[CurFrame]);
    //    faceImage = findface((unsigned char*)imageBuffer[CurFrame], imageHeight, imageWidth);
    sprintf(filename,"frame%s%d.ppm",(frame<10)?"0":"",frame);
    writePPM(imageBuffer[CurFrame],imageHeight,imageWidth,255,filename);
    //    sprintf(filename,"face%s%d.ppm",(frame<10)?"0":"",frame);
    //  writePGM(faceImage,imageHeight,imageWidth,255,filename);
    printf("wrote %s at %d %d %d %d\n",filename, x, y, cx+t, cy);
  }

  vm(0,0);
  disconnectRobot(); // Disconnect and shut down sensors
  return 0;
}







