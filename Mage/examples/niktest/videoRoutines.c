/* 
 =========================================================================
 File: videoRoutines.c

 This set of routines sets up the video system and starts
 fast capture of images into a user-supplied buffer
  =========================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev.h>
#include "videoRoutines.h"

#define V4L_DEVICE "/dev/video0"

#define VERBOSE 1

struct video_capability vcap;
struct video_channel    *vc;
struct video_picture    vp;
struct video_mmap       mm[30];
int                     video_fd = -1;
static struct video_mbuf        gb_buffers = { 2*0x151000, 0, {0,0x151000 }};
static int curChannel = 1;
static int curFrame = 0;
static int videoDeviceOpen = 0;

/* 
   This routine opens the video stream and tests the requested height
   and width of the images to be capture against the video card
*/
unsigned char *OpenVideoDevice(long *height, long *width, long nBytes, long nFrames, int verbose, long offsets[]) {
  unsigned char* buf;
  int i;
  
  /* opens the device for R/W and assigns a file/stream number to video_fd */
  video_fd = open(V4L_DEVICE, O_RDWR);
  if(video_fd<=0){
    perror("open");
    exit(-1);
  }

  /* queries the video device to find out its capabilities 
      - name
      - type
      - channels 
      - audios
      - maxwidth
      - maxheight
      - minwidth
      - minheight
  */
  if(ioctl(video_fd, VIDIOCGCAP, &vcap)<0){
    perror("VIDIOCGCAP");
    exit(-1);
  }

  if(verbose) {
    fprintf(stderr,"Video Capture Device Name : %s\n",
	    vcap.name);
    fprintf(stderr,"Maximum size (w, h): (%d, %d)\n",
	    vcap.maxwidth,
	    vcap.maxheight);
    fprintf(stderr,"Minimum size (w, h): (%d, %d)\n",
	    vcap.minwidth,
	    vcap.minheight);
  }

  /* bound the height and width values passed to this function */
  if(*width < vcap.minwidth)
    *width = vcap.minwidth;
  else if(*width > vcap.maxwidth)
    *width = vcap.maxwidth;
  if(*height < vcap.minheight)
    *height = vcap.minheight;
  else if(*height > vcap.maxheight)
    *height = vcap.maxheight;

  /* queries the video device for the channel information
     - channel
     - name
     - tuners
     - flags (channel has a tuner (1), channel has audio (2))
     - type (tv (1), camera (2))
     - norm (set by channel)
  */
  vc = (struct video_channel *)malloc(sizeof(struct video_channel) * vcap.channels);
  if(vc == NULL) {
    fprintf(stderr, "OpenVideoDevice: Unable to allocate memory\n");
    return(NULL);
  }

  for(i=0;i<vcap.channels;i++){
    vc[i].channel = i;
    if(ioctl(video_fd, VIDIOCGCHAN, &(vc[i]))<0){
      perror("VIDIOCGCHAN");
      exit(-1);
    }

    if(verbose) {
      fprintf(stderr,"Video Source (%d) Name : %s\n",i, vc[i].name);
      fprintf(stderr, "Flags %d, type %d, norm %d\n", 
	      (int)vc[i].flags, 
	      (int)vc[i].type, 
	      (int)vc[i].norm);
    }

  }

  /* This allocates a memory map from the video device (fd) to 
     a block of memory.
  */
  if (-1 == ioctl(video_fd, VIDIOCGMBUF, &gb_buffers)) {
    perror("ioctl VIDIOCGMBUF");
  }
  for(i=0;i<nFrames;i++) {
    offsets[i] = gb_buffers.offsets[i];
    printf("buffer %d = %ld\n", i, offsets[i]);
  }

  if(verbose) {
    printf("buffer size: %ld\n", (long)gb_buffers.size);
    printf("buffer 0 offset: %ld\n", (long)gb_buffers.offsets[0]);
    if(nFrames == 2)
      printf("buffer 1 offset: %ld\n", (long)gb_buffers.offsets[1]);
  }

  buf = (unsigned char*)mmap(0, gb_buffers.size, PROT_READ|PROT_WRITE, MAP_SHARED, video_fd, 0);
  if((int)buf < 0){
    perror("mmap");
    return(NULL);
  }

  /* tells the device to set the video channel */
  curChannel = 1;
  vc[curChannel].channel = 1;
  vc[curChannel].norm = 1; // this seems to be necessary (?)
  if(ioctl(video_fd, VIDIOCSCHAN, &(vc[curChannel])) < 0){
    perror("VIDIOCSCHAN");
    munmap(buf, gb_buffers.size);
    return(NULL);
  }

  /* sets up the memory map parameters */
  for(i=0;i<nFrames;i++) {
    /* A frame of 0 means only capture a single frame (frame 0) */
    mm[i].frame  = i;

    /* specifies the size of the image to capture */
    mm[i].height = (*height);
    mm[i].width  = (*width);

    switch(nBytes) {
    case 1:
      /* specifies grayscale */
      mm[i].format = VIDEO_PALETTE_GREY;
      break;
    case 2:
      /* specifies 16 bit 565 RGB */
      mm[i].format = VIDEO_PALETTE_RGB565;
      break;
    case 3:
      /* specifies 24 bit RGB */
      mm[i].format = VIDEO_PALETTE_RGB24;
      break;
    case 4:
      /* specifies 32 bit RGB */
      mm[i].format = VIDEO_PALETTE_RGB32;
      break;
    default:
      printf("Error in number of bytes\n");
      return(NULL);
    }
  }

  videoDeviceOpen = 1;

  return(buf);
}

int CloseVideoDevice(unsigned char *buf, long height, long width, long nBytes, long nFrames) {

  if(!videoDeviceOpen) {
    fprintf(stderr, "CloseVideoDevice: Device is not open\n");
    return(-1);
  }

  if(buf)  /* unmap the memory */
    munmap(buf, gb_buffers.size);
  
  if(video_fd >= 0)  /* close the video device */
    close(video_fd);

  free(vc);

  videoDeviceOpen = 0;

  return(0);
}

int CaptureVideo(int frame) {

  if(!videoDeviceOpen) {
    fprintf(stderr, "CaptureVideo: Device is not open\n");
    return(-1);
  }

  /* tells the video device to capture the specified frame */
  if(ioctl(video_fd, VIDIOCMCAPTURE, &(mm[frame]))<0){
    perror("VIDIOCMCAPTURE");
    return(-1);
  }

  curFrame = frame;

  return(0);
}

int SyncVideo(long frame) {

  if(!videoDeviceOpen) {
    fprintf(stderr, "SyncVideo: Device is not open\n");
    return(-1);
  }

  /* waits until the mmap from the current frame is finished */
  if(ioctl(video_fd, VIDIOCSYNC, &(mm[frame].frame))<0){
    perror("VIDIOCSYNC");
    return(-1);
  }

  return(0);
}

int SetVideoChannel(int newChannel) {

  if(!videoDeviceOpen) {
    fprintf(stderr, "SetVideoChannel: Device is not open\n");
    return(-1);
  }

  if(newChannel != curChannel) {
    vc[newChannel].norm = 1;

    if(ioctl(video_fd, VIDIOCSCHAN, &(vc[newChannel])) < 0){
      perror("VIDIOCSCHAN");
      return(-1);
    }

    curChannel = newChannel;
  }

  return(0);
}

