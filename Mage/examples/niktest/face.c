
#include <stdio.h>
#include <stdlib.h>
#include "ppmIO.h"

#define DEBUG 0
#define MAX_BOX_SIZE 5000

#define locateThresh 15
#define HistThresh 30

void threshHist(unsigned char *hist,int thresh);
void gauss(unsigned char *image,unsigned char *out,int width,int height);
void sobel(unsigned char *image,unsigned char *out, int width, int height);
void addImages(unsigned char *image,unsigned char *pic,int width,int height);
void multImages(unsigned char *image,unsigned char *pic,int width,int height);
void smoothKeep(unsigned char *image,unsigned char *out,int width,int height);
//int combineBoxes(Box_t *bbox,Point_t *points,int numBoxes);

unsigned char* findface(unsigned char *raw_image,long rows,long cols){
  int width=cols;
  int height=rows;
  int locations;
  static unsigned char *Histogram=NULL;
  static unsigned char *hist_image=NULL;
  static unsigned char *rawChrome_image=NULL;

  if (!Histogram)
    Histogram = loadRGhist("face.pgm");
  if (!hist_image)
    hist_image=(unsigned char*)malloc(sizeof(unsigned char)*width*height*3);
  if (!rawChrome_image)
    rawChrome_image=(unsigned char*)malloc(sizeof(unsigned char)*
					   width*height*3);
  threshHist(Histogram,HistThresh);
  memcpy(hist_image,raw_image,height*width*3);
  makeChromeImage(raw_image,height,width,rawChrome_image);
  applyHist(rawChrome_image, height, width, Histogram,hist_image);
  gauss(hist_image,rawChrome_image,width,height);
  gauss(rawChrome_image,hist_image,width,height);
  sobel(raw_image,rawChrome_image,width,height);
  addImages(hist_image,rawChrome_image,width,height);
  multImages(rawChrome_image,hist_image,width,height);
  smoothKeep(hist_image,rawChrome_image,width,height);

  //  locations=locateRegions(rawChrome_image,height,width,locateThresh,location,bbox,sizeList);

  return rawChrome_image;
}

void threshHist(unsigned char *hist,int thresh){
  int x,end;

  end=toNum(hist[1]);
  end*=toNum(hist[2]);
  
  for(x=4;x<end+4;x++)
    if(thresh>hist[x])
      hist[x]=0;
}

void gauss(unsigned char *image,unsigned char *out,int width,int height){
  int x,y,temp;
  
  for (y=0;y<height*width;y++)
    out[y]=0;
  
  for (y=1;y<height-1;y++)
    for (x=1;x<width-1;x++){
      temp=image[(x-1)+(y-1)*width];
      temp+=2*image[(x-1)+y*width];
      temp+=image[(x-1)+(y+1)*width];
      temp+=2*image[x+(y-1)*width];
      temp+=4*image[x+y*width];
      temp+=2*image[x+(y+1)*width];
      temp+=image[(x+1)+(y-1)*width];
      temp+=2*image[(x+1)+y*width];
      temp+=image[(x+1)+(y+1)*width];
      temp/=16;
      out[y*width+x]=image[x+y*width];
    }
} 

void sobel(unsigned char *image,unsigned char *out, int width, int height){
  int x,y,temp,ptr;

  for (y=0;y<height*width*3;y++)
    out[y]=0;

  for (y=1;y<height-1;y++)
    for (x=3,ptr=1;x<3*width-3;x+=3,ptr++){
      temp =0;
      temp+=image[(x+3)+(y-1)*width*3];
      temp+=image[(x+4)+(y-1)*width*3];
      temp+=image[(x+5)+(y-1)*width*3];
      temp+=2*image[(x+3)+y*width*3];
      temp+=2*image[(x+4)+y*width*3];
      temp+=2*image[(x+5)+y*width*3];
      temp+=image[(x+3)+(y+1)*width*3];
      temp+=image[(x+4)+(y+1)*width*3];
      temp+=image[(x+5)+(y+1)*width*3];
      temp-=image[(x-3)+(y-1)*width*3];
      temp-=image[(x-2)+(y-1)*width*3];
      temp-=image[(x-1)+(y-1)*width*3];
      temp-=2*image[(x-3)+y*width*3];
      temp-=2*image[(x-2)+y*width*3];
      temp-=2*image[(x-1)+y*width*3];
      temp-=image[(x-3)+(y+1)*width*3];
      temp-=image[(x-2)+(y+1)*width*3];
      temp-=image[(x-1)+(y+1)*width*3];
      temp=abs(temp);
      //      if (temp>1000)
      //temp=0;
      if (temp>255)
	temp=0;
      out[y*width+ptr]=temp;
    }
}

void addImages(unsigned char *image,unsigned char *pic,int width,int height){
  int x;

  for (x=0;x<width*height;x++)
    if (image[x]>pic[x])
      pic[x]=image[x];
}

void multImages(unsigned char *image,unsigned char *pic,int width,int height){
  int x,temp;

  for (x=0;x<width*height;x++){
    temp=image[x]*pic[x];
    if (temp>255)
      pic[x]=255;
    else
      pic[x]=temp;
  }
}

void smoothKeep(unsigned char *image,unsigned char *out,int width,int height){
  int x,y,temp;
  
  for (y=0;y<height*width;y++)
    out[y]=0;
  
  for (y=1;y<height-1;y++)
    for (x=1;x<width-1;x++){
      temp=image[(x-1)+(y-1)*width];
      temp+=image[(x-1)+y*width];
      temp+=image[(x-1)+(y+1)*width];
      temp+=image[x+(y-1)*width];
      temp+=image[x+y*width];
      temp+=image[x+(y+1)*width];
      temp+=image[(x+1)+(y-1)*width];
      temp+=image[(x+1)+y*width];
      temp+=image[(x+1)+(y+1)*width];
      temp/=9;
      if (50<abs(temp-image[x+y*width]))
	out[y*width+x]=0;
      else
	out[y*width+x]=image[x+y*width];
    }
} 
/*
int combineBoxes(Box_t *bbox,Point_t *points,int numBoxes){
  int x,y,box;
  int list[numBoxes];
  
  for (x=0;x<numBoxes;x++)
    list[x]=1;
  
  box=0;
  for (x=0;x<numBoxes-1;x++){
    if (list[x]==1)
      for (y=x+1;y<numBoxes;y++) {
	if ((bbox[x][1]<bbox[y][3])||
	    (bbox[x][3]>bbox[y][1])) {
	  list[y]=0;
	}
      }
  }
  
  for (x=0,y=0;y<numBoxes;y++){
    if(list[y] == 1) {
      bbox[x][0]=bbox[y][0];
      bbox[x][1]=bbox[y][1];
      bbox[x][2]=bbox[y][2];
      bbox[x][3]=bbox[y][3];
      points[x][0]=points[y][0];
      points[x][1]=points[y][1];
      points[x][2]=points[y][2];
      x++;
    }
  }
  
  return x;
}
*/





