#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ppmIO.h"

typedef struct
{
  long item;
  void* next;
} list;

#define XSIZE 160
#define YSIZE 120

#define QSIZE 15

#define MERGESPEED 4
//1 = no merge, 2 = fast, n = slow

long HistResolution = 1;

void NewQ(unsigned char*** imageq){
  int i;
  *imageq = malloc((QSIZE+1)*sizeof(char*));
  for (i=0;i<QSIZE;i++){
    (*imageq)[i]=0;
  }
  (*imageq)[QSIZE]=0; //length is stored at the end
}

void addImage(unsigned char **imageq, unsigned char* I){
  int i;
  if (imageq[QSIZE-1]!=0){
    free(imageq[QSIZE-1]);
  }
  for (i=QSIZE-1;i>0;i--){
    imageq[i]=imageq[i-1];
  }
  imageq[0]=malloc(3*XSIZE*YSIZE);
  memcpy(imageq[0],I,3*XSIZE*YSIZE);

  //length is stored at the end
  if (((int)imageq[QSIZE])<QSIZE){
    imageq[QSIZE]++;
  }
}  

void flipRedBluePixel(
		      unsigned char* imageptr, 
		      unsigned char* outptr){
  unsigned char tempchar;
  tempchar = *imageptr;
  *(outptr+1) = *(imageptr+1);
  *outptr = *(imageptr+2);
  *(outptr+2) = tempchar;    
}

void flipRedBlue(unsigned char *imagestart,
		 long height,
		 long width,
		 unsigned char *outimage){
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char* imageend = &imagestart[height*width*3];

  outptr = outimage;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3){
    flipRedBluePixel(imageptr,outptr);
    outptr+=3;
  }
}

void threshholdEachLowPixel(unsigned char *imageptr,
			    unsigned char red,
			    unsigned char green, 
			    unsigned char blue,
			    unsigned char *outptr){
  if (*imageptr>red){
    *outptr=red;
  }
  else{
    *outptr=0;
  }
  if (*(imageptr+1)>green){
    *(outptr+1)=green;
  }
  else{
    *(outptr+1)=0;
  }
  if (*(imageptr+2)>blue){
    *(outptr+2)=blue;
    }
  else{
    *(outptr+2)=0;
  }
}

void treshholdAllLowPixel(unsigned char* imageptr,
			  unsigned char red,
			  unsigned char green, 
			  unsigned char blue,
			  unsigned char* outptr){
  if ((*imageptr<red)||
      (*(imageptr+1)<green)||
      (*(imageptr+2)<blue)){
    *outptr=0;
    *(outptr+1)=0;
    *(outptr+2)=0;
  }
}

void treshholdEachLow(unsigned char *imagestart,
		      long height,
		      long width,
		      unsigned char red,
		      unsigned char green, 
		      unsigned char blue,
		      unsigned char *outimage){

  unsigned char* imageend = &imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;

  outptr = outimage;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3){
    threshholdEachLowPixel(imageptr,red,green,blue,outptr);
    outptr+=3;
  }
}


void treshholdAllLow(unsigned char* imagestart,
		     long height,
		     long width,
		     unsigned char red,
		     unsigned char green, 
		     unsigned char blue,
		     unsigned char* outimage){

  unsigned char* imageptr;
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* outptr=outimage; 
    
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3){
    treshholdAllLowPixel(imageptr,red,blue,green,outptr);
    outptr+=3;
  }
}

void median(unsigned char* imagestart, 
	    long height,
	    long width,
	    unsigned char* outimage){
  int i;
  int j;
  unsigned char* imageptr;
  unsigned char* begin;
  unsigned char* end;
  int temp;
  int tempred;
  int tempgreen;
  int tempblue;
  static unsigned char* outim=NULL;
  unsigned char* outptr;
  static int* imoffset=NULL;
  unsigned char* imageend=&imagestart[height*width*3];
  int filterhi=3;
  int filterwid=9;
  int filtersize=27;
  int imagewid=width*3;

  if (!outim){
    outim = malloc(width*height*3);
    imoffset = malloc(sizeof(int)*filterhi*filterwid);
  }
  
  outptr=outim;

  for (i=0;i<imagewid+3;i++){
    *(outptr++)=0;
  }
  
  for (i=0;i<filterhi;i++){
    for (j=0;j<filterwid;j++){
      imoffset[i*filterwid+j]=(i-(filterhi>>1))*imagewid+j-3;
    }
  }
  begin = &imagestart[(filterhi>>1)*imagewid+3];
  end = &imageend[-((filterhi>>1)*imagewid+3)];
  for (imageptr=begin;imageptr<end;imageptr+=3){
    temp = 0;
    tempred=0;
    tempgreen=0;
    tempblue=0;
    for (i=0;i<filtersize;i++){
        if (imageptr[imoffset[i]]>0){
	  temp++;
	  switch (i%3){
	  case 0:tempred+=imageptr[imoffset[i]];break;
	  case 1:tempgreen+=imageptr[imoffset[i]];break;
	  default:tempblue+=imageptr[imoffset[i]];break;
	  }
	}
    }
    if (temp>filtersize>>1){
      *(outptr++)=3*tempred/temp;
      *(outptr++)=3*tempgreen/temp;
      *(outptr++)=3*tempblue/temp;
    }
    else{
      *(outptr++)=0;
      *(outptr++)=0;
      *(outptr++)=0;
    }
  }
  
  for (i=0;i<imagewid+3;i++){
    *(outptr++)=0;
  }
  memcpy(outimage,outim,width*height*3);
}

void bwBinMedian(unsigned char* imagestart, 
	      long height,
	      long width,
	      unsigned char* outimage){
  int i;
  int j;
  unsigned char* imageptr;
  unsigned char* begin;
  unsigned char* end;
  int temp;
  static unsigned char* outim=NULL;
  unsigned char* outptr;
  static int* imoffset=NULL;
  unsigned char* imageend=&imagestart[height*width];
  int filterhi=3;
  int filterwid=3;
  int filtersize=9;
  int imagewid=width;

  if (!outim){
    outim = malloc(width*height);
    imoffset = malloc(sizeof(int)*filterhi*filterwid);
  }

  outptr=outim;

  for (i=0;i<imagewid+1;i++){
    *(outptr++)=0;
  }
  
  for (i=0;i<filterhi;i++){
    for (j=0;j<filterwid;j++){
      imoffset[i*filterwid+j]=(i-(filterhi>>1))*imagewid+j-1;
    }
  }
  begin = &imagestart[(filterhi>>1)*imagewid+1];
  end = &imageend[-((filterhi>>1)*imagewid+1)];
  for (imageptr=begin;imageptr<end;imageptr+=1){
    temp = 0;
    for (i=0;i<filtersize;i++){
        if (imageptr[imoffset[i]]>0){
	  temp++;
	}
    }
    if (temp>filtersize>>1){
      *(outptr++)=255;
    }
    else{
      *(outptr++)=0;
    }
  }
  
  for (i=0;i<imagewid+1;i++){
    *(outptr++)=0;
  }
  memcpy(outimage,outim,width*height*3);
}

void runFilter(unsigned char* imagestart, 
	       long height,
	       long width,
	       char* filter, 
	       int filterhi,
	       int filterwid,
	       int filterlength,
	       int imagewid,
	       unsigned char* outimage){
  /* 
 int i;
  int j;
  unsigned char* imageptr;
  unsigned char* begin;
  unsigned char* end;
  int temp;
  unsigned char* outim;
  unsigned char* outptr;
  unsigned char* imoffset;
  unsigned char* imageend=&imagestart[height*width*3];
  int filtersize;

  filtersize = filterhi+filterwid;

  outim = malloc(width*height*3);
  outptr=outim;

  for (i=0;i<imagewid+3;i++){
    *(outptr++)=0;
  }
  
  imoffset = malloc(sizeof(char)*filterhi*filterwid);
  for (i=0;i<filterhi;i++){
    for (j=0;j<filterwid;j++){
      imoffset[i]=(i-(filterhi>>1))*imagewid+j-(filterwid>>1);
    }
  }
  
  
  begin = &imagestart[(filterhi>>1)*imagewid+(filterwid>>1)];
  end = &imageend[-((filterhi>>1)*imagewid+(filterwid>>1))];
  for (imageptr=begin;imageptr<end;imageptr++){
    temp = 0;
    for (i=0;i<filtersize;i++){
      temp +=((int)filter[i])*((int)imageptr[imoffset[i]]);
    }    
    *(outptr++)=temp>>filterlength;
  } 

  for (i=0;i<imagewid+3;i++){
    *(outptr++)=0;
  }
  memcpy(outimage,outim,width*height*3);
  free(outim);
  */
}



void blur(unsigned char* imagestart,
	  long height,
	  long width,
	  unsigned char* outimage){
  //  char filter[27] = {1,0,0,2,0,0,1,0,0,2,0,0,4,0,0,2,0,0,1,0,0,2,0,0,1,0,0};
  //  runFilter(imagestart,height,width,filter,3,9,4,XSIZE*3,outimage);
}

unsigned char* loadRGhist(char *filename){
  int width,height,intensities, x, y;
  unsigned char *image;
  unsigned char *outfile;
  int rbuckets;
  int gbuckets;

  image = readPGM(&height,&width,&intensities,filename);
  outfile = (unsigned char *)malloc(sizeof(unsigned char) 
                                    * (height * width + 4));

  for (rbuckets=1;!((1<<rbuckets)&height);rbuckets++)
    ;
  for (gbuckets=1;!((1<<gbuckets)&width);gbuckets++)
    ;
  
  outfile[0]=2;
  outfile[1]=(unsigned char)rbuckets;
  outfile[2]=(unsigned char)gbuckets;
  outfile[3]=0;

  for (x=0,y=4;x<height*width;x++,y++) {
    outfile[y]=image[x];
  }
  return outfile;
}

void saveRGhist(char *filename,
                unsigned char* infile){
  int width,height;
  unsigned char* image;
  
  image = &infile[4];

  height=1<<infile[1];
  width=1<<infile[2];
  
  writePGM(image,height,width,255,filename);

}

void makeChromeImage(unsigned char *imagestart,
		    long height,
		    long width,
		    unsigned char *outdata){
  unsigned char *imageptr;
  unsigned char *outptr;
  unsigned char *imageend=&imagestart[height*width*3];
   
  long r;
  long g;
  long i;
  long tempred;
  long tempgreen;
  long tempblue;

  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
    tempred  = (long)*(imageptr+2);
    tempgreen = (long)*(imageptr+1);
    tempblue   = (long)*imageptr;
    
    if ((tempred+tempgreen+tempblue)==0){
      r=85;
      g=85;
      i=0;
    }else{
      r = (tempred<<8)/(tempred+tempgreen+tempblue);
      g = (tempgreen<<8)/(tempred+tempgreen+tempblue);
      i = (tempred+tempgreen+tempblue)/3;
    }
    
    *outptr = i;
    *(outptr+1) = g;
    *(outptr+2) = r;
    outptr+=3;
  }
}
void makeRGIHist(unsigned char *imagestart,
		 long height,
		 long width,
		 long rbuckets,
		 long gbuckets,
		 long ibuckets,
		 unsigned char *histout){
  unsigned char *imageptr;
  unsigned char *imageend=&imagestart[height*width*3];
  static long *raw=NULL;
  long k;
  long j;
  long max=0;
  
  long i;
  long r;
  long g;
  
  *histout++=3;
  *histout++=rbuckets;
  *histout++=gbuckets;
  *histout++=ibuckets;
  
  if (!raw){
    raw = malloc(sizeof(long)*1<<18);
    // r+g+i must be < 18, should never need more than 64 bins per dimention
  }

  //  printf("%x-%x\n",imagestart,imageend);
  
  for (j=0;j<(1<<(rbuckets+gbuckets+ibuckets));j++){
    raw[j]=0;
  }

  for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
    g = (long)*(imageptr+1);
    r = (long)*(imageptr+2);
    i = (long)*imageptr;
    raw[(g>>(8-gbuckets))+((r>>(8-rbuckets))<<gbuckets)+((i>>(8-ibuckets))<<(gbuckets+rbuckets))]++;
  }
  for (k=0;k<(1<<(rbuckets+gbuckets+ibuckets));k++){
    if (raw[k]>max)
      max = raw[k];
  }
  for (k=0;k<(1<<(rbuckets+gbuckets+ibuckets));k++){
    histout[k]=(raw[k]<<8)/max;
  }
}

void makeRGHist(unsigned char *imagestart,
	      long height,
	      long width,
	      long rbuckets,
	      long gbuckets,
	      unsigned char *histout){
  unsigned char *imageptr;
  unsigned char *imageend=&imagestart[height*width*3];
  static long *raw=NULL;
  long i;
  long j;
  long max=0;
  

  long r;
  long g;
  
  *histout++=2;
  *histout++=rbuckets;
  *histout++=gbuckets;
  *histout++=0;
  
  if (!raw){
    raw = malloc(sizeof(long)*1<<12);
    // r+g+i must be < 18, should never need more than 64 bins per dimention
  }
  
  //  printf("%x-%x\n",imagestart,imageend);
  
  for (j=0;j<(1<<(rbuckets+gbuckets));j++){
    raw[j]=0;
  }

  for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
    g = (long)*(imageptr+1);
    r = (long)*(imageptr+2);
    raw[(g>>(8-gbuckets))+((r>>(8-rbuckets))<<gbuckets)]++;
  }
  for (i=0;i<(1<<(rbuckets+gbuckets));i++){
    if (raw[i]>max)
      max = raw[i];
  }
  for (i=0;i<(1<<(rbuckets+gbuckets));i++){
    histout[i]=(raw[i]<<8)/max;
  }
}

void mergePixel(unsigned char* imageptr, int mergeSpeed, unsigned char* outptr){
  *outptr=(unsigned char)(((((((long)1<<mergeSpeed)-1)*
			      ((long)*outptr))<<(8-mergeSpeed))+
			    (((long)*imageptr)<<(8-mergeSpeed)))>>8);
}

void mergeHistPixel(unsigned char* imageptr, int mergeSpeed, unsigned char* outptr){
  long oldhistvalue=0;
  long newhistvalue=0;
  
  oldhistvalue+=*outptr;
  oldhistvalue+=(*(outptr+1))<<8;
  oldhistvalue+=(*(outptr+2))<<16;

  newhistvalue+=*imageptr;
  newhistvalue+=(*(imageptr+1))<<8;
  newhistvalue+=(*(imageptr+2))<<16;
  
  oldhistvalue=(long)(((((((long)1<<mergeSpeed)-1)*
			 oldhistvalue)<<(8-mergeSpeed))+
		       (newhistvalue<<(8-mergeSpeed)))>>8);
  *outptr=oldhistvalue&0xFF;
  *(outptr+1)=(oldhistvalue>>8)&0xFF;
  *(outptr+2)=(oldhistvalue>>16)&0xFF;
}

void updateBackHist(unsigned char** imageq,
		    long height,
		    long width,
		    unsigned char** backhist){

  static unsigned char *outdata=NULL;
  static unsigned char *histout=NULL;
  unsigned char *histptr;
  unsigned char *backptr;
  unsigned char *backend;
  
  if (((int)imageq[QSIZE])==QSIZE){
    if (!outdata){
      outdata = malloc(width*height*3);
      histout = malloc(3<<16);
    }
    makeChromeImage(imageq[QSIZE-1],height,width,outdata);
    makeRGHist(outdata,height,width,8,8,histout);

    if (*backhist==NULL){
      *backhist = histout;
    }
    else{
      backend = &((*backhist)[3<<16]);
      histptr = histout;
      for (backptr=*backhist;backptr<backend;backptr+=3){
	mergeHistPixel(histptr,MERGESPEED,backptr);
	histptr+=3;
      }
    }
  }
}

  

void getCentroid(unsigned char* image, 
		 long height,
		 long width,
		 long* xpos, 
		 long* ypos,
		 long* size){
  unsigned char* imageptr;
  unsigned char* imageend;
  long x,y,s,n;
  
  unsigned char temp;
  
  x=y=s=n=0;
  imageend = &image[3*XSIZE*YSIZE];
  for(imageptr=image;imageptr<imageend;imageptr+=3){
    /*
      temp  = (((*imageptr>*(imageptr+1))?
      ((*imageptr>*(imageptr+2))?*imageptr:*(imageptr+2)):
      ((*(imageptr+1)>*(imageptr+2))?*(imageptr+1):*(imageptr+2)))-
      ((*imageptr<*(imageptr+1))?
      ((*imageptr<*(imageptr+2))?*imageptr:*(imageptr+2)):
      ((*(imageptr+1)<*(imageptr+2))?*(imageptr+1):*(imageptr+2))));
      
      threshholdOneLow(&temp,64,&temp);
	      
    */
    temp  = *imageptr+*(imageptr+1)+*(imageptr+2);
	     
    x+=temp*(((imageptr-image)/3)%width);
    y+=temp*(((imageptr-image)/3)/width);
    s+=temp;
    if (temp)
      n++;
  }
  if (s>0){
    *xpos = x/s;
    *ypos = y/s;
    *size = n;
  }
  else{
    *xpos = 0;
    *ypos = 0;
    *size = n;
  }  
}

void getBWCentroid(unsigned char* image, 
		   long height,
		   long width,
		   long* xpos, 
		   long* ypos,
		   long* size){
  unsigned char* imageptr;
  unsigned char* imageend;
  long x,y,s,n;
  
  unsigned char temp;
  
  x=y=s=n=0;
  imageend = &image[XSIZE*YSIZE];
  for(imageptr=image;imageptr<imageend;imageptr++){
    /*
      temp  = (((*imageptr>*(imageptr+1))?
      ((*imageptr>*(imageptr+2))?*imageptr:*(imageptr+2)):
      ((*(imageptr+1)>*(imageptr+2))?*(imageptr+1):*(imageptr+2)))-
      ((*imageptr<*(imageptr+1))?
      ((*imageptr<*(imageptr+2))?*imageptr:*(imageptr+2)):
      ((*(imageptr+1)<*(imageptr+2))?*(imageptr+1):*(imageptr+2))));
      
      threshholdOneLow(&temp,64,&temp);
      
    */
    temp  = *imageptr;
	     
    x+=temp*((imageptr-image)%width);
    y+=temp*((imageptr-image)/width);
    s+=temp;
    if (temp)
      n++;
  }
  if (s>0){
    *xpos = x/s;
    *ypos = y/s;
    *size = n;
  }
  else{
    *xpos = 0;
    *ypos = 0;
    *size = n;
  }  
}


void filterImageByBinary(unsigned char* imagestart,
			 long height,
			 long width,
			 unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr++) {
    if (*imageptr==0){
    //   if (*outptr!=0){
      //	printf("to the abyss with thee!\n");
      //   }
      *outptr=0;
    }
    outptr++;
  }
}

void removeHist(unsigned char* imagestart,
		long height,
		long width,
		unsigned char* hist,
		unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char rbuckets;
  unsigned char gbuckets;
  hist++;
  rbuckets=*hist++;
  gbuckets=*hist++;
  hist++;
  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
    if (hist[((*(imageptr+1))>>(8-gbuckets))+(((*(imageptr+2))>>(8-rbuckets))<<gbuckets)]!=0){
      *outptr =0;
      *(outptr+1) = 0;
      *(outptr+2) = 0;
    }
    outptr+=3;
  }
}

void applyHist(unsigned char* imagestart,
	       long height,
	       long width,
	       unsigned char* hist,
	       unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char rbuckets;
  unsigned char gbuckets;
  unsigned char ibuckets;
  unsigned char channels;
  
  channels=*hist++;
  rbuckets=*hist++;
  gbuckets=*hist++;
  ibuckets=*hist++;
  outptr = outdata;
  if (channels==2){
    for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
      *outptr++=hist[((*(imageptr+1))>>(8-gbuckets))+(((*(imageptr+2))>>(8-rbuckets))<<gbuckets)];
    }
  }
  else{
    for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
      *outptr++=hist[((*(imageptr+1))>>(8-gbuckets))
		    +(((*(imageptr+2))>>(8-rbuckets))<<gbuckets)
		    +(((*imageptr)>>(8-ibuckets))<<(gbuckets+rbuckets))];
    }
  }    
}

void filterHist(unsigned char* imagestart,
	       long height,
	       long width,
	       unsigned char* hist,
	       unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char rbuckets;
  unsigned char gbuckets;
  hist++;
  rbuckets=*hist++;
  gbuckets=*hist++;
  hist++;
  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3) {
    if (hist[((*(imageptr+1))>>(8-gbuckets))+(((*(imageptr+2))>>(8-rbuckets))<<gbuckets)]==0){
      *outptr =0;
      *(outptr+1) = 0;
      *(outptr+2) = 0;
    }
    outptr+=3;
  }
}

/*
void applyColor(unsigned char* imagestart,
	       long height,
	       long width,
	       ColorBound_t color,
	       unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3,outptr++) {
    *outptr=0;
    if(*(imageptr+2) < color[0])
      continue;
    if(*(imageptr+2) > color[1])
      continue;
    if(*(imageptr+1) < color[2])
      continue;
    if(*(imageptr+1) > color[3])
      continue;
    if(*(imageptr) < color[4])
      continue;
    if(*(imageptr) > color[5])
      continue;
    
    *outptr=255;
  }
}

void filterColor(unsigned char* imagestart,
		 long height,
		 long width,
		 Color_t color,
		 char rrange,
		 char grange,
		 char irange,
		 unsigned char* outdata){
  
  unsigned char* imageend=&imagestart[height*width*3];
  unsigned char* imageptr;
  unsigned char* outptr;
  outptr = outdata;
  for(imageptr=imagestart;imageptr<imageend;imageptr+=3,outptr++) {
    if (((*(imageptr)>>irange)==color[2])&&
	((*(imageptr+1)>>grange)==color[1])&&
	((*(imageptr+2)>>rrange)==color[0])){
      *outptr=255;
    }
    else{
      *outptr=0;
    }
  }
}

*/
void shrink4(unsigned char* imagestart,
	     long height,
	     long width,
	     int shrinkage,
	     unsigned char* outdata){
  unsigned char* imageptr;
  unsigned char* imageend;
  static unsigned char* tmpimage=NULL;
  unsigned char* tmpptr;
  
  if (!tmpimage){
    tmpimage=malloc(height*width*3);
  }
  tmpptr=tmpimage;
  imageend=&imagestart[3*(width+1)];
  
  *tmpptr++=(*imagestart>0);
  *tmpptr++=(*(imagestart+1)>0);
  *tmpptr++=(*(imagestart+2)>0);

  for(imageptr=&imagestart[3];imageptr<imageend;imageptr++) {
    if(*imageptr>0){
      *tmpptr=*(tmpptr-3)+1;
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  imageend=&imagestart[3*height*width];
  for(imageptr=&imagestart[3*(width+1)];imageptr<imageend;imageptr++) {
    if(*imageptr>0){
      *tmpptr=(*(tmpptr-3)>*(tmpptr-3*width))?
	*(tmpptr-3*width)+1:
	*(tmpptr-3)+1;
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  for(imageptr=&imagestart[3];imageptr<imageend;imageptr++) {
    if(*imageptr>0){
      *tmpptr=*(tmpptr-3)+1;
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  imageend=&imagestart[3*height*width];
  for(imageptr=&imagestart[3*(width+1)];imageptr<imageend;imageptr++) {
    if(*imageptr>0){
      *tmpptr=(*(tmpptr-3)>*(tmpptr-3*width))?
	*(tmpptr-3*width)+1:
	*(tmpptr-3)+1;
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  for(imageptr=&imageend[-4];imageptr>&imageend[-3*(width+1)];imageptr--) {
    if(*imageptr>0){
      *tmpptr=(*tmpptr>*(tmpptr+3))?*(tmpptr+3)+1:*tmpptr;
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  for(imageptr=&imageend[-3*(width+1)];imageptr>=imagestart;imageptr--) {
    if(*imageptr>0){
      *tmpptr=(*(tmpptr+3)>*(tmpptr+3*width))?
	((*tmpptr>*(tmpptr+3*width))?*(tmpptr+3*width)+1:*tmpptr):
	((*tmpptr>*(tmpptr+3))?*(tmpptr+3)+1:*tmpptr);
      tmpptr++;
    }
    else{
      *tmpptr++=0;
    }
  }
  treshholdAllLow(tmpimage,height,width,2,2,2,outdata);
}

void makeBottomVector(unsigned char* imagestart,
		      long height,
		      long width,
		      unsigned char* vector){
  unsigned char* imageptr;
  unsigned char* lastrow=&imagestart[width*3];
  unsigned char* imageend=&imagestart[height*width*3];
  int count=0;
  int i=0;
  int j=0;

  for (i=0;i<width;i++){
    vector[i]=0;
  }
  
  for (imageptr=imageend-3;(imageptr>=imagestart)&&(count<width);imageptr-=3){
    i--;
    if (i<0){
      i+=width;
      j++;
    }
    if ((vector[i]==0)&&(((*imageptr)+
	(*(imageptr+1))+
	(*(imageptr+2))>0)||(imageptr<lastrow))){
      vector[i]=j;
      count++;
      *imageptr=0xFF;
    }
  }
}


void vectorMedian(unsigned char* vector, 
		  int size, 
		  unsigned char* outvector){
  unsigned char* vectorptr;
  static unsigned char* tempvector=NULL;
  unsigned char* tmpptr;
  unsigned char* vectorend;
  unsigned char backup[5];
  int i,j,k,t;
  
  if (!tempvector){
    tempvector = malloc(size-4);
  }
  tmpptr = tempvector;
  vectorend=&vector[size-2];

  vector = &vector[2];
 
  for (vectorptr = vector; vectorptr<vectorend; vectorptr++){
    memcpy(backup,&vectorptr[-2],5);
    for (k=5;k>2;k--){
      j=0;
      for (i=1;i<k;i++){
	if (backup[i]>backup[j]){
	  j=i;
	}
      }
      i--;
      t=backup[j];
      backup[j]=backup[i];
      backup[i]=t;
    }
    *(tmpptr++)=backup[2];
  }
  memcpy(&outvector[2],tempvector,size-4);
}

void vectorDerivative(unsigned char* vector, 
		      int size, 
		      char* outvector){
  unsigned char* vectorptr;
  static char* tempvector=NULL;
  char* tmpptr;
  unsigned char* vectorend;
  
  if (!tempvector){
    tempvector = malloc(size);
  }
  tmpptr = tempvector;
  vectorend=&vector[size-2];
  
  vector = &vector[1];
 
  for (vectorptr = vector; vectorptr<vectorend; vectorptr++){
    *(tmpptr++)=vectorptr[1]-vectorptr[-1];
  }
  memcpy(&outvector[1],tempvector,size-2);
  free(tempvector);
}

void vectorMin(unsigned char* vector, 
	       int start, 
	       int stop, 
	       long* min){
  *min = vector[start];
  while((++start)<stop){
    if ((*min)>vector[start]){
      *min = vector[start];
    }
  }
}

void getRealDist(int pixels,
		 int height,
		 long* dist){

  pixels=height-pixels;
  if (height>120){
    pixels>>=1;
  }
  *dist = (long)(pow((double)pixels,-1.4416)*501270.0);
}
  
void cleanList(list** l){
  if (*l!=NULL){
    cleanList((list**)(&((*l)->next)));
    free(*l);
  }
}

void dispGroundObjects(char* deltavector,
		       int start,
		       int end,
		       list** bounds){
  int object = 2;
  list *current = NULL;
  
  while (++start<end){
    if ((object!=0)&&(deltavector[start]>10)){
      if (object==2){
	current = *bounds = malloc(sizeof(list));
      }
      else{
	current->next = malloc(sizeof(list));
	current = current->next;
      }
      current->item=start;
      current->next=NULL;
      object=0;
      //      printf("end : %d\n",start);
    }
    if ((object!=1)&&(deltavector[start]<-10)){
      if (object==2){
	current = *bounds = malloc(sizeof(list));
      }
      else{
	current->next = malloc(sizeof(list));
	current = current->next;
      }
      current->item=start;
      current->next=NULL;
      object=1;
      //      printf("start : %d\n",start);
    }
  }
}


void makeSaturationImage(unsigned char* image,
			 int height, 
			 int width,
			 unsigned char* outimage){
  
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char* imageend;
  
  unsigned char min,sat,intense,val; 
  
  imageend = &image[height*width*3];
  outptr = outimage;
  for (imageptr = image;imageptr<imageend;imageptr+=3){
    min = (*imageptr<*(imageptr+1))?
      ((*imageptr>*(imageptr+2))?*(imageptr+2):*imageptr):
      ((*(imageptr+1)>*(imageptr+2))?*(imageptr+2):*(imageptr+1));
    if ((*imageptr+*(imageptr+1)+*(imageptr+2))==0){
      sat = 0;
    }
    else{
      sat = 256 - (min*(3<<8)/(*imageptr+*(imageptr+1)+*(imageptr+2)));
    }
    intense = (*imageptr+*(imageptr+1)+*(imageptr+2))/3;
    val = ((int)sat*(int)intense)>>8;
    *(outptr++)=val;
  }
}

void makeGrayImage(unsigned char* image,
		   int height, 
		   int width,
		   unsigned char* outimage){
  
  unsigned char* imageptr;
  unsigned char* outptr;
  unsigned char* imageend;
  
  unsigned char intense; 
  
  imageend = &image[height*width*3];
  outptr = outimage;
  for (imageptr = image;imageptr<imageend;imageptr+=3){
    intense = (*imageptr+*(imageptr+1)+*(imageptr+2))/3;
    *(outptr++)=intense;
  }
}


		       
unsigned char toLog(long num){
  unsigned char i;
  for (i=1;!((1<<i)&num);i++)
    ;
  return i;
}

long toNum(unsigned char log){
  return 1<<log;
}



