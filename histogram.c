
/* histogram.c 
   Read in a set of images and output a chromaticity histogram
   Generates histograms appropriate for use with particle filter,
   SVM_OP_pftrack
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ppmIO.h"
//#include "track.h"

#define HDIM 64

int main(int argc, char *argv[]) {
    int hist[HDIM*HDIM];
    unsigned char chist[HDIM*HDIM];
    int i, j, max;
    char name[256];
    
    // read image file names from stdin
    if(argc < 3) {
        printf("Usage: %s <histogramFile.pgm> <input1.ppm> <input2.ppm> <more input files...>\n", argv[0]);
	exit(1);
    }

    // init hist to 1
    for(i=0; i<HDIM*HDIM; i++) {
	hist[i] = 1;
    }

    // process each image 
    for(i=2; i<argc; i++) {
	int rows, cols, colors;
	Pixel *img, *px;

	// read in the file
	printf("reading in %s\n", argv[i]);
	img = readPPM(&rows, &cols, &colors, argv[i]);
	if(!img) {
	    printf("skipped\n");
	    continue;
	}

	// plot its pixels in the histogram
	for(j=0, px=img; j<rows*cols; j++, px++) {
	    float cr, cg;
	    int ridx, gidx;

	    //printf("idx %d\n", j);
	    //printf("pixel (%d, %d, %d)\n", px->r, px->g, px->b);

	    // calculate chromaticity
	    if(px->r==0 && px->g==0) {
		//printf("zero\n");
		cr = cg = 0;
	    }
	    else {
		cr = px->r/(float)(px->r + px->g + px->b);
		cg = px->g/(float)(px->r + px->g + px->b);
	    }
	    //printf("cr   %4.2f  ", cr);
	    //printf("cg   %4.2f\n", cg);
	    
	    // plot
	    ridx = (HDIM-1)*cr;
	    gidx = (HDIM-1)*cg;
	    hist[ridx*HDIM+gidx]++;
            //printf("hist coords (%4d, %4d)\n", ridx, gidx);
            //printf("\n");
	}

	free(img);
    }

    // scale histogram to (1, 255) 
    max=0;
    for(i=0; i<HDIM*HDIM; i++)
	if(hist[i]>max) max = hist[i];
    for(i=0; i<HDIM*HDIM; i++) {
	int c = hist[i];
	c = c-1;
	c = c * (254/(float)(max-1));
	printf("scaled value %d\n", c);
	hist[i] = c+1;
    }

    // blur histogram into char image and threshold
    for(i=0; i<HDIM; i++) {
	for(j=0; j<HDIM; j++) {
	    int sum=0, count=2, avg;


	    // self
	    sum += 2*hist[i*HDIM+j];
	    
	    // N
	    if(i>0) {
		sum += hist[(i-1)*HDIM + j];
		count++;
	    }

	    // E
	    if(j<HDIM-1) {
		sum += hist[i*HDIM + j+1];
		count++;
	    }

	    // S
	    if(i<HDIM-1) {
		sum += hist[(i+1)*HDIM + j];
		count++;
	    }

	    // W
	    if(j>0) {
		sum += hist[i*HDIM + j-1];
		count++;
	    }
	    avg = sum/(float)count;

	    chist[i*HDIM+j] = avg;
	} 
    }
    
    // write out histogram
    strncpy(name, argv[1], 255);
    printf("writing histogram to %s\n", name);
    writePGM(chist, HDIM, HDIM, 255, name);

    return 0;
}
