
#ifndef SOCKETUTIL_H
#define SOCKETUTIL_H

#include <stdio.h>
#include <stdlib.h>

int makeFilenameSocket( const char *filename );

int closeFilenameSocket( int sock, const char *filename );

int sendFilenameData( int sock, char *filename, int *values, int numValues);

int *readFilenameData( int sock, int *numValues);

int sendFilenameSocket( int sock, const char *filename, char *message, unsigned msg_length );

int readFilenameSocket( int sock, char *message, unsigned MaxLength, char *filename );

int sendFilenameImage( int sock, char *filename, char *src, int rows, int cols, int bytesPerPixel );

char *readFilenameImage( int sock, int *rows, int *cols, int *bytesPerPixel, char *filename );

#endif
