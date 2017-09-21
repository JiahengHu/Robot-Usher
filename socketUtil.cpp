// based on the GNU C Library example

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "socketUtil.h"

int makeFilenameSocket( const char *filename ) {
	struct sockaddr_un name;
	int sock;
	size_t size;

	// create a socket
	// PF_UNIX -> programs share the same filesystem
	// SOCK_DGRAM -> sending data in chunks, not as a stream
	sock = socket( PF_UNIX, SOCK_DGRAM, 0 );
	if( sock < 0 ) {
		printf("makeFilenameSocket: unable to open socket\n");
		perror("socket");
		exit( -1 );
	}

	// bind a name to the socket
	memset( (char *) &name, 0, sizeof(name) );
	name.sun_family = AF_UNIX;
	strcpy( name.sun_path, filename );

	size = strlen( name.sun_path) + sizeof( name.sun_family ) + 1;
	//	name.sun_len = size;
	
	if( bind(sock, (struct sockaddr *) &name, size) < 0 ) {
		printf("makeFilenameSocket: unable to bind socket\n");
		perror("bind");
		exit( -2 );
	}

	return(sock);
}

int closeFilenameSocket( int sock, const char *filename ) {

	// remove the file
	unlink( filename );

	// close the socket
	close( sock );
	
	return(0);
}

int sendFilenameSocket( int sock, const char *filename, char *message, unsigned msg_length ) {
	struct sockaddr_un name;
	int nbytes;
	size_t size;

	name.sun_family = AF_UNIX;
	strcpy( name.sun_path, filename );
	size = strlen( name.sun_path) + sizeof( name.sun_family ) + 1;
	//	name.sun_len = size;

	nbytes = sendto( sock, message, msg_length, 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror( "sendFilenameSocket: sendto failure" );
		exit( -1 );
	}

	return(nbytes);
}

int readFilenameSocket( int sock, char *message, unsigned MaxLength, char *filename ) {
	int nbytes;
	size_t namesize;
	struct sockaddr_un name;
	int end;

	// sets the namesize
	namesize = sizeof( name );

	nbytes = recvfrom( sock, message, MaxLength, 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readFilenameSocket: recvfrom failure");
		exit( -1 );
	}

	if( filename != NULL ) {
		end = MaxLength - sizeof(name.sun_family) - 1;
		strncpy( filename, name.sun_path, end );
		filename[end] = '\0';
	}

	return(nbytes);
}

int sendFilenameData( int sock, char *filename, int *values, int numValues) {
	int i;
	struct sockaddr_un name;
	size_t size;
	int nbytes;

	name.sun_family = AF_UNIX;
	strcpy( name.sun_path, filename );
	size = strlen( name.sun_path ) + sizeof( name.sun_family ) + 1;
	//	name.sun_len = size;

	if(numValues > 512) {
		printf("numValues is too large to send, cutting to 512.\n");
		numValues = 512;
	}
	printf("numValues to send: %d\n", numValues);

	// send the number of data points
	nbytes = sendto( sock, &numValues, sizeof(int), 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror("sendFilenameData: sendto failure" );
		exit(-1);
	}

	// send the array of values
	nbytes = sendto( sock, values, sizeof(int)*numValues, 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror( "sendFilenameData: sendto failure (2)");
		exit(-1);
	}

	return( numValues );
}

int *readFilenameData( int sock, int *numValues) {
	int nbytes;
	struct sockaddr_un name;
	size_t namesize;
	const int MaxLength = 128;
	char message[MaxLength];
	int *iptr = (int *)message;
	int *src = NULL;

	namesize = sizeof(name);

	nbytes = recvfrom( sock, message, MaxLength, 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readFilenameData: recvfrom failure");
		exit(-1);
	}
	*numValues = *iptr;

	src = (int *)malloc(sizeof(int) * (*numValues) );
	nbytes = recvfrom( sock, src, *numValues * sizeof(int), 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readfilenameData: recvfrom failure (2)");
		exit(-1);
	}

	return(src);
}


int sendFilenameImage( int sock, char *filename, char *src, int rows, int cols, int bytesPerPixel ) {
	int i;
	char *cptr = NULL;
	struct sockaddr_un name;
	int nbytes;
	size_t size;
	const int MaxLength = 128;
	char message[128];

	name.sun_family = AF_UNIX;
	strcpy( name.sun_path, filename );
	size = strlen( name.sun_path) + sizeof( name.sun_family ) + 1;
	//	name.sun_len = size;
	
	// send the rows
	nbytes = sendto( sock, &rows, sizeof(int), 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror( "sendFilenameSocket: sendto failure" );
		exit( -1 );
	}

	// send the columns
	nbytes = sendto( sock, &cols, sizeof(int), 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror( "sendFilenameSocket: sendto failure" );
		exit( -1 );
	}

	// send the bytes per pixel
	nbytes = sendto( sock, &bytesPerPixel, sizeof(int), 0, (struct sockaddr *) &name, size );
	if( nbytes < 0 ) {
		perror( "sendFilenameSocket: sendto failure" );
		exit( -1 );
	}

	// loop over the rows and send each one individually
	for(i=0;i<rows;i++) {
		cptr = &(src[i*cols*bytesPerPixel]);

		nbytes = sendto( sock, cptr, bytesPerPixel * cols, 0, (struct sockaddr *) &name, size );
		if( nbytes < 0 ) {
			perror( "sendFilenameSocket: sendto failure" );
			exit( -1 );
		}

		// wait for handshake
		nbytes = recvfrom(sock, message, MaxLength, 0, NULL, NULL );
		if( nbytes < 0 ) {
			perror("readFilenameImage: recvfrom handshake failure");
			exit( -1 );
		}
	}

	return( rows*cols*bytesPerPixel );
}


char *readFilenameImage( int sock, int *rows, int *cols, int *bytesPerPixel, char *filename ) {
	int nbytes;
	size_t namesize;
	struct sockaddr_un name;
	int end;
	const int MaxLength = 128;
	char message[MaxLength];
	int *iptr = (int *)message;
	char *src = NULL;
	char *cptr = NULL;
	int i;

	// sets the namesize
	namesize = sizeof( name );

	// get the # of rows
	nbytes = recvfrom( sock, message, MaxLength, 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readFilenameSocket: recvfrom failure");
		exit( -1 );
	}
	*rows = *iptr;

	end = MaxLength - sizeof(name.sun_family) - 1;
	if( filename != NULL ) {
		strncpy( filename, name.sun_path, end );
		filename[end] = '\0';
	}

	// get the # of cols
	nbytes = recvfrom( sock, message, MaxLength, 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readFilenameSocket: recvfrom failure");
		exit( -1 );
	}
	*cols = *iptr;

	// get the # of bytes per pixel
	nbytes = recvfrom( sock, message, MaxLength, 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
	if( nbytes < 0 ) {
		perror("readFilenameSocket: recvfrom failure");
		exit( -1 );
	}
	*bytesPerPixel = *iptr;

	if( *rows < 1 || *rows > 10000 || *cols < 1 || *cols > 10000) {
		printf("readFilenameImage: rows or cols is too large %d %d\n", *rows, *cols);
		exit(-1);
	}

	// allocate the image
	src = (char *)malloc((*bytesPerPixel) * (*rows) * (*cols));

	// get the rows of the image
	for(i=0;i<*rows;i++) {

		cptr = &(src[i* (*cols) * (*bytesPerPixel)]);
		nbytes = recvfrom( sock, cptr, (*bytesPerPixel) * (*cols), 0, (struct sockaddr *)&name, (socklen_t *)&namesize );
		if( nbytes < 0 ) {
			perror("readFilenameSocket: recvfrom failure");
			exit( -1 );
		}

		nbytes = sendto( sock, &i, sizeof(int), 0, (struct sockaddr *) &name, (socklen_t)namesize );
		if( nbytes < 0 ) {
			perror( "sendFilenameSocket: sendto failure" );
			exit( -1 );
		}

	}

	// done

	return(src);
}
