/*
	Bruce A. Maxwell
	January 2016

	Scans data from the laser once, then exits
*/

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include "libfreenect/libfreenect.h"
#include "socketUtil.h"
#include <iostream>

#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <flexiport/flexiport.h>

int gbl_fd = 0;
char gbl_serverSocketFilename[512];
hokuyoaist::Sensor *gbl_laser;


static void sighandler( int signal ) {

	if(gbl_laser->is_open())
		gbl_laser->close();
	
	closeFilenameSocket( gbl_fd, gbl_serverSocketFilename );

	return;
}

int main(int argc, char *argv[]) {
	const int verbose = 1;
	char port[256];
	char portOptions[256];
	unsigned int baud = 19200;
	int speed = 0;
	char filename[512];
	char message[512];
	int nbytes;
	hokuyoaist::Sensor laser;  // laser object
	hokuyoaist::ScanData data; // data object
	int *dptr;
	int numReadings;
	int i;
	int state = 1;
	int iterations = 0;

	gbl_laser = &laser;

	// set up the server socket
	// create the server socket
	printf("Creating socket\n");
	strcpy(gbl_serverSocketFilename, "/tmp/socket-urglaser-server");

	gbl_fd = makeFilenameSocket( gbl_serverSocketFilename );
	printf("Socket opened with descriptor %d\n", gbl_fd);

	// catch signals
  signal ( SIGINT,  sighandler );
  signal ( SIGQUIT, sighandler );
  signal ( SIGTERM, sighandler );
  signal ( SIGHUP,  sighandler );
  signal ( SIGFPE,  sighandler );

	
	// set up the laser port
	strcpy(port, "/dev/ttyACM0");
	if( argc > 1 ) {
		strcpy(port, argv[1]);
	}
	printf("Using port %s\n", port);
	sprintf(portOptions, "type=serial,device=%s,timeout=1", port);

	try {

		// open the laser
		laser.open(portOptions);

		// Calibrate the laser time stamp
		laser.calibrate_time();

		// turn the laser on
		laser.set_power(true);

		// Set the baud rate
		/*
		try {
				laser.set_baud(baud);
		}
		catch(hokuyoaist::BaudrateError &e) {
				std::cerr << "Failed to change baud rate: " << e.what() << '\n';
		}
		catch(hokuyoaist::ResponseError &e) {
				std::cerr << "Failed to change baud rate: " << e.what() << '\n';
		}
		catch(...) 	{
				std::cerr << "Failed to change baud rate\n";
		}
		*/
				
		// Set the motor speed
		try {
				laser.set_motor_speed(speed);
		}
		catch(hokuyoaist::MotorSpeedError &e) {
				std::cerr << "Failed to set motor speed: " << e.what() << '\n';
		}
		catch(hokuyoaist::ResponseError &e) {
				std::cerr << "Failed to set motor speed: " << e.what() << '\n';
		}

		// start the loop waiting for a message
		for(;state;) {
			printf("waiting for a message\n");

			// look for a message
			nbytes = readFilenameSocket( gbl_fd, message, 512, filename );
			if(verbose)
				printf("received message %s from: %s\n", message, filename );

			if(nbytes > 0) {
				switch(message[0]) {

				case 'Q': // terminate
					state = 0;
					printf("Terminating\n");
					break;
				
				case 'I': // send an image

					// get all of the laser data from a single scan
					laser.get_ranges( data, -1, -1, 1 );
					numReadings = data.ranges_length();

					if(dptr == NULL) {
						dptr = (int *)malloc(sizeof(int) * numReadings/2);
					}

					for(i=0;i<numReadings/2;i++) {
						dptr[i] = 0.5*(data[2*i] + data[2*i + 1]);
					}

					nbytes = sendFilenameData( gbl_fd, filename, dptr, numReadings/2 );

					break;

				default:
					break;
				}
			}

			if(verbose) {
				const int numBoxes = 16;
				const int closeThreshold = 10;
				int box[numBoxes];
				int divisor = numReadings/numBoxes;

				laser.get_ranges( data, -1, -1, 1 );
				numReadings = data.ranges_length();

				for(i=0;i<numBoxes;i++) {
					box[i] = 20000;
				}

				for(i=0;i<data.ranges_length();i++) {
					int ix = i/divisor >= numBoxes ? numBoxes-1 : i/divisor;
					box[ix] = data[i] > closeThreshold && box[ix] > data[i] ? data[i] : box[ix];
				}

				printf("Readings:\n");
				for(i=0;i<numBoxes;i++) {
					printf("%4d ", box[i]);
				}
				printf("\n");
			}
			iterations++;

		}
		
		// turn the laser off
		//		laser.set_power(false);
		
		// close the connection to the laser
		printf("Closing laser\n");
		laser.close();
		printf("Laser closed\n");
		
	}
	catch(hokuyoaist::BaseError &e) {
		std::cerr << "Caught Exception: " << e.what() << '\n';
		return(-1);
	}

	printf("Closing socket %s\n", gbl_serverSocketFilename );
	closeFilenameSocket( gbl_fd, gbl_serverSocketFilename );
	printf("Socket terminated\n");
	
	return(0);
}
