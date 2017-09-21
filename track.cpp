/*
	19/02/2017
	Jiaheng Hu
	Liwei Jiang
	CS363 S17
	track.c
	Project3
	
  The Robot will track the obstacle in front of it
  Hit cntl-c to stop the robot.

 */

#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <Mage.h>
#include <iostream>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <flexiport/flexiport.h>


void sighandler(int signal) {

  // turn everything off and disconnect
  irOff();
  sonarOff();
  disconnectRobot();
  
  fprintf(stderr, "Rotate: Exiting on signal %d\n", signal);

  exit(-1);

}

/* returns the front position that has the shortest distance to the robot*/
int shortest_front_position(int *box, int number){
  int min=5000;
  int p=-1;
  int n=(int)((float)number/3);
  for(int i = n;i<number-n;i++) {
  if(box[i]!=0)
	if(box[i]<min){
		min=box[i];
		p=i;
	}	
  }	
  if (min==5000)
    return -1;
  return p; 
}

/* returns the back sonar which detects the closest object. Return 0 if the shortest distance
 is less than min_dist */
short shortest_back_sonar(long *state, short min_dist) {
  short i, shortest, shortest_sonar;
  
  shortest = 8000;
  shortest_sonar = 1;
  
  i = 23;
  do {
    if (state[i] < shortest) {
      shortest = (short)state[i];
      shortest_sonar = i-16;
    }
    printf("%d: %d ", i-17, state[i]);
    i++;
  } while (i < 28);
  
  if (shortest < min_dist)
    shortest_sonar = 0;
  
  printf("\n");
  return(shortest_sonar);
}

/* returns the shortest front distance detected by the front sonar */
int shortest_front_distance(int *box, int number){
  int min=5000;
  int p=-1;
  int n=(int)((float)number/3);
  for(int i = n;i<number-n;i++) {
  	if(box[i]!=0)
		if(box[i]<min){
			min=box[i];
			p=i;
		}	
	}	
  return min; 
}

/* returns the position that has the shortest distance to the robot*/
int shortest_position(int *box, int number){
  int min=5000;
  int p=-1;
  for(int i=0;i<number;i++) {
  if(box[i]!=0)
	if(box[i]<min){
		min=box[i];
		p=i;
	}	
  }	
  if (min==5000)
    return -1;
  return p; 
}

/* returns the shortest distance detected by the front sonar */
int shortest_distance(int *box, int number){
  int min=5000;
  int p=-1;
  for(int i=0;i<number;i++) {
  	if(box[i]!=0)
		if(box[i]<min){
			min=box[i];
			p=i;
		}	
	}	
  return min; 
}

/* turn help function */
int turnHelp(int turn_degree) {	
	if (turn_degree > 200) {
		turn_degree = 200;
	}
	else if (turn_degree < -200) {
		turn_degree = -200;
	}
	vm(0, turn_degree);
	return(0);
}


/* turn function, customize the turning behavior based on initial position and turning degrees */
int turn(int turn_degree, long State[]) {	
	turn_degree = turn_degree % 360;	
	turn_degree = 1000 * turn_degree * M_PI / 180;
	
	long delta_degree = 0;
	long init_degree;
	
	init_degree = State[STATE_T];
		
	printf("Start radian is %d\n", State[STATE_T]);
		
	/* positive: left turn */
	/* orientation value increa while turning left */
	if (turn_degree >= 0) {
	
		if (6283 - init_degree >= turn_degree) {
			while( State[STATE_T] - init_degree <= turn_degree ) {
				delta_degree = turn_degree - (State[STATE_T] - init_degree);
			
				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((delta_degree + 50))) {
					break;
				}
			}	
		}
		else {
			while( State[STATE_T] - init_degree >= 0 ) {		
				delta_degree = turn_degree - (State[STATE_T] - init_degree);
			
				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((delta_degree + 50))) {
					break;
				}
			}	
			
			
			while( State[STATE_T] <= turn_degree - (6283 - init_degree) ) {					
				delta_degree = turn_degree - (6283 - init_degree + State[STATE_T]);
			
				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((delta_degree + 50))) {
					break;
				}
			}	
		}	
	}	
	
	/* negative: right turn */
	/* compass value increases while turning right */
	else if (turn_degree < 0) {
		
		while( State[STATE_T] == 0 ) {
			turnHelp(-10);		
		}	
		
		if (init_degree >= -turn_degree) {
			while( init_degree - State[STATE_T] <= -turn_degree ) {	
				delta_degree = -turn_degree - (init_degree - State[STATE_T]);
				
				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((- delta_degree - 50))) {
					break;
				}
			}
		}
		
		else {
			while( init_degree - State[STATE_T] >= 0 ) {
				delta_degree = -turn_degree - (init_degree - State[STATE_T]);

				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((- delta_degree - 50))) {
					break;
				}
			}
			
			while( 6283 - State[STATE_T] <= -turn_degree - init_degree ) {
				delta_degree = -turn_degree - (init_degree + 6283 - State[STATE_T]);

				/* the magnitude of the velocity decreases when it approaches the goal angle */
				if(turnHelp((- delta_degree - 50))) {
					break;
				}
			}				
		}	
	}
	return(0);
}



int main(int argc, char *argv[])
{
  long State[NUM_STATE]; // State is where Mage dumps all of its info
  int x0, y0;
  int current;
  int target;
  int distance;
  int last = 0;
  int position;
  
  //the state of the robot
  int status=0;
  
  char port[256];
  char portOptions[256];
  unsigned int baud = 19200;
  int speed = 0;
  connectRobot(State, MAGE_MODEL_MAGELLAN, (char *)"/dev/ttyUSB0");   // Connect (serial line) with the robot 

  irOn(); // Start up IRs
  sonarOn(); // Start up Sonars
  //laser sensor
  vm(0,0);
  usleep(500000);
  strcpy(port, "/dev/ttyACM0");
	if( argc > 1 ) {
		strcpy(port, argv[1]);
	}
	printf("Using port %s\n", port);
	sprintf(portOptions, "type=serial,device=%s,timeout=1", port);
  try {
		hokuyoaist::Sensor laser;  // laser object
		hokuyoaist::ScanData data; // data object
		int *box;
		int numReadings;
		int numBoxes;
		int i;

		// open the laser
		laser.open(portOptions);

		// Calibrate the laser time stamp
		laser.calibrate_time();

		// turn the laser on
		laser.set_power(true);

			// catch cntl-c to get a clean shutdown
		  signal( SIGINT, sighandler );
		  //State[STATE_T]=0;	
		  fprintf(stderr,"Theta: %ld\n",State[STATE_T]);

		  printf("Starting motion\n");

		
  			//the loop
 while( 1 ) {
			float error, ratio;
			int velocity;
	
			float turn_degree;
			
	
			// get all of the laser data from a single scan
			laser.get_ranges( data, -1, -1, 1 );

					
			
			numReadings = data.ranges_length();
			numBoxes = numReadings/10;
			box = new int[numBoxes];

			for(i=0;i<numBoxes;i++) {
					box[i] = 10000000;
			}

			for(i=0;i<data.ranges_length();i++) {
				int ix = i/10 >= numBoxes ? numBoxes-1 : i/10;
				if (data[i] >= 30) {
					box[ix] = box[ix] > data[i] ? data[i] : box[ix];
				}
			}    
			printf("Readings:\n");
			for(i=0;i<numBoxes;i++) {
				printf("%5d ", box[i]);
			}
			printf("\n");
			
			//If the robot hasn't locate a follower	
			if(status==1){
						//locate the direction of the obstacle
						position=shortest_position(box,numBoxes);

						distance=shortest_distance(box,numBoxes);
						printf("whole\n");
						
						if(distance<500)
							status=2;
			}

			//If the robot has already locate a follower	
			else {
						//locate the direction of the obstacle
						position=shortest_front_position(box,numBoxes);

						distance=shortest_front_distance(box,numBoxes);
						printf("front\n");
						if(distance>1000)
							status=1;
			}						
	
			printf("position: %d, distance: %d\n",position,distance);
	
	
	
	
			//do nothing if the sonar doesn't detect anything
			if(position==-1)
				continue;
		
			turn_degree = (position-33.5)*2.5;
			printf("turn_degree: %f\n",turn_degree);	
	
			//if the robot don't need to turn, then it moves towards the obstacle
			if(turn_degree>=-15&&turn_degree<=15){
				error = shortest_distance(box,numBoxes)-300;
				
				if (shortest_back_sonar(State,500)==0&&error<0){
					vm(0,0);
					continue;
				}

				if( error > -50 && error < 50 ) { // 5cm error
				  vm(0,0);
				  continue;
				}

				velocity = error * 0.5;

				velocity = velocity - last > 40 ? last + 40 : velocity;
				velocity = velocity - last < -40 ? last - 40 : velocity;
	  
				velocity = velocity > 500 ? 500 : velocity; // 1m/s max velocity
				velocity = velocity < -500 ? -500 : velocity;
				velocity = velocity >= 0 && velocity < 20 ? 20 : velocity;
				velocity = velocity < 0 && velocity > -20 ? -20 : velocity;

				printf("Error %.2f  Velocity %d\n", error, velocity);

				vm( velocity, 0 );

				// update the last velocity sent
				last = velocity;
		
				usleep( 100000 );    	
			}	
	
			//else the robot turn towards the obstacle
			else{
				printf("should turn\n");

				turn(turn_degree, State);
	
				usleep( 100000 );
			}
		  

		  }
  		
  		delete[] box;
  		laser.close();
  
 	}
  catch(hokuyoaist::BaseError &e) {
		std::cerr << "Caught Exception: " << e.what() << '\n';
		return(-1);
  }

  vm(0,0);
  

  
  irOff();
  sonarOff();
  disconnectRobot(); // Disconnect and shut down sensors

  return 0;
}
