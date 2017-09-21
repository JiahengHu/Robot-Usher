/*
    03/27/2017
    CS363 S17
    Project4
    Liwei Jiang
    Jiaheng Hu
    
    robotStructure.c
*/

#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "robotStructure.h"
#include "URGclient.h"

/* turn everything off and disconnect */
void sighandler(int signal) {
  irOff();
  sonarOff();
  disconnectRobot();
  
  fprintf(stderr, "Turn: Exiting on signal %d\n", signal);
  exit(-1);
}


/* group laser signals to 24 groups of virtual data across 240 degrees */
int *group_data(int *datain) {
	int *datain;
	int *data = NULL;
	int sum;
	int i, j;

	data = (int *)malloc(sizeof(int) * 24);	
	for (i=0; i<24; i++ ) {
		sum = 0;
		for (j=0; j<14; j++) {
			sum += datain[2 + i*14 + j];
		}
		sum /= 14;
		data[i] = sum;	
	}
	return(data);
}

/* returns the angle that is the closest direction */ 
int get_best_dir_laser(robotState *state) {
  	int i;
  	int min_value = 3000; // min. sonar reading 
  	int best_dir; // to remember the direction
  	
  	int min_value1 = 500; // min. sonar reading
  	int best_dir1; // to remember the direction
  	
  	int angle;

  	best_dir = 12;
  
  	for (i=0; i<24; i++) {
  		if (state->laser_data_virtual[i] < min_value) {
  			min_value = state->laser_data_virtual[i];
  			best_dir = i;
  		}
  	}
  	
  	for (i = STATE_SONAR_5; i <= STATE_SONAR_11; i++) { 
  		printf(" min: %d ", State[i]);
    	if (State[i] <= min_value1) {
   		 	min_value1 = state->state[i];
    		best_dir1 = i - STATE_SONAR_0;
    		printf(" min: %d ", min_value1);
		}
	}
	
	if (best_dir1 < 8) {
  		best_dir1 = best_dir1 - 16;
  	}
  			
	if (min_value1 < 500) {
		if (min_value1 < min_value) {			
			angle = best_dir1 * 225;
			return angle;
		}	
	}
  		
	best_dir = best_dir - 12;
  	angle = best_dir * 100;
  	return angle;
}


/* check the direction that is short to the barrier using laser */
int check_shortest_laser(int *lasers, int num, robotState *state) {
	int i, shortest_laser;
	int laser_index = 0;
	int shortest;

	shortest = 3000;
	shortest_laser = 0;
    
	for (i = 0; i < num; i++) {
		if (state->laser_data_virtual[sonar_index] < shortest) {
      		shortest = state->laser_data_virtual[sonar_index];
      		shortest_laser = lasers[i];
      		printf("%d\n", shortest);
    	}
	}
	printf("Shortest laser: %d \n", shortest_laser);
	printf("Shortest distance: %d \n", shortest);	
	return shortest;
}


/* rotate the robot by a certain angle */
int *rotate(robotState *state) {
	int turn_degree = robot->current_goal_angles % 360;	
	turn_degree = 1000 * turn_degree * M_PI / 180;
	printf("Start radian is %d\n", robot->init_orientation);
	int velocity;
	velocity[0] = 0;
	velocity[1] = 0;
		
	/* positive: left turn */
	/* orientation value increase while turning left */
	if (state->current_goal_angles >= 0) {
		if (6283 - state->init_orientation >= state->current_goal_angles) {
			if( state->state[STATE_T] - state->init_orientation <= state->current_goal_angles ) {
				velocity[1] = state->current_goal_angles - (state->state[STATE_T] - state->init_orientation) + 50;
			}
			else {
				state->current_state = 10;
			}
		}
		else {
			if( state->state[STATE_T] - state->init_orientation >= 0 ) {		
				velocity[1] = state->current_goal_angles - (state->state[STATE_T] - state->init_orientation) + 50;
			}	
			else if ( state->state[STATE_T] <= state->current_goal_angles - (6283 - state->init_orientation) ) {					
				velocity[1] = state->current_goal_angless - (6283 - state->init_orientation + state->state[STATE_T]) + 50;
			}	
			else {
				state->current_state = 10;
			}
		}	
	}	
	
	/* negative: right turn */
	/* compass value increases while turning right */
	else if (state->current_goal_angles < 0) {
		
		if ( state->state[STATE_T] == 0 ) {
			velocity[1] = -10;
		}	

		else {
			if (state->init_orientation >= -state->current_goal_angles) {
				if( state->init_orientation - State[STATE_T] <= -state->current_goal_angles ) {	
					velocity[1] =  state->current_goal_angles + (state->init_orientation - state->state[STATE_T]) - 50;
				}
				else {
					state->current_state = 10;
				}	
			}
			else {
				if( state->init_orientation - state->state[STATE_T] >= 0 ) {
					velocity[1] = state->current_goal_angles + (state->init_orientation - state->state[STATE_T]) - 50;
				}
				else if ( 6283 - state->state[STATE_T] <= -state->current_goal_angles - state->init_orientation ) {
					velocity[1] = - state->current_goal_angles + (state->init_orientation + 6283 - state->state[STATE_T]) - 50;					
				}
				else {
					state->current_state = 10;
				}				
			}	
		}	
	}
	return(velocity);
}


/* check wether the next state of way point following */
void WaypointRotateCheck(robotState *state) {
	if (state->current_state == 2) {
		if (state->index_waypoints < state->number_waypoints) {
			state->current_goal_angle = state->waypoints_orientation[state->index_waypoints + 1];
			state->init_orientation = state->state[STATE_T];
		}
		else {
			state->current_state = 10;
			state->waypoints_if_dis_ori = 0;
		}
	}
	else {
	state->current_state = 10;
	}
}






/* ------------------------------------------------------------------------------------------------------------------------------------------ */

/* rotate the robot by a certain angle */
int *rotate1(robotState *state) {
	int turn_degree = robot->current_goal_angles % 360;	
	turn_degree = 1000 * turn_degree * M_PI / 180;
	printf("Start radian is %d\n", robot->init_orientation);
	int velocity;
	velocity[0] = 0;
	velocity[1] = 0;
		
	/* positive: left turn */
	/* orientation value increase while turning left */
	if (state->current_goal_angles >= 0) {
		if (6283 - state->init_orientation >= state->current_goal_angles) {
			if( state->state[STATE_T] - state->init_orientation <= state->current_goal_angles ) {
				velocity[1] = state->current_goal_angles - (state->state[STATE_T] - state->init_orientation) + 50;
			}
			else {
				WaypointRotateCheck(state):
			}
		}
		else {
			if( state->state[STATE_T] - state->init_orientation >= 0 ) {		
				velocity[1] = state->current_goal_angles - (state->state[STATE_T] - state->init_orientation) + 50;
			}	
			else if ( state->state[STATE_T] <= state->current_goal_angles - (6283 - state->init_orientation) ) {					
				velocity[1] = state->current_goal_angless - (6283 - state->init_orientation + state->state[STATE_T]) + 50;
			}	
			else {
				state->current_state = 10;
			}
		}	
	}	
	
	/* negative: right turn */
	/* compass value increases while turning right */
	else if (state->current_goal_angles < 0) {
		
		if ( state->state[STATE_T] == 0 ) {
			velocity[1] = -10;
		}	
s
		else {
			if (state->init_orientation >= -state->current_goal_angles) {
				if( state->init_orientation - State[STATE_T] <= -state->current_goal_angles ) {	
					velocity[1] =  state->current_goal_angles + (state->init_orientation - state->state[STATE_T]) - 50;
				}
				else {
					state->current_state = 10;
				}	
			}
			else {
				if( state->init_orientation - state->state[STATE_T] >= 0 ) {
					velocity[1] = state->current_goal_angles + (state->init_orientation - state->state[STATE_T]) - 50;
				}
				else if ( 6283 - state->state[STATE_T] <= -state->current_goal_angles - state->init_orientation ) {
					velocity[1] = - state->current_goal_angles + (state->init_orientation + 6283 - state->state[STATE_T]) - 50;					
				}
				else {
					state->current_state = 10;
				}				
			}	
		}	
	}
	return(velocity);
}







































/* move the robot by a certain distance */
int *move(robotState *state) {
	printf("Initial x location is %d. Initial y location is %d. \n", (int)(robot->init_x_loc), (int)(robot->init_y_loc));	
	int delta_x_loc;
	int delta_y_loc;	
	int velocity[2];

	velocity[0] = 0;
	velocity[1] = 0;

	/* move forward */
	if (state->current_goal_distance >= 0) {
		if(state->distance_traveled < robot->current_goal_distance) {
			delta_x_loc = (int)(robot->init_x_loc) - (int)(robot->state[STATE_X]);
			delta_y_loc = (int)(robot->init_y_loc) - (int)(robot->state[STATE_Y]);		
			state->distance_traveled = (int)sqrt(delta_x_loc*delta_x_loc + delta_y_loc*delta_y_loc);
			printf("Distance travelled: %d\n", delta_distance);
			velocity[0] = robot->current_goal_distance - state->distance_traveled;
		}
		else {
			if (state->current_state == 2) {
				if (state->index_waypoints < state->number_waypoints) {
					state->index_waypoints = state->index_waypoints + 1;
					state->current_goal_distance = state->waypoints_distance[state->index_waypoints];
					state->distance_traveled = 0;
				}
				else {
					state->current_state = 10;
					state->waypoints_if_dis_ori = 0;
				}
			}
			else {
				state->current_state = 10;
			}
		}
	}
	
	/* move backward */
	else if (state->current_goal_distance < 0) {
		if(state->distance_traveled < -robot->current_goal_distance) {		
			delta_x_loc = (int)(robot->init_x_loc) - (int)(robot->state[STATE_X]);
			delta_y_loc = (int)(robot->init_y_loc) - (int)(robot->state[STATE_Y]);		
			state->distance_traveled = (int)sqrt(delta_x_loc*delta_x_loc + delta_y_loc*delta_y_loc);
			printf("Distance travelled: %d\n", delta_distance);
			velocity[0] = robot->current_goal_distance + state->distance_traveled;
		}
		else {
			if (state->current_state == 2) {
				if (state->index_waypoints < state->number_waypoints) {
					state->index_waypoints = state->index_waypoints + 1;
					state->current_goal_distance = state->waypoints_distance[state->index_waypoints];
					state->distance_traveled = 0;
				}
				else {
					state->current_state = 10;
					state->waypoints_if_dis_ori = 0;
				}
			}
			else {
				state->current_state = 10;
			}
		}
	}	
	return(velocity);
}


/* move the robot to a position (x, y) and an orientation theta */
void locate(robotState *state) {
	

	if (state->waypoints_if_dis_ori == 0) {
		turn(robotState *state);	
	}	


	else if (state->waypoints_if_dis_ori == 1) {
		move(robotState *state);
	}


	// compass = (int)State[STATE_T];
	
	// if ( compass / 10 >= r_loc ) {
	// 	if ( compass / 10 - r_loc < 180 ) {
	// 		turn( (compass / 10 - r_loc), State );
	// 	}
	// 	else if ( compass / 10 - r_loc >= 180 ) {
	// 		turn( (compass / 10 - r_loc) - 360, State );
	// 	}			
	// }
	
	// else if ( compass / 10 < r_loc ) {	
	// 	if ( compass / 10 - r_loc > -180 ) {
	// 		turn( (compass / 10 - r_loc), State );
	// 	}
	// 	else if ( compass / 10 - r_loc <= -180 ) {
	// 		turn( (compass / 10 - r_loc) + 360, State );
	// 	}
	// }	
	return(0);

}

/* allow the robot walk along a wall */
void wall() {
	
}


/* allow the robot wander without bumping into any obstacle */
void wander(robotState *state) {
	int front;
	int left;
	int right;
	int front_laser[3] = {11, 12, 13};
	int right_laser[6] = {3, 4, 5, 6, 7, 8};
	int left_laser[6] = {16, 17, 18, 19, 20, 21};
	int right_laser_small[3] = {20, 21, 22};
	int velocity[2];
	velocity[0] = 0;
	velocity[1] = 0;
	
	front = check_shortest_laser(front_laser, 3, state);
	left = check_shortest_laser(left_laser, 6, state);
	right = check_shortest_laser(right_laser, 6, state);
	
	// if far from front, left, and right
	if (front >= 2000 && left >= 1500 && right >= 1500) {
		velocity[0] = front + 10;
		velocity[1] = 0;
		printf("--- too far from front ---\n");
	}
	
	else {
		// if front is the most hindered direction
		if (front <= left && front <= right) {
			if (front > 1000) {
				velocity[0] = front + 10;
				velocity[1] = 0;
				printf("--- far from front ---\n");
			}
			// after moving close enough to the front wall, turn left while monitoring the right side
			else if (front <= 1000) {
				if (front < 1000) {
					velocity[1] = (get_best_dir_laser(state));
				}
				else if (front < 1600 && front >= 1000) {
					velocity[0] = front + 10;
					velocity[1] = (get_best_dir_laser(state));
				}
				else if (front >= 1600) {	
					velocity[0] = front + 5;
				}	
			}
		}
		else if (left < front && left < right) {
			velocity[0] = front + 10;
			velocity[1] = -40;
			printf("--- close to left ---\n");
		}
		else if (right < front && right < left) {
			velocity[0] = front + 10;
			velocity[1] = 40;
			printf("--- close to right ---\n");
		}	
	}
	return(velocity);	
}

/* allow the robot traverse a maze */
void maze() {
	
}

/* allow the robot track an obstacle in front of it */
void track() {
	
}




/* ---------------------------------------- Functions regarding Ransac ---------------------------------------- */
//return the x value of a certain measure ( in respect to the robot)
float getX(int index, int distance){
	float degree=(index-(float)67/8)*3.33/180*pi;
	float x=cos(degree);
	return x*distance;
}	

//return the y value of a certain measure ( in respect to the robot)
float getY(int index, int distance){
	float degree=(index-(float)67/8)*3.33/180*pi;
	float y=sin(degree);
	return y*distance;
}	

//calculate the abs of a 2-d vector
float vabs(float *v){
	float abs=v[0]*v[0]+v[1]*v[1];
	abs=sqrt(abs);
	return abs;
}	

/*Use the ransac algorithm to locate the wall near the robot
  The returned value will be the slope and intercept of the wall*/ 
float *getWall(int *rawLaser){
		int *box;
		int numReadings;
		int numBoxes;
		int i;
		
		numReadings = rawLaser.size();
		'''Could have bug here, dont know if its correct'''
		
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
		

		// printf("Readings:\n");
// 		for(i=0;i<numBoxes;i++) {
// 			printf("%5d ", box[i]);
// 		}
// 		printf("\n");
		
		//decide interation time
		float inlier=0.5;
		float points=2;
		float success=0.98;
		
		float loop = (log(1-success))/(log(1-pow(inlier,points)));
		
		//sending the x and y position into two arrays for later use
		//printf("k = %f\n",loop);
		srand (time(NULL));
		//printf("size of box = %d\n",numBoxes);
		float xpo[numBoxes];
		float ypo[numBoxes];
		for( int i = 0; i < numBoxes; i++){
			xpo[i]=getX(i,box[i]);
			ypo[i]=getY(i,box[i]);
		}
		
		
		int maxnum=0;
			//The max number of points in a model
			
		float pointset[numBoxes][2];
			//the points that are inliers	
		for( int i = 0 ;i < loop; i++){
			int rand1=rand() % 68;
			while(box[rand1]==10000000)
				rand1=rand() % 68;
				
			int rand2=rand() % 68;
			
			while(rand2 == rand1 || box[rand2]==10000000)
				rand2=rand() % 68;
			
			//printf("rand1 = %d, rand2 = %d\n",rand1, rand2);
			
			float p1[2], p2[2],v[2];	
			p1[0]=getX(rand1,box[rand1]);
			p1[1]=getY(rand1,box[rand1]);
			
			
			p2[0]=getX(rand2,box[rand2]);
			p2[1]=getY(rand2,box[rand2]);
			
			
			for( int k = 0; k < 2 ; k++ ){
				v[k] = p2[k]-p1[k];
			}
			float absv=vabs(v);
			
			for( int k = 0; k < 2 ; k++ ){
				v[k] = v[k]/absv;
			}
// 			printf("v[0]: %f\n",v[0]);
// 			printf("v[1]: %f\n",v[1]);
			
			float tmp[numBoxes][2];
			//float d[numBoxes];
			int numOfValid = 0;
			for( int k = 0; k < numBoxes ; k++ ){
			
				float r1=p2[0]-xpo[k];				
				float r2=p2[1]-ypo[k];//the vector from point i to p2
// 				printf("%d r1: %f\n",k,r1);
// 				printf("%d r2: %f\n",k,r2);
				/*have problem*/				
				float dt=r1*v[0]+r2*v[1];//distance from p1 to the closest point on the line
				dt=-dt;
				//dt have problem
				//it is not the closest point
				
// 				printf("%d dt: %f\n",k,dt);
				
				float dv[2];//the vertical vector from the point to the line

				dv[0]=p2[0]+dt*v[0]-xpo[k];
				dv[1]=p2[1]+dt*v[1]-ypo[k];
				float d=vabs(dv);
				if(d<=50){//adjust the erro threshold here
					tmp[numOfValid][0]=xpo[k];
					tmp[numOfValid][1]=ypo[k];
					numOfValid++;
				}	
// 				printf("%d point: %f\n",k,d);
			}
// 			printf("valid points: %d\n", numOfValid);
			
			if (numOfValid>maxnum){
				for( int k = 0; k < numOfValid; k++){				
					pointset[k][0]=tmp[k][0];
					pointset[k][1]=tmp[k][1];
				}	
				maxnum=numOfValid;
			}	
			
		}
		
		//printf("max: %d\n", maxnum);
		float sumx=0, sumy=0, sumxx=0, sumyy=0, sumxy=0;
		for( int i = 0; i < maxnum; i++){
			printf("point%d: %f, %f\n",i,pointset[i][0],pointset[i][1]);
			sumx+=pointset[i][0];
			sumy+=pointset[i][1];
			sumxx+=pointset[i][0]*pointset[i][0];
			sumyy+=pointset[i][1]*pointset[i][1];
			sumxy+=pointset[i][0]*pointset[i][1];
		}	
		
		float a = (maxnum*sumxy-sumx*sumy)/(maxnum*sumxx-sumx*sumx);
		float b = (sumxx*sumy-sumx*sumxy)/(maxnum*sumxx-sumx*sumx);
		
		//printf("y = %f x + %f\n", a, b);
		
		float *line = new float[2];
		
		line[0] = a;
		
		line[1] = b; 	
		
		delete[] box;
		
		return line;
}




/* ---------------------------------------- End of Functions regarding Ransac ---------------------------------------- */


/* read way points from the text file and put them into x and y way points arries. */
void WayPointFileReading(robotState *state) {
	char str[999];
	FILE * file;
	int i;
	i = 0;
	file = fopen( "waypoints.txt" , "r");
	if (file) {
		while (fscanf(file, "%s", str)!=EOF) {
			if (i == 0) {
				printf("allocate memory for the watpoint array %s\n",str);
				state->number_waypoints = atoi(str);
				state->waypoints_x = malloc(sizeof(int) * atoi(str));
				state->waypoints_y = malloc(sizeof(int) * atoi(str));
			}
			else {
				if (i%2 == 1) {
					state->waypoints_x[(i-1) / 2] = atoi(str);
					printf("x: index: %d value: %d\n", (i-1) / 2, state->waypoints_x[(i-1) / 2]);
				}
				else {
					state->waypoints_y[(i-1) / 2] = atoi(str);
					printf("y: index: %d value: %d\n", (i-1) / 2, state->waypoints_y[(i-1) / 2]);
				}
			}
			i++;
		}
		fclose(file);
	}
	state->waypoints_distance = malloc(sizeof(int) * state->number_waypoints);
	state->waypoints_orientation = malloc(sizeof(int) * state->number_waypoints);
}


/* compute the distances that the robot needs to translate and the angle that the robot needs to rotates from one point to another */
void WayPointDistanceOrientationCalculate(robotState *state) {
	int delta_x_loc;
	int delta_y_loc;	
	int i;
	
	for (i=0; i<state->number_waypoints; i++) {
		if (i == 0) {
			delta_x_loc = state->waypoints_x[i] - 0;
			delta_y_loc = state->waypoints_y[i] - 0;
		}
		else {
			delta_x_loc = state->waypoints_x[i] - state->waypoints_x[i-1];
			delta_y_loc = state->waypoints_y[i] - state->waypoints_y[i-1];
		}
		state->waypoints_distance[i] = (int)sqrt(delta_x_loc * delta_x_loc + delta_y_loc * delta_y_loc);

		if(delta_x_loc > 0) {
			state->waypoints_orientation[i] = (int) 360 * (atan((double)delta_y_loc / (double)delta_x_loc) / ((double)2 * M_PI) ) ;
		}
		
		else if(delta_x_loc < 0) {
			state->waypoints_orientation[i] = (int) 360 * (atan((double)delta_y_loc / -(double)delta_x_loc) / ((double)2 * M_PI) ) ;
			if (state->waypoints_orientation[i] > 0) {
				state->waypoints_orientation[i] = 180 - state->waypoints_orientation[i];
			}		
			else if (state->waypoints_orientation[i] < 0) {
				state->waypoints_orientation[i] = -180 - state->waypoints_orientation[i];
			}	
		}
		else {
			state->waypoints_orientation[i] = 0;
		}
	}
}



/* deallocate the memory that is allocated when initializing the robotState struct */
void RobotStateFree(robotState *state) {

	if (state->laser_data_raw != NULL) {
		free(state->laser_data_raw);
	}
	if (state->laser_data_virtual != NULL) {
		free(state->laser_data_virtual);
	}
	if (state->waypoints_x != NULL) {
		free(state->waypoints_x);
	}
	if (state->waypoints_y != NULL) {
		free(state->waypoints_y);
	}
	if (state->waypoints_orientation != NULL) {
		free(state->waypoints_orientation);
	}
	if (state->waypoints_distance != NULL) {
		free(state->waypoints_distance);
	}
}




/* initialize the robotState structure. */
void RobotStateInitialize(robotState *state, char *argv[]) {

	state->laser_data_raw = NULL;
	state->laser_data_virtual = NULL;
	state->waypoints_x = NULL;
	state->waypoints_y = NULL;
	state->waypoints_orientation = NULL;
	state->waypoints_distance = NULL;

	long State[NUM_STATE]; 

	if ( strcmp(argv[1], "rotate") == 0 ){
		state->current_state = 0;
		state->current_goal_angle = atoi(argv[1]);
	}
	else if ( strcmp(argv[1], "move") == 0 ) {
		state->current_state = 1;
		state->current_goal_distance = atoi(argv[1]);
	}
	else if ( strcmp(argv[1], "locate") == 0 ) {
		state->current_state = 2;
		WayPointFileReading(state);
		state->index_waypoints = 0;
	}
	else if ( strcmp(argv[1], "wall") == 0 ) {
		state->current_state = 3;
	}
	// else if ( stricmp(argv[0], "findwall") == 0 ) {
	// 	state->current_state = 4;
	// }
	else if ( strcmp(argv[1], "wander") == 0 ) {
		state->current_state = 5;
	}
	else if ( strcmp(argv[1], "maze") == 0 ) {
		state->current_state = 6;
	}
	else if ( strcmp(argv[1], "track") == 0 ) {
		state->current_state = 7;
	}
	else {
		state->current_state = 10;
	}

	state->state = State;
	state->laser_data_raw = read_data();
	state->laser_data_virtual = group_data(state->laser_data_raw);
	state->init_x_location = State[STATE_X];
	state->init_y_location = State[STATE_Y];
	state->init_orientation = State[STATE_T];
	state->distance_traveled = 0;
}


 /*
 	evaluate the current sensors; 
	evaluate internal robot status;
	evalluate current state; 
	set the next state; 
	set goals of the robot. 
*/

void RobotStateUpdate(robotState *state) {
	long State[NUM_STATE];
	state->state = State;
	state->laser_data_raw = read_data();
	state->laser_data_virtual = group_data(state->laser_data_raw);
}


/* take in the current state and any other status information and determine the appropriate velocities for the robot */
int *RobotNavigation(robotState *state) {
	int[2] v;

	switch(state->current_state) {

		case 0:
			v = rotate(state);
			break;
		case 1:
			v = move(state);
			break; 
		case 2:
			v = locate(state);
			break; 
		case 3:
			v = wall(state);
			break; 
		// case 4:
		// 	v = findwall(state);
		// 	break; 
		case 5:
			v = wander(state);
			break; 
		case 6:
			v = maze(state);
			break; 
		case 7:
			v = track(state);
			break;
	}
	return v;
}


/* safe check of the rotational verlocity (w) and transitional verlocity (v) */
int *RobotAction(robotState *state, int *velocity) {
	int translate, rotate, rotate1, translate1;

	if (velocity[0] > 30) {
		translate = 30;
		rotate = translate * velocity[1] / velocity[0];
	}
	else if (velocity[0] < -30) {
		translate = -30;
		rotate = translate * velocity[1] / velocity[0];
	}
	else {
		translate = velocity[0];
		rotate = velocity[1];
	}

	if (rotate > 20) {
		rotate1 = 20;
		translate1 = rotate1 * translate / rotate;
		rotate = rotate1;
		translate = translate1;
	}
	else if (rotate < -20) {
		rotate1 = -20;
		translate1 = rotate1 * translate / rotate;
		rotate = rotate1;
		translate = translate1;
	}
	vm(translate, rotate);
}


/* robot main loop */
int main(int argc, char *argv[]) {
	robotState *state;
	int *velocity;
	state = malloc(sizeof(robotState));

	WayPointFileReading(state);


	long th;

/* ---------------------------------------- Start Setup Robot ---------------------------------------- */
	connectRobot(State, MAGE_MODEL_MAGELLAN, (char *)"/dev/ttyUSB0"); /* connect (serial line) with the robot */
	irOn(); /* start up IRs */
	sonarOn(); /* start up Sonars */
	signal( SIGINT, sighandler ); /* catch cntl-c to get a clean shutdown */
	vm(0,0); /* set the speed */
	usleep(500000); /* wait for things to start up */
	RobotStateInitialize(state, argv);

/* ----------------------------------------- End Setup Robot ------------------------------------------ */

	while (state->current_state != 10) {
		RobotStateUpdate(state);
	 	velocity = RobotNavigation(state);	
	 	RobotAction(state, velocity);	
	}

/* ----------------------------------------- Clean Up Robot ------------------------------------------ */
	RobotStateFree(state);
	usleep(100000); 
	vm(0,0);
	disconnectRobot(); /* disconnect the robot */
	free(state);
	return 0;
}





