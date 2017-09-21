/*
    05/05/2017
    CS363 S17
    Project7
    Liwei Jiang
    Jiaheng Hu
    
    robotStructure.cpp
*/

#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <ipc.h>
#include <SVM_VisionModule.h>
#include <GCM_IPC.h>
#include <GCM_Log.h>
// #include <SDL2/SDL.h>
#include "play_audio.h"
#include "URGclient.h"
#include "robotStructure.h"
// #include <SDL2/SDL.h>

#define pi 3.14159265358979323846
Display *curDisplay;
Window   curWindow;
GC       curGC; /* graphics context */

int datahand[2];
SVM_Operator_Command svmCommand;
int flags;
struct termios t;
int red[2];
int green[2];
int pink[2];
int yellow[2];

/* top, left, bottom, right in image coordinates */
int redBbox[4]; 
int greenBbox[4]; 
int pinkBbox[4]; 
int yellowBbox[4]; 
int redSize;
int greenSize;
int pinkSize;
int yellowSize;

void dataHandler(MSG_INSTANCE msgInstance, void *callData,void *clientData);

// monitoring the audio length and the current position it is playing
// void MyAudioCallback(void* userdata, Uint8* stream, int streamLength) {
// 	AudioData* audio = (AudioData*)userdata;
// 
// 	if (audio->length == 0) {
// 		return;
// 	}
// 
// 	Uint32 length = (Uint32) streamLength;
// 	length = (length > audio->length ? audio->length : length);
// 
// 	SDL_memcpy(stream, audio->pos, length);
// 
// 	audio->pos += length;
// 	audio->length -= length;
// }
// 
// play the audio
// int audio_play(std::string FILE_PATH, int *velocity, int stateIndex, robotState *state) {
// 	SDL_Init(SDL_INIT_EVERYTHING);
// 	char c;
// 	SDL_AudioSpec wavSpec;
// 	Uint8* wavStart;
// 	Uint32 wavLength;
// 	const char *file_path = FILE_PATH.c_str();
// 
// 	if (SDL_LoadWAV(file_path, &wavSpec, &wavStart, &wavLength) == NULL) {
// 		// TODO: Proper error handling
// 		std::cerr << "Error: " << FILE_PATH << " could not be loaded as an audio file" << std::endl;
// 		return 1;
// 	}
// 
// 	AudioData audio;
// 	audio.pos = wavStart;
// 	audio.length = wavLength;
// 
// 	wavSpec.callback = MyAudioCallback;
// 	wavSpec.userdata = &audio;
// 
// 	SDL_AudioDeviceID device = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, SDL_AUDIO_ALLOW_ANY_CHANGE);
// 
// 	if (device == 0) {
// 		// TODO: Proper error handling
// 		std::cerr << "Error: " << SDL_GetError() << std::endl;
// 		return 1;
// 	}
// 
// 	SDL_PauseAudioDevice(device, 0);
// 
// 	while (audio.length > 0) {
// 		printf("length: %d\n", audio.length);
// 		read(0, &c, 1);
// 		velocity[0] = 0;
// 		velocity[1] = 0;
// 		
// 		if((c == 't') || (c == 'T')) {
// 			break;
// 		}
// 		SDL_Delay(100);
// 	}
// 
// 	state->visual_state = stateIndex;
// 	SDL_CloseAudioDevice(device);
// 	SDL_FreeWAV(wavStart);
// 	SDL_Quit();
//     return 0;
//  }
//  

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


/* check the direction that is short to the barrier using laser */
int check_shortest_laser(int *lasers, int num, robotState *state) {
	int i, shortest_laser;
	int shortest;
	
	shortest = 3000;
	shortest_laser = 0;   
	for (i = 0; i < num; i++) {
		if (state->laser_data_virtual[lasers[i]] < shortest) {
      		shortest = state->laser_data_virtual[lasers[i]];
      		shortest_laser = lasers[i];
    	}
	}
	return shortest;
}


/* returns the angle that is the closest direction */ 
int get_best_dir_laser(robotState *state) {
  	int i;
  	int min_value = 500; // min. sonar reading 
  	int best_dir; // to remember the direction
  	int angle;
  	
  	best_dir = 12;
  
  	for (i=0; i<24; i++) {
  		if (state->laser_data_virtual[i] > min_value) {
  			min_value = state->laser_data_virtual[i];
  			best_dir = i;
  		}
  	}
  		
	best_dir = best_dir - 12;
  	angle = best_dir * 100;
  	return angle;
}


/* check wether the next state of way point following */
void WaypointRotateCheck(robotState *state) {
	if (state->current_state == 2) {
		if (state->index_waypoints < state->number_waypoints) {
			state->current_goal_angle = state->waypoints_orientation[state->index_waypoints + 1];
			state->init_x_location = state->state[STATE_X];
			state->init_y_location = state->state[STATE_Y];
			state->init_orientation = state->state[STATE_T];
			state->waypoints_if_dis_ori = 1;
		}
	}
	else {
		state->current_state = 10;
	}
}


/* rotate the robot by a certain angle */
void rotate(robotState *state, int *velocity) {
	velocity[0] = 0;
	velocity[1] = 0;

	if (state->current_goal_angle < 400 && state->current_goal_angle > -400) {
		state->waypoints_if_dis_ori = 1;
		return;
	}
			
	/* positive: left turn */
	/* orientation value increase while turning left */
	if (state->current_goal_angle >= 0) {
		if (6283 - state->init_orientation >= state->current_goal_angle) {
			if( state->state[STATE_T] - state->init_orientation <= state->current_goal_angle ) {
				velocity[1] = state->current_goal_angle - (state->state[STATE_T] - state->init_orientation) + 10;
			}
			else {
				WaypointRotateCheck(state);
			}
		}
		else {
			if( state->state[STATE_T] - state->init_orientation >= 0 ) {		
				velocity[1] = state->current_goal_angle - (state->state[STATE_T] - state->init_orientation) + 10;
			}	
			else if ( state->state[STATE_T] <= state->current_goal_angle - (6283 - state->init_orientation) ) {					
				velocity[1] = state->current_goal_angle - (6283 - state->init_orientation + state->state[STATE_T]) + 10;
			}	
			else {
				WaypointRotateCheck(state);		
			}
		}	
	}	
	
	/* negative: right turn */
	/* compass value increases while turning right */
	else if (state->current_goal_angle < 0) {
		
		if ( state->state[STATE_T] == 0 ) {
			velocity[1] = -10;
		}	
		else {
			if (state->init_orientation >= -state->current_goal_angle) {
				if( state->init_orientation - state->state[STATE_T] <= -state->current_goal_angle ) {	
					velocity[1] =  state->current_goal_angle + (state->init_orientation - state->state[STATE_T]) - 10;
				}
				else {
					WaypointRotateCheck(state);
				}	
			}
			else {
				if( state->init_orientation - state->state[STATE_T] >= 0 ) {
					velocity[1] = state->current_goal_angle + (state->init_orientation - state->state[STATE_T]) - 10;
				}
				else if ( 6283 - state->state[STATE_T] <= -state->current_goal_angle - state->init_orientation ) {
					velocity[1] = - state->current_goal_angle + (state->init_orientation + 6283 - state->state[STATE_T]) - 10;	
				}
				else {
					WaypointRotateCheck(state);
				}				
			}	
		}	
	}
	return;
}


/* check wether the next state of way point following */
void WaypointTranslateCheck(robotState *state) {
	if (state->current_goal_angle < 400 && state->current_goal_angle > -400) {
		WaypointRotateCheck(state);
	}
	if (state->current_state == 2) {
		if (state->index_waypoints < state->number_waypoints - 1) {
			state->index_waypoints = state->index_waypoints + 1;
			state->current_goal_distance = state->waypoints_distance[state->index_waypoints];
			state->init_x_location = state->state[STATE_X];
			state->init_y_location = state->state[STATE_Y];
			state->init_orientation = state->state[STATE_T];
			state->distance_traveled = 0;
			state->waypoints_if_dis_ori = 0;
		}
		else {
			state->current_state = 10;
		}
	}
	else {
		state->current_state = 10;
	}
}


/* move the robot by a certain distance */
int *move(robotState *state, int *velocity) {
	int delta_x_loc;
	int delta_y_loc;	
	velocity[0] = 0;
	velocity[1] = 0;

	if (state->current_goal_angle < 400 && state->current_goal_angle > -400) {
		velocity[1] = state->current_goal_angle;
	}

	/* move forward */
	if (state->current_goal_distance >= 0) {
		if(state->distance_traveled < state->current_goal_distance) {
			delta_x_loc = (int)(state->init_x_location) - (int)(state->state[STATE_X]);
			delta_y_loc = (int)(state->init_y_location) - (int)(state->state[STATE_Y]);		
			state->distance_traveled = (int)sqrt(delta_x_loc*delta_x_loc + delta_y_loc*delta_y_loc);
			printf("Distance travelled: %d\n", state->distance_traveled);
			velocity[0] = state->current_goal_distance - state->distance_traveled + 40;
		}
		else {
			WaypointTranslateCheck(state);
		}
	}
	
	/* move backward */
	else if (state->current_goal_distance < 0) {
		if(state->distance_traveled < -state->current_goal_distance) {		
			delta_x_loc = (int)(state->init_x_location) - (int)(state->state[STATE_X]);
			delta_y_loc = (int)(state->init_y_location) - (int)(state->state[STATE_Y]);		
			state->distance_traveled = (int)sqrt(delta_x_loc*delta_x_loc + delta_y_loc*delta_y_loc);
			printf("Distance travelled: %d\n", state->distance_traveled);
			velocity[0] = state->current_goal_distance + state->distance_traveled - 40;
		}
		else {
			WaypointTranslateCheck(state);
		}
	}	
}


/* move the robot across a set of way points */
void locate(robotState *state, int *velocity){
	printf("locate index %d\n", state->waypoints_if_dis_ori);
	if (state->waypoints_if_dis_ori == 0) {
		rotate(state, velocity);
	}
	else {
		move(state, velocity);
	}
}


/* ---------------------------------------- Functions regarding Ransac ---------------------------------------- */
//return the x value of a certain measure ( in respect to the robot)
float getX(int index, int distance){
	float degree=((index)*3.53+60)/180*pi;
	float x=cos(degree);
	return x*distance;
}	


//return the y value of a certain measure ( in respect to the robot)
float getY(int index, int distance){
	float degree=((index)*3.53+60)/180*pi;
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
  The returned value will be the slope and intercept of the wall
  Side:0->right 1->front 2->left*/ 
float *getWall(int *rawLaser, int side){
		int *box;
		int numReadings;
		int numBoxes;
		int i;
		numReadings = 85;
		numBoxes = 17;
		box = new int[numBoxes];

		for(i=0;i<numBoxes;i++) {
			box[i] = 10000000;
		}
		
		if(side==0){
			for(i=0;i<numReadings;i++) {
				int ix = i/5 >= numBoxes ? numBoxes-1 : i/5;
				
				if (rawLaser[i] >= 30 && rawLaser[i]!=3000) {
					box[ix] = box[ix] > rawLaser[i] ? rawLaser[i] : box[ix];
				}
			}
		}	
		
		else if(side==1){
			for(i=0;i<numReadings;i++) {
				int ix = i/5 >= numBoxes ? numBoxes-1 : i/5;
				
				if (rawLaser[i+128] >= 30&&rawLaser[i+128]!=3000) {
					box[ix] = box[ix] > rawLaser[i+128] ? rawLaser[i+128] : box[ix];
				}
			}	
		}	
		
		else if(side==2){
			for(i=0;i<numReadings;i++) {
				int ix = i/5 >= numBoxes ? numBoxes-1 : i/5;
				
				if (rawLaser[i+255] >= 30&&rawLaser[i+255]!=3000) {
					box[ix] = box[ix] > rawLaser[i+255] ? rawLaser[i+255] : box[ix];
				}
			}
		}	
			
		//decide interation time
		float inlier=0.5;
		float points=2;
		float success=0.98;
		
		float loop = (log(1-success))/(log(1-pow(inlier,points)));
		
		srand (time(NULL));
		
		float xpo[numBoxes];
		float ypo[numBoxes];
		for( int i = 0; i < numBoxes; i++){
			xpo[i]=getX(i,box[i]);
			ypo[i]=getY(i,box[i]);
		}
		printf("Readings:\n");
		for(i=0;i<numBoxes;i++) {
			printf("%5d ", box[i]);
		}
		printf("\n");
		
		int maxnum=0;
			//The max number of points in a model
			
		float pointset[numBoxes][2];
			//the points that are inliers	
		for( int i = 0 ;i < loop; i++){
			int rand1=rand() % numBoxes;
			while(box[rand1]==10000000)
				rand1=rand() % numBoxes;
				
			int rand2=rand() % numBoxes;
			
			while(rand2 == rand1 || box[rand2]==10000000)
				rand2=rand() % numBoxes;
			
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
			
			float tmp[numBoxes][2];
			int numOfValid = 0;
			for( int k = 0; k < numBoxes ; k++ ){
			
				float r1=p2[0]-xpo[k];				
				float r2=p2[1]-ypo[k];//the vector from point i to p2
				/* have problem */				
				float dt=r1*v[0]+r2*v[1];//distance from p1 to the closest point on the line
				dt=-dt;
				//dt have problem
				//it is not the closest point
				
				float dv[2];//the vertical vector from the point to the line

				dv[0]=p2[0]+dt*v[0]-xpo[k];
				dv[1]=p2[1]+dt*v[1]-ypo[k];
				float d=vabs(dv);
				//adjust the erro threshold here
				if (d<=50) {	
					tmp[numOfValid][0]=xpo[k];
					tmp[numOfValid][1]=ypo[k];
					numOfValid++;
				}	
			}
			
			if (numOfValid>maxnum){
				for( int k = 0; k < numOfValid; k++){				
					pointset[k][0]=tmp[k][0];
					pointset[k][1]=tmp[k][1];
				}	
				maxnum=numOfValid;
			}	
		}

		float sumx=0, sumy=0, sumxx=0, sumyy=0, sumxy=0;
		for( int i = 0; i < maxnum; i++){
			//printf("point%d: %f, %f\n",i,pointset[i][0],pointset[i][1]);
			sumx+=pointset[i][0];
			sumy+=pointset[i][1];
			sumxx+=pointset[i][0]*pointset[i][0];
			sumyy+=pointset[i][1]*pointset[i][1];
			sumxy+=pointset[i][0]*pointset[i][1];
		}	
		
		float a = (maxnum*sumxy-sumx*sumy)/(maxnum*sumxx-sumx*sumx);
		float b = (sumxx*sumy-sumx*sumxy)/(maxnum*sumxx-sumx*sumx);
		
		float d=fabs(b)/sqrt(a*a+1);
		float degree=atan(a);
		//printf("degree=%f\n",degree);
		//printf("distance=%f\n",d);
		float *line = new float[2];
		
		line[0] = d;
		line[1] = degree; 	

		delete[] box;
		return line;
}


/* allow the robot walk along a wall */
void wall(robotState *state,int *velocity ) {
	int front;
	int left;
	int right;
	int front_laser[3] = {11, 12, 13};
	int right_laser[6] = {3, 4, 5, 6, 7, 8};
	int left_laser[6] = {16, 17, 18, 19, 20, 21};
	int right_laser_small[3] = {20, 21, 22};
	velocity[0] = 0;
	velocity[1] = 0;
	
	front = check_shortest_laser(front_laser, 3, state);
	left = check_shortest_laser(left_laser, 6, state);
	right = check_shortest_laser(right_laser, 6, state);
	
	// if far from front, left, and right
	if (front >= 1000 && left >= 1000 && right >= 1000) {
		velocity[0] = front + 10;
		velocity[1] = 0;
		printf("--- too far from front ---\n");
	}
	
	else {
		// if front is the most hindered direction
		if (front <= 1000) {
			//printf("should turn\n");
			if(right<=left){
				velocity[0] = 0;
				velocity[1] = 120;
			}
			
			else{
				velocity[0] = 0;
				velocity[1] = -120;
			}	
		}
		
		else if (left < right) {
			printf("follow left wall\n");
			float *wallValue = getWall(state->laser_data_raw, 2);
				if(wallValue[0]<1250){
					if(wallValue[1]<=-0.5){
						printf("angle<-0.5\n");
						velocity[0] = 0;
						velocity[1] = -120;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = wallValue[1]*120-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+20;
						velocity[1] = 0;
					}
					else {
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = -40;
						}
						else	{
							velocity[0] = wallValue[0]*0.1;
							velocity[1] = wallValue[1]*120+5;
						}
					}	
				}
				
				else if(wallValue[0]<1500){
					if(wallValue[1]<=-0.5){
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120-5;
					}
					else if(wallValue[1]<=0){
						velocity[0] = front+20;
						velocity[1] = wallValue[1]*120-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*120+5;
					}
					else {
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = -40;
						}
						else	{
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120+5;
						}
					}	
				}	
				
				else{
					if(wallValue[1]<=-0.5){
						velocity[0] = front+20;
						velocity[1] = -120;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = 0;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+20;
						velocity[1] = 0;
					}
					else {
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = -40;
						}
						else	{
							velocity[0] = 0;
							velocity[1] = wallValue[1]*120+5;
						}
					}	
				}
		delete []wallValue;	
		}			
				
		else {
			printf("follow right wall\n");
			float *wallValue = getWall(state->laser_data_raw, 0);
				if(wallValue[0]<1250){
					if(wallValue[1]<=-0.5){
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = 80;
							}
						else	{
							velocity[0] = 0;
							velocity[1] = -120;
						}
					}	
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = wallValue[1]*120-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+20;
						velocity[1] = 0;
					}
					else {
						velocity[0] = front+20;
						velocity[1] = wallValue[1]*120+5;
					}	
				}
				
				else if(wallValue[0]<1500){
					if(wallValue[1]<=-0.5){
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = 80;
							}
						else	{
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120-5;
						}
					}
					else if(wallValue[1]<=0){
						velocity[0] = front+20;
						velocity[1] = wallValue[1]*120-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+20;
						velocity[1] = wallValue[1]*120+5;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120+5;
					}	
				}	
				else{
					if(wallValue[1]<=-0.5){
						if (front <= 1250){
								velocity[0] = 0;
								velocity[1] = 80;
							}
						else	{
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120-5;
						}
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = 0;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+20;
						velocity[1] = 0;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*120;
					}	
				}
		delete []wallValue;	
		}	
	}
	return;
}


/* allow the robot wander without bumping into any obstacle */
void wander(robotState *state, int *velocity) {
	int front;
	int left;
	int right;
	int front_laser[3] = {11, 12, 13};
	int right_laser[6] = {3, 4, 5, 6, 7, 8};
	int left_laser[6] = {16, 17, 18, 19, 20, 21};
	int right_laser_small[3] = {20, 21, 22};
	velocity[0] = 0;
	velocity[1] = 0;
	
	front = check_shortest_laser(front_laser, 3, state);
	left = check_shortest_laser(left_laser, 6, state);
	right = check_shortest_laser(right_laser, 6, state);
	
	// if far from front, left, and right
	if (front >= 2000 && left >= 1500 && right >= 1500) {
		velocity[0] = front + 10;
		velocity[1] = 0;
	}
	
	else {
		// if front is the most hindered direction
		if (front <= left && front <= right) {
			if (front > 1000) {
				velocity[0] = front + 10;
				velocity[1] = 0;
			}
			// after moving close enough to the front wall, turn left while monitoring the right side
			else if (front <= 1000) {
				if (front < 1500) {
					velocity[1] = get_best_dir_laser(state);
				}
				else if (front < 2500 && front >= 1500) {
					velocity[0] = 40;
					velocity[1] = (get_best_dir_laser(state));
				}
				else {	
					velocity[0] = front + 5;
				}	
			}
		}
		// if left is the most hindered direction
		else if (left < front && left < right) {
			if (left < 1000) {
				velocity[1] = -100;		
			}
			else {
				velocity[0] = front + 10;
				velocity[1] = -120;
			}
		}
		// if right is the most hindered direction
		else if (right < front && right < left) {
			if (right < 1000) {
				velocity[1] = 100;		
			}
			else {
				velocity[0] = front + 10;
				velocity[1] = 120;			
			}
		}	
	}
	return;	
}


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
				state->number_waypoints = atoi(str);
				state->waypoints_x = (int *) malloc(sizeof(int) * atoi(str));
				state->waypoints_y = (int *)malloc(sizeof(int) * atoi(str));
			}
			else {
				if (i%2 == 1) {
					state->waypoints_x[(i-1) / 2] = atof(str) * 1000;
				}
				else {
					state->waypoints_y[(i-1) / 2] = atof(str) * 1000;
				}
			}
			i++;
		}
		fclose(file);
	}
	state->waypoints_distance = (int *) malloc(sizeof(int) * state->number_waypoints);
	state->waypoints_orientation = (int *) malloc(sizeof(int) * state->number_waypoints);
}


/* compute the distances that the robot needs to translate and the angle that the robot needs to rotates from one point to another */
void WayPointDistanceOrientationCalculate(robotState *state) {
	int delta_x_loc;
	int delta_y_loc;	
	int old_orientation;
	int i;

	old_orientation = 0;
	
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
			state->waypoints_orientation[i] = (int) 360 * (atan((double)delta_y_loc / (double)delta_x_loc) / ((double)2 * M_PI)) - old_orientation;
			old_orientation = (int) 360 * (atan(((double)delta_y_loc / (double)delta_x_loc) ) / ((double)2 * M_PI));
		}
		
		else if(delta_x_loc < 0) {

			state->waypoints_orientation[i] = (int) 360 * (atan((double)delta_y_loc / -(double)delta_x_loc) / ((double)2 * M_PI));
			
			if (state->waypoints_orientation[i] > 0) {
				state->waypoints_orientation[i] = (180 - state->waypoints_orientation[i]) - old_orientation;
				old_orientation = (180 - state->waypoints_orientation[i]);
			}		
			else if (state->waypoints_orientation[i] < 0) {
				state->waypoints_orientation[i] = (-180 - state->waypoints_orientation[i]) - old_orientation;
				old_orientation = (-180 - state->waypoints_orientation[i]);
			}	
			else if (state->waypoints_orientation[i] == 0) {
				state->waypoints_orientation[i] = 180 - old_orientation;
				old_orientation = 180;
			}
		}
		else {
			if (delta_y_loc > 0) {
				state->waypoints_orientation[i] =  90 - old_orientation;
				old_orientation = 90;
			}
			else if (delta_y_loc < 0) {
				state->waypoints_orientation[i] =  -90 - old_orientation;
				old_orientation = -90;
			}
			else {
				state->waypoints_orientation[i] =  0 - old_orientation;
				old_orientation = 0;
			}
		}
		state->waypoints_orientation[i] = state->waypoints_orientation[i] % 360;	
		state->waypoints_orientation[i] = 1000 * state->waypoints_orientation[i] * M_PI / 180;		
	}
}


/* subscribe data from the SVM operator */
void dataHandler(MSG_INSTANCE msgInstance, void *callData,void *clientData) {
	int i;
	SVM_Operator_Data *d = (SVM_Operator_Data *)callData;
	datahand[0] = d->location[0][0];
	datahand[1] = d->location[0][1];
	green[0] = d->location[0][0];
	green[1] = d->location[0][1];
	pink[0] = d->location[1][0];
	pink[1] = d->location[1][1];
	yellow[0] = d->location[2][0];
	yellow[1] = d->location[2][1];
	red[0] = d->location[3][0];
	red[1] = d->location[3][1];
	
	/* top, left, bottom, right in image coordinates */	
	for (i=0; i<4; i++) {
		redBbox[i] = d->bbox[3][i]; 
		greenBbox[i] = d->bbox[0][i]; 
		pinkBbox[i] = d->bbox[1][i]; 
		yellowBbox[i] = d->bbox[2][i]; 
	}
	
	redSize = d->size[3];
	greenSize = d->size[0];
	pinkSize = d->size[1];
	yellowSize = d->size[2];

	IPC_freeData(IPC_msgFormatter(SVM_DATA_RESPONSE), callData);
	return;
}


/* calculate the size of the bbox and the position of the bbox and return if the box is in the middle of the screen */
int sizeOfBbox(int *bbox) {
	int width;
	int height;
	width = bbox[3] - bbox[1];
	height = bbox[0] - bbox[2];	
	printf("top: %d \n", bbox[0]);
	printf("left: %d \n", bbox[2]);
	printf("bottom: %d \n", bbox[0]);
	printf("right: %d \n", bbox[0]);
	return width * height;
}



/*  
	connects to central;
	tells the vision module to turn on a specific SVM operator;
*/
void visualOn(int ID) {
	
	// command line arguments, initialized to reasonable defaults
	int operatorID = SVM_OP_Pink_Blob;
	int priority = 1;
	int ptzatt = SVM_PTZ_DEFAULT;
	float ptzpos[3] = {0.0, 1.571, 0.0};
	int arg = 0;
	int streaming = 250;
	int timing = SVM_TIMING_STOCHASTIC;
	int i;
	struct termios newt;
	char c;
	char *hname = NULL;
	operatorID = ID;

	// This connects to the IPC central server using a call to the GCM library
	// init IPC
	GCM_initIPC((char *)"listener", hname);

	// This fills out the data structure used to send a command to SVM
	// publish the command to SVM
	svmCommand.operatorID = (SVM_Operator)operatorID; // which operator to turn on
	svmCommand.priority = priority;                   // priority for the operator, 0 is off, non-zero is on
	svmCommand.timing = (SVM_Operator_Timing)timing;  // stochastic timing (0) or fixed timing (1)
	svmCommand.ptzatt = (SVM_PTZ_Attribute)ptzatt;    // default (0), fixed (1), search (2), tracking(3)
	svmCommand.ptzpos[0] = ptzpos[0];   // pan
	svmCommand.ptzpos[1] = ptzpos[1];   // tilt
	svmCommand.ptzpos[2] = ptzpos[2];   // zoom
	svmCommand.ptzMaxMisses = 0;        
	svmCommand.ptzMaxFrames = 0;
	svmCommand.notifyMiss = 0;
	svmCommand.arg = arg;               // operator specific argument
	svmCommand.streaming = streaming;   // 0 (not streaming or the minimum time between 
	// broadcasts of detections in ms
									  
	printf("Sending command:\n");
	printf("  operatorID = %d\n", svmCommand.operatorID);
	printf("  priority =   %d\n", svmCommand.priority);
	printf("  ptzatt =     %d\n", svmCommand.ptzatt);
	printf("  ptzpos[0] =  %f\n", svmCommand.ptzpos[0]);
	printf("  ptzpos[1] =  %f\n", svmCommand.ptzpos[1]);
	printf("  ptzpos[2] =  %f\n", svmCommand.ptzpos[2]);
	printf("  MaxMisses =  %d\n", svmCommand.ptzMaxMisses);
	printf("  MaxFrames =  %d\n", svmCommand.ptzMaxFrames);
	printf("  notifyMiss = %d\n", svmCommand.notifyMiss);
	printf("  arg =        %d\n", svmCommand.arg);
	printf("  streaming =  %d\n\n", svmCommand.streaming);
	
	// this sends the command to SVM
	IPC_publishData(SVM_COMMAND, &svmCommand);

	// put the terminal in a no-blocking mode
	tcgetattr(0, &t);
	newt = t;

	newt.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
	newt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	newt.c_cflag &= ~(CSIZE | PARENB);
	newt.c_cflag |= CS8;
	newt.c_oflag &= ~(OPOST);
	newt.c_cc[VMIN] = 1;
	newt.c_cc[VTIME] = 0;

	tcsetattr(0, TCSAFLUSH, &newt);

	// subscribe to IPC messages so that we listen for responses from SVM
	IPC_subscribeData(SVM_DATA_RESPONSE, dataHandler, NULL);

	flags = fcntl(0, F_GETFL, 0);
	fcntl(0, F_SETFL, flags | O_NONBLOCK);
}


/* disconnected to the IPC */
void visualOff() {
	fcntl(0, F_SETFL, flags);
	// turn off the operator we turned on earlier by setting the priority to zero
	svmCommand.priority = 0;
	IPC_publishData(SVM_COMMAND, &svmCommand);

	// terminate the IPC connection by calling the GCM function
	GCM_terminateIPC((char *)"Listener");

	tcsetattr(0, TCSAFLUSH, &t);
	printf("Terminating\n");
}


/* 
	detect a target and follow it with a specific distance
 */
void visual(robotState *state, int *velocity) {
	int front_laser[3] = {11, 12, 13};
	int front;
	
	state->visual_target_x = (int)(datahand[0]);
	state->visual_target_y = (int)(datahand[1]);
	front = check_shortest_laser(front_laser, 3, state);
	printf("front: %d\n", front);
	printf("visual: x: %d, y: %d\n", state->visual_target_x, state->visual_target_y);
	
	velocity[0] = 0;
	velocity[1] = 0;
	
	// STATE wander: searching the target
	if (state->visual_state == 0) {
		printf("wander\n");
		wander(state, velocity);
	
		if (state->visual_target_x != state->visual_last_x || state->visual_target_y != state->visual_last_y) {
			printf("see target\n");
			state->visual_state = 1;
		}
	}
	
	// STATE rotate: rotating towards the target
	else if (state->visual_state == 1) {
		printf("rotate target\n");
		if (state->visual_target_x < 130 && state->visual_target_x > 0 ) {
			velocity[1] = (150 - state->visual_target_x + 10) * 3;
			state->visual_track_counter = 0;
			return;
		}
		else if (state->visual_target_x > 170 && state->visual_target_x < 300) {
			velocity[1] = (150 - state->visual_target_x - 10) * 3;
			state->visual_track_counter = 0;
			return;
		}
		else if (state->visual_target_x > 130 && state->visual_target_x < 170) {	
			state->visual_track_counter = state->visual_track_counter + 1;
			if (state->visual_track_counter >= 3) {
				state->visual_state = 2;
				state->visual_track_counter = 0;
			}
		}
	}
	
	// STATE rotate: following the target and keeping a specific distance from the target
	else if (state->visual_state == 2) {
		printf("track target\n");
		if (state->visual_target_x == state->visual_last_x && state->visual_target_y == state->visual_last_y) {
			state->visual_wander_counter = state->visual_wander_counter + 1;
			if (state->visual_wander_counter >= 10) {
				state->visual_state = 0;
				state->visual_wander_counter = 0;
				return;
			}
		}
		else {
			state->visual_wander_counter = 0;
		}
	
		if (front > 1500) {
			velocity[0] = front - 1500 + 50;
		}
		else if (front < 1500) {
			velocity[0] = front - 1500 - 50;
		}
	
		if (state->visual_target_x < 130 && state->visual_target_x > 0 ) {
			state->visual_rotate_counter = state->visual_rotate_counter + 1;
			if (state->visual_rotate_counter >= 2) {
				state->visual_state = 1;
				state->visual_rotate_counter = 0;
			}
		}
		else if (state->visual_target_x > 170 && state->visual_target_x < 300) {
			state->visual_rotate_counter = state->visual_rotate_counter + 1;
			if (state->visual_rotate_counter >= 2) {
				state->visual_state = 1;
				state->visual_rotate_counter = 0;
			}
		}
		else if (state->visual_target_x > 130 && state->visual_target_x < 170) {
			state->visual_rotate_counter = 0;
		}
	}
	state->visual_last_x = (int)(datahand[0]);
	state->visual_last_y = (int)(datahand[1]);
}



/* identify color, wander, find same color wall paper */
void visualColor(robotState *state, int *velocity) {
	velocity[0] = 0;
	velocity[1] = 0;
	int i;
	
	// STATE 0: wander + identify target
	if (state->visual_state == 0) {
		printf("Wander while searching for the target\n");
		wander(state, velocity);
		
		if ((red[0] != state->visual_last_x_red || red[1] != state->visual_last_y_red)) {
			printf("orange\n");
			
			if (redSize > 4000) {
				state->visual_red_counter = state->visual_red_counter + 1;
			}
	
			if (state->visual_red_counter == 1) {
				printf("orange detected!\n");
				state->visual_state = 8;
				state->visual_red_counter = 0;
			}
		}
		
		else {
			state->visual_red_counter = 0;
		}
	}
	
	// STATE 1: identify the sticker
	else if (state->visual_state == 1) {
		printf("looking for colors!\n");
	
		if (green[0] != state->visual_last_x_green || green[1] != state->visual_last_y_green) {
			printf("green detected!\n");
			if (int(green[0]) > 50 && int(green[0]) < 200 && greenSize > 2500){
				printf("green counter: %d\n", state->visual_green_counter);
				state->visual_green_counter = state->visual_green_counter + 1;
				state->visual_yellow_counter = 0;
				state->visual_pink_counter = 0;
				state->visual_track_counter = 0;
			}
		
			if (state->visual_green_counter == 1) {
				audio_play("pick_green.wav");
				usleep(500000);
				state->visual_state = 2;
				state->visual_track_counter = 0;
				state->visual_green_counter = 0;
			}
		}
	
		else if (pink[0] != state->visual_last_x_pink || pink[1] != state->visual_last_y_pink) {
			printf("pink detected!\n");
			
			if (int(pink[0]) > 50 && int(pink[0]) < 200 && pinkSize > 2500){
				printf("pink counter: %d\n", state->visual_pink_counter);
				state->visual_pink_counter = state->visual_pink_counter + 1;
				state->visual_yellow_counter = 0;
				state->visual_green_counter = 0;
				state->visual_track_counter = 0;
			}

			if (state->visual_pink_counter == 1) {
				audio_play("pick_pink.wav");
				usleep(500000);
				state->visual_state = 3;
				state->visual_track_counter = 0;
				state->visual_pink_counter = 0;
			}
		}
	
		else if (yellow[0] != state->visual_last_x_yellow || yellow[1] != state->visual_last_y_yellow) {
			printf("yellow detected!\n");
			
			if (int(yellow[0]) > 50 && int(yellow[0]) < 200 && yellowSize > 2500){
				printf("yellow counter: %d\n", state->visual_yellow_counter);
				state->visual_yellow_counter = state->visual_yellow_counter + 1;	
				state->visual_green_counter = 0;
				state->visual_pink_counter = 0;
				state->visual_track_counter = 0;
			}
		
			if (state->visual_yellow_counter == 1) {
				audio_play("pick_yellow.wav");
				usleep(500000);
				state->visual_state = 4;
				state->visual_track_counter = 0;
				state->visual_yellow_counter = 0; 
			}
		}
	
		else {
			state->visual_green_counter = 0;
			state->visual_pink_counter = 0;
			state->visual_yellow_counter = 0;
			state->visual_track_counter = state->visual_track_counter + 1;
			
			if (state->visual_track_counter == 40) {
				state->visual_state = 0;
				state->visual_track_counter = 0;
			}
		}
	}
	
	
	else if (state->visual_state == 2) {
		printf("wander to find green wall paper\n");
		wall(state, velocity);
		
		if(state->visual_green_counter <= -3){
			state->visual_state = 5;
		}
		
		if (green[0] != state->visual_last_x_green || green[1] != state->visual_last_y_green) {
			state->visual_green_counter = state->visual_green_counter + 1;

			if (state->visual_green_counter == 15 && state->green_detected == 0) {
				printf("green detected!\n");
				state->green_detected = 1; 				
			}
		}

		else {
			if(state->green_detected == 1){
				state->visual_green_counter -= 1;
				printf("after_green: %d\n",state->visual_green_counter );
			}
		}
	}
	
	// STATE 3: wander to find the wall paper with the pink color
	else if (state->visual_state == 3) {
		printf("wander to find pink wall paper\n");
		wall(state, velocity);
		
		if(state->visual_pink_counter <= -3){
			state->visual_state = 6;
		}
		
		if (pink[0] != state->visual_last_x_pink || pink[1] != state->visual_last_y_pink) {
				state->visual_pink_counter = state->visual_pink_counter + 1;

			if (state->visual_pink_counter == 15 && state->pink_detected == 0) {
				printf("pink detected!\n");
				state->pink_detected = 1; 				
			}
		}

		else {
			if(state->pink_detected == 1){
				state->visual_pink_counter -= 1;
				printf("before_pink: %d\n",state->visual_pink_counter );
			}
		}
	}
	
	// STATE 4: wander to find the wall paper with the yellow color
	else if (state->visual_state == 4) {
		printf("wander to find yellow wall paper\n");
		wall(state, velocity);
		
		if(state->visual_yellow_counter <= -3){
			state->visual_state = 7;
		}
		
		if (yellow[0] != state->visual_last_x_yellow || yellow[1] != state->visual_last_y_yellow) {
			state->visual_yellow_counter = state->visual_yellow_counter + 1;

			if (state->visual_yellow_counter == 15 && state->yellow_detected == 0) {
				printf("yellow detected!\n");
				state->yellow_detected = 1; 				
			}
		}

		else {
			if(state->yellow_detected == 1){
				state->visual_yellow_counter -= 1;
				printf("after_yellow: %d\n",state->visual_yellow_counter );
			}
		}
	}


	// STATE 5: play the audio of finding the poster: green */ 
	else if (state->visual_state == 5) {
		velocity[0] = 0;
		velocity[1] = 0;
		printf("play found_green.wav\n");
		audio_play("found_green.wav");
		usleep(500000);
		state->visual_state = 0;
	}	
	
	// STATE 6: play the audio of finding the poster: pink */ 
	else if (state->visual_state == 6) {
		velocity[0] = 0;
		velocity[1] = 0;
		printf("play found_pink.wav\n");
		audio_play("found_pink.wav");
		usleep(500000);
		state->visual_state = 0;
	}		
	
	// STATE 7: play the audio of finding the poster: yellow */ 
	else if (state->visual_state == 7) {
		velocity[0] = 0;
		velocity[1] = 0;
		printf("play found_yellow.wav\n");
		audio_play("found_yellow.wav");
		usleep(500000);
		state->visual_state = 0;
	}		

	// STATE 8: play the introductory audio */ 
	else if (state->visual_state == 8) {
		velocity[0] = 0;
		velocity[1] = 0;
		printf("play introduction.wav\n");
		audio_play("introduction.wav");
		state->visual_state = 1;
		usleep(2000000);	
	}	
	
	state->visual_last_x_red = red[0];
	state->visual_last_y_red = red[1];	
	state->visual_last_x_green = green[0];
	state->visual_last_y_green = green[1];
	state->visual_last_x_pink = pink[0];
	state->visual_last_y_pink = pink[1];
	state->visual_last_x_yellow = yellow[0];
	state->visual_last_y_yellow = yellow[1];
}








/* allow the robot walk along a wall */
void wall_back(robotState *state,int *velocity ) {
	int front;
	int left;
	int right;
	int front_laser[3] = {11, 12, 13};
	int right_laser[6] = {3, 4, 5, 6, 7, 8};
	int left_laser[6] = {16, 17, 18, 19, 20, 21};
	int right_laser_small[3] = {20, 21, 22};
	velocity[0] = 0;
	velocity[1] = 0;
	
	front = check_shortest_laser(front_laser, 3, state);
	left = check_shortest_laser(left_laser, 6, state);
	right = check_shortest_laser(right_laser, 6, state);
	
	// if far from front, left, and right
	if (front >= 1000 && left >= 1000 && right >= 1000) {
		velocity[0] = front + 10;
		velocity[1] = 0;
		printf("--- too far from front ---\n");
	}
	
	else {
		// if front is the most hindered direction
		if (front <= 1000) {
			printf("should turn\n");
			if(right<=left){
				velocity[0] = 0;
				velocity[1] = 40;
			}
			
			else{
				velocity[0] = 0;
				velocity[1] = -40;
			}	
		}
		
		else if (left < right) {
			printf("follow left wall\n");
			float *wallValue = getWall(state->laser_data_raw, 2);
				if(wallValue[0]<750){
					if(wallValue[1]<=-0.5){
						velocity[0] = 0;
						velocity[1] = -40;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = 0;
					}
					else {
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40+5;
					}	
				}
				
				else if(wallValue[0]<1000){
					if(wallValue[1]<=-0.5){
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0){
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40+5;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40+5;
					}	
				}	
				
				else{
					if(wallValue[1]<=-0.5){
						velocity[0] = front+10;
						velocity[1] = -40;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = 0;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = 0;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40;
					}	
				}
		delete []wallValue;	
		}			
				
		else {
			printf("follow right wall\n");
			float *wallValue = getWall(state->laser_data_raw, 0);
				if(wallValue[0]<750){
					if(wallValue[1]<=-0.5){
						velocity[0] = 0;
						velocity[1] = -40;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = 0;
					}
					else {
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40+5;
					}	
				}
				
				else if(wallValue[0]<1000){
					if(wallValue[1]<=-0.5){
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0){
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40-5;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = wallValue[1]*40+5;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40+5;
					}	
				}	
				else{
					if(wallValue[1]<=-0.5){
						velocity[0] = front+10;
						velocity[1] = -40;
					}
					else if(wallValue[1]<=0){
						velocity[0] = wallValue[0];
						velocity[1] = 0;
					}
					else if(wallValue[1]<=0.5){
						velocity[0] = front+10;
						velocity[1] = 0;
					}
					else {
						velocity[0] = 0;
						velocity[1] = wallValue[1]*40;
					}	
				}
		delete []wallValue;	
		}	
	}
	return;
}



/* detect a face and interact with the person */
void visualFace(robotState *state, int *velocity) {
	int front_laser[3] = {11, 12, 13};
	int front;

	printf("visual: x: %d, y: %d\n", datahand[0], datahand[1]);
	state->visual_target_x = datahand[0];
	state->visual_target_y = datahand[1];
	front = check_shortest_laser(front_laser, 3, state);
	printf("front: %d\n", front);
	
	velocity[0] = 0;
	velocity[1] = 0;
	
	// STATE wander: searching the target
	if (state->visual_state == 0) {
		wander(state, velocity);
		
		if (state->visual_target_x != state->visual_last_x || state->visual_target_y != state->visual_last_y) {
			state->visual_state = 1;
		}
	}
	
	// STATE rotate: rotating towards the target
	if (state->visual_state == 1) {
		if (state->visual_target_x < 130 && state->visual_target_x > 0 ) {
			velocity[1] = (150 - state->visual_target_x + 10) * 3;
			state->visual_track_counter = 0;
			return;
		}
		else if (state->visual_target_x > 170 && state->visual_target_x < 300) {
			velocity[1] = (150 - state->visual_target_x - 10) * 3;
			state->visual_track_counter = 0;
			return;
		}
		else if (state->visual_target_x > 130 && state->visual_target_x < 170) {	
			state->visual_track_counter = state->visual_track_counter + 1;
			if (state->visual_track_counter >= 10) {
				state->visual_state = 2;
				state->visual_track_counter = 0;
			}
		}
	}
	
	// STATE rotate: following the target and keeping a specific distance from the target
	else if (state->visual_state == 2) {
	
		if (front > 1500) {
			velocity[0] = front - 1500 + 30;
		}
		else if (front < 1000) {
			velocity[0] = front - 1000 - 30;
		}	
		else {
			state->visual_state = 3;
			return;
		}
	
		if (state->visual_target_x < 130 && state->visual_target_x > 0 ) {
			state->visual_rotate_counter = state->visual_rotate_counter + 1;
			if (state->visual_rotate_counter >= 2) {
				state->visual_state = 1;
				state->visual_rotate_counter = 0;
			}
		}
		else if (state->visual_target_x > 170 && state->visual_target_x < 300) {
			state->visual_rotate_counter = state->visual_rotate_counter + 1;
			if (state->visual_rotate_counter >= 2) {
				state->visual_state = 1;
				state->visual_rotate_counter = 0;
			}
		}
		else if (state->visual_target_x > 130 && state->visual_target_x < 170) {
			state->visual_rotate_counter = 0;
		}
	}
	
	// STATE greet: rotating to greet people
	else if (state->visual_state == 3) {
		state->visual_greet_counter = state->visual_greet_counter + 1;
		velocity[0] = (state->visual_target_x - 150) * 30;
		
		// after finishing greeting, return back to wander state
		if (state->visual_greet_counter == 25) {
			state->visual_state = 0;
			state->visual_greet_counter = 0;
		}
	}
	state->visual_last_x = datahand[0];
	state->visual_last_y = datahand[1];
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
void RobotStateInitialize(robotState *state, char *argv[], long *State) {
	int i;
	
	state->laser_data_raw = NULL;
	state->laser_data_virtual = NULL;
	state->waypoints_x = NULL;
	state->waypoints_y = NULL;
	state->waypoints_orientation = NULL;
	state->waypoints_distance = NULL;

	if ( strcmp(argv[1], "rotate") == 0 ) {
		state->current_state = 0;
		state->current_goal_angle = atoi(argv[2]);
		state->current_goal_angle = state->current_goal_angle % 360;	
		state->current_goal_angle = 1000 * state->current_goal_angle * M_PI / 180;	
	}
	else if ( strcmp(argv[1], "move") == 0 ) {
		state->current_state = 1;
		state->current_goal_distance = atoi(argv[2]) * 1000;
	}

	else if ( strcmp(argv[1], "locate") == 0 ) {
		state->current_state = 2;
		WayPointFileReading(state);
		WayPointDistanceOrientationCalculate(state);
		state->current_goal_angle = state->waypoints_orientation[0];
		state->current_goal_distance = state->waypoints_distance[0];
		state->index_waypoints = 0;
		state->waypoints_if_dis_ori = 0;
	}

	else if ( strcmp(argv[1], "wall") == 0 ) {
		state->current_state = 3;
	}
	else if ( strcmp(argv[1], "maze") == 0 ) {
		state->current_state = 4;
	}
	else if ( strcmp(argv[1], "wander") == 0 ) {
		state->current_state = 5;
	}
	else if ( strcmp(argv[1], "track") == 0 ) {
		state->current_state = 6;
	}
	else if ( strcmp(argv[1], "visual") == 0 ) {
		state->current_state = 7;
		state->visual_detection = atoi(argv[2]);
// 		printf("ID: %d", state->visual_detection);
		visualOn(state->visual_detection);
		state->visual_target_x = 0;
		state->visual_target_y = 0;
		state->visual_state = 0;
		state->visual_rotate_counter = 0;
		state->visual_track_counter = 0;
		state->visual_wander_counter = 0;
		state->visual_greet_counter = 0;
	}	
	else if ( strcmp(argv[1], "face") == 0 ) {
		state->current_state = 8;
		state->visual_detection = atoi(argv[2]);
		visualOn(state->visual_detection);
		state->visual_target_x = 0;
		state->visual_target_y = 0;
		state->visual_state = 0;
		state->visual_rotate_counter = 0;
		state->visual_track_counter = 0;
		state->visual_wander_counter = 0;
		state->visual_greet_counter = 0;
		state->visual_last_x = 0;
		state->visual_last_y = 0;		
	}
	else if ( strcmp(argv[1], "color") == 0 ) {
		state->current_state = 9;
		visualOn(1);
		state->visual_target_x = 0;
		state->visual_target_y = 0;
		state->visual_state = 0;
		state->visual_rotate_counter = 0;
		state->visual_track_counter = 0;
		state->visual_wander_counter = 0;
		state->visual_greet_counter = 0;
		state->visual_last_x = 0;
		state->visual_last_y = 0;	
		state->visual_last_x_green = 0;
		state->visual_last_y_green = 0;
		state->visual_last_x_pink = 0;
		state->visual_last_y_pink = 0;
		state->visual_last_x_yellow = 0;
		state->visual_last_y_yellow = 0;
		state->visual_last_x_red = 0;
		state->visual_last_y_red = 0;
		state->pink_detected = 0;
		state->green_detected = 0;
		state->yellow_detected = 0;
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
	state->pink_detected = 0;
	state->green_detected = 0;
	state->yellow_detected = 0;
	
}


 /*
 	evaluate the current sensors; 
	evaluate internal robot status;
	evalluate current state; 
	set the next state; 
	set goals of the robot. 
*/
void RobotStateUpdate(robotState *state, long *State) {
	state->state = State;
	state->laser_data_raw = read_data();
	state->laser_data_virtual = group_data(state->laser_data_raw);
}


/* take in the current state and any other status information and determine the appropriate velocities for the robot */
void RobotNavigation(robotState *state, int *velocity) {
	switch(state->current_state) {
		case 0:
		 	rotate(state, velocity);
			break;
		case 1:
		 	move(state, velocity);
			break; 
		case 2:
		 	locate(state, velocity);
			break; 
		case 3:
		 	wall(state,velocity);
			break; 
		case 5:
		 	wander(state,velocity);
			break; 
		case 7:
		 	visual(state,velocity);
		 	IPC_listenClear(100);
			break; 
		case 8:
		 	visualFace(state,velocity);
		 	IPC_listenClear(100);
			break; 
		case 9:
		 	visualColor(state,velocity);
		 	IPC_listenClear(100);
			break; 
		case 10:
			break;	
	}
}


/* safe check of the rotational velocity (w) and transitional verlocity (v) */
int *RobotAction(robotState *state, int *velocity) {
	int translate, rotate, rotate1, translate1;

	if (velocity[0] > 100) {
		translate = 100;
		rotate = translate * velocity[1] / velocity[0];
	}
	else if (velocity[0] < -100) {
		translate = -100;
		rotate = translate * velocity[1] / velocity[0];
	}
	else {
		translate = velocity[0];
		rotate = velocity[1];
	}

	if (rotate >120) {
		rotate1 = 120;
		translate1 = rotate1 * translate / rotate;
		rotate = rotate1;
		translate = translate1;
	}
	else if (rotate < -120) {
		rotate1 = -120;
		translate1 = rotate1 * translate / rotate;
		rotate = rotate1;
		translate = translate1;
	}
	vm(translate, rotate);
}


/* robot main loop */
int main(int argc, char *argv[]) {
	if (argc == 1) {
		printf("ERROR: Please enter an instruction!\n");
		return 0;
	}
	
	robotState *state;
	long *State;
	int *velocity;
	char c;
	state = (robotState *)malloc(sizeof(robotState));
	velocity = (int *) malloc(sizeof(int) * 2);
	State = (long *) malloc(sizeof(long) * NUM_STATE);
	
	/* ---------------------------------------- Start Setup Robot ---------------------------------------- */
	connectRobot(State, MAGE_MODEL_MAGELLAN, (char *)"/dev/ttyUSB0"); /* connect (serial line) with the robot */
	irOn(); /* start up IRs */
	sonarOn(); /* start up Sonars */
	signal( SIGINT, sighandler ); /* catch cntl-c to get a clean shutdown */
	vm(0,0); /* set the speed */
	RobotStateInitialize(state, argv, State);
	usleep(1000000); /* wait for things to start up */

	/* ----------------------------------------- End Setup Robot ------------------------------------------ */

	while (state->current_state != 10) {
        RobotStateUpdate(state, State);
        RobotNavigation(state, velocity);	
 		RobotAction(state, velocity);	
		read(0, &c, 1);
		if((c == 'q') || (c == 'Q')) {
			break;
		}
	}

	/* ----------------------------------------- Clean Up Robot ------------------------------------------ */
	if (state->current_state == 7 || state->current_state == 8 || state->current_state == 9) {
		visualOff();
	}
	RobotStateFree(state);
	usleep(100000); 
	vm(0,0);
	disconnectRobot(); /* disconnect the robot */
	free(state);
	free(velocity);
	free(State);
	return 0;
}





