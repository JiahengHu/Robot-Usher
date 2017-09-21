/*
	19/02/2017
	Jiaheng Hu
	Liwei Jiang
	CS363 S17
	track.c
	Project3
	
	  This program will use the laser's read in to try to figure out the equation of
	  the line in front of it.
	
The default parameters:
	Y axis: The direction the robot is facing is positive
	X axis: The right hand side of the robot is positive
	% of inlier: 0.5
	# of points: 2
	% of success: 0.98
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include <flexiport/flexiport.h>
#include <vector>
#include <time.h>
// #include <codecogs/maths/approximation/regression/linear.h>
// #include <cmath>
// #include <iomanip>
using namespace std;

#define pi 3.14159265358979323846

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

int main(int argc, char *argv[]) {
	char port[256];
	char portOptions[256];
	unsigned int baud = 19200;
	int speed = 0;

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

		// Set the baud rate
		/*
		try
			{
				laser.set_baud(baud);
			}
		catch(hokuyoaist::BaudrateError &e)
			{
				std::cerr << "Failed to change baud rate: " << e.what() << '\n';
			}
		catch(hokuyoaist::ResponseError &e)
			{
				std::cerr << "Failed to change baud rate: " << e.what() << '\n';
			}
		catch(...)
			{
				std::cerr << "Failed to change baud rate\n";
			}
		*/
				
		// Set the motor speed
		try
			{
				laser.set_motor_speed(speed);
			}
		catch(hokuyoaist::MotorSpeedError &e)
			{
				std::cerr << "Failed to set motor speed: " << e.what() << '\n';
			}
		catch(hokuyoaist::ResponseError &e)
			{
				std::cerr << "Failed to set motor speed: " << e.what() << '\n';
			}		

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
		
		//calculate a and b by linear regression
		printf("max: %d\n", maxnum);
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
		
		printf("y = %f x + %f\n", a, b);
			
		//Maths::Regression::Linear A(maxnum, pointset[0],pointset[1]);

		// turn the laser off
		//		laser.set_power(false);
		laser.close();
		
		delete[] box;
	}
	catch(hokuyoaist::BaseError &e) {
		std::cerr << "Caught Exception: " << e.what() << '\n';
		return(-1);
	}

	return(0);
}