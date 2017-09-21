/*
    03/27/2017
    CS363 S17
    Project4
    Liwei Jiang
    Jiaheng Hu
    robotStructure.h

    The state machine of robot navigation.
*/

#include <Mage.h>


typedef struct {
	/* the robot's state vector */
  long *state;
	/* the robot's current state */
	int current_state;
	
	/* the robot's raw laser reading */
	int *laser_data_raw;
	
	/* the robot's grouped virtual laser reading, total 24 groups */
	int *laser_data_virtual;
	
	/* the robot's initial x location, used for rotation and waypoint tracking */
	long init_x_location;
	
	/* the robot's initial y location, used for rotation and waypoint tracking */
	long init_y_location;
	
	/* the robot's initial orientation, used for rotation and waypoint tracking */
	long init_orientation;
	
	/* the distance the robot already traveled, used for move and waypoint tracking */
	int distance_traveled;

	/* an array of way following points x read from a txt file */
	int *waypoints_x;

	/* an array of way following points y read from a txt file */
	int *waypoints_y;

	/* an array of way following points orientation computed based on reading from a txt file */
	int *waypoints_orientation;

	/* an array of way following points distance computed based on reading from a txt file */
	int *waypoints_distance;

	/* a flag to decide if it is the stage to translate or to rotate: 0 rotate, 1 translate */
	int waypoints_if_dis_ori;

	/* the index of way following points */
	int index_waypoints;

	/* the number of way following points */
	int number_waypoints;

	/* the robot's current goal turning angle */
	int current_goal_angle;
	
	/* the robot's current goal distance */
	int current_goal_distance;
	
	/* the robot's current goal position */
	int current_goal_position[2];

	/* an array of x coordinate of points used for ransac */
	int *ransac_x;

	/* an array of y coordinate of points used for ransac */
	int *ransac_y;
	
	/* x location of the visual target */
	int visual_target_x;
	
	/* y location of the visual target */
	int visual_target_y;
	
	/* x location of the last visual target */
	int visual_last_x;
	
	/* y location of the last visual target */
	int visual_last_y;
	
	/* state of the visual target tracking */
	int visual_state;
	
	/* count how long has the robot been facing the visual target */
	int visual_rotate_counter;

	/* count how long has the robot been tracking the visual target */
	int visual_track_counter;
	
	/* count how long has the robot not been tracking the visual target, and then going to the wander state */
	int visual_wander_counter;
	
	/* count how long has the robot been greeting people */
	int visual_greet_counter;
	
	/* x location of the last visual target */
	int visual_last_x_green;
	
	/* y location of the last visual target */
	int visual_last_y_green;
		
	/* x location of the last visual target */
	int visual_last_x_pink;
	
	/* y location of the last visual target */
	int visual_last_y_pink;
	
	/* x location of the last visual target */
	int visual_last_x_yellow;
	
	/* y location of the last visual target */
	int visual_last_y_yellow;
	
	/* x location of the last visual target */
	int visual_last_x_red;
	
	/* y location of the last visual target */
	int visual_last_y_red;
	
	int visual_green_counter;
	int visual_pink_counter;
	int visual_yellow_counter;
	int visual_red_counter;
	
	/* whether to detect face or detect pink badges, 0 for pink badge; 24 face detection */
	int visual_detection;
	
	/* state of pink detection */
	int pink_detected;
	
	/* state of green detection */
	int green_detected;
	
	/* state of green detection */
	int yellow_detected;
	
} robotState;
























 









