#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h> 
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "../../include/Mage.h"

int main(int argc, char *argv[])
{
  long State[NUM_STATE]; // State is where Mage dumps all of its info
  connectRobot(State);   // Connect (serial line) with the robot 
  irOn(); // Start up IRs
  sonarOn(); // Start up Sonars

  while (State[STATE_BUMPER] != 1) { // front bumper
    
    if ((State[STATE_SONAR_0] < 400) || // front sonar
	(State[STATE_SONAR_2] < 400) || // front left
	(State[STATE_SONAR_14] < 400)){  // front right
      vm(0, 300); // turn
    } else {
      vm (200, 0); // go straight
    }
    usleep(50000); 
  }
  disconnectRobot(); // Disconnect and shut down sensors
  return 0;
}







