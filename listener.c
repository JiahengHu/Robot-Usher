/*
  This program permits command-line control and communication with
  SVM.  The command-line arguments to Listener allow the user to turn
  on an operator, which is turned off when the program exits.

  It provides a good skeleton for people wishing to write applications
  that communicate with SVM.

  Listener uses the GCM library to initialize and terminate IPC
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ipc.h>
#include <SVM_VisionModule.h>
#include <GCM_IPC.h>
#include <GCM_Log.h>

void dataHandler(MSG_INSTANCE msgInstance, void *callData,void *clientData);
void dataHandler(MSG_INSTANCE msgInstance, void *callData,void *clientData) {
  SVM_Operator_Data *d = (SVM_Operator_Data *)callData;

  printf("green: (%d, %d)\r\n", d->location[0][0], d->location[0][1]);
  printf("pink: (%d, %d)\r\n", d->location[1][0], d->location[1][1]);
  printf("yellow: (%d, %d)\r\n", d->location[2][0], d->location[2][1]);

  IPC_freeData(IPC_msgFormatter(SVM_DATA_RESPONSE), callData);

  return;
}

int main(int argc, char *argv[]) {
  SVM_Operator_Command svmCommand;

  // command line arguments, initialized to reasonable defaults
  int operatorID = SVM_OP_Pink_Blob;
  int priority = 1;
  int ptzatt = SVM_PTZ_DEFAULT;
  float ptzpos[3] = {0.0, 1.571, 0.0};
  int arg = 0;
  int streaming = 250;
  int timing = SVM_TIMING_STOCHASTIC;
  int flags;
  int i;
  struct termios t, newt;
  char c;
  char *hname = NULL;
  
  // process the command line arguments
  for(i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      switch(argv[i][1]) {
      case 'h': // hostname
	if( argc > i+1 ) {
	  hname = argv[i+1];
	  i++;
	}
	break;
      case 'o': // operator
	if(argc > i+1) {
	  operatorID = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing operator argument\n");
	  exit(-1);
	}
	break;
	
      case 'p': // priority
	if(argc > i+1) {
	  priority = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing priority argument\n");
	  exit(-1);
	}
	break;

      case 'a': // arg
	if(argc > i+1) {
	  arg = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing arg argument\n");
	  exit(-1);
	}
	break;

      case 's': // streaming
	if(argc > i+1) {
	  streaming = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing streaming argument\n");
	  exit(-1);
	}
	break;

      case 't': // timing
	if(argc > i+1) {
	  timing = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing timing argument\n");
	  exit(-1);
	}
	break;

      case 'f': // ptzatt
	if(argc > i+1) {
	  ptzatt = atoi(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing ptzatt argument\n");
	  exit(-1);
	}
	break;

      case 'z': // ptzpos
	if(argc > i+1) {
	  ptzpos[0] = atof(argv[i+1]);
	  i++;
	  ptzpos[1] = atof(argv[i+1]);
	  i++;
	  ptzpos[2] = atof(argv[i+1]);
	  i++;
	}
	else {
	  printf("Missing ptzpos arguments (pan [rad], tilt [rad], zoom [1..127])\n");
	  exit(-1);
	}
	break;

      default:
	printf("Unable to process command line option '%s'\n", argv[i]);
	exit(-1);
      }
    }
    else {
      printf("Usage: Listener \n");
      printf("-h <hostname> \n");
      printf("-o <operator> \n");
      printf("-p <priority> \n");
      printf("-a <arg> \n");
      printf("-s <streaming> \n");
      printf("-t <timing>\n");
      printf("-f <ptz att>\n");
      printf("-z <ptz pan> <ptz tilt> <ptz zoom>\n");
      exit(-1);
    }
  }

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

  // enter the main loop
  while(read(0, &c, 1)) {

    // Every time through the main loop we let IPC listen for a message for 30ms
    // IPC_listenClear will call the message handler if a message arrives or is waiting.
    IPC_listenClear(30);

    if((c == 'q') || (c == 'Q')) 
      break;
  }

  fcntl(0, F_SETFL, flags);

  // turn off the operator we turned on earlier by setting the priority to zero
  svmCommand.priority = 0;
  IPC_publishData(SVM_COMMAND, &svmCommand);

  // terminate the IPC connection by calling the GCM function
  GCM_terminateIPC((char *)"Listener");

  tcsetattr(0, TCSAFLUSH, &t);

  printf("Terminating\n");

  return(0);
}
