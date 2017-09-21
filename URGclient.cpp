#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "socketUtil.h"


int *read_data() {
	int sock;
	char serverName[128];
	char clientName[128];
	char message[512];
	int N;
	int *data = NULL;
	int i;
		
// 	system("rm /tmp/socket-urglaser-server");
// 	system("rm /tmp/socket-urglaser-client");
	
	strcpy( serverName, "/tmp/socket-urglaser-server" );
	strcpy( clientName, "/tmp/socket-urglaser-client" );

	sock = makeFilenameSocket( clientName );

	// send request message
	strcpy( message, "I" );
	N = sendFilenameSocket( sock, serverName, message, strlen(message)+1 );

	// read the image
	data = (int *)readFilenameData( sock, &N );
	
	if(data != NULL)  {
		for(i=0;i<341;i++) {
			if (data[i] == 0) {
				data[i] = 3000;
			}
			//printf("data %d: %d \n", i, data[i]);
		}
	}
	
	closeFilenameSocket( sock, clientName );

	return(data);
}

// 
// /* free data */
// void free_data(int *data) {
// 	if(data != NULL) {
// 		free(data);
// 	}
// }

// 
// int main(int argc, char *argv[]) {
// 	int i;
// 	int *data;	
//  	data = read_data();
// 	free_data(data);	
//  	return(0);
//  	
// }








