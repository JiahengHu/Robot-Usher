/*
	Sends a quit message to the server
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "socketUtil.h"

int main(int argc, char *argv[]) {
	int sock;
	char serverName[128];
	char clientName[128];
	char message[512];
	int N;

	strcpy( serverName, "/tmp/socket-urglaser-server" );
	strcpy( clientName, "/tmp/socket-urglaser-client" );

	sock = makeFilenameSocket( clientName );

	// send request message
	strcpy( message, "Q" );
	N = sendFilenameSocket( sock, serverName, message, strlen(message)+1 );

	closeFilenameSocket( sock, clientName );

	return(0);
}
