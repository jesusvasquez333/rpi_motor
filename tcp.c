#include <stdio.h>      
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <netinet/tcp.h>
#include <stdlib.h>     
#include <string.h>     
#include <unistd.h>    
#include <stdio.h>  
#include <stdlib.h>

#define MAXPENDING 1000

void die(char *errorMessage)
{
  perror(errorMessage);
  exit(1);
}

int main(int argc, char *argv[])
{
  int servSock;                    /* Socket descriptor for server */
  int clntSock;                    /* Socket descriptor for client */
  struct sockaddr_in echoServAddr; /* Local address */
  struct sockaddr_in echoClntAddr; /* Client address */
  unsigned short echoServPort;     /* Server port */
  unsigned short quickAck;
  unsigned int clntLen;            /* Length of client address data structure */
  int value = 1;
  char mesg[1000];
  int n;


  if (argc != 2)     /* Test for correct number of arguments */
  {
    fprintf(stderr, "Usage:  %s <Server Port>\n", argv[0]);
    exit(1);
  }

  echoServPort = atoi(argv[1]);  /* First arg:  local port */

  /* Create socket for incoming connections */
  if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    die("socket() failed");

  /* Construct local address structure */
  memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
  echoServAddr.sin_family = AF_INET;                /* Internet address family */
  echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
  echoServAddr.sin_port = htons(echoServPort);      /* Local port */

  /* Bind to the local address */
  if (bind(servSock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
    die("bind() failed");

  /* Mark the socket so it will listen for incoming connections */
  if (listen(servSock, MAXPENDING) < 0)
    die("listen() failed");

  for (;;) /* Run forever */
  {
    /* Set the size of the in-out parameter */
    clntLen = sizeof(echoClntAddr);

    printf("Waiting for client...\n");

    /* Wait for a client to connect */
    if ((clntSock = accept(servSock, (struct sockaddr *) &echoClntAddr, &clntLen)) < 0)
      die("accept() failed");

    /* clntSock is connected to a client! */

    printf("Connected to client %s\n", inet_ntoa(echoClntAddr.sin_addr));

	for (;;) /* Run forever */
	{
		n = recvfrom(clntSock,mesg,1000,0,(struct sockaddr *)&echoClntAddr,&clntLen);
		
		if (n == 0)
			break;
			
		sendto(clntSock,mesg,n,0,(struct sockaddr *)&echoClntAddr,sizeof(echoClntAddr));
		printf("-------------------------------------------------------\n");
		mesg[n] = 0;
		printf("Received the following:\n");
		printf("%s\n",mesg);
		printf("-------------------------------------------------------\n");
	}

  }
}
