
#ifndef SERVERUDP3_H
# define SERVERUDP3_H

#include <iostream>

//#include <system-dynamics.h>
#//include <integrate-dynamics.h>
//#include <controller.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <Eigen/Core>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
//#include <serverudp.h>
//#include <clientudp.h>
 #include "port.h"
 
using namespace std;
using namespace Eigen;


#define BUFSIZE 2048

class ServerUDP
  {
    public:
            
            ServerUDP();
        
            
            virtual ~ServerUDP();
        
            bool server_start();
            bool server_send(char* pBuffer);
            bool server_recv(char* pBuffer);
            
            
            
    private:
          
          int _sFd, n;   //Socket
        
            
          struct sockaddr_in myaddr;	/* our address */
        	struct sockaddr_in remaddr;	/* remote address */
        	socklen_t addrlen;		/* length of addresses */
        	int recvlen;			/* # bytes received */
        	int fd;				/* our socket */
        	int msgcnt;			/* count # of messages we received */
        	//char buf[BUFSIZE];	/* receive buffer */
    
  };
  
  
    
  
#endif
