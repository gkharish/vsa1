
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

//#include <native/task.h>
//#include <native/timer.h>

#include <string.h>
#include <sstream>
#include "serverudp3.h"
//#include "clientudp3.h"
#include "plantmodel.h"


  
    
    /* Server UDP program */
    
    ServerUDP::ServerUDP()
    {
    	if (SERVICE_PORT == 0) cout << "\n Invalid Server port number";//_inputport = inputport;
    }
    
    
    ServerUDP::~ServerUDP()
    {
    	close (fd);
    }
    
    
    bool ServerUDP::server_start()
    {
    	
    
        /* create a UDP socket */

    	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    		perror("cannot create socket\n");
    		return 0;
    	}
    
    	/* bind the socket to any valid IP address and a specific port */
    
    	memset((char *)&myaddr, 0, sizeof(myaddr));
    	myaddr.sin_family = AF_INET;
    	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    	myaddr.sin_port = htons(SERVICE_PORT);
    
    	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
    		perror("bind failed");
    		return 0;
    	}
    	addrlen = sizeof(remaddr);
    	cout <<  "\n" << "Server started";
    	
    	//write(1,"Server started on UDP: ",21);
    	return true;
    }
    
    
    bool ServerUDP::server_send(char* buf)
    {
        
        
        
    	cout << "\n Server sending response link position : " << buf;
    	
    	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, addrlen) < 0)
			perror("sendto");
    	
    	cout <<  " .... Server send done \n";
    	return true;
    }
    
    
    
    bool ServerUDP::server_recv(char* buf)
    {
    	
    	
    	cout << "\n waiting on port " <<  SERVICE_PORT << "\n";
		recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
		if (recvlen > 0) {
			buf[recvlen] = 0;
			cout << "\n received message  control value u :" << buf;
		}
		else
			cout << "\n uh oh - something went wrong!\n";
			
			
    	cout <<  "\n" << "...Server recv done";
    	return true;
    }
    