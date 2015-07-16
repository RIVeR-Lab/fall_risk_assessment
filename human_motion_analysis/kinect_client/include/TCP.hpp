#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>


#ifndef TCP_HPP_
#define TCP_HPP_

class TCP
{
public:
  TCP(char * _port, char * _addr);
  virtual ~TCP();
  
  int Connect();
  bool charSearch(char *toSearch, char *searchFor);
  int s; //the socket descriptor
  
private:
  addrinfo hints, *servinfo;
  char *addr;
  char *port;
  
  
  bool Send(char *msg);
  void Receive(int numbytes, char * buf);
};

#endif

