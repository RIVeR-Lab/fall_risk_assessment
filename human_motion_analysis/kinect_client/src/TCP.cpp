#include "TCP.hpp"
#include <iostream>

using namespace std;

TCP::TCP(char * _port, char * _addr)
{
	port = _port;
	addr = _addr;

	//Ensure that servinfo is clear
	memset(&hints, 0, sizeof hints); // make sure the struct is empty

	//setup hints
	hints.ai_family = AF_UNSPEC; // don't care IPv4 or IPv6
	hints.ai_socktype = SOCK_STREAM; // TCP stream sockets

	//Setup the structs if error print why
	int res;
	if ((res = getaddrinfo(addr,port,&hints,&servinfo)) != 0)
	{
		fprintf(stderr,"getaddrinfo: %s\n", gai_strerror(res));
	}

	//setup the socket
	if ((s = socket(servinfo->ai_family,servinfo->ai_socktype,servinfo->ai_protocol)) == -1)
	{
		perror("client: socket");
	}
}

TCP::~TCP()
{
}

int TCP::Connect()
{
  //This goes into the send/rcv loop
  //Connect
  if (connect(s,servinfo->ai_addr, servinfo->ai_addrlen) == -1)
    {
      close (s);
      perror("Client Connect");
      return 1;
    }
  
  //We dont need this anymore
  freeaddrinfo(servinfo);

  // no error
  return 0;  
}

bool TCP::charSearch(char *toSearch, char *searchFor)
{
	int len = strlen(toSearch);
	int forLen = strlen(searchFor); // The length of the searchfor field

	//Search through each char in toSearch
	for (int i = 0; i < len;i++)
	{
		//If the active char is equil to the first search item then search toSearch
		if (searchFor[0] == toSearch[i])
		{
			bool found = true;
			//search the char array for search field
			for (int x = 1; x < forLen; x++)
			{
				if (toSearch[i+x]!=searchFor[x])
				{
					found = false;
				}
			}

			//if found return true;
			if (found == true)
				return true;
		}
	}

	return 0;
}

bool TCP::Send(char *msg)
{//Send some data
	//Send some data
	int len = strlen(msg);
	int bytes_sent = send(s,msg,len,0);

	if (bytes_sent == 0)
		return false;
	else
		return true;
}

void TCP::Receive(int numbytes, char * buf)
{
  cout << "Here! " << numbytes << endl;
}

