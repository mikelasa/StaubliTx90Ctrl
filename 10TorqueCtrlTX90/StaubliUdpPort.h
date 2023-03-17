#pragma once

#include <winsock2.h>


class StaubliUdpPort
{
protected:
	int client_socket;
	sockaddr_in server;
public:
	void open();
	void close();
	bool send(char* msg, int nBytes);
	bool recv(char* msg, int nBytes);
};

