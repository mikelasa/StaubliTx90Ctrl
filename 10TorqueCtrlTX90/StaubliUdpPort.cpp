#include "StaubliUdpPort.h"

#include <iostream>

#pragma comment(lib,"ws2_32.lib") 
#pragma warning(disable:4996) 

#define SERVER "192.168.1.3"  // or "localhost" - ip address of UDP server
#define BUFLEN 512  // max length of answer
#define PORT 8888  // the port on which to listen for incoming data

void StaubliUdpPort::open()
{
	WSADATA ws;
	std::printf("Initialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &ws) != 0)
	{
		std::printf("Failed. Error Code: %d", WSAGetLastError());
		throw std::string("WSAStartup not found");
	}
	std::printf("Initialised.\n");

	if ((client_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) // <<< UDP socket
	{
		std::printf("socket() failed with error code: %d", WSAGetLastError());
		throw std::string("could not open socket ");
	}

	// setup address structure
	memset((char*)&server, 0, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_port = htons(PORT);
	server.sin_addr.S_un.S_addr = inet_addr(SERVER);
}

void StaubliUdpPort::close()
{
	closesocket(client_socket);
	WSACleanup();
}

bool StaubliUdpPort::send(char* msg, int nBytes)
{
	if (sendto(client_socket, msg, nBytes, 0, (sockaddr*)&server, sizeof(sockaddr_in)) == SOCKET_ERROR)
	{
		return false;
	}
	return true;
}

bool StaubliUdpPort::recv(char* msg, int nBytes)
{
	int slen = sizeof(struct sockaddr_in);

	if (recvfrom(client_socket, msg, nBytes, 0, (struct sockaddr*)&server, &slen) == SOCKET_ERROR)
	{
		return false;
	}
	return true;
}
