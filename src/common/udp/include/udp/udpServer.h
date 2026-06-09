#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include <iostream>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
#include <string.h>
//#include <sys/types.h>
#ifdef linux
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#endif
#include <vector>

#ifdef WIN32
#include <winsock2.h>
#endif

using namespace std;

class udpServer
{
public:
  udpServer();
  bool  setup(int port,std::string &ip);
  bool  addMulticastAddr(std::string multiAddr,std::string localAddr);
  bool  setTimeOut(int milliSecond);
  bool  send( char *data, int len, std::string sIp, unsigned short sPort);
  int   receive( char *recvBuf, int size);
  void  exit();

private:
#ifdef WIN32
  SOCKET mSock;
  WSADATA WSAData;
#elif linux
  int mSock;
#endif

  std::string mAddress;
  int mPort;
  struct sockaddr_in mClient; //服务器的地址等信息
};

#endif // UDPCLIENT_H
