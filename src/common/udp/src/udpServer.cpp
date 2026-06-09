#include "udp/udpServer.h"
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
using namespace std;
udpServer::udpServer()
{
    mSock = -1;
    mPort = 0;
    mAddress = "";
    if(mSock == -1)
    {
        mSock = socket(AF_INET , SOCK_DGRAM , 0);               /*IPPROTO_UDP*/
        if (mSock == -1)
        {
            cout << "Could not create socket" << endl;
        }
    }
#ifdef WINDOWS
    if (WSAStartup(MAKEWORD(2, 2), &WSAData) != 0)
    {
        printf("初始化失败!");
        return;
    }
#endif
}

bool udpServer::setup(int port,std::string &ip){

    mPort = port;
	mAddress = ip;
    struct sockaddr_in server_addr;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    server_addr.sin_port = htons(mPort);
    bind(mSock, (struct sockaddr *) &server_addr, sizeof(server_addr));
    return true;
}

bool udpServer::addMulticastAddr(std::string multiAddr,std::string localAddr)
{

        /*-------------------------------------------------------------------------------------------------*/
        struct ip_mreqn mreq;
        mreq.imr_multiaddr.s_addr = inet_addr(multiAddr.c_str());	  //多播组ip地址 組播IP
        //  mreq.imr_multiaddr.s_addr = htonl(INADDR_ANY);
        mreq.imr_address.s_addr = inet_addr(localAddr.c_str());//本地ip地址
        mreq.imr_ifindex = 0; 							  // 接口索引 0 表示任意接口
        //                   /套接字  /IP级别     /在指定接口上加入组播组
        int ret = setsockopt(mSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(struct ip_mreqn));
        if (0 > ret)
        {
            perror("setsockopt multicast error");
            return false;
        }
        return true;
}
bool udpServer::setTimeOut(int milliSecond)
{
    struct timeval timeOut;
    timeOut.tv_sec = milliSecond / 1000;
    timeOut.tv_usec = (milliSecond- timeOut.tv_sec * 1000) * 1000;
    int ret = setsockopt(mSock, SOL_SOCKET,SO_RCVTIMEO,&timeOut,sizeof (timeval));
    if(ret < 0)
    {
#ifdef SPD_LOG
            spdlog::apply_all([&](std::shared_ptr<spdlog::logger> l) {
                l->error("udp set timeout error!"); });
#endif
            return  false;
    }
    return true;
}

bool udpServer::send(char data[], int len, std::string sIp, unsigned short sPort)
{

    memset(&mClient, 0, sizeof(mClient));
    mClient.sin_addr.s_addr = inet_addr(sIp.c_str());
    mClient.sin_family = AF_INET;
    mClient.sin_port = htons(sPort);
    if(mSock != -1 && len > 0) {
        int ret = sendto(mSock, data, len, 0, (struct sockaddr *)&mClient, sizeof(struct sockaddr));
    }
    else
        return false;
	
    return true;
}

int udpServer::receive( char recvBuf[], int size)
{
    int length = 0;
    memset(recvBuf, 0, size);
    struct sockaddr_in addr;
    int len = sizeof(addr);
#ifdef linux
    //std::cout <<"receive 1"<<std::endl;
    length = recvfrom(mSock, recvBuf, size, 0, (struct sockaddr *)&addr, (socklen_t *)&len);
    ///std::cout <<"receive 2"<<std::endl;
#endif
    //
#ifdef WIN32
    length = recvfrom(mSock, recvBuf, size, 0, (struct sockaddr *)&mClient, &len);
#endif
//        printf("!!!!!!!!!!!!!!!!!!recv:%s  len:%d\n", recvBuf, length);
//        qDebug() << "recv:" << recvBuf[0] << "len:" << length;
    return length;
}

void udpServer::exit()
{
#ifdef linux
    close(mSock);
#endif

#ifdef WIN32
    closesocket(mSock);
    WSACleanup();
#endif
}
