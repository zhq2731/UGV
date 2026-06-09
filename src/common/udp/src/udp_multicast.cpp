#include "udp/udp_multicast.h"
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <cstring>
#include <iostream>

// ----------------- UDPMulticast 基础实现 -----------------

UDPMulticast::UDPMulticast(const std::string& multicast_group, int port)
    : multicast_group_(multicast_group), port_(port), sockfd_(-1) {
    
    // 初始化组播地址结构
    memset(&multicast_addr_, 0, sizeof(multicast_addr_));
    multicast_addr_.sin_family = AF_INET;
    multicast_addr_.sin_port = htons(port);
    multicast_addr_.sin_addr.s_addr = inet_addr(multicast_group.c_str());
    //if (inet_pton(AF_INET, multicast_group_.c_str(), &multicast_addr_.sin_addr) != 1) {
       // throw std::runtime_error("Invalid multicast address: " + multicast_group_);
   // }
    
    //setupSocket(ttl, loopback);
}

UDPMulticast::~UDPMulticast() {
    commonCleanup();
}



void UDPMulticast::commonCleanup() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

// ----------------- MulticastSender 实现 -----------------

MulticastSender::MulticastSender(const std::string& multicast_group, int port,
                                   const std::string& bind_interface,unsigned char ttl,bool loopback)
    : UDPMulticast(multicast_group, port), bind_interface_(bind_interface)
{

	// 创建UDP套接字
	sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd_ < 0) {
		throw std::runtime_error("Socket creation failed");
	}

	// 设置TTL
	if (setsockopt(sockfd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl))) {
		commonCleanup();
		throw std::runtime_error("Failed to set multicast TTL");
	}

	// 设置回环
	unsigned char loop = loopback ? 1 : 0;
	if (setsockopt(sockfd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop))) {
		commonCleanup();
		throw std::runtime_error("Failed to set multicast loopback");
	}

	struct in_addr multicast_if;
	multicast_if.s_addr = inet_addr(bind_interface_.c_str());  
	if (setsockopt(sockfd_, IPPROTO_IP, IP_MULTICAST_IF, &multicast_if, sizeof(multicast_if)) == -1) {
        perror("setsockopt IP_MULTICAST_IF failed");
        close(sockfd_);
        return ;
    }

	
}


ssize_t MulticastSender::send(const std::string& message) const {
    return send(message.data(), message.size());
}

ssize_t MulticastSender::send(const void* data, size_t length) const {
    if (sockfd_ < 0) return -1;

	//std::cout <<"sned "<< length <<std::endl;
    return sendto(sockfd_, data, length, 0,
                (struct sockaddr*)&multicast_addr_, sizeof(multicast_addr_));
}

// ----------------- MulticastReceiver 实现 -----------------

MulticastReceiver::MulticastReceiver(const std::string& multicast_group, int port,
                                   const std::string& bind_interface)
    : UDPMulticast(multicast_group, port), 
      bind_interface_(bind_interface),
      async_running_(false) {

	  // 创建UDP套接字
	  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
	  if (sockfd_ < 0) {
		  throw std::runtime_error("Socket creation failed");
	  }
	  
	 /*
    // 设置地址重用
    int reuse = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse))) {
        throw std::runtime_error("Failed to set SO_REUSEADDR");
    }
     */
    // 绑定到指定接口和端口
    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(port_);
    ///bind_addr.sin_addr.s_addr = inet_addr(bind_interface.c_str());
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	/*
    if (inet_pton(AF_INET, bind_interface_.c_str(), &bind_addr.sin_addr) != 1) {
        throw std::runtime_error("Invalid bind interface: " + bind_interface_);
    }
    */
    
    if (bind(sockfd_, (struct sockaddr*)&bind_addr, sizeof(bind_addr))) {
        throw std::runtime_error("Failed to bind socket");
    }

    // 加入组播组
    struct ip_mreq mreq;
     mreq.imr_multiaddr.s_addr = inet_addr(multicast_group_.c_str());
     mreq.imr_interface.s_addr = inet_addr(bind_interface.c_str());
	 
    if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) {
        throw std::runtime_error("Failed to join multicast group");
    }
}

MulticastReceiver::~MulticastReceiver() {
    stopAsync();
    
    // 离开组播组
    if (sockfd_ >= 0) {
        struct ip_mreq mreq;
        if (inet_pton(AF_INET, multicast_group_.c_str(), &mreq.imr_multiaddr) == 1 &&
            inet_pton(AF_INET, bind_interface_.c_str(), &mreq.imr_interface) == 1) {
            setsockopt(sockfd_, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        }
    }
}

ssize_t MulticastReceiver::receive(char* buffer, size_t buf_size, int timeout_ms) const {
    std::string dummy;
    return receiveWithSender(buffer, buf_size, dummy, timeout_ms);
}

ssize_t MulticastReceiver::receiveWithSender(char* buffer, size_t buf_size,
                                           std::string& sender_ip, int timeout_ms) const {
    if (sockfd_ < 0) return -1;
    
    // 设置超时
    if (timeout_ms >= 0) {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        
        int ret = poll(fds, 1, timeout_ms);
        if (ret == 0) return 0; // 超时
        if (ret < 0) return -1; // 错误
    }
    
    // 接收数据
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    ssize_t recv_len = recvfrom(sockfd_, buffer, buf_size, 0,
                               (struct sockaddr*)&sender_addr, &sender_len);
    
    if (recv_len > 0) {
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
        sender_ip = ip_str;
    }
    return recv_len;
}

void MulticastReceiver::startAsync(
    std::function<void(const char*, size_t, const std::string&)> callback,
    std::function<void(const std::string&)> error_callback,
    size_t buffer_size) {
    
    if (async_running_) return;
    async_running_ = true;
    
    async_thread_ = std::thread([this, callback, error_callback, buffer_size]() {
        std::unique_ptr<char[]> buffer(new char[buffer_size]);
        
        while (async_running_) {
            std::string sender_ip;
            ssize_t recv_len = receiveWithSender(buffer.get(), buffer_size, sender_ip, 100);
            
            if (recv_len > 0) {
                callback(buffer.get(), recv_len, sender_ip);
            } else if (recv_len < 0 && error_callback) {
                error_callback("Receive error");
                break;
            }
            // recv_len == 0 表示超时，继续循环
        }
    });
}

void MulticastReceiver::stopAsync() {
    if (!async_running_) return;
    
    async_running_ = false;
    if (async_thread_.joinable()) {
        async_thread_.join();
    }
}
