#ifndef UDP_MULTICAST_H
#define UDP_MULTICAST_H

#include <string>
#include <functional>
#include <stdexcept>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class UDPMulticast {
public:
    /**
     * @brief 初始化UDP组播
     * @param multicast_group 组播地址(如"239.255.255.250")
     * @param port 端口号
     * @param ttl 生存时间(默认1，仅本地网络)
     * @param loopback 是否回环(默认true，本机可接收自己发送的消息)
     */
    UDPMulticast(const std::string& multicast_group, int port);
    
    virtual ~UDPMulticast();
    
    // 禁用拷贝和赋值
    UDPMulticast(const UDPMulticast&) = delete;
    UDPMulticast& operator=(const UDPMulticast&) = delete;

protected:
    std::string multicast_group_;
    int port_;
    int sockfd_;
    struct sockaddr_in multicast_addr_;
    
    void commonCleanup();
};

class MulticastSender : public UDPMulticast {
public:
    using UDPMulticast::UDPMulticast;

	MulticastSender(const std::string& multicast_group, int port,
                                   const std::string& bind_interface,unsigned char ttl,bool loopback);
    /**
     * @brief 发送组播消息
     * @param message 要发送的消息
     * @return 成功返回发送的字节数，失败返回-1
     */
    ssize_t send(const std::string& message) const;
    
    /**
     * @brief 发送二进制数据
     * @param data 数据指针
     * @param length 数据长度
     * @return 成功返回发送的字节数，失败返回-1
     */
    ssize_t send(const void* data, size_t length) const;
	
    std::string bind_interface_;
};

class MulticastReceiver : public UDPMulticast {
public:
    /**
     * @brief 接收器构造函数
     * @param multicast_group 组播地址
     * @param port 端口号
     * @param bind_interface 绑定接口IP(默认"0.0.0.0"表示所有接口)
     */
    MulticastReceiver(const std::string& multicast_group, int port, 
                     const std::string& bind_interface = "0.0.0.0");
    
    ~MulticastReceiver();
    
    /**
     * @brief 接收消息
     * @param buffer 接收缓冲区
     * @param buf_size 缓冲区大小
     * @param timeout_ms 超时时间(毫秒，0表示阻塞，-1表示不设置)
     * @return 成功返回接收的字节数，失败返回-1
     */
    ssize_t receive(char* buffer, size_t buf_size, int timeout_ms = 0) const;
    
    /**
     * @brief 接收消息并获取发送者地址
     * @param buffer 接收缓冲区
     * @param buf_size 缓冲区大小
     * @param sender_ip 输出参数，发送者IP
     * @param timeout_ms 超时时间(毫秒)
     * @return 成功返回接收的字节数，失败返回-1
     */
    ssize_t receiveWithSender(char* buffer, size_t buf_size, 
                            std::string& sender_ip, int timeout_ms = 0) const;
    
    /**
     * @brief 开始异步接收
     * @param callback 消息回调函数
     * @param error_callback 错误回调函数
     * @param buffer_size 内部缓冲区大小(默认4096)
     */
    void startAsync(
        std::function<void(const char*, size_t, const std::string&)> callback,
        std::function<void(const std::string&)> error_callback = nullptr,
        size_t buffer_size = 4096);
    
    /**
     * @brief 停止异步接收
     */
    void stopAsync();

private:
    std::string bind_interface_;
    bool async_running_;
    std::thread async_thread_;
};

#endif // UDP_MULTICAST_H
