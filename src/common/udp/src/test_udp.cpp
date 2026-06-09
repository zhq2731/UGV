#include "udp/udp_multicast.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    const std::string multicast_group = "239.255.255.250";
    const int port = 1900;
    
    try {
        // 示例1: 基本发送接收
        {
            MulticastSender sender(multicast_group, port);
            MulticastReceiver receiver(multicast_group, port);
            
            // 发送测试消息
            sender.send("Hello Multicast!");
            
            // 接收消息
            char buffer[1024];
            std::string sender_ip;
            ssize_t recv_len = receiver.receiveWithSender(buffer, sizeof(buffer), sender_ip, 1000);
            
            if (recv_len > 0) {
                std::cout << "Received from " << sender_ip << ": " 
                          << std::string(buffer, recv_len) << std::endl;
            }
        }
        
        // 示例2: 异步接收
        {
            MulticastSender sender(multicast_group, port);
            MulticastReceiver receiver(multicast_group, port);
            
            // 启动异步接收
            receiver.startAsync(
                [](const char* data, size_t len, const std::string& sender_ip) {
                    std::cout << "[Async] Received from " << sender_ip << ": " 
                              << std::string(data, len) << std::endl;
                },
                [](const std::string& error) {
                    std::cerr << "[Async] Error: " << error << std::endl;
                }
            );
            
            // 发送几条测试消息
            for (int i = 0; i < 3; ++i) {
                sender.send("Async message " + std::to_string(i));
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            // 给点时间处理最后的消息
            std::this_thread::sleep_for(std::chrono::seconds(1));
            receiver.stopAsync();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
