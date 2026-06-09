// #ifndef JOYSTICKSERIAL_H
// #define JOYSTICKSERIAL_H

// #include <string>
// #include <thread>
// #include <mutex>
// #include "serialport.h"
// #include <sys/stat.h>

// #define MSG_ID 0x55AA

// namespace JOY {
// /* rs432 serial parameter */
// enum Rs432Para_E {
//     BAUDRATE = 115200,
//     DATA_BITS = 8,
//     START_BITS = 1,
//     STOP_BITS = 1,
// };
// /* 按钮状态 */
// enum BTNSTA_E{
//     RELEASE = 0,  // 弹起
//     PRESS,        // 按下
// };

// #pragma pack(1)
// struct BtnType1_st{
//     u_char emerge:1; //急停位
//     u_char leftStick1:1;
//     u_char leftStick2:1;
//     u_char rightStick1:1;
//     u_char rightStick2:1;
//     u_char F1:1;
//     u_char F5:1;
//     u_char F2:1;
// };
// struct BtnType2_st{
//     u_char F6:1;
//     u_char F3:1;
//     u_char F7:1;
//     u_char F4:1;
//     u_char F8:1;
//     u_char res:3;
// };

// struct JoyProtocol_ST{
//     u_short headId; //帧头
//     u_char updateFlag;
//     u_char btnRes;
//     BtnType1_st btnGroup1;
//     BtnType2_st btnGroup2;
//     u_short status;
//     short leftAxisY;
//     short leftAxisX;
//     short rightAxisY;
//     short rightAxisX;
//     u_char reserved[14];
//     u_short check; //校验位
// };
// //所有按键
// struct JoyData_ST{
//     bool isOnline = true;
//     bool isinit = true;
//     BTNSTA_E leftStick1 = RELEASE;
//     BTNSTA_E leftStick2 = RELEASE;
//     BTNSTA_E rightStick1 = RELEASE;
//     BTNSTA_E rightStick2 = RELEASE;
//     BTNSTA_E emergeBtn = RELEASE;
//     BTNSTA_E F1 = RELEASE;
//     BTNSTA_E F2 = RELEASE;
//     BTNSTA_E F3 = RELEASE;
//     BTNSTA_E F4 = RELEASE;
//     BTNSTA_E F5 = RELEASE;
//     BTNSTA_E F6 = RELEASE;
//     BTNSTA_E F7 = RELEASE;
//     BTNSTA_E F8 = RELEASE;
//     BTNSTA_E engineState = RELEASE;//映射为发动机状态
//     short leftAxisY = 0;
//     short rightAxisX = 0;
// };

// #pragma pack()
// }

// class JoyStickSerial
// {
// public:
//     JoyStickSerial();
//     ~JoyStickSerial();
    
//     bool InitJoySerial(const std::string &devName);
    
//     JOY::JoyData_ST getJoyData() {
//         std::lock_guard<std::mutex> lock(mMtx);
//         return mJoyData;
//     }

// private:
//     void RecvDataThread();
//     bool isDevExit(const std::string& dev);
//     void setJoyData(JOY::JoyData_ST data) {
//         std::lock_guard<std::mutex> lock(mMtx);
//         mJoyData = std::move(data);
//     }

// private:
//     SerialPort *mSerialPort;
//     std::thread mRevThread;
//     int mFd = -1;
//     bool bCapStatus;
//     std::string mDevName;

//     std::mutex mMtx;
//     JOY::JoyData_ST mJoyData;
// };

// #endif // JOYSTICKSERIAL_H


#ifndef JOYSTICKSERIAL_H
#define JOYSTICKSERIAL_H

#include <string>
#include <thread>
#include "serialport.h"
#include <iostream>

typedef double  float64;
typedef float  float32;
typedef int int32;
typedef unsigned int uint32;
typedef short int16;
typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed char int8;

struct RemoteControl{
    uint16 header;
    uint8 update_sign;
    uint8 light[3];
    int16 updown1;
    int16 leftright1;
    int16 updown2;
    int16 leftright2;
    uint16 correct; 
};
// 假设的JOY命名空间
namespace JOY {
    enum Rs432Para_E {
        BAUDRATE = 115200,
        DATA_BITS = 8,
        START_BITS = 1,
        STOP_BITS = 1,
    };
/* 按钮状态 */
    enum BTNSTA_E{
        RELEASE = 0,  // 弹起
        PRESS,        // 按下
    };
};

class JoyStickSerial
{
public:
    JoyStickSerial();
    ~JoyStickSerial();
    
    // 初始化串口
    bool InitJoySerial(const std::string &devName);

    RemoteControl getData(){
        return remoteControl;
    };
    
private:
    // 数据接收线程函数
    void RecvDataThread();
    
    // 检查设备是否存在
    bool isDevExit(const std::string& dev);
    
private:
    SerialPort* mSerialPort;  // 串口对象指针
    int mFd;                  // 文件描述符
    bool bCapStatus;          // 捕获状态
    std::thread mRevThread;   // 接收线程
    std::string mDevName;     // 设备名称
    RemoteControl remoteControl;
};

#endif // JOYSTICKSERIAL_H