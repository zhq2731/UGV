#include "cloud_comm/joystickserial.h"
#include <thread>
#include <unistd.h>
#include <cstring>
#include <sys/stat.h>

#define REUSE_F1 // F1代替两档旋钮

JoyStickSerial::JoyStickSerial()
    : mSerialPort(nullptr), bCapStatus(false), mFd(-1)
{
}

JoyStickSerial::~JoyStickSerial()
{
    if (mSerialPort != nullptr) {
        delete mSerialPort;
    }
    if (mFd > 0) {
        mSerialPort->UartClose(mFd);
    }
    mFd = -1;
    bCapStatus = false;
    if (mRevThread.joinable()) {
        mRevThread.join();
    }
}

bool JoyStickSerial::InitJoySerial(const std::string &devName)
{
    // std:: cout << "enterffff" << std::endl;
    mDevName = devName;
    mSerialPort = new SerialPort();
    mFd = mSerialPort->UartOpen(const_cast<char*>(mDevName.c_str()), JOY::Rs432Para_E::BAUDRATE, 
                               JOY::Rs432Para_E::DATA_BITS, JOY::Rs432Para_E::STOP_BITS, NO_CHECK);
    if (mFd > 0) {
        bCapStatus = true;
        mRevThread = std::thread(&JoyStickSerial::RecvDataThread, this);
        return true;
    }
    return false;
}

void JoyStickSerial::RecvDataThread()
{
    // std:: cout << "entereeeee" << std::endl;
    if (mSerialPort == nullptr) {
        return;
    }
    
    int nRead = 0;
    u_char rBuf[16];
    
    while (bCapStatus) {
        // 检测设备文件是否在线
        bool isOnline = isDevExit(mDevName);
        if (!isOnline) {
            usleep(100 * 1000);
            continue;
        }
        
        memset(rBuf, 0, sizeof(rBuf));
        nRead = mSerialPort->UartRead(mFd, reinterpret_cast<char*>(rBuf), sizeof(rBuf));
        if (nRead < 16) {
            continue;
        }

        memset(&remoteControl, 0, sizeof(remoteControl));
        memcpy(&remoteControl, rBuf, sizeof(remoteControl));
        


        // 这里添加你的数据处理逻辑
        // 例如: JOY::JoyProtocol_ST joyData;
        // memcpy(&joyData, rBuf, sizeof(joyData));
        // 处理接收到的数据...
    }
}

bool JoyStickSerial::isDevExit(const std::string& dev)
{
    struct stat fileStat;
    memset(&fileStat, 0, sizeof(struct stat));
    if (stat(dev.c_str(), &fileStat) == -1) {
        return false;
    }
    if (!S_ISCHR(fileStat.st_mode)) {
        return false;
    }
    return true;
}
