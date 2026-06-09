/******************************************************************************
 * Copyright (c) 2020-2021 中国航天科工集团二院206所 All rights reserved.
 *                              成都研发中心
 *文件说明：RS485通讯相关接口
 *作者：闫春秀
 *时间：2021-11-12
 *版权所有，侵权必究！
 ******************************************************************************/
#include <stdint.h>
typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
#include "cloud_comm/serialport.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <pthread.h>


#define UART3_FR_REG  (0x12103000)
#define UART2_FR_REG  (0x12102000)
#define UART4_FR_REG  (0x12104000)
#define SHUB0_UART0_FR_REG (0x18060000)
#define MAP_SIZE        0xFF

int speed_arr[] = { B2000000, B1500000, B921600, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                    B2000000, B1500000, B921600, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {2000000, 1500000, 921600, 115200, 38400,  19200,  9600,  4800,  2400,  1200,  300,
                    2000000, 1500000, 921600, 115200, 38400,  19200,  9600, 4800, 2400, 1200,  300, };

pthread_mutex_t uart_write_lock;
pthread_mutex_t uart_read_lock;

SerialPort::SerialPort()
{
    m_UartFd = -10; //szk modify bug 11.16 add m_UartFd initialize
}

void SerialPort::set_speed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if  (speed == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0) {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

int SerialPort::UartInit(int fd, int baud,int databits,int stopbits,int mParity)
{
    struct termios Opt;

    tcgetattr(fd,&Opt);
    set_speed(fd,baud);
    pthread_mutex_init(&uart_write_lock,NULL);
    pthread_mutex_init(&uart_read_lock,NULL);

    /* set databits */
    Opt.c_cflag &= (~CSIZE);
    switch (databits)
    {
    case 5:
        Opt.c_cflag |= CS5;
        break;
    case 6:
        Opt.c_cflag |= CS6;
        break;
    case 7:
        Opt.c_cflag |= CS7;
        break;
    case 8:
        Opt.c_cflag |= CS8;
        break;
    default:
        Opt.c_cflag |= CS8;
        printf("Unsupported data size/n");
        break;
    }

    /* Set the parity check */
    switch (mParity)
    {
        case NO_CHECK:
            Opt.c_cflag &= ~PARENB;   /* Clear parity enable */
            Opt.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case ODD_CHECK:
            Opt.c_cflag |= (PARODD | PARENB); /* Odd Checking */
            Opt.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case PARITY_CHECK:
            Opt.c_cflag |= PARENB;     /* Enable parity */
            Opt.c_cflag &= ~PARODD;   /* Even Checking */
            Opt.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        default:
            printf("Unsupported parity/n");
            Opt.c_cflag &= ~PARENB;
            Opt.c_cflag &= ~CSTOPB;
            break;
    }

    /* Set Stobits */
    switch (stopbits)
    {
        case 1:
            Opt.c_cflag &= ~CSTOPB;
            break;
        case 2:
            Opt.c_cflag |= CSTOPB;
           break;
        default:
             printf("Unsupported stop bits/n");
             return UART_FALSE;
    }
    Opt.c_cc[VTIME] = 0;
    Opt.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    Opt.c_cflag |= (CLOCAL | CREAD);

    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    Opt.c_lflag &= ~(ICANON | ISIG | ECHO | IEXTEN);

    Opt.c_oflag &= ~OPOST;
    Opt.c_oflag &= ~(ONLCR | OCRNL);

    Opt.c_iflag &=  ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    /* Write to the configuration, make the configuration take effect */
    if (tcsetattr(fd,TCSANOW,&Opt) != 0)
    {
        printf("SetupSerial!\n");
        close(fd);
        return UART_FALSE;
    }
    return fd;
}

int SerialPort::UartWrite(int uartFd,char * buf,int bufLen)
{
    int writelen = 0;
    int writesize;

    if(uartFd <0){
        return UART_FALSE;
    }
    pthread_mutex_lock(&uart_write_lock);
//    printf("wtrie data[%d]\n", bufLen);
    while (writelen != bufLen)
    {
        writesize = write(uartFd, buf + writelen, bufLen - writelen);
        writelen+=writesize;
    }

    pthread_mutex_unlock(&uart_write_lock);
    return writelen;
}

int SerialPort::UartRead(int uartFd,char *buff,int bufLen)
{
    int  nRead = 0;
    int ret = 0;

    if(uartFd<0)
        return -1;
    ret = myselect(uartFd,1000*1000,READ_SELECT);
    if(ret<0)
    {
        usleep(1000);
        return UART_FALSE;
    }
    else if(ret==0)
    {
        return UART_TIMEOUT;
    }
    pthread_mutex_lock(&uart_read_lock);
    while (1)
    {
        int readSize = 0;
        readSize = read(uartFd, &buff[nRead],bufLen-nRead);
        nRead = nRead + readSize;
        if(readSize <= 0  || nRead >= bufLen){
            pthread_mutex_unlock(&uart_read_lock);
//            printf("read end! [%d][%d]\n", readSize, nRead);
//            fflush(stdout);
            break;
        }
        usleep(5 * 1000);
    }
    pthread_mutex_unlock(&uart_read_lock);
    tcflush(uartFd, TCIFLUSH);
    return nRead;
}

int SerialPort::UartOpen(char* UartName,int baud,int databits,int stopbits,int mParity)
{
    int comFd=open(UartName, O_RDWR|O_NONBLOCK);//阻塞式读写
    int ret = 0;
    char cmdBuf[50]={0};
    ret = sprintf(cmdBuf,"stty -F %s %d",UartName,baud);
    cmdBuf[ret]=0x0;
    system(cmdBuf);
    if(comFd<=0)
    {
        printf("##############################ERROR! open %s err\n",UartName);
        return UART_FALSE;
    }
    if(UartInit(comFd,baud,databits,stopbits,mParity)<=0)
    {
        printf("uartPort init error");
        return UART_FALSE;
    }
    tcflush(comFd,TCIOFLUSH);
    return comFd;
}

void SerialPort::UartClose(int &uartFd)
{
    if(uartFd > 0){
        close(uartFd);
        uartFd = -1;
    }
}

int SerialPort::myselect(int sockfd, int usec, int mtype)
{
    struct timeval tv;
    fd_set readfds, writefds, exceptfds;
    int receive = 0;
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_ZERO(&exceptfds);
    tv.tv_sec = usec/1000000;
    tv.tv_usec = usec%1000000;
    FD_SET(sockfd, &exceptfds);
    if(mtype == READ_SELECT){
        FD_SET(sockfd, &readfds);
        receive = select(sockfd+1, &readfds, NULL, &exceptfds, &tv);//查询串口，再用read避免堵塞
    }
    else{
        FD_SET(sockfd, &writefds);
        receive = select(sockfd+1, NULL, &writefds, &exceptfds, &tv);
    }
    if(receive < 0){
        perror("select error:");
        return M_FALSE;
    }
    else if(receive == 0)
    {
        return M_TIMEOUT;
    }
    else{
        if(mtype == READ_SELECT){
            if(FD_ISSET(sockfd, &readfds) > 0)
                return M_TRUE;
        }
        else
        {
            if(FD_ISSET(sockfd, &writefds) > 0)
                return M_TRUE;
        }
        return M_FALSE;
    }
    return M_TRUE;
}


u_int16_t SerialPort::crc_packet(u_int8_t *data, int len)
{
    u_int16_t crc = 0x0000;

    while (len-- > 0)
        crc = crc_byte(crc, *data++);

    return crc;
}

u_int16_t SerialPort::crc16_ccitt(u_int8_t *data, int len)
{
    u_int16_t crc = 0x0000;
    u_int16_t polynomial = 0x1021;
    for(size_t i=0;i<len;i++){
        crc^=(data[i]<<8);
        for(int j=0;j<8;j++){
            if(crc & 0x8000){
                crc = (crc<<1)^polynomial;
            }else{
                crc<<=1;
            }
        }
        crc &= 0xFFFF;
    }
    return crc^0x0000;
}

u_int16_t  SerialPort::crc_byte(u_int16_t crc, u_int8_t b)
{
    u_int8_t I;

    crc = crc ^ b << 8;
    I = 8;
    do
        if (crc & 0x8000)
            crc = crc << 1 ^ 0x1021;
        else
            crc = crc << 1;
    while (--I);

    return crc;
}

double SerialPort::unsignedchar2double(unsigned char* data, int count, double LSB)
{
    double result = 0.;
    unsigned temp = 0;

    if (count == 2) {
        if ( (data[0]&0x80) == 0x00 ) {
            temp = data[0]*256+data[1];
            result = (double)temp*LSB;
            return result;
        } else {
            temp = (data[0]&0x7F)*256+data[1];
            result = - (double)temp*LSB;
            return result;
        }
    } else {
        return result;
    }

}



