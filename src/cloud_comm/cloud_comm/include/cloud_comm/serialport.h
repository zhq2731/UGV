/******************************************************************************
 * Copyright (c) 2020-2021 中国航天科工集团二院206所 All rights reserved.
 *                              成都研发中心
 *作者：闫春秀
 *时间：2021-11-12
 *版权所有，侵权必究！
 ******************************************************************************/
#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>

#define UART_TRUE  		1
#define UART_TIMEOUT 	0
#define UART_FALSE 		-1

#define M_TRUE			1
#define M_FALSE			-1
#define M_TIMEOUT	    0


enum SelectType
{
    READ_SELECT=0,
    WRITE_SELECT=1
};


enum UartCheck{NO_CHECK=0,ODD_CHECK=1,PARITY_CHECK=2};

enum UART_TYPE{
    UART_DETECTCARD = 0x01,
    UART_ULTRASONIC = 0x02
};

//szk modify bug 11.16 add static
class SerialPort
{
public:
    SerialPort();
    static int UartOpen(char* UartName,int baud,int databits,int stopbits,int mParity);
    static int UartWrite(int uartFd,char *buf,int bufLen);
    static int UartRead(int uartFd,char *buf,int bufLen);
    static void UartClose(int &uartFd);
    static u_int16_t crc_packet(u_int8_t *data, int len); // CRC校验
    static u_int16_t crc16_ccitt(u_int8_t *data, int len); //crc多项式校验
    static double unsignedchar2double(unsigned char* data, int count, double LSB);


private:
    static void set_speed(int fd, int speed);
    static int myselect(int sockfd, int usec, int mtype);
    static int UartInit(int fd, int baud,int databits,int stopbits,int mParity);
    static u_int16_t crc_byte(u_int16_t crc, u_int8_t b); // CRC校验
    int m_UartFd;




};
#endif // SERIALPORT_H

