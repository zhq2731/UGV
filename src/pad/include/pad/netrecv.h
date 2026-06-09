#ifndef NETCOMM_H
#define NETCOMM_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QtNetwork>
#include <QByteArray>
#include<QString>

#include "pad/allheader.h"

class NetRecv : public QThread
{
  Q_OBJECT
public:
  explicit NetRecv(QObject* parent = Q_NULLPTR);
  ~NetRecv() Q_DECL_OVERRIDE;

  QUdpSocket* m_pUdpChassisRecvSocket;     /*接收套接字*/
  QUdpSocket* m_pUdpAutoDrivingRecvSocket; /*接收套接字*/
  QUdpSocket* m_pUdpPerceptionRecvSocket;  /*接收套接字*/
  QUdpSocket* m_pUdpTaskpointsRecvSocket;  /*接收套接字*/
  QUdpSocket* m_pUdpRemoteRecvSocket;      /*运动开始套接字*/
  QUdpSocket* m_pUdpRemoteDriveRecvSocket; /*遥控驾驶套接字*/


  bool m_bQuit; /*程序运行停止标志*/

  void run() Q_DECL_OVERRIDE; /*重载run函数*/

signals:
  void RecvRemoteControlAutoDrivingMsg(QByteArray datagram, INT32 iLen); /*接收到综控的网络报文*/
  void RecvRemoteControlChassisMsg(QByteArray datagram, INT32 iLen);     /*接收到综控的网络报文*/
  void RecvPerceptionMsg(QByteArray datagram, INT32 iLen);               /*接收到综控的网络报文*/
  void RecvRemoteMsg(QByteArray datagram, INT32 iLen); /*接收到综控的网络报文*/
  void RecvTaskpointsMsg(QByteArray datagram, INT32 iLen); /*接收到综控的网络报文*/
  void RecvRemoteDriveMsg(QByteArray datagram, INT32 iLen); /*接收到综控的网络报文*/

  //   void MsgTableDispSign(QString strCol2, QString strCol3, UINT8 ucDispType = DISP_PROCESS); /*界面报文表格显示*/
  //   void WriteRecord(const QString& strInfo, UINT8 ucFlag); /*改成发送信号形式，避免在记录线程中冲突，耽误时间*/

public slots:
  void ReadChassisDatagram();
  void ReadAutoDrivingDatagram();
  void ReadPerceptionDatagram();
  void ReadTaskpointsDatagram();
  void ReadRemoteDatagram();
  void ReadRemoteDriveDatagram();
};

#endif  // NETCOMM_H
