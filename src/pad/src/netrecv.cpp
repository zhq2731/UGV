#include "pad/netrecv.h"
#include <iostream>
#include <QMessageBox>
NetRecv::NetRecv(QObject *parent) : QThread(parent)
{
  m_bQuit = false;                         /*程序运行停止标志*/
  m_pUdpChassisRecvSocket = Q_NULLPTR;     /*接收套接字*/
  m_pUdpAutoDrivingRecvSocket = Q_NULLPTR; /*接收套接字*/
  m_pUdpPerceptionRecvSocket = Q_NULLPTR;
  m_pUdpTaskpointsRecvSocket = Q_NULLPTR; //任务下发端接收套接字
  m_pUdpRemoteRecvSocket = Q_NULLPTR;     //远程操控设备接收套接字

  m_pUdpRemoteDriveRecvSocket = Q_NULLPTR;
}

NetRecv::~NetRecv()
{
  m_bQuit = true; /*退出接收线程*/

  //  m_pUdpChassisRecvSocket->abort(); /*接收套接字*/
  //  delete m_pUdpChassisRecvSocket;
  //  m_pUdpChassisRecvSocket = Q_NULLPTR;

  m_pUdpAutoDrivingRecvSocket->abort(); /*接收套接字*/
  delete m_pUdpAutoDrivingRecvSocket;
  m_pUdpAutoDrivingRecvSocket = Q_NULLPTR; /*接收套接字*/

  m_pUdpTaskpointsRecvSocket->abort();
  delete m_pUdpTaskpointsRecvSocket;
  m_pUdpTaskpointsRecvSocket = Q_NULLPTR;

  m_pUdpRemoteRecvSocket->abort();
  delete m_pUdpRemoteRecvSocket;
  m_pUdpRemoteRecvSocket = Q_NULLPTR;
  //  m_pUdpPerceptionRecvSocket->abort(); /*接收套接字*/
  //  delete m_pUdpPerceptionRecvSocket;
  //  m_pUdpPerceptionRecvSocket = Q_NULLPTR; /*接收套接字*/

  m_pUdpRemoteDriveRecvSocket->abort();
  delete m_pUdpRemoteDriveRecvSocket;
  m_pUdpRemoteDriveRecvSocket = Q_NULLPTR;
  qDebug() << "~NetRecv()";
}

/********************************重载run函数********************************/
void NetRecv::run()
{
  // std::cout << "Net recv run " << std::endl;
  //  m_pUdpChassisRecvSocket = new QUdpSocket(); /*实例化socket*/
  //  if (m_pUdpChassisRecvSocket == Q_NULLPTR)
  //  {
  //    QMessageBox::information(Q_NULLPTR, "错误", "UDP创建失败");
  //    return;
  //  }
  //  bool bChassisBindRet = m_pUdpChassisRecvSocket->bind(QHostAddress(PNC_IP), PNC_CHASSIS_PORT);
  //  if (!bChassisBindRet)
  //  {
  //    QMessageBox::information(Q_NULLPTR, "错误", "绑定与远程遥控端底盘端口失败");
  //    return;
  //  }
  // std::cout << "Net recv run " << std::endl;

  // m_pUdpAutoDrivingRecvSocket = new QUdpSocket(); /*实例化socket*/

  // if (m_pUdpAutoDrivingRecvSocket == Q_NULLPTR)
  // {
  //   QMessageBox::information(Q_NULLPTR, "错误", "UDP创建失败");
  //   return;
  // }
  // bool bAutoDrivingBindRet = m_pUdpAutoDrivingRecvSocket->bind(QHostAddress(PNC_IP), PNC_AUTODRIVING_PORT);
  // if (!bAutoDrivingBindRet)
  // {
  //   QMessageBox::information(Q_NULLPTR, "错误", "绑定与远程遥控端自动驾驶端口失败");
  //   return;
  // }

  m_pUdpTaskpointsRecvSocket = new QUdpSocket(); /*任务下发套接字实例化*/

  if (m_pUdpTaskpointsRecvSocket == Q_NULLPTR)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "UDP创建失败");
    return;
  }
  bool bTaskpointsBindRet = m_pUdpTaskpointsRecvSocket->bind(QHostAddress::Any, TANK_TASKPOINTS_PORT);
  if (!bTaskpointsBindRet)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "绑定与taskpoint端自动驾驶端口失败");
    return;
  }

  m_pUdpRemoteRecvSocket = new QUdpSocket(); /*任务下发套接字实例化*/

  if (m_pUdpRemoteRecvSocket == Q_NULLPTR)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "UDP创建失败");
    return;
  }

  bool bRemoteBindRet = m_pUdpRemoteRecvSocket->bind(QHostAddress::Any, TANK_REMOTE_PORT);
  if (!bRemoteBindRet)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "绑定与远程遥控端自动驾驶端口失败");
    return;
  }
  // std::cout << "Net recv run " << std::endl;

  m_pUdpRemoteDriveRecvSocket = new QUdpSocket(); /*任务下发套接字实例化*/
  if (m_pUdpRemoteDriveRecvSocket == Q_NULLPTR)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "遥控驾驶UDP创建失败");
    return;
  }
  bool bRemoteDriveBindRet = m_pUdpRemoteDriveRecvSocket->bind(QHostAddress::Any, REMOTE_CONTROL_PORT);
  if (!bRemoteDriveBindRet)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "绑定与远程遥控驾驶端口失败");
    return;
  }

  //  m_pUdpPerceptionRecvSocket = new QUdpSocket(); /*实例化socket*/
  //  if (m_pUdpPerceptionRecvSocket == Q_NULLPTR)
  //  {
  //    QMessageBox::information(Q_NULLPTR, "错误", "UDP创建失败");
  //    return;
  //  }
  //  bool bPerceptionBindRet = m_pUdpPerceptionRecvSocket->bind(QHostAddress(PNC_IP), PNC_PERCEPTION_PORT);
  //  if (!bPerceptionBindRet)
  //  {
  //    QMessageBox::information(Q_NULLPTR, "错误", "绑定与感知端口失败");
  //    return;
  //  }
  // std::cout << "Net recv run " << std::endl;
  // std::cout << " net recev bind port " << std::endl;
  //  connect(m_pUdpChassisRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadChassisDatagram()), Qt::DirectConnection);

  // connect(m_pUdpAutoDrivingRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadAutoDrivingDatagram()),
  //         Qt::DirectConnection);
  connect(m_pUdpTaskpointsRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadTaskpointsDatagram()),
          Qt::DirectConnection);
  connect(m_pUdpRemoteRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadRemoteDatagram()),
          Qt::DirectConnection);

  connect(m_pUdpRemoteDriveRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadRemoteDriveDatagram()),
          Qt::DirectConnection);

  //  connect(m_pUdpPerceptionRecvSocket, SIGNAL(readyRead()), this, SLOT(ReadPerceptionDatagram()), Qt::DirectConnection);

  //     if (true == bRet)
  //     {
  //       QString strInfo = QString("NET端口 IP:%1 PORT:%2 UDP bind Succeed!").arg(FIGHT_OPT_IP).arg(VEHICLE_CTRL_PORT);
  //       // emit WriteRecord(strInfo, RECORD_PROCESS);

  //       qint64 optVal = m_pUdpRecvSocket->readBufferSize(); /*目前是0，那么接收缓存区没限制*/
  //                                                           // INT32 optlen = sizeof(optVal);
  //       // getsockopt(m_pUdpRecvSocket, SOL_SOCKET, SO_RCVBUF, (char *)&optVal, &optlen);
  //       std::cout << "网络接收缓存区大小:" << optVal;
  //     }
  //     else
  //     {
  //       QString strInfo = QString("NET端口 IP:%1 PORT:%2 UDP bind failed!").arg(FIGHT_OPT_IP).arg(VEHICLE_CTRL_PORT);
  //       // emit WriteRecord(strInfo, RECORD_ERROR);
  //       std::cout << strInfo.toStdString() << std::endl;
  //     }
  exec();
  //  std::cout << "net recevie thread end" <<std::endl;
  //    if(Q_NULLPTR != m_pUdpRecvSocket)
  //    {
  //        m_pUdpRecvSocket->close();/*关闭socket，可能没关闭，因此一直在缓存区存数*/
  //        delete m_pUdpRecvSocket;
  //        m_pUdpRecvSocket = Q_NULLPTR;
  //    }
}

void NetRecv::ReadChassisDatagram()
{
  // std::cout << " ReadChassisDatagram " << std::endl;
  while (true == m_pUdpChassisRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    // std::cout << " ReadChassisDatagram " << std::endl;
    QByteArray datagram;
    datagram.clear();
    datagram.resize((INT32)m_pUdpChassisRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpChassisRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                              &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                           /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
      strInfo = strTemp + strInfo;
      // std::cout << strTemp.toStdString() << std::endl;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      //   strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      //   for (INT32 i = 0; i < iLen; i++)
      //   {
      //     strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      //     strInfo.append(strTemp);
      //   }
      //            emit WriteRecord(strInfo, RECORD_DATA);

      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      if (REMOTE_CONTROL_IP == senderIP.toString())
      {
        emit RecvRemoteControlChassisMsg(datagram, iLen); /*综合控制设备网络报文处理*/
      }
      else
      {
        /*非法IP，不处理*/
      }
    }
  }
}
void NetRecv::ReadAutoDrivingDatagram()
{
  while (true == m_pUdpAutoDrivingRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    QByteArray datagram;
    datagram.clear();
    datagram.resize(
        (INT32)m_pUdpAutoDrivingRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpAutoDrivingRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                                  &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                               /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
      strInfo = strTemp + strInfo;
      // std::cout << strInfo.toStdString() << std::endl;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      for (INT32 i = 0; i < iLen; i++)
      {
        strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
        strInfo.append(strTemp);
      }
      //            emit WriteRecord(strInfo, RECORD_DATA);

      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      if (REMOTE_CONTROL_IP == senderIP.toString())
      {
        emit RecvRemoteControlAutoDrivingMsg(datagram, iLen); /*综合控制设备网络报文处理*/
      }
      else
      {
        /*非法IP，不处理*/
      }
    }
  }
}

void NetRecv::ReadTaskpointsDatagram() /*北理任务下发*/
{
  qDebug() << "debug";
  // QByteArray task_points_data;
  // task_points_data.clear();
  // while (m_pUdpTaskpointsRecvSocket->hasPendingDatagrams())
  // {
  //    task_points_data.resize(m_pUdpTaskpointsRecvSocket->pendingDatagramSize());
  //    m_pUdpTaskpointsRecvSocket->readDatagram(task_points_data.data(),task_points_data.size());
  // }
  // //jiexi
  while (m_pUdpTaskpointsRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    QByteArray datagram;
    datagram.clear();
    datagram.resize(
        (INT32)m_pUdpTaskpointsRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpTaskpointsRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                                 &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                              /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
      strInfo = strTemp + strInfo;
      // std::cout << strInfo.toStdString() << std::endl;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      for (INT32 i = 0; i < iLen; i++)
      {
        strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
        strInfo.append(strTemp);
      }
      //            emit WriteRecord(strInfo, RECORD_DATA);

      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      // if (REMOTE_CONTROL_IP == senderIP.toString())
      // {
      emit RecvTaskpointsMsg(datagram, iLen); /*综合控制设备网络报文处理*/

      // }
      // else
      // {
      //   /*非法IP，不处理*/
      // }
    }
  }
}

void NetRecv::ReadRemoteDatagram() /*远程操控设备报文接收回调函数*/
{
  qDebug() << "debug  ReadRemoteDatagram";
  // QByteArray remote_data;
  // remote_data.clear();
  // while (m_pUdpRemoteRecvSocket->hasPendingDatagrams())
  // {
  //    remote_data.resize(m_pUdpRemoteRecvSocket->pendingDatagramSize());
  //    m_pUdpRemoteRecvSocket->readDatagram(remote_data.data(),remote_data.size());
  // }
  // //jiexi
  // // qDebug()<<remote_data;
  // emit RecvRemoteMsg(datagram,iLen); /*综合控制设备网络报文处理*/
  while (m_pUdpRemoteRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    QByteArray datagram;
    datagram.clear();
    datagram.resize(
        (INT32)m_pUdpRemoteRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpRemoteRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                             &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                          /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
      strInfo = strTemp + strInfo;
      // std::cout << strInfo.toStdString() << std::endl;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      for (INT32 i = 0; i < iLen; i++)
      {
        strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
        strInfo.append(strTemp);
      }
      //            emit WriteRecord(strInfo, RECORD_DATA);

      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      // if (REMOTE_CONTROL_IP == senderIP.toString())
      // {
      emit RecvRemoteMsg(datagram, iLen); /*综合控制设备网络报文处理*/
      // }
      // else
      // {
      /*非法IP，不处理*/
      // }
    }
  }
}

void NetRecv::ReadRemoteDriveDatagram() /*遥控驾驶设备报文接收回调函数*/
{
  while (m_pUdpRemoteDriveRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    QByteArray datagram;
    datagram.clear();
    datagram.resize(
        (INT32)m_pUdpRemoteDriveRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpRemoteDriveRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                                  &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                               /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d,").arg(iLen);
      strInfo = strTemp + strInfo;
      // std::cout << strInfo.toStdString() << std::endl;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      for (INT32 i = 0; i < iLen; i++)
      {
        strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
        strInfo.append(strTemp);
      }
      //            emit WriteRecord(strInfo, RECORD_DATA);


      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      // if (REMOTE_CONTROL_IP == senderIP.toString())
      // {
      emit RecvRemoteDriveMsg(datagram, iLen); /*综合控制设备网络报文处理*/
      // }
      // else
      // {
      /*非法IP，不处理*/
      // }
    }
  }
}

void NetRecv::ReadPerceptionDatagram()
{
  while (true == m_pUdpPerceptionRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
  {
    QByteArray datagram;
    datagram.clear();
    datagram.resize(
        (INT32)m_pUdpPerceptionRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
    QHostAddress senderIP;
    UINT16 usSenderPort = 0;
    INT32 iLen = (INT32)m_pUdpPerceptionRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
                                                                 &usSenderPort); /*读取数据报*/
    if (-1 == iLen)                                                              /*读取报文失败*/
    {
      //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收失败");
    }
    else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
    {
      QString strInfo;
      for (int i = 0; i < iLen; i++)
      {
        strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
      }
      QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
      strInfo = strTemp + strInfo;
      //            emit WriteRecord(strInfo, RECORD_ERROR);
      //   emit MsgTableDispSign("错误", "网络接收长度过小");
    }
    else /*接收报文正确*/
    {
      QString strInfo, strTemp;
      strInfo = QString("网络接收 长度=%1 ").arg(iLen);
      for (INT32 i = 0; i < iLen; i++)
      {
        strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
        strInfo.append(strTemp);
      }
      //            emit WriteRecord(strInfo, RECORD_DATA);

      // std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
      if (PERCETION_IP == senderIP.toString())
      {
        // std::cout <<"rece percept msg" << std::endl;
        emit RecvPerceptionMsg(datagram, iLen); /*综合控制设备网络报文处理*/
      }
      else
      {
        /*非法IP，不处理*/
      }
    }
  }
}
// void NetRecv::ReadDatagram() /*socket接收到报文后，自动进入此函数*/
// {
//   while (true == m_pUdpRecvSocket->hasPendingDatagrams()) /*有数据报在等待,非阻塞，检查一下就返回*/
//   {
//     QByteArray datagram;
//     datagram.clear();
//     datagram.resize((INT32)m_pUdpRecvSocket->pendingDatagramSize()); /*pendingDatagramSize返回第一条阻塞的报文*/
//     QHostAddress senderIP;
//     UINT16 usSenderPort = 0;
//     INT32 iLen = (INT32)m_pUdpRecvSocket->readDatagram(datagram.data(), datagram.size(), &senderIP,
//                                                        &usSenderPort); /*读取数据报*/
//     if (-1 == iLen)                                                    /*读取报文失败*/
//     {
//       //            emit WriteRecord("网络接收失败，readDatagram返回-1", RECORD_ERROR);
//       emit MsgTableDispSign("错误", "网络接收失败");
//     }
//     else if ((INT32)(sizeof(NetHeaderST) + sizeof(NetEndST)) > iLen) /*接收报文长度不正确*/
//     {
//       QString strInfo;
//       for (int i = 0; i < iLen; i++)
//       {
//         strInfo = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
//       }
//       QString strTemp = QString("NET端口 网络接收长度过小 长度=%d，").arg(iLen);
//       strInfo = strTemp + strInfo;
//       //            emit WriteRecord(strInfo, RECORD_ERROR);
//       emit MsgTableDispSign("错误", "网络接收长度过小");
//     }
//     else /*接收报文正确*/
//     {
//       QString strInfo, strTemp;
//       strInfo = QString("网络接收 长度=%1 ").arg(iLen);
//       for (INT32 i = 0; i < iLen; i++)
//       {
//         strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
//         strInfo.append(strTemp);
//       }
//       //            emit WriteRecord(strInfo, RECORD_DATA);

//       std::cout << "recevie from " << senderIP.toString().toStdString() << std::endl;
//       if (VEHICLE_CTRL_IP == senderIP.toString())
//       {
//         emit RecvVCNetMsg(datagram, iLen); /*综合控制设备网络报文处理*/
//       }
//       else
//       {
//         /*非法IP，不处理*/
//       }
//     }
//   }
// }
