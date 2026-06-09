#include "pad/netsend.h"
#include <QMessageBox>
#include <iostream>

NetSend::NetSend(QObject* parent) : QObject(parent)
{
  m_listSendMsg.clear(); /*发送报文列表*/
  m_uiSendMsgNo = 0;
  m_uiSendChassisMsgNo = 0; /*发送底盘报文序号*/
  m_uiSendAutoDrivingHeartMsgNo = 0;
  m_uiSendPNC2PADPercptMsgNo = 0;

  m_pUdpSendSocket = Q_NULLPTR;
  //   m_pUdpSendAutoDrivingSocket = Q_NULLPTR;
  m_pSendTimer = Q_NULLPTR;
  // remoteSocket = new QUdpSocket(this);
  // remoteSocket->bind(QHostAddress("127.0.0.1"),20460); /**/
  // connect(remoteSocket,&QUdpSocket::readyRead,this,&NetSend::readPingDatagrams);
  qDebug()<<"debug";
}

NetSend::~NetSend()
{
  m_pSendTimer->stop();

  if (Q_NULLPTR != m_pUdpSendSocket)
  {
    m_pUdpSendSocket->close();
    m_pUdpSendSocket->deleteLater();
    m_pUdpSendSocket = Q_NULLPTR;
  }

  if (Q_NULLPTR != m_pSendTimer)
  {
    m_pSendTimer->stop();
    m_pSendTimer->deleteLater();
    m_pSendTimer = Q_NULLPTR;
  }
  qDebug() << "~NetSend()";
}

/********************************网络初始化********************************/
bool NetSend::Init()
{
  m_pUdpSendSocket = new QUdpSocket(this); /*实例化socket*/


  if (m_pUdpSendSocket == Q_NULLPTR)
  {
    QMessageBox::information(Q_NULLPTR, "错误", "发送自动驾驶UDP创建失败");
    return false;
  }
  m_pSendTimer = new QTimer(this);
  m_pSendTimer->setTimerType(Qt::PreciseTimer);
  connect(m_pSendTimer, SIGNAL(timeout()), this, SLOT(SendProcTimer()));

  m_pSendTimer->start(5);
  /*开启命令报文重发定时器*/

  return true;
}

/********************************网络报文发送函数********************************/
void NetSend::write(QByteArray datagram, INT32 iLen, QHostAddress DestIP, UINT16 usPort)
{
  NetHeaderST stNetHeader;
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));

  qint64 iReturnLen = m_pUdpSendSocket->writeDatagram(datagram.data(), iLen, DestIP, usPort);
  // std::cout << " send udp " << static_cast<int>(stNetHeader.usMsgType) << std::endl;
  // if (iLen != iReturnLen) /*发送网络报文*/
  // {
  //   QString strInfo = QString("NET端口 报文%1H发送失败!").arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'));
  //   // emit WriteRecord(strInfo, RECORD_ERROR);
  //   std::cout << strInfo.toStdString() << std::endl;
  // }
  //   QString strInfo, strTemp;
  //   strInfo = QString("网络发送 长度=%1 ").arg(iReturnLen);
  //   for (INT32 i = 0; i < iLen; i++)
  //   {
  //     strTemp = QString("%1 ").arg((UINT8)datagram.at(i), 2, 16, QChar('0'));
  //     strInfo.append(strTemp);
  //   }
  //   emit WriteRecord(strInfo, RECORD_DATA);
}

/******************************和校验算法******************************/
UINT8 NetSend::CheckSum(const UINT8* pucData, INT32 iLen)
{
  UINT8 ucTemp = 0;
  for (int i = 0; i < (iLen - 1); i++)
  {
    ucTemp ^= (pucData[i]); /*全部字节异或*/
  }
  return ucTemp;
}

/******************************报文重发定时器******************************/
void NetSend::SendProcTimer()
{
  //   /*槽函数在同一线程顺序执行，运行到此函数中时，m_mapSendNo队列不会变化*/
  //   INT32 sMsgCnt = m_mapSendNo.size(); /*需重发的报文数量*/
  //   if (0 < sMsgCnt)                    /*存在需要重发的报文*/
  //   {
  //     QMutableMapIterator<UINT32, UINT32> itMap(m_mapSendNo); /*迭代器*/
  //     while (itMap.hasNext())                                 /*有对象*/
  //     {
  //       itMap.next();                        /*移至下一个*/
  //       itMap.setValue((itMap.value() + 5)); /*等待时间累计*/
  //       if (150 > itMap.value())             /*未到150ms*/
  //       {
  //         if ((0 == itMap.value() % 50)) /*50ms一个周期*/
  //         {
  //           ReSendMsg(itMap.key()); /*重发当前帧号报文*/
  //         }
  //       }
  //       else /*150ms到，超时，删除当前报文并记录*/
  //       {
  //         QString strInfo = QString("接收确认超时 NetSend::SendProcTimer 帧号%1H报文已重发3次，未收到接收确认报文")
  //                               .arg(itMap.key(), 8, 16, QChar('0'));
  //         emit WriteRecord(strInfo, RECORD_ERROR);

  //         m_mapSendMsg.remove(itMap.key()); /*删除重发报文的设置*/
  //         itMap.remove();                   /*从重发报文中删除*/
  //       }
  //     }
  //   }
  //   else
  //   {
  //     /*无等待确认的命令报文，不用处理*/
  //   }

  if (m_listSendMsg.size() > 0) /*存在需发送的报文*/
  {
    write(m_listSendMsg.at(0).arrMsg, m_listSendMsg.at(0).iLen, m_listSendMsg.at(0).DestIP,
          m_listSendMsg.at(0).DestPort);
          // std::cout <<  "m_listSendMsg   size= " <<   m_listSendMsg.size()  <<   std::endl;
    m_listSendMsg.takeAt(0); /*发送完成后，删除第1项元素*/
  }
}

/******************************重发网络报文处理******************************/
void NetSend::ReSendMsg(UINT32 uiMsgNo)
{
  m_listSendMsg.append(m_mapSendMsg[uiMsgNo]);
}

void NetSend::SendMsgListAdd(QByteArray datagram, INT32 iLen, QHostAddress DestIP, UINT16 DestPort) /*发送报文列表添加*/
{
  /*发送帧号、发送时间、发送校验和统一在此处再赋值*/
  NetHeaderST stNetHeader;
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));
//  if (stNetHeader.usMsgType == PNC_TO_REMOTE_CONTROL_STATE_MSG)  //底盘状态报文
//  {
//    if (0xFFFFFFFF <= m_uiSendChassisMsgNo) /*发送报文序号*/
//    {
//      m_uiSendChassisMsgNo = 1;
//    }
//    else
//    {
//      m_uiSendChassisMsgNo++;
//    }
//    stNetHeader.uiMsgNo = m_uiSendChassisMsgNo;
//  }
   if (stNetHeader.usMsgType == PNC2PAD_HEART_MSG)
  {
    if (0xFFFFFFFF <= m_uiSendAutoDrivingHeartMsgNo) /*发送报文序号*/
    {
      m_uiSendAutoDrivingHeartMsgNo = 1;
    }
    else
    {
      m_uiSendAutoDrivingHeartMsgNo++;
    }
    stNetHeader.uiMsgNo = m_uiSendAutoDrivingHeartMsgNo;
  }
//  else if (stNetHeader.usMsgType == PNC2PAD_INS_MSG)
//  {
//    if (0xFFFFFFFF <= m_uiSendPNC2PADINSMsgNo) /*发送报文序号*/
//    {
//      m_uiSendPNC2PADINSMsgNo = 1;
//    }
//    else
//    {
//      m_uiSendPNC2PADINSMsgNo++;
//    }
//    stNetHeader.uiMsgNo = m_uiSendPNC2PADINSMsgNo;
//  }
//  else if (stNetHeader.usMsgType == PERC2VC_OBS_CLUSTER_MSG)
//  {
//    if (0xFFFFFFFF <= m_uiSendPNC2PADPercptMsgNo) /*发送报文序号*/
//    {
//      m_uiSendPNC2PADPercptMsgNo = 1;
//    }
//    else
//    {
//      m_uiSendPNC2PADPercptMsgNo++;
//    }
//    stNetHeader.uiMsgNo = m_uiSendPNC2PADPercptMsgNo;
//  }
//  else if (stNetHeader.usMsgType == PNC2PAD_TRAJECTORY_UP_MSG)
//  {
//    if (0xFFFFFFFF <= m_uiSendPathMsgNo) /*发送报文序号*/
//    {
//      m_uiSendPathMsgNo = 1;
//    }
//    else
//    {
//      m_uiSendPathMsgNo++;
//    }
//    stNetHeader.uiMsgNo = m_uiSendPathMsgNo;
//    std::cout << "send trajectory " << std::endl;
//  }
//  else if (stNetHeader.usMsgType == PNC2PAD_MOVE_DATA_MSG)
//  {
//    if (0xFFFFFFFF <= m_uiMoveDataMsgNo) /*发送报文序号*/
//    {
//      m_uiMoveDataMsgNo = 1;
//    }
//    else
//    {
//      m_uiMoveDataMsgNo++;
//    }
//    stNetHeader.uiMsgNo = m_uiMoveDataMsgNo;
//    std::cout << "send move data " << std::endl;
//  }
  else
  {
    // std::cout << "donot have this send type" << std::endl;
  }

  /*发送帧号赋值*/
  stNetHeader.uiMsgTime = Acquire24AbsTime(); /*发送时间赋值*/
  memcpy(datagram.data(), &stNetHeader, sizeof(NetHeaderST));
  INT16 sNo = (INT16)stNetHeader.usMsgLen - 1;
  datagram[sNo] = (INT8)CheckSum((UINT8*)datagram.data(), stNetHeader.usMsgLen); /*和校验赋值*/

  /*只有从外面来的报文需要做重发准备*/
  if (NET_MSG_ACK == stNetHeader.usAck) /*报文需要确认*/
  {
    NetSendST stNetSend;
    stNetSend.arrMsg.append(datagram);
    stNetSend.iLen = iLen;
    stNetSend.DestIP = DestIP;
    stNetSend.DestPort = DestPort;

    m_mapSendMsg.insert(m_uiSendMsgNo, stNetSend); /*插入发送的命令报文*/
    m_mapSendNo.insert(m_uiSendMsgNo, 0);
  }
  else
  {
    /*报文不需确认，不用处理*/
  }

  NetSendST stNetSend;
  stNetSend.arrMsg.append(datagram);
  stNetSend.iLen = iLen;
  stNetSend.DestIP = DestIP;
  stNetSend.DestPort = DestPort;
  m_listSendMsg.append(stNetSend); /*把本次要发送的报文加入发送列表*/
}

/******************************接收确认报文处理******************************/
void NetSend::AckMsgProc(QByteArray datagram, INT32 iLen)
{
  if (sizeof(VC2FPAckMsgST) == iLen)
  {
    VC2FPAckMsgST stVC2FPAckMsg;
    memcpy(&stVC2FPAckMsg, datagram.data(), sizeof(VC2FPAckMsgST));

    /*槽函数在同一线程顺序执行，运行到此函数中时，m_mapSendNo队列不会变化*/
    INT32 sRemoveCnt = m_mapSendNo.remove(stVC2FPAckMsg.uiMsgNo); /*删除回复的接收确认帧号*/
    m_mapSendMsg.remove(stVC2FPAckMsg.uiMsgNo);                   /*删除已回复的重发报文*/
    if (0 == sRemoveCnt)
    {
      QString strInfo = QString("接收到确认帧号%1H报文，无该发送报文").arg(stVC2FPAckMsg.uiMsgNo, 8, 16, QChar('0'));
      // emit WriteRecord(strInfo, RECORD_ERROR);
    }
    else if (1 < sRemoveCnt) /*多余一个被删除*/
    {
      QString strInfo = QString("程序错误，删除%1条接收确认帧号%2H报文")
                            .arg(sRemoveCnt)
                            .arg(stVC2FPAckMsg.uiMsgNo, 8, 16, QChar('0'));
      // emit WriteRecord(strInfo, RECORD_ERROR);
    }
    else
    {
      /*删除正确，不用处理*/
    }

    /*报文表格增加一行*/
    // emit MsgTableDispSign("综控", "回复接收确认");
  }
  else
  {
    RecvLenErrorProc(datagram, iLen, sizeof(VC2FPAckMsgST)); /*接收报文长度错误处理*/
  }
}

/******************************接收报文长度错误处理******************************/
void NetSend::RecvLenErrorProc(QByteArray datagram, INT32 iLen, UINT32 uiRightLen)
{
  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST)); /*获取报文头内容*/

  QString strInfo =
      QString("帧号%1H报文长度%2错误，应为%3").arg(stNetHeader.uiMsgNo, 8, 16, QChar('0')).arg(iLen).arg(uiRightLen);
  emit WriteRecord(strInfo, RECORD_ERROR);

  strInfo = QString("%1H报文长度错误").arg(stNetHeader.usMsgType, 4, 16, QChar('0'));
  emit MsgTableDispSign("错误", strInfo);
}

/******************************获取24小时绝对时间******************************/
UINT32 NetSend::Acquire24AbsTime()
{
  QDateTime dateTime = QDateTime::currentDateTime();
  UINT32 uiTime =
      (UINT32)(dateTime.time().msec() +
               ((dateTime.time().hour() * 60 + dateTime.time().minute()) * 60 + dateTime.time().second()) * 1000);
  return uiTime;
}

void NetSend::readPingDatagrams()
{
  // QByteArray recv_data;              //具体报文内容

  // while (remoteSocket->hasPendingDatagrams())
  // {
  //   recv_data.resize(remoteSocket->pendingDatagramSize());
  //   remoteSocket->readDatagram(recv_data.data(),recv_data.size());
  // }
  // qDebug()<<recv_data;
  // qDebug()<<"debug"; 
}