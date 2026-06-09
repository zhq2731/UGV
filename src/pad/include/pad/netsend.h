#ifndef NETSEND_H
#define NETSEND_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QThread>
#include <QByteArray>
//#include "windows.h"
#include "pad/allheader.h"
#include <QDebug>
#include "taskPoints_msgs/taskPoints.h"
#include "taskPoints_msgs/TaskNode.h"
typedef struct
{
  QByteArray arrMsg; /*报文信息*/
  INT32 iLen;        /*报文长度*/
  QHostAddress DestIP;
  UINT16 DestPort;
} NetSendST; /*网络发送结构体*/

class NetSend : public QObject
{
  Q_OBJECT
public:
  explicit NetSend(QObject* parent = nullptr);
  ~NetSend();

  QUdpSocket* m_pUdpSendSocket;         /*发送套接字*/
  QUdpSocket* remoteSocket;             /*云控端通信套接字*/  
  taskPoints_msgs::taskPoints recvmsg;  /*接受任务下发赋值用*/
  QMap<UINT32, NetSendST> m_mapSendMsg; /*等待确认的报文帧号与报文内容Map*/
  QMap<UINT32, UINT32> m_mapSendNo;     /*等待确认的报文帧号与等待时间Map*/
  QList<NetSendST> m_listSendMsg;       /*发送报文列表*/
  QTimer* m_pSendTimer;                 /*5ms报文发送定时器，以防帧间隔太短，发生丢包现象*/
  UINT32 m_uiSendChassisMsgNo;          /*发送底盘报文序号*/
  UINT32 m_uiSendAutoDrivingHeartMsgNo; /*发送无人驾驶心跳报文序号*/
  UINT32 m_uiSendPNC2PADINSMsgNo = 0;   /*发送定位定向报文序号*/

  UINT32 m_uiSendPNC2PADPercptMsgNo = 0;

  UINT32 m_uiSendPathMsgNo = 0;
  UINT32 m_uiMoveDataMsgNo = 0;

  UINT32 m_uiSendMsgNo;           /*发送报文序号*/
  void ReSendMsg(UINT32 uiMsgNo); /*重发网络报文*/

  UINT8 CheckSum(const UINT8* pucData, INT32 iLen);                          /*和校验算法*/
  void RecvLenErrorProc(QByteArray datagram, INT32 iLen, UINT32 uiRightLen); /*接收长度错误处理*/
  UINT32 Acquire24AbsTime();                                                 /*获取24小时绝对时间*/
  public slots:
    
private:
  void write(QByteArray datagram, INT32 iLen, QHostAddress DestIP, UINT16 usPort); /*网络发送函数*/

signals:
  void MsgTableDispSign(QString strCol2, QString strCol3, UINT8 ucDispType = DISP_PROCESS); /*界面报文表格显示*/
  void WriteRecord(const QString& strInfo, UINT8 ucFlag); /*改成发送信号形式，避免在记录线程中冲突，耽误时间*/

public slots:
  bool Init(); /*网络初始化*/
  void SendMsgListAdd(QByteArray datagram, INT32 iLen, QHostAddress DestIP, UINT16 DestPort);
  void SendProcTimer();                             /*报文发送定时器*/
  void AckMsgProc(QByteArray datagram, INT32 iLen); /*接收确认报文处理*/
  void readPingDatagrams();
};

#endif  // NETSEND_H
