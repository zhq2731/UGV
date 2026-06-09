#ifndef PARENTDLG_H
#define PARENTDLG_H

#include <QDialog>
#include <QApplication>
#include <QDesktopWidget>
#include <QGuiApplication>
#include <QScreen>
#include <QThread>
#include <QMessageBox>
#include <QFileDialog>
#include <QPainter>
#include <QScrollBar>
#include <QDate>
#include <QTime>
#include <QDateTime>
#include <QMouseEvent>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <ros/ros.h>
#include <unistd.h>

#include "pad/allheader.h"
#include "pad/netrecv.h"
#include "pad/netsend.h"
#include "pad/astar.h"
#include "pad/PosMeasureDlg.h"
#include "pad/configdialog.h"
#include "pad/mapfilegeneration.h"
// #include "pad/rclcomm.h"
#include "eigen3/Eigen/Eigen"
#include "pad/qnode.h"
#include <QVector>

#include "pad/chassislighthornwiperdialog.h"
#include "pad/vehiclewirecontrol.h"
#include "pad/platoondlg.h"
#include "pad/updatefile.h"
namespace Ui
{
  class ParentDlg;
}

class ParentDlg : public QDialog
{
  Q_OBJECT

public:
  explicit ParentDlg(int argc, char **argv, QWidget *parent = nullptr);
  ~ParentDlg() Q_DECL_OVERRIDE;

  void FoldAndSendPathBindMsg(UINT8 ucFrameMultiFlag, UINT16 usFrameNum, UINT32 uiTotalByteNum, UINT16 usFrameNo,
                              UINT16 usFrameDataBytes, QByteArray arrFrameData);

  void SendHeartMsg();                                     /*发送心跳报文*/
  void SendPathBindMsg();                                  /*发送路径装订报文*/
  void SendMapCordtBindMsg();                              /*发送地图坐标系装订报文*/
  void SendPathAskMsg();                                   /*发送跟踪路径查询报文*/
  void SendMapCordtAskMsg();                               /*发送运动坐标系查询报文*/
  void SendAckMsg(UINT32 uiFrameNo);                       /*发送接收确认报文*/
  void SendChasCtrlMsg(FP2VCChasCtrlMsgST stFP2VCMsg);     /*发送底盘设备控制报文*/
  void SendMoveCtrlMsg(FP2VCChasMoveCtrlMsgST stFP2VCMsg); /*发送运动控制报文*/

  void RecvHeartMsgProc(QByteArray datagram, INT32 iLen);            /*接收心跳报文处理*/
  void RecvPathBindRsltMsgProc(QByteArray datagram, INT32 iLen);     /*接收路径装订结果报文处理*/
  void RecvMapCordtBindRlstMsgProc(QByteArray datagram, INT32 iLen); /*地图坐标系装订结果报文处理*/
  void RecvAbNormalMsgProc(QByteArray datagram, INT32 iLen);         /*接收异常信息处理*/
  void RecvCanTransMsgProc(QByteArray datagram, INT32 iLen);         /*CAN透传报文处理*/
  void RecvCmdAckMsgProc(VC2FPCanTransMsgST stCanMsg);               /*接收命令应答透传报文处理*/
  void RecvStateMsgProc(VC2FPCanTransMsgST stCanMsg);                /*接收状态透传报文处理*/
  void RecvChasCanCmdAckMsgProc(VC2FPCanTransMsgST stCanMsg);        /*接收底盘CAN命令应答报文处理*/
  void RecvChasCanStateMsgProc(VC2FPCanTransMsgST stCanMsg);         /*接收底盘CAN状态报文处理*/

  void ChasDataDisp(Chas2VCWorkDataST stChas2VCWorkData);      /*底盘数据显示处理*/
  void INSDataDisp(INS2VCWorkDataST stINS2VCWorkData);         /*INS数据显示处理*/
  void ChasCmdDataDisp(VC2ChasCmdST stVC2ChasCmd);             /*综控发给底盘的命令数据显示处理*/
  void EqStateDispColor(QLabel *label, UINT8 ucColor);         /*设备状态文本背景颜色设置*/
  void ChasDetailStateDispColor(QLabel *label, UINT8 ucColor); /*底盘设备状态文本背景颜色设置*/
  void CameraDataDisp(CameraDataST stCameraData);              /*摄像头信息显示*/
  void RadarDataDisp(RadarDataST stRadarData);                 /*激光雷达信息显示*/

  void BindRsltValidJudge();                                            /*路径装订结果是否有效标志*/
  void PathBindRsltDisp();                                              /*路径装订结果信息显示*/
  void TrackPathDisp();                                                 /*需要跟踪的路径信息显示*/
  void MapCorditBindRsltDisp(LongLatHeightST stOrigin, ENUCorST stEnd); /*地图坐标系装订结果信息显示*/

  UINT8 CheckSum(const UINT8 *pucData, INT32 iLen);                          /*和校验函数*/
  void RecvLenErrorProc(QByteArray datagram, INT32 iLen, UINT32 uiRightLen); /*接收报文长度错误处理*/

  // void AnomalyInfoDisp(QString strInfo, UINT8 ucDispType); /*异常信息显示*/
  void TextToVoice(QString strVoice); /*文本转声音*/

  void DrawChessBoard();       /*绘制一张白板,清除之前的图*/
  void DrawAxis();             /*画坐标轴*/
  void DrawStartAndEndPoint(); /*绘制路径起点和终点*/
  void DrawPath();             /*绘制路径*/
  void DrawPathPrependPoint(); /*绘制路径前置点*/
  void DrawVehPos();           /*绘制车辆当前位置*/

  void ReadConfigFile(); /*读取配置文件*/

  void DrawSelectPoint();

  void DrawLocalPath();

  void GenerateRoadBoundary(); /*生成道路左右边界数据*/

  bool CheckStartCommunicationStateEnable(); /*查看与下位机开始通信状态 qixianyu 20220210*/

  void ADCTrajectorySubscripeFunc(const planning_msgs::TrajectoryPointArray::ConstPtr msg);
  void SendRouteAndLocalPathMsg(unsigned char ucPathType, std::vector<GlobalPositionST> listPathPoint);

  void FoldAndSendPathBindMsg(unsigned char ucPathType, unsigned char ucFrameMultiFlag, unsigned short int usFrameNum,
                              unsigned int uiTotalByteNum, unsigned short int usFrameNo,
                              unsigned short int usFrameDataBytes, std::vector<GlobalPositionST> arrFrameData);
  void SendObstacleMsg(uint8 ucFrameMultiFlag, unsigned short int usFrameNum, unsigned short int usFrameNo, std::vector<ObstacleST> arrFrameData);

  void SendPNCMoveDataMsg(PNC2PadMoveDataMsgST stMsg); /*发送决策控制数据信息*/

  void SendRouteAndGlobalPathMsg(unsigned char ucPathType, std::vector<localPositionST> listPathPoint);
  void FoldAndSendPathBindGlobalMsg(unsigned char ucPathType, unsigned char ucFrameMultiFlag, unsigned short int usFrameNum, unsigned int uiTotalByteNum, unsigned short int usFrameNo, unsigned short int usFrameDataBytes, std::vector<localPositionST> arrFrameData);

  /*将与X轴的夹角，一二象限逆时针旋转为0～+180度，四三象限顺时针旋转为0～-180度的角度转换成，与东北天坐标系Y轴顺时针0～360度(Y轴为0度)的角度*/
  float Angle2X180TransAngle2Y360(float fAngle2X)
  {
    /*与Y轴夹角0~360转成与X夹角-180~+180*/
    float fAngle2Y = 0;
    if (fAngle2X <= -90)
    {
      fAngle2Y = 90 - fAngle2X;
    }
    else if (fAngle2X < 0)
    {
      fAngle2Y = 90 - fAngle2X;
    }
    else if (fAngle2X < 90)
    {
      fAngle2Y = 90 - fAngle2X;
    }
    else
    {
      fAngle2Y = 450 - fAngle2X;
    }
    return fAngle2Y;
  }

  inline float angle2Y360TransAngle2X180(float fAngle2Y) /*与Y轴夹角0~360转成与X夹角-180~+180*/
  {
    float fAngle2X = 0;
    if (fAngle2Y <= 90)
    {
      fAngle2X = 90 - fAngle2Y;
    }
    else if (fAngle2Y <= 180)
    {
      fAngle2X = 90 - fAngle2Y;
    }
    else if (fAngle2Y <= 270)
    {
      fAngle2X = 90 - fAngle2Y;
    }
    else
    {
      fAngle2X = 450 - fAngle2Y;
    }
    return fAngle2X;
  }
  // UINT32 Acquire24AbsTime()
  // {
  //   QDateTime dateTime = QDateTime::currentDateTime();
  //   UINT32 uiTime =
  //       (UINT32)(dateTime.time().msec() +
  //                ((dateTime.time().hour() * 60 + dateTime.time().minute()) * 60 + dateTime.time().second()) * 1000);
  //   return uiTime;
  // }

private:
  QString m_strTableNewInfo;                                                                    /*表格最新一行信息*/
  QList<ENUCorST> m_listVehPos;                                                                 /*车辆位置列表*/
  QList<SmoothPathPointST> m_listTrackPathCmd;                                                  /*要跟踪的路径数据点,将其改为vector*/
  void SendNetMsg(char data[], int iLen, std::string strDestIP, unsigned short int usDestPort); /*发送以太网报文*/

  QVector<ENUCorST> m_vectorTrackPathLeftBoundary;  /*要跟踪的路径数据点的左侧边界 qixianyu 20220124*/
  QVector<ENUCorST> m_vectorTrackPathRightBoundary; /*要跟踪的路径数据点的右侧边界 qixianyu 20220124*/

  QList<SmoothPathPointST> m_listTrackPathBindRslt; /*综控中装订的跟踪路径点*/
  bool m_bTrackPathBindRsltValid;                   /*路径装订结果有效*/
  UINT32 m_uiTrackPathFrameNum;                     /*要跟踪的路径数据点的总数据包数*/
  UINT32 m_uiTrackPathTotalBytes;                   /*要跟踪的路径数据点的数据总字节数*/
  UINT32 m_uiRecvTrackPathFrameCnt;                 /*接收到的要跟踪的路径数据点的数据帧号计数*/
  UINT32 m_uiRecvTrackPathTotalBytesCnt;            /*要跟踪的路径数据点的数据总字节数自计数*/

  /*接收报文数显示，属于调试信息*/
  UINT32 m_uiLstRecordRecvHeartMsgNum;   /*上周期记录到的接收到心跳报文计数*/
  UINT32 m_uiRecvHeartMsgNum;            /*接收到心跳报文计数*/
  UINT32 m_uiRecvPathBindRsltMsgNum;     /*接收到路径装订结果报文计数*/
  UINT32 m_uiRecvMapCordtBindRlstMsgNum; /*接收到地图坐标系装订结果报文计数*/
  UINT32 m_uiRecvAbNormlaMsgNum;         /*接收到异常报文报文计数*/
  UINT32 m_uiRecvAckMsgNum;              /*接收到接收确认报文计数*/
  UINT32 m_uiRecvCmdAckMsgNum;           /*接收到命令应答透传报文计数*/
  UINT32 m_uiRecvStateMsgNum;            /*接收到状态透传报文计数*/

  /*发送报文数显示，属于调试信息*/
  PNC2PadHeartMsgST m_stPNC2PadHeartMsg; /*心跳上报报文，用于100ms定时器发送*/
  UINT32 m_uiSendHeartMsgNum;            /*发送心跳报文计数*/
  UINT32 m_uiSendPathBindMsgNum;         /*发送路径装订报文计数*/
  UINT32 m_uiSendMapCordtBindMsgNum;     /*发送运动坐标系装订报文计数*/
  UINT32 m_uiSendPathAskMsgNum;          /*发送跟踪路径信息查询报文计数*/
  UINT32 m_uiSendMapCordtAskMsgNum;      /*发送运动坐标系查询报文计数*/
  UINT32 m_uiSendAckMsgNum;              /*发送接收确认报文计数*/
  UINT32 m_uiSendChasCtrlMsgNum;         /*发送底盘设备控制报文计数*/
  UINT32 m_uiSendMoveCtrlMsgNum;         /*发送底盘运动控制报文计数*/
  int m_sockClient;                      /*与PAD通信，发送socket*/
  int m_sockServer;                      /*与PAD通信，接收socket*/
  unsigned short int FLAGS_PNC_PORT;     /*与PAD通信的决策控制单元以太网端口*/

  QString m_strErrorInfo; /*错误信息提示*/

  taskPoints_msgs::taskPoints taskpointbridgemsg;

  NetRecv *m_pNetRecv;           /*网络通信类*/
  NetSend *m_pNetSend;           /*网络发送类*/
  QThread *m_pThreadNetSend;     /*承载网络发送线程的线程*/
  Record *m_pRecord;             /*记录类，为了传递对象给其他类，使用指针对象*/
  QTimer *m_pNetSendTimer;       /*100ms心跳以太网发送定时器*/
  AStar *m_pAStar;               /*路径规划处理类*/
  QThread *m_pThreadAstar;       /*承载控制线程的线程*/
  PosMeasureDlg m_dlgPosMeasure; /*位置量测对话框*/
  ConfigDialog m_dlgConfig;      /*配置文件对话框*/

  ChassisLightHornWiperDialog *chassis_light_horn_wiper_dlg_ptr_ = nullptr;
  VehicleWireControl * vehicle_wire_control_ptr_ = nullptr;
  platoondlg* platoondlg_ptr_ = nullptr;
  UpdateFile* updatefile_ptr_ = nullptr;



  MapFileGeneration m_dlgMapFileGenration;
  QTimer *m_timerUIInit;       /*10ms等待界面初始化完成的定时器*/
  QTimer *m_timerTimeDisp;     /*1s系统时间及运行时间显示*/
  UINT32 m_uiRunTime;          /*已运行时间*/
  QTimer *m_timerAnomalyDisp;  /*异常信息显示定时器*/
  QTimer *m_timerTaskPointPub; /*任务下发计时器*/

  bool m_bMapInfoValid;   /*地图信息有效标志，每次点击打开文件按钮，将其置为false*/
  bool m_bMapInfoRepaint; /*地图内的信息需重新绘制*/

  UINT8 m_ucVehENUPosValid;  /*车辆ENU位置有效*/
  ENUCorST m_stENUPathStart; /*马路起点*/
  ENUCorST m_stENUPathPrependPt, m_stENUVehFrontWheel, m_stENUVehCenterPos, m_stENUVehBackWheel,
      m_stLstRecordENUVehCenterPos; /*路径前置跟踪点、车辆前后轮，上周期记录的后轮位置在ENU坐标系下的位置*/
  double m_fVehCourse;              /*车辆航向*/
  QString m_strChasCmdText;         /*综控向底盘发送的命令文本*/
  float m_fVehTurnAngle;            /*车辆前轮转角*/
  UINT8 m_ucMoveCtrlFlag;           /*运动控制标志*/

  QFile m_objPathFile;          /*路径文件*/
  QString m_strCurOpenFileName; /*当前打开的文件名称*/

  double m_fMapZoomArray
      [10];                   /*地图缩放级数数组，即1个栅格等于多少像素.此处必须用double，使用float会导致精度低，明明是2.0，但是乘以很大1000000以上的数后，就多1了。*/
  double m_fMapMeterPerPixel; /*当前地图缩放级数,即1个像素等于多少米*/
  double m_fPixelPerGrid;     /*当前1个栅格等于多少像素*/

  bool m_bCorditBindAreaDisp; /*地图坐标系装订编辑区和显示区显示标志*/

  QTimer *m_timerMoveCtrl; /*50ms遥控运动控制定时器*/

  bool m_bMousePressed;   /*鼠标按下标志*/
  QPoint m_PressPosition; /*鼠标按下的位置*/

  bool m_bMoveStart;                         /*运动开始标志*/
  bool m_bMoveStartFirstComputeCurrentPoint; /*第一次计算当前点*/

  INT32 m_iCurrenPointIndex;      /*当前点的index*/
  INT32 m_iLastCurrentPointIndex; /*保留的上一次当前点的index*/

  qint64 m_InitQDateTimeSinceMSecond;

  QString m_sEllipseTime;
  UINT16 m_usUTCYear;  /*UTC年*/
  UINT8 m_ucUTCMonth;  /*UTC月*/
  UINT8 m_ucUTCDay;    /*UTC日*/
  UINT8 m_ucUTCHour;   /*UTC时*/
  UINT8 m_ucUTCMinute; /*UTC分*/
  UINT8 m_ucUTCSecInt; /*UTC整秒*/
  UINT8 m_ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/

  double m_fSelectPointX;
  double m_fSelectPointY;
  bool m_bPointSelect;
  double lon_task_point_get;
  double lat_task_point_get;

  QTimer *m_timerOneKeyStartMotion; /*100ms一键运动开始定时器，20220117 qixianyu*/
  int m_iOneKeyStartMotionCounter;  /*定时器回调计数器，20220117 qixianyu*/
  uint8 m_uiPlanMode = 0;

signals:
  void SendNetMsg(QByteArray datagram, INT32 iLen, QHostAddress DestIP, UINT16 usPort); /*向NetComm网络发送函数*/
  void WriteRecord(const QString &strInfo, UINT8 ucFlag);                               /*改成发送信号形式，避免在记录线程中冲突，耽误时间*/
  void NetSendInit();                                                                   /*NetSend类初始化信号*/
  void AckMsg(QByteArray datagram, INT32 iLen);                                         /*接收确认报文处理*/
  void ReadMapFile(const QString filePath, const QString fileName);                     /*读取地图文件*/

  void sendAduModeCmd(uint8 vehicle_adu_mode);
  void sendDriveModeCmd(uint8 drive_mode);
  void sendTargetGearCmd(uint8 gear_cmd);
  void sendMaxSpeedLimitCmd(double max_speed);
  void sendEngineCmd(bool engine_cmd);
  // void sendTargetAccPctAndDeAccCmd(float target_acc_pct, float target_de_acc);         // change

  void sendTargetThrottleAndBrakePct(float throttle_pct, float brake_pct);         // change

  void sendTargetSteeringAngleAndAngleSpeedCmd(float target_angle, float angle_speed); // to change
  void sendParkBrakeCmd(bool cmd);
  void sendEmcyBrakeCmd(bool cmd);
  void sendLeftLightCmd(bool cmd);
  void sendRightLightCmd(bool cmd);
  void sendEmcyFlasherCmd(bool cmd);
  void sendLowBeamCmd(bool cmd);
  void sendHighBeamCmd(bool cmd);
  void sendHonkCmd(bool cmd);
  void sendTaskdist(uint8 taskdistclick);
  void sendTaskstart(uint8 taskdistclick);
  void sendTasksuspend(uint8 taskdistclick);
  void sendTaskstop(uint8 taskdistclick);

  void sendTaskPointGet(double lon, double lat, uint8 taskpointgetclicked);

  void sendReferencePath(QList<SmoothPathPointST>);
  void sendReferencePathCheckCmd();
  void sendMotionStart(uint8 cmd);
  void sendRemoteDrive(RemoteDriveST data);
  void sendVehicleResetCmd();
  void sendTaskPoints(taskPoints_msgs::taskPoints task_list);
  void emittaskpoints(std::vector<TaskPointST> task_points_, uint8 tasktype);

  void sendControlMode(uint8 mode);
  void sendPlanningMode(uint8 mode);

  //轨迹录制信号
  void sendRecordCmd(int recordcmd);

  //发送倒车信号
  void sendBackWordCmd(int backwordcmd);

  void Init();
  //  std::vector<std::vector< std::pair<float, float>> obs_vertex_; /*障碍物多边形报文件*/
  //  std::vector<int>  obs_vertex_number_;

  void sendObstacle(std::vector<std::vector<std::pair<float, float>>> obs);

private slots:
  void on_btnWndClose_clicked(); /*窗口关闭按钮单击槽函数*/
                                 //  void on_radioMoveByMan_clicked();      /*运动模式_遥控模式单选框单击槽函数*/
                                 //  void on_radioMovePathTrack_clicked();  /*运动模式_路径跟踪单选框单击槽函数*/
                                 //  void on_radioMoveCourseKeep_clicked(); /*运动模式_航向不变单选框单击槽函数*/
  //  void on_sliderVehSpeedCmd_valueChanged(int value);       /*遥控速度滚动条值变化槽函数*/
  //  void on_sliderTurnAngleCmd_valueChanged(int value);      /*遥控转向角滚动条值变化槽函数*/
  //  void on_sliderTurnAngleSpeedCmd_valueChanged(int value); /*遥控转向角速度滚动条值变化槽函数*/

  void timerTimeDispFunc(); /*系统时间显示定时器*/
  void timerUIInitFunc();   /*等待界面初始化完成延时定时器*/
  void timerMoveCtrlFunc(); /*运动控制定时器处理*/
  // void timerTaskPointFunc();   /*任务下发定时器处理*/
  // void timerAnomalyDispFunc(); /*异常信息显示定时器处理*/

  void RecvMsgProc(QByteArray datagram, INT32 iLen);                         /*接收NetComm的网络报文处理*/
  void RecvRemoteControlAutoDrivingMsgProc(QByteArray datagram, INT32 iLen); /*接收到综控的网络报文*/
  void RecvRemoteControlChassisMsgProc(QByteArray datagram, INT32 iLen);     /*接收到综控的网络报文*/
  void RecvPerceptionMsgProc(QByteArray datagram, INT32 iLen);               /*接收到综控的网络报文*/
  void RecvRemoteMsgProc(QByteArray datagram, INT32 iLen);                   /*接收到综控的网络报文*/
  void RecvRemoteDriveMsgProc(QByteArray datagram, INT32 iLen);              // 遥控驾驶控制
  void TaskpointsMsgProc(QByteArray datagram, INT32 iLen);                   /*接收到综控的网络报文*/

  void NetSendTimer();                                                                  /*以太网心跳报文发送定时器*/
  void MsgTableDisp(QString strCol2, QString strCol3, UINT8 ucDispType = DISP_PROCESS); /*报文表格显示*/

  bool eventFilter(QObject *watched, QEvent *event) Q_DECL_OVERRIDE; /*scroll区重绘重载函数*/
  void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;               /*鼠标移动重载函数*/
  void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;              /*鼠标按下重载函数*/
  void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;            /*鼠标释放重载函数*/
  void on_btnGoToVehPos_clicked();                                   /*将车辆位置挪至地图的中心*/

  void on_btnGridMapFileOpen_clicked(); /*栅格地图打开按钮点击槽函数*/
  void on_btnPathPlan_clicked();        /*路径规划按钮点击槽函数*/
  void on_btnMapAndPathBind_clicked();  /*地图坐标系和路径装订按钮单击槽函数*/
  void on_btnMapAndPathAsk_clicked();   /*地图坐标系和路径查询按钮单击槽函数*/
  void on_btnPathFileOpen_clicked();    /*路径文件打开按钮单击槽函数*/
  void on_btnPosMeasure_clicked();      /*位置量测按钮单击槽函数*/

  void on_sliderMapZoom_valueChanged(int value); /*地图缩放滑动条值变化槽函数*/

  void ReadMapFileRslt(bool bRslt); /*读取地图文件结果槽函数*/

  void MapOriginSet(LongLatHeightST stLLHOrigin);            /*设定了地图原点信号*/
  void PathCollectDisp(QList<SmoothPathPointST> listENUPos); /*采集到的路径点进行显示*/

  //  void on_btnEngineStart_clicked();              /*发动机启动按钮单击槽函数*/
  //  void on_btnEngineStop_clicked();               /*发动机停止按钮单击槽函数*/
  void on_btnGearSet_clicked();         /*档位设置按钮单击槽函数*/
                                        //  void on_btnParkCtrl_clicked();                 /*驻车制动按钮单击槽函数*/
                                        //  void on_btnReleasePark_clicked();              /*解除驻车按钮单击槽函数*/
  void on_btnChasWorkModeSet_clicked(); /*底盘被控模式按钮单击槽函数*/
  void on_btnMoveStart_clicked();       /*运动开始按钮单击槽函数*/
  void on_btnMoveStop_clicked();        /*运动停止按钮单击槽函数*/
  void on_btnParkBrak_clicked();        /*紧急制动按钮单击槽函数*/
                                        //  void on_btnCorditBindAreaDispSwitch_clicked(); /*地图坐标系装订编辑区和显示区相互切换按钮单击槽函数*/
  void on_btnMapCordtEnter_clicked();   /*坐标系数据确认输入按钮单击槽函数*/
  //  void on_btnTurnAngleCmdReset_clicked(); /*转向角回零按钮单击槽函数*/
  void on_btnTabelInfoClear_clicked(); /*信息显示列表清空*/

  void onTrackPathTableVSliderMove(INT32 iValue);    /*规划路径列表竖滚条滚动槽函数*/
  void onPathBindRsltTableVSliderMove(INT32 iValue); /*路径装订结果反馈列表竖滚条滚动槽函数*/

  void on_btnClearPath_clicked(); /*车辆轨迹清空按钮单击槽函数*/

  void on_btConfigDlg_clicked();

  void on_btnMapFileGeneration_clicked();

  void on_tableTrackPathInfo_cellPressed(int row, int column);

  //  void on_btnOneKeyStartMotion_clicked();

  void onOneKeyStartMotionTimerCallBack();

  void onHeartBeatTimerCallBack();

  void on_btnStartCommunication_clicked();

  void on_btnStopCommunication_clicked();

  void onXW5651(GNSS_IMU_ST msg);
  void onVehicleChassisState(VehicleChassisStateST msg);
  void onVehicleMotionState(VehicleMotionStateST msg);
  void onReferencePathFeedback(QList<SmoothPathPointST> msg);

  void onTrajectory(std::vector<GlobalPositionST> listPoint);
  void onGlobalPath84(std::vector<localPositionST> listPoint);
  void onDilixinxi(DilixinxiST listPoint);
  void onObstacle(std::vector<ObstacleST> listPoint);
  void onHeartBeat(HeartBeatST heart_beat);

  // void onLightHornWiperTopicCallback(ChassisLightHornWiper state);

  // void onControlError(ControlErrorST msg);
  // void onLocalPath(QList<SmoothPathPointST> mgs);

  //  void on_btnEmcyReset_clicked();

  // void on_btnLeftLight_clicked();

  // void on_btnRightLight_clicked();

  // void on_btnDoubleLight_clicked();

  // void on_btnHighBeam_clicked();

  // void on_btnLowBeam_clicked();

  // void on_btnHorn_clicked();

  void on_btnLockThirdPersonView_clicked();

  void on_btnUnLockThirdPersonView_clicked();

  void on_btnResetVehicleState_clicked();

  //  void on_btnSendSteerWheelAndThrottleCmd_clicked();

  void on_btnSendSteerWheelCmd_clicked();

  void on_btnSendThrottleCmd_clicked();

  //  void on_btnSendStopCmd_clicked();

  //  void on_btnComputeFrontAngle_clicked();

  // void on_btnConfirmMode_clicked();

  //  void on_btnPlanMode_clicked();

  void on_btntaskdist_clicked();

  void on_btntaskstart_clicked();

  void on_btntasksuspend_clicked();

  void on_btntaskstop_clicked();

  void on_taskpointget_clicked();

  void on_btnTest_clicked();
  void on_btnParkingBrakeOn_clicked();

  void on_btnParkingBrakeOff_clicked();



  void on_btnWireControl_clicked();


  void on_btnSwarm_clicked();
  void on_btnUpdate_clicked();//进入文件更新界面
  void close_update();//关闭文件更新界面

  void on_btnRecoad_clicked();//开始/结束轨迹录制

  void on_btnbackword_clicked();//开始/停止倒车
private:
  Ui::ParentDlg *ui;
  // RCLCommon* rcl_common_ = nullptr;
  QRosNode *qnode_ = nullptr;
  std::mutex xw5651_mutex_;
  GNSS_IMU_ST xw5651_msg_;

  std::mutex vehicle_chassis_state_mutex_;
  VehicleChassisStateST vehicle_chassis_state_msg_;

  std::mutex vehicle_motion_state_mutex_;
  VehicleMotionStateST vehicle_motion_state_msg_;

  bool is_lock_third_person_view_ = false;
  bool is_move_forward_ = true;

  bool m_bInit;

  ControlErrorST control_error_msg_;
  std::mutex control_error_msg_mutex_;
  UINT8 driving_cmd_mode_ = 0;
  std::string FLAGS_PNC_IP;                        /*与PAD通信的决策控制单元以太网IP*/
  std::string FLAGS_PAD_IP_OWN;                    /*与决策控制单元通信的PAD以太网IP*/
  std::string FLAGS_PAD_IP_INDEX1;                 /*PAD群通信以太网IP起始*/
  std::string FLAGS_PAD_IP_GROUP = "192.168.1.60"; /*PAD群通信组播地址*/
  unsigned short int FLAGS_PAD_PORT = 9007;        /*与决策控制单元通信的PAD以太网端口*/

  UINT32 m_uiRemoteControlToPadHeartNo = 0; /*发送底盘报文序号*/
  UINT32 m_uiPadToRemoteControlHeartNo = 0; /*发送底盘报文序号*/

  UINT32 m_uiSendChassisNo = 0;        /*发送底盘报文序号*/
  UINT32 m_uiSendInsNo = 0;            /*发送底盘报文序号*/
  UINT32 m_uiSendChassisNoThres = 0;   /*发送底盘报文序号*/
  UINT32 m_uiSendInsNoThres = 0;       /*发送底盘报文序号*/
  UINT32 m_uiSendDilixinxiNoThres = 0; /*发送地理信息报文序号*/
  UINT32 m_uiSendDilixinxiNo = 0;      /*发送地理信息报文序号*/
  UINT32 m_uiSendObstacleNo = 0;
  UINT32 m_uiSendObstacleNoThres = 0;

  unsigned int m_uiRecvObsClusterMsgNum = 0;   /*接收到障碍聚类报文信息计数*/
  unsigned int m_uiRecvObsClusterFrameCnt = 0; /*接收到的障碍的数据帧号计数*/
  unsigned int m_uiObsClusterFrameNum = 0;     /*障碍的总数据包数*/
  std::list<ObsClusterInfoST> m_listObsClusterInfo;
  double perception_vehicle_x_ = 0.0;
  double perception_vehicle_y_ = 0.0;
  double perception_vehicle_ins_heading_ = 0.0;

  std::vector<std::vector<std::pair<float, float>>> obs_;   /*障碍物多边形报文件*/
  std::vector<std::pair<float, float>> obs_vertex_one_dim_; /*障碍物多边形报文件*/
  std::vector<TaskPointST> task_points_;

  std::vector<int> obs_vertex_number_;
  unsigned int m_uiRecvObsMsgNum = 0;   /*接收到障碍聚类报文信息计数*/
  unsigned int m_uiRecvObsFrameCnt = 0; /*接收到的障碍的数据帧号计数*/
  unsigned int m_uiObsFrameNum = 0;     /*障碍的总数据包数*/
  unsigned int m_uiSendMsgNo;           /*发送报文序号*/

  QList<SmoothPathPointST> m_listLocalPath;    /*存储接收的局部路径*/
  std::vector<GlobalPositionST> m_PNC2PADPath; /*PNC发送到遥控端的路径/轨迹数据*/
  std::mutex pnc2pad_move_data_mutex_;
  PNC2PadMoveDataMsgST m_stPNC2PadMoveDataMsg; /*存储下来，用于100ms定时器发送*/
  bool rec_task_ = false;

  QTime heart_beat_ref_time_;
  QTime heart_beat_planning_time_;
  QTime heart_beat_control_time_;
  QTime heart_beat_estop_;
  QTime heart_beat_lidar_pos_;
  QTime heart_beat_lidar_neg_;

  QTime heart_beat_radar_;
  QTime heart_beat_camera_sense_;
  QTime heart_beat_camera_segment_;

  QTimer *m_pHearBeatTimer; /*心跳定时器*/


  bool is_vehicle_test_dlg_pressed_ =false;
  bool is_vehicle_wire_control_pressed_ =false;
  bool is_swarm_pressed_ =false;
  bool is_update_pressed_ =false;

};

#endif // PARENTDLG_H
