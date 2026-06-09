#include "pad/parentdlg.h"
#include "ui_parentdlg.h"
#include "pad/allglobal.h"

// #include "pcl/sample_consensus/ransac.h"
// #include "pcl/sample_consensus/sac_model_circle.h"
#include <vector>
#include <iostream>

ParentDlg::ParentDlg(int argc, char **argv, QWidget *parent) : QDialog(parent), ui(new Ui::ParentDlg)
{
  /*加载界面显示*/
  ui->setupUi(this);

  QSize deskRect = QGuiApplication::screens().at(0)->availableSize(); /*获取可用区域大小*/

  // setWindowFlags(Qt::FramelessWindowHint); /*设置窗口边框隐藏*/
  setGeometry(0, 0, deskRect.width() - 50, deskRect.height());

  // 设置窗体最大化和最小化
  Qt::WindowFlags windowFlag  = Qt::Window;
  windowFlag |= Qt::WindowMinimizeButtonHint;
  windowFlag |= Qt::WindowCloseButtonHint;
  setWindowFlags(windowFlag);

  QPalette p = this->palette();
  p.setColor(QPalette::Window, QColor(35, 35, 35));
  this->setPalette(p); /*设置窗口背景颜色*/

  /*全局变量赋初始值*/
  memset(&g_stMapCorditCmd, 0, sizeof(ENUCorST));
  g_bSetMapOrigin = false; /*是否选定了原点的标志*/

  //  ReadConfigFile(); /*读取配置文件*/

  /*第1步，开一个系统时间显示的定时器*/
  m_timerTimeDisp = new QTimer(this);
  connect(m_timerTimeDisp, SIGNAL(timeout()), this, SLOT(timerTimeDispFunc()));
  m_uiRunTime = 0; /*已运行时间*/

  /*第2步，实例记录对象，开启记录线程*/
  //  m_pRecord = new Record(this); /*创建记录类*/
  //  connect(m_pRecord, &Record::finished, m_pRecord, &QObject::deleteLater);
  //  m_pRecord->Init();
  //  m_pRecord->write("记录线程开启", RECORD_PROCESS);

  /*第3步，初始化Net端口，开启以太网接收线程*/
  m_pNetRecv = new NetRecv(this);
  connect(m_pNetRecv, &NetRecv::finished, m_pNetRecv, &QObject::deleteLater);

  /*第4步，初始化以太网发送线程*/
  m_pThreadNetSend = new QThread;
  m_pNetSend = new NetSend;
  m_pNetSend->moveToThread(m_pThreadNetSend);
  connect(m_pThreadNetSend, &QThread::finished, m_pNetSend, &QObject::deleteLater);

  /*第5步，开启地图和路径操作线程*/
  m_pThreadAstar = new QThread;
  m_pAStar = new AStar;
  m_pAStar->moveToThread(m_pThreadAstar);
  connect(m_pThreadAstar, &QThread::finished, m_pAStar, &QObject::deleteLater);

  /*开启文件更新界面*/
  updatefile_ptr_ = new UpdateFile(this);
  updatefile_ptr_->hide();

  /*开一个10ms延时定时器，等待网格重绘完成*/
  m_timerUIInit = new QTimer(this);
  connect(m_timerUIInit, SIGNAL(timeout()), this, SLOT(timerUIInitFunc()));

  /*开100ms心跳报文发送定时器*/
  m_pNetSendTimer = new QTimer(this);
  m_pNetSendTimer->setTimerType(Qt::PreciseTimer);
  connect(m_pNetSendTimer, SIGNAL(timeout()), this, SLOT(NetSendTimer()));

  /*实例化一个运动控制定时器*/
  m_timerMoveCtrl = new QTimer(this);
  m_timerMoveCtrl->setTimerType(Qt::PreciseTimer); /*高精度定时器*/
  connect(m_timerMoveCtrl, SIGNAL(timeout()), this, SLOT(timerMoveCtrlFunc()), Qt::DirectConnection);

  //  m_timerAnomalyDisp = new QTimer(this);              /*异常信息显示定时器*/
  //  m_timerAnomalyDisp->setTimerType(Qt::PreciseTimer); /*高精度定时器*/
  //  connect(m_timerAnomalyDisp, SIGNAL(timeout()), this, SLOT(timerAnomalyDispFunc()), Qt::DirectConnection);

  //  m_timerTaskPointPub = new QTimer(this);              /*异常信息显示定时器*/
  //  m_timerTaskPointPub->setTimerType(Qt::PreciseTimer); /*高精度定时器*/

  //  connect(m_timerTaskPointPub, SIGNAL(timeout()), this, SLOT(timerTaskPointFunc()), Qt::DirectConnection);

  //  m_timerTaskPointPub->start(100);

  m_listVehPos.clear();              /*车辆位置列表*/
  m_listTrackPathCmd.clear();        /*要跟踪的路径数据点*/
  m_listTrackPathBindRslt.clear();   /*综控中装订的跟踪路径点*/
  m_bTrackPathBindRsltValid = false; /*路径装订结果,默认无效*/

  m_uiTrackPathFrameNum = 0;          /*要跟踪的路径数据点的总数据包数*/
  m_uiTrackPathTotalBytes = 0;        /*要跟踪的路径数据点的数据总字节数*/
  m_uiRecvTrackPathFrameCnt = 0;      /*接收到的要跟踪的路径数据点的数据帧号计数*/
  m_uiRecvTrackPathTotalBytesCnt = 0; /*要跟踪的路径数据点的数据总字节数自计数*/

  m_uiLstRecordRecvHeartMsgNum = 0;   /*上周期记录到的接收到心跳报文计数*/
  m_uiRecvHeartMsgNum = 0;            /*接收到心跳报文计数*/
  m_uiRecvPathBindRsltMsgNum = 0;     /*接收到路径装订结果报文计数*/
  m_uiRecvMapCordtBindRlstMsgNum = 0; /*接收到地图坐标系装订结果报文计数*/
  m_uiRecvAbNormlaMsgNum = 0;         /*接收到异常报文报文计数*/
  m_uiRecvAckMsgNum = 0;              /*接收到接收确认报文计数*/
  m_uiRecvCmdAckMsgNum = 0;           /*接收到命令应答透传报文计数*/
  m_uiRecvStateMsgNum = 0;            /*接收到状态透传报文计数*/

  /*发送报文数显示，属于调试信息*/
  m_uiSendHeartMsgNum = 0;        /*发送心跳报文计数*/
  m_uiSendPathBindMsgNum = 0;     /*发送路径装订报文计数*/
  m_uiSendMapCordtBindMsgNum = 0; /*发送运动坐标系装订报文计数*/
  m_uiSendPathAskMsgNum = 0;      /*发送跟踪路径信息查询报文计数*/
  m_uiSendMapCordtAskMsgNum = 0;  /*发送运动坐标系查询报文计数*/
  m_uiSendAckMsgNum = 0;          /*发送接收确认报文计数*/
  m_uiSendChasCtrlMsgNum = 0;     /*发送底盘设备控制报文计数*/
  m_uiSendMoveCtrlMsgNum = 0;     /*发送底盘运动控制报文计数*/

  m_strErrorInfo.clear(); /*错误信息提示*/

  QStringList strListWorkMode;
  strListWorkMode << "5"
                  << "6"
                  << "7"
                  << "8"
                  << "9"
                  << "10";
  ui->comboCourseVehSpeed->addItems(strListWorkMode); /*巡航速度组框*/
  ui->comboCourseVehSpeed->setCurrentIndex(0);

  m_bCorditBindAreaDisp = true; /*地图坐标系装订编辑区和显示区相互切换，默认显示地图坐标系装订显示区*/

  /*地图缩放级数,序号越大，越放大，1个栅格覆盖几个像素*/
  m_fMapZoomArray[0] = 0.05;
  m_fMapZoomArray[1] = 0.1;
  m_fMapZoomArray[2] = 0.2;
  m_fMapZoomArray[3] = 0.4;
  m_fMapZoomArray[4] = 1;
  m_fMapZoomArray[5] = 2;
  m_fMapZoomArray[6] = 5;
  m_fMapZoomArray[7] = 10;
  m_fMapZoomArray[8] = 20;
  m_fMapZoomArray[9] = 30;

  ui->sliderMapZoom->setValue(5);
  m_fPixelPerGrid = m_fMapZoomArray[4]; /*默认1个栅格等于1个像素*/
  ui->labMapPixelPerGrid->setText(QString("%1P/G").arg(m_fPixelPerGrid, 0, 'f', 2));
  m_fMapMeterPerPixel = 0; /*当前地图缩放级数,即1个像素等于多少米*/

  m_bMapInfoRepaint = false;                /*地图需重新绘制*/
  ui->labGridMap->installEventFilter(this); /*为MapArea安装事件监控器*/
  m_bMapInfoValid = false;                  /*地图信息有效标志，每次点击打开文件按钮，将其置为false*/
  m_fVehCourse = 0;                         /*车辆航向*/
  m_fVehTurnAngle = 0;                      /*车辆前轮转角*/
  m_ucMoveCtrlFlag = 0;                     /*未处于运动控制中*/

  m_bMoveStart = false;
  m_bMoveStartFirstComputeCurrentPoint = false;
  m_iCurrenPointIndex = 0;
  m_iLastCurrentPointIndex = 0;

  m_usUTCYear = 0;   /*UTC年*/
  m_ucUTCMonth = 0;  /*UTC月*/
  m_ucUTCDay = 0;    /*UTC日*/
  m_ucUTCHour = 0;   /*UTC时*/
  m_ucUTCMinute = 0; /*UTC分*/
  m_ucUTCSecInt = 0; /*UTC整秒*/
  m_ucUTCSecDOt = 0; /*UTC秒小数部分，1LSB = 0.01s*/

  m_fSelectPointX = 0;
  m_fSelectPointY = 0;
  m_bPointSelect = false;

  m_ucVehENUPosValid = 0;                                     /*车辆ENU位置有效*/
  memset(&m_stENUPathStart, 0, sizeof(ENUCorST));             /*路径起点*/
  memset(&m_stENUPathPrependPt, 0, sizeof(ENUCorST));         /*车辆前置跟踪点*/
  memset(&m_stENUVehFrontWheel, 0, sizeof(ENUCorST));         /*车辆前轮位置*/
  memset(&m_stENUVehCenterPos, 0, sizeof(ENUCorST));          /*车辆质心位置*/
  memset(&m_stENUVehBackWheel, 0, sizeof(ENUCorST));          /*车辆后轮位置*/
  memset(&m_stLstRecordENUVehCenterPos, 0, sizeof(ENUCorST)); /*上周期记录的车辆后轮位置*/
  memset(&control_error_msg_, 0, sizeof(ControlErrorST));

  //  ui->grpManCtrl->hide(); /*初始默认隐藏遥控控制区*/

  //  ui->radioGearN->setChecked(true);           /*默认档位选择在N档*/
  //  ui->radioMovePathTrack->setChecked(true);   /*默认进行路径跟踪模式控制*/
  //  ui->radioChasCtrledByMan->setChecked(true); /*默认放在人控模式下*/

  //  ui->sliderVehSpeedCmd->setSingleStep(1);
  //  ui->sliderVehSpeedCmd->setMinimum(0);
  //  ui->sliderVehSpeedCmd->setMaximum(MAX_LINE_SPEED * 10);
  //  ui->sliderVehSpeedCmd->setValue(0);
  //  ui->sliderVehSpeedCmd->setTickPosition(QSlider::TicksAbove);

  //  ui->sliderTurnAngleCmd->setSingleStep(1);
  //  ui->sliderTurnAngleCmd->setMinimum(MIN_TURN_ANGLE * 10);
  //  ui->sliderTurnAngleCmd->setMaximum(MAX_TURN_ANGLE * 10);
  //  ui->sliderTurnAngleCmd->setValue(0);
  //  ui->sliderTurnAngleCmd->setTickPosition(QSlider::TicksAbove);

  //  ui->sliderTurnAngleSpeedCmd->setSingleStep(1);
  //  ui->sliderTurnAngleSpeedCmd->setMinimum(0);
  //  ui->sliderTurnAngleSpeedCmd->setMaximum(TURN_ANGLE_SPEED * 10);
  //  ui->sliderTurnAngleSpeedCmd->setValue(0);
  //  ui->sliderTurnAngleSpeedCmd->setTickPosition(QSlider::TicksAbove);

  //  ui->labVehSpeedCmd->setText("0");       /*遥控运动速度命令值*/
  //  ui->labTurnAngleCmd->setText("0");      /*遥控转向角度命令值*/
  //  ui->labTurnAngleSpeedCmd->setText("0"); /*遥控转向角速度命令值*/

  //  ui->grpMapCorditBindDisp->hide(); /*默认隐藏坐标系编辑区*/ 0

  QStringList strTableHeaderText;
  strTableHeaderText << "时戳"
                     << "信源"
                     << "信息";
  ui->tableMsgDisp->setColumnCount(
      strTableHeaderText.count()); /*不设置列数量，则列标题无法显示。并且要放在设置列标题前面*/
  ui->tableMsgDisp->setHorizontalHeaderLabels(strTableHeaderText);
  ui->tableMsgDisp->setEditTriggers(QAbstractItemView::NoEditTriggers);   /*设置表格不可编辑*/
  ui->tableMsgDisp->setSelectionBehavior(QAbstractItemView::SelectRows);  /*设置按行选择*/
  ui->tableMsgDisp->setSelectionMode(QAbstractItemView::SingleSelection); /*设置单行选择*/
  ui->tableMsgDisp->setAlternatingRowColors(false);                       /*设置行间隔颜色*/
  ui->tableMsgDisp->setRowCount(0);
  ui->tableMsgDisp->horizontalHeader()->setVisible(true); /*设置列表头显示*/
  ui->tableMsgDisp->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
  ui->tableMsgDisp->verticalHeader()->setVisible(false); /*设置行表头显示*/
  //    ui->tableMsgDisp->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  //    ui->tableMsgDisp->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  //    ui->tableMsgDisp->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  ui->tableMsgDisp->horizontalHeaderItem(0)->setTextAlignment(Qt::AlignLeft);
  ui->tableMsgDisp->horizontalHeaderItem(1)->setTextAlignment(Qt::AlignLeft);
  ui->tableMsgDisp->horizontalHeaderItem(2)->setTextAlignment(Qt::AlignLeft);
  ui->tableMsgDisp->setFocusPolicy(Qt::NoFocus); /*设置选择单元格不出现虚框*/

  strTableHeaderText.clear();
  strTableHeaderText << "序号"
                     << "ENUX"
                     << "ENUY"
                     << "T(s)";
  ui->tableTrackPathInfo->setColumnCount(
      strTableHeaderText.count()); /*不设置列数量，则列标题无法显示。并且要放在设置列标题前面*/
  ui->tableTrackPathInfo->setHorizontalHeaderLabels(strTableHeaderText);
  ui->tableTrackPathInfo->setEditTriggers(QAbstractItemView::NoEditTriggers);   /*设置表格不可编辑*/
  ui->tableTrackPathInfo->setSelectionBehavior(QAbstractItemView::SelectRows);  /*设置按行选择*/
  ui->tableTrackPathInfo->setSelectionMode(QAbstractItemView::SingleSelection); /*设置单行选择*/
  ui->tableTrackPathInfo->setAlternatingRowColors(false);                       /*设置行间隔颜色*/
  ui->tableTrackPathInfo->setRowCount(0);
  ui->tableTrackPathInfo->horizontalHeader()->setVisible(true); /*设置列表头显示*/
  ui->tableTrackPathInfo->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
  ui->tableTrackPathInfo->verticalHeader()->setVisible(false); /*设置行表头显示*/

  ui->tableTrackPathInfo->horizontalHeaderItem(0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  ui->tableTrackPathInfo->horizontalHeaderItem(1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

  // ui->tableTrackPathInfo->horizontalHeadm_uiMoveDataMsgNoerItem(1)->setTextAlignment(Qt::AlignCenter |
  // Qt::AlignVCenter);
  ui->tableTrackPathInfo->horizontalHeaderItem(2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  ui->tableTrackPathInfo->horizontalHeaderItem(3)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

  ui->tableTrackPathInfo->setFocusPolicy(Qt::NoFocus); /*设置选择单元格不出现虚框*/

  strTableHeaderText.clear();
  strTableHeaderText << "序号"
                     << "ENUX"
                     << "ENUY";
  ui->tablePathBindRsltInfo->setColumnCount(
      strTableHeaderText.count()); /*不设置列数量，则列标题无法显示。并且要放在设置列标题前面*/
  ui->tablePathBindRsltInfo->setHorizontalHeaderLabels(strTableHeaderText);
  ui->tablePathBindRsltInfo->setEditTriggers(QAbstractItemView::NoEditTriggers);   /*设置表格不可编辑*/
  ui->tablePathBindRsltInfo->setSelectionBehavior(QAbstractItemView::SelectRows);  /*设置按行选择*/
  ui->tablePathBindRsltInfo->setSelectionMode(QAbstractItemView::SingleSelection); /*设置单行选择*/
  ui->tablePathBindRsltInfo->setAlternatingRowColors(false);                       /*设置行间隔颜色*/
  ui->tablePathBindRsltInfo->setRowCount(0);
  ui->tablePathBindRsltInfo->horizontalHeader()->setVisible(true); /*设置列表头显示*/
  ui->tablePathBindRsltInfo->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
  ui->tablePathBindRsltInfo->verticalHeader()->setVisible(false); /*设置行表头显示*/

  ui->tablePathBindRsltInfo->horizontalHeaderItem(0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  ui->tablePathBindRsltInfo->horizontalHeaderItem(1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  ui->tablePathBindRsltInfo->horizontalHeaderItem(2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  ui->tablePathBindRsltInfo->setFocusPolicy(Qt::NoFocus); /*设置选择单元格不出现虚框*/

  // connect((QObject *)(ui->tableTrackPathInfo->verticalScrollBar()), SIGNAL(valueChanged(INT32)), this,
  //         SLOT(onPathBindRsltTableVSliderMove(INT32)));
  // connect((QObject *)(ui->tablePathBindRsltInfo->verticalScrollBar()), SIGNAL(valueChanged(INT32)), this,
  //         SLOT(onTrackPathTableVSliderMove(INT32)));

  // EqStateDispColor(ui->labVCState, 0); /*设备状态初始化显示*/
  // EqStateDispColor(ui->labChasState, 0);
  // EqStateDispColor(ui->labMemsState, 0);
  // EqStateDispColor(ui->labPercepState, 0);

  // ChasDetailStateDispColor(ui->labEMSState, 0); /*底盘分设备状态显示*/
  // ChasDetailStateDispColor(ui->labEBSState, 0);
  // ChasDetailStateDispColor(ui->labEPBState, 0);
  // ChasDetailStateDispColor(ui->labEPSState, 0);
  // ChasDetailStateDispColor(ui->labTCUState, 0);
  // ChasDetailStateDispColor(ui->labChasCtrlerState, 0);
  // ChasDetailStateDispColor(ui->labBCMState, 0);

  //  connect(this, SIGNAL(WriteRecord(QString, UINT8)), m_pRecord, SLOT(write(QString, UINT8))); /*写记录*/
  // connect(m_pNetRecv, SIGNAL(MsgTableDispSign(QString, QString, UINT8)), this,
  //         SLOT(MsgTableDisp(QString, QString, UINT8))); /*界面显示*/
  //  connect(m_pNetRecv, SIGNAL(RecvVCNetMsg(QByteArray, INT32)), this, SLOT(RecvMsgProc(QByteArray, INT32)));

  connect(m_pNetRecv, SIGNAL(RecvRemoteControlAutoDrivingMsg(QByteArray, INT32)), this,
          SLOT(RecvRemoteControlAutoDrivingMsgProc(QByteArray, INT32)));

  connect(m_pNetRecv, SIGNAL(RecvRemoteMsg(QByteArray, INT32)), this,
          SLOT(RecvRemoteMsgProc(QByteArray, INT32)));

  connect(m_pNetRecv, SIGNAL(RecvRemoteDriveMsg(QByteArray, INT32)), this,
          SLOT(RecvRemoteDriveMsgProc(QByteArray, INT32)));

  connect(m_pNetRecv, SIGNAL(RecvTaskpointsMsg(QByteArray, INT32)), this,
          SLOT(TaskpointsMsgProc(QByteArray, INT32)));
  //  connect(m_pNetRecv, SIGNAL(RecvRemoteControlChassisMsg(QByteArray, INT32)), this,
  //          SLOT(RecvRemoteControlChassisMsgProc(QByteArray, INT32)));
  //  connect(m_pNetRecv, SIGNAL(RecvPerceptionMsg(QByteArray, INT32)), this,
  //          SLOT(RecvPerceptionMsgProc(QByteArray, INT32)));

  /*接收到网络报文，发送到战斗操作台处理报文*/

  // connect(m_pNetSend, SIGNAL(WriteRecord(QString, UINT8)), m_pRecord, SLOT(write(QString, UINT8))); /*写记录*/
  // connect(m_pNetSend, SIGNAL(MsgTableDispSign(QString, QString)), this,
  //         SLOT(MsgTableDisp(QString, QString))); /*界面显示*/

  // connect(this, SIGNAL(AckMsg(QByteArray, INT32)), m_pNetSend,
  //         SLOT(AckMsgProc(QByteArray, INT32))); /*以太网应答报文处理*/
  connect(this, SIGNAL(SendNetMsg(QByteArray, INT32, QHostAddress, UINT16)), m_pNetSend, SLOT(SendMsgListAdd(QByteArray, INT32, QHostAddress, UINT16))); /*发送以太网报文*/

  connect(this, SIGNAL(NetSendInit()), m_pNetSend, SLOT(Init()));

  connect(this, SIGNAL(ReadMapFile(const QString, const QString)), m_pAStar,
          SLOT(ReadMapFile(const QString, const QString))); /*读取地图文件信息命令*/

  connect(m_pAStar, SIGNAL(ReadMapFileRslt(bool)), this, SLOT(ReadMapFileRslt(bool))); /*地图打开成功信号*/
  connect(m_pAStar, SIGNAL(MsgTableDispSign(QString, QString, UINT8)), this,
          SLOT(MsgTableDisp(QString, QString, UINT8))); /*界面显示*/

  connect(&m_dlgPosMeasure, SIGNAL(MapOriginSet(LongLatHeightST)), this,
          SLOT(MapOriginSet(LongLatHeightST))); /*设置了地图原点标志*/
  connect(&m_dlgPosMeasure, SIGNAL(PathCollectDisp(QList<SmoothPathPointST>)), this,
          SLOT(PathCollectDisp(QList<SmoothPathPointST>))); /*设置了地图原点标志*/
  connect(m_pAStar, SIGNAL(MapOriginSet(LongLatHeightST)), this,
          SLOT(MapOriginSet(LongLatHeightST))); /*设置了地图原点标志*/

  m_timerTimeDisp->start(1000); /*开启定时器*/
                                //  m_pRecord->start();           /*开启run函数*/

  m_pThreadAstar->start();
  m_timerUIInit->start(10); /*开启定时器*/

  m_timerOneKeyStartMotion = new QTimer(this);
  m_timerOneKeyStartMotion->setTimerType(Qt::PreciseTimer);
  connect(m_timerOneKeyStartMotion, SIGNAL(timeout()), this, SLOT(onOneKeyStartMotionTimerCallBack()));
  m_iOneKeyStartMotionCounter = 0;

  m_pHearBeatTimer = new QTimer(this);
  m_pHearBeatTimer->setTimerType(Qt::PreciseTimer);
  connect(m_pHearBeatTimer, SIGNAL(timeout()), this, SLOT(onHeartBeatTimerCallBack()));

  qnode_ = new QRosNode(argc, argv);

  /*打开组队界面*/
  platoondlg_ptr_ = new platoondlg(this);
  platoondlg_ptr_->connectQnode(qnode_);
  platoondlg_ptr_->hide();
  //连接轨迹录制信号与槽
  connect(this, SIGNAL(sendRecordCmd(int)),
          qnode_, SLOT(OnSendRecordCmd(int)));
  //连接倒车命令信号与槽
  connect(this, SIGNAL(sendBackWordCmd(int)),
          qnode_, SLOT(OnSendBackWordCmd(int)));

  qDebug() << "init";

  connect(qnode_, SIGNAL(emitXW5651(GNSS_IMU_ST)), this, SLOT(onXW5651(GNSS_IMU_ST)));
  connect(qnode_, SIGNAL(emitVehicleChassisState(VehicleChassisStateST)), this,
          SLOT(onVehicleChassisState(VehicleChassisStateST)));
  connect(qnode_, SIGNAL(emitTrajectory(std::vector<GlobalPositionST>)), this,
          SLOT(onTrajectory(std::vector<GlobalPositionST>)));
  connect(qnode_, SIGNAL(emitGlobalPath84(std::vector<localPositionST>)), this,
          SLOT(onGlobalPath84(std::vector<localPositionST>)));
  connect(qnode_, SIGNAL(emitObstacle(std::vector<ObstacleST>)), this,
          SLOT(onObstacle(std::vector<ObstacleST>)));
  connect(qnode_, SIGNAL(emitDilixinxi(DilixinxiST)), this,
          SLOT(onDilixinxi(DilixinxiST)));
  connect(qnode_, SIGNAL(emitHeartBeat(HeartBeatST)), this,
          SLOT(onHeartBeat(HeartBeatST)));

  //  connect(qnode_, SIGNAL(emitVehicleMotionState(VehicleMotionStateST)), this,
  //          SLOT(onVehicleMotionState(VehicleMotionStateST)));

  //  connect(this, SIGNAL(sendAduModeCmd(uint8)), qnode_, SLOT(onSendAduModeCmd(uint8)));
  connect(this, SIGNAL(sendDriveModeCmd(uint8)), qnode_, SLOT(onSendDriveModeCmd(uint8)));

  connect(this, SIGNAL(sendTargetGearCmd(uint8)), qnode_, SLOT(onSendTargetGearCmd(uint8)));
  // connect(this, SIGNAL(sendMaxSpeedLimitCmd(double)), qnode_, SLOT(onSendMaxSpeedLimitCmd(double)));
  //  connect(this, SIGNAL(sendEngineCmd(bool)), qnode_, SLOT(onSendEngineCmd(bool)));
  connect(this, SIGNAL(sendTargetThrottleAndBrakePct(float, float)), qnode_,
          SLOT(onsendTargetThrottleAndBrakePct(float, float)));
  connect(this, SIGNAL(sendTaskdist(uint8)), qnode_,
          SLOT(onsendTaskdist(uint8)));

  connect(this, SIGNAL(sendTaskstart(uint8)), qnode_,
          SLOT(onsendTaskdist(uint8)));

  connect(this, SIGNAL(sendTasksuspend(uint8)), qnode_,
          SLOT(onsendTaskdist(uint8)));

  connect(this, SIGNAL(sendTaskstop(uint8)), qnode_,
          SLOT(onsendTaskdist(uint8)));

  connect(this, SIGNAL(sendTaskPointGet(double, double, uint8)), qnode_,
          SLOT(onsendTaskPointGet(double, double, uint8)));

  connect(this, SIGNAL(sendTargetSteeringAngleAndAngleSpeedCmd(float, float)), qnode_,
          SLOT(onSendTargetSteeringAngleAndAngleSpeedCmd(float, float)));
  connect(this, SIGNAL(sendParkBrakeCmd(bool)), qnode_, SLOT(onSendParkBrakeCmd(bool)));
  //  connect(this, SIGNAL(sendEmcyBrakeCmd(bool)), qnode_, SLOT(onSendEmcyBrakeCmd(bool)));
  // connect(this, SIGNAL(sendLeftLightCmd(bool)), qnode_, SLOT(onSendLeftLightCmd(bool)));
  // connect(this, SIGNAL(sendRightLightCmd(bool)), qnode_, SLOT(onSendRightLightCmd(bool)));
  // connect(this, SIGNAL(sendEmcyFlasherCmd(bool)), qnode_, SLOT(onSendEmcyFlasherCmd(bool)));
  // connect(this, SIGNAL(sendLowBeamCmd(bool)), qnode_, SLOT(onSendLowBeamCmd(bool)));
  // connect(this, SIGNAL(sendHighBeamCmd(bool)), qnode_, SLOT(onSendHighBeamCmd(bool)));
  // connect(this, SIGNAL(sendHonkCmd(bool)), qnode_, SLOT(onSendHonkCmd(bool)));
  //  connect(this, SIGNAL(sendReferencePath(QList<SmoothPathPointST>)), qnode_,
  //          SLOT(onSendReferencePath(QList<SmoothPathPointST>)));
  connect(this, SIGNAL(sendMotionStart(uint8)), qnode_, SLOT(onSendMotionStart(uint8)));
  connect(this, SIGNAL(sendVehicleResetCmd()), qnode_, SLOT(onSendVehicleResetCmd()));
  // connect(this, SIGNAL(sendTaskPoints(taskPoints_msgs::taskPoints)), qnode_, SLOT(onSendTaskPoints(taskPoints_msgs::taskPoints)));
  connect(this, SIGNAL(emittaskpoints(std::vector<TaskPointST>, uint8)), qnode_, SLOT(ontaskpoints(std::vector<TaskPointST>, uint8)));
  connect(this, SIGNAL(sendRemoteDrive(RemoteDriveST)), qnode_, SLOT(onSendRemoteDrive(RemoteDriveST)));

  //  connect(this, SIGNAL(sendReferencePathCheckCmd()), qnode_, SLOT(onSendReferencePathCheckCmd()));

  //  connect(qnode_, SIGNAL(emitReferencePathFeedback(QList<SmoothPathPointST>)), this,
  //          SLOT(onReferencePathFeedback(QList<SmoothPathPointST>)));

  //  connect(this, SIGNAL(sendObstacle(std::vector<std::vector<std::pair<float, float>>>)), qnode_,
  //          SLOT(onSendObstacle(std::vector<std::vector<std::pair<float, float>>>)));

  //  connect(rcl_common_, SIGNAL(emitControlError(ControlErrorST)), this, SLOT(onControlError(ControlErrorST)));
  //  connect(rcl_common_, SIGNAL(emitLocalPath(QList<SmoothPathPointST>)), this,
  //          SLOT(alPath(QList<SmoothPathPointST>)));

  //  connect(this, SIGNAL(sendPlanningMode(uint8)), qnode_, SLOT(onSendPlanningMode(uint8)));

  //  if (is_move_forward_)
  //  {
  //    ui->btnOneKeyStartMotion->setText("向前运动");
  //  }
  //  else
  //  {
  //    ui->btnOneKeyStartMotion->setText("后退运动");
  //  }

  m_uiSendChassisNo = 0;
  m_uiSendInsNo = 0;
  m_uiSendDilixinxiNo = 0;
  m_uiSendObstacleNo = 0;
  m_uiSendChassisNoThres = 10;
  m_uiSendInsNoThres = 5;
  m_uiSendDilixinxiNoThres = 5;
  m_uiSendObstacleNoThres = 5;
  m_uiPlanMode = 0;

  Init();

  heart_beat_ref_time_ = QTime::currentTime();
  heart_beat_planning_time_ = QTime::currentTime();
  heart_beat_control_time_ = QTime::currentTime();
  heart_beat_estop_ = QTime::currentTime();
  heart_beat_lidar_pos_ = QTime::currentTime();
  heart_beat_lidar_neg_ = QTime::currentTime();

  heart_beat_radar_ = QTime::currentTime();
  heart_beat_camera_sense_ = QTime::currentTime();
  heart_beat_camera_segment_ = QTime::currentTime();

  m_pHearBeatTimer->start(500);
  ui->btnStartCommunication->click(); // 默认开启软件就点击建立通信连接按钮
}

// void ParentDlg::alPath(QList<SmoothPathPointST> msg)
// {
//  if (m_uiPlanMode == 1)  //只有在避障模式才绘制
//  {
//    // 1 本地绘制
//    m_listLocalPath = msg;
//    DrawLocalPath();
//    // 2 发送到遥控器
//    m_PNC2PADPath.clear();
//    for (int i = 0; i < msg.size(); ++i)
//    {
//      GlobalPositionST point;
//      memset(&point, 0, sizeof(GlobalPositionST));
//      point.fX = msg.at(i).stENUPoint.x;
//      point.fY = msg.at(i).stENUPoint.y;
//      m_PNC2PADPath.push_back(point);
//    }
//    emit SendRouteAndLocalPathMsg(PATH_TYPE_LOCAL_PLAN, m_PNC2PADPath);
//  }
// }

void ParentDlg::DrawLocalPath()
{
  if ((0 == m_pAStar->m_iMapColNum) || (0 == m_pAStar->m_iMapRowNum) || (0 == m_fPixelPerGrid))
  {
    return;
  }

  QPainter painter(ui->labGridMap);
  painter.setPen(QPen(QColor(208, 78, 164, 255), 2));
  painter.setBrush(QColor(208, 78, 164, 255));
  for (INT32 i = 0; i < m_listLocalPath.size() - 1; i++)
  {
    GridMapIndexST stFrontGridIndex, stBackGridIndex;
    stFrontGridIndex.iGridColNo =
        fabs(m_listLocalPath.at(i).stENUPoint.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stFrontGridIndex.iGridRowNo =
        fabs(m_listLocalPath.at(i).stENUPoint.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    double fs = sqrt(pow((m_listLocalPath.at(i + 1).stENUPoint.x - m_listLocalPath.at(i).stENUPoint.x), 2) +
                     pow((m_listLocalPath.at(i + 1).stENUPoint.y - m_listLocalPath.at(i).stENUPoint.y), 2));
    INT32 j = i;
    while ((fs < 0.5) && (i < m_listLocalPath.size() - 2))
    {
      i++;
      fs = sqrt(pow((m_listLocalPath.at(i + 1).stENUPoint.x - m_listLocalPath.at(j).stENUPoint.x), 2) +
                pow((m_listLocalPath.at(i + 1).stENUPoint.y - m_listLocalPath.at(j).stENUPoint.y), 2));
    }

    stBackGridIndex.iGridColNo =
        fabs(m_listLocalPath.at(i + 1).stENUPoint.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stBackGridIndex.iGridRowNo =
        fabs(m_listLocalPath.at(i + 1).stENUPoint.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;

    INT32 iStartPXPosX, iStartPXPosY, iEndPXPosX, iEndPXPosY;
    iStartPXPosX = stFrontGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iStartPXPosY = stFrontGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iEndPXPosX = stBackGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iEndPXPosY = stBackGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

    painter.drawLine(QPoint(iStartPXPosX, iStartPXPosY), QPoint(iEndPXPosX, iEndPXPosY)); /*绘制路径点连线*/

    //        float fAngle2Y = Calt_A2B_North_Angle(m_listLocalPath.at(j).stENUPoint,
    //        m_listLocalPath.at(i+1).stENUPoint); ENUCorST stCenterPos; memset(&stCenterPos, 0, sizeof(ENUCorST));
    //        stCenterPos.x = m_listLocalPath.at(j).stENUPoint.x + fs/2.0*sin(fAngle2Y*DEGREE_RADIAN);
    //        stCenterPos.y = m_listLocalDynmPath.at(j).stENUPoint.y + fs/2.0*cos(fAngle2Y*DEGREE_RADIAN);
    //        /*对每个点画一个矩形*/
    //        if(fs > 0.5)
    //        {
    //            fs += 0.5;
    //            //DrawRotateRect(stCenterPos, fAngle2Y, ROAD_WIDTH, fs);
    //            /*计算矩形中心在控件中的x和y像素位置*/
    //            GridMapIndexST stGridVehIndex;
    //            stGridVehIndex.iGridColNo = (INT32)(fabs(stCenterPos.x -
    //            m_pAStar->m_fMapENU_XMin)/m_pAStar->m_fResolution); stGridVehIndex.iGridRowNo =
    //            (INT32)(fabs(stCenterPos.y - m_pAStar->m_fMapENU_YMax)/m_pAStar->m_fResolution); INT32 iPXVehPosX =
    //            (INT32)(stGridVehIndex.iGridColNo*m_fPixelPerGrid + m_fPixelPerGrid/2);/*控件中位置的X像素*/ INT32
    //            iPXVehPosY = (INT32)(stGridVehIndex.iGridRowNo*m_fPixelPerGrid +
    //            m_fPixelPerGrid/2);/*控件中位置的Y像素*/

    //            /*根据车辆航向，旋转车辆在地图中的姿态*/
    //            painter.resetTransform();/*重置变换*/
    //            painter.translate(iPXVehPosX, iPXVehPosY);/*坐标原点定到矩形中心*/
    //            float fRotateAngle;/*绕X轴旋转坐标系*/
    //            if(fAngle2Y <= 90)/*车辆航向角*/
    //            {
    //                fRotateAngle = fAngle2Y-90;/*逆时针旋转*/
    //            }
    //            else if(fAngle2Y <= 180)
    //            {
    //                fRotateAngle = fAngle2Y-90;/*顺时针旋转*/
    //            }
    //            else if(fAngle2Y <= 270)
    //            {
    //                fRotateAngle = fAngle2Y-90;/*顺时针旋转*/
    //            }
    //            else
    //            {
    //                fRotateAngle = fAngle2Y-450;/*逆时针旋转*/
    //            }
    //            painter.rotate(fRotateAngle);/*旋转坐标系*/
    //            if(0 != m_fMapMeterPerPixel)
    //            {
    //                painter.fillRect((-fs)/2.0/m_fMapMeterPerPixel, -VEHICLE_WIDTH/2.0/m_fMapMeterPerPixel,
    //                fs/m_fMapMeterPerPixel, VEHICLE_WIDTH/m_fMapMeterPerPixel, QBrush(QColor(255,128,64,115)));
    //            }
    //            painter.resetTransform();/*恢复translate和rotate对painter作的修改*/
    //        }
  }
}
// void ParentDlg::onControlError(ControlErrorST msg)
//{
//   control_error_msg_mutex_.lock();
//   control_error_msg_ = msg;
//   auto info = control_error_msg_;
//   control_error_msg_mutex_.unlock();

//  memset(&m_stENUPathPrependPt, 0, sizeof(ENUCorST)); /*车辆前置跟踪点*/
//  m_stENUPathPrependPt =
//      m_listTrackPathCmd.at(static_cast<int>(control_error_msg_.pp_look_ahead_point_index)).stENUPoint;

//  QString text = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13 %14 %15 %16 %17")
//                     .arg(info.engine)
//                     .arg(info.vehicle_gear)
//                     .arg(info.park_state)
//                     .arg(info.lat_error, 0, 'f', 3)
//                     .arg(info.pp_look_ahead_point_index)
//                     .arg(info.pp_look_ahead_dist, 0, 'f', 3)
//                     .arg(info.steer_angle_cmd)
//                     .arg(info.steer_angle_speed_cmd)
//                     .arg(info.front_angle_cmd)
//                     .arg(info.throttle_cmd)
//                     .arg(info.de_acc_cmd, 0, 'f', 3)
//                     .arg(info.vehicle_x, 0, 'f', 3)
//                     .arg(info.vehicle_y, 0, 'f', 3)
//                     .arg(info.vehicle_theta, 0, 'f', 3)
//                     .arg(info.vehicle_v, 0, 'f', 3)
//                     .arg(info.vehicle_kappa, 0, 'f', 5)
//                     .arg(info.controller_state);
//  emit WriteRecord(text, 3);

//  pnc2pad_move_data_mutex_.lock();
//  memset(&m_stPNC2PadMoveDataMsg, 0, sizeof(m_stPNC2PadMoveDataMsg));
//  m_stPNC2PadMoveDataMsg.stMoveEvaluateData.fPathHorizErr = info.lat_error;
//  m_stPNC2PadMoveDataMsg.stMoveCaltData.stMoveCaltData.fFrontAxleAngle = info.steer_angle_cmd;
//  if (m_uiPlanMode == 0)
//  {
//    m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fX =
//        m_listTrackPathCmd.at(info.pp_look_ahead_point_index).stENUPoint.x;
//    m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fY =
//        m_listTrackPathCmd.at(info.pp_look_ahead_point_index).stENUPoint.y;
//  }
//  if (m_uiPlanMode == 1)
//  {
//    int index = 0;
//    if (info.pp_look_ahead_point_index > m_PNC2PADPath.size())
//    {
//      index = m_PNC2PADPath.size() - 1;
//    }
//    else
//    {
//      index = info.pp_look_ahead_point_index;
//    }
//    m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fX = m_PNC2PADPath.at(index).fX;
//    m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fY = m_PNC2PADPath.at(index).fY;
//  }

//  SendPNCMoveDataMsg(m_stPNC2PadMoveDataMsg);
//  pnc2pad_move_data_mutex_.unlock();
//}

// void ParentDlg::SendPNCMoveDataMsg(PNC2PadMoveDataMsgST stMsg) /*发送决策控制数据信息*/
//{
//   unsigned int RIGHTMSGLEN = sizeof(PNC2PadMoveDataMsgST);
//   stMsg.stNetHeader.usMsgType = PNC2PAD_MOVE_DATA_MSG;
//   stMsg.stNetHeader.usMsgLen = RIGHTMSGLEN;
//   stMsg.stNetHeader.uiMsgNo = 0;
//   stMsg.stNetHeader.uiMsgTime = 0; /*暂时写0*/
//   stMsg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
//   stMsg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
//   stMsg.stNetHeader.usAck = NET_MSG_NOACK;

//  stMsg.stNetEnd.ucCheckSum = CheckSum((unsigned char*)(&stMsg), RIGHTMSGLEN);

//  emit SendNetMsg(QByteArray((char*)&stMsg, sizeof(PNC2PadMoveDataMsgST)), sizeof(PNC2PadMoveDataMsgST),
//                  QHostAddress(REMOTE_CONTROL_IP), REMOTE_CONTROL_AUTO_DRIVING_PORT);
//}

void ParentDlg::onXW5651(GNSS_IMU_ST msg)
{
  xw5651_mutex_.lock();
  xw5651_msg_ = msg;
  //  m_dlgPosMeasure.setXW5651(xw5651_msg_);
  auto xw5651_msg = msg;
  xw5651_mutex_.unlock();

  /*导航信息显示*/
  //  QString strUTCTime = QString("%1年%2月%3日%4时%5分%6.%7秒")
  //                           .arg(xw5651_msg.utc_year)
  //                           .arg(xw5651_msg.utc_month)
  //                           .arg(xw5651_msg.utc_day)
  //                           .arg(xw5651_msg.utc_hour)
  //                           .arg(xw5651_msg.utc_minute)
  //                           .arg(xw5651_msg.utc_second_int_part)
  //                           .arg(xw5651_msg.utc_second_float_part * 1000);
  //  ui->labINSUTCTime->setText(strUTCTime);

  //  switch (xw5651_msg.combined_navigation_state)
  //  {
  //    case INS_STATE_INIT:
  //    {
  //      ui->labINSState->setText("初始化");
  //      break;
  //    }
  //    case INS_STATE_COARSE_ALIGN:
  //    {
  //      ui->labINSState->setText("粗对准");
  //      break;
  //    }
  //    case INS_STATE_FINE_ALIGN:
  //    {
  //      ui->labINSState->setText("精对准");
  //      break;
  //    }
  //    case INS_STATE_ONE_ANTENA_LOCATE:
  //    {
  //      ui->labINSState->setText("单天线定位");
  //      break;
  //    }
  //    case INS_STATE_DOUBLE_ANTENA_LOCATE:
  //    {
  //      ui->labINSState->setText("双天线定位");
  //      break;
  //    }
  //    case INS_STATE_ONE_ANTENA_RTK:
  //    {
  //      ui->labINSState->setText("单天线差分");
  //      break;
  //    }
  //    case INS_STATE_ONLY_IMU:
  //    {
  //      ui->labINSState->setText("纯惯");
  //      break;
  //    }
  //    case INS_STATE_ZERO_SPEED_REVISE:
  //    {
  //      ui->labINSState->setText("零速校正");
  //      break;
  //    }
  //    case INS_STATE_DOUBLE_ANTENA_RTK:
  //    {
  //      ui->labINSState->setText("双天线差分");
  //      break;
  //    }
  //    case INS_STATE_DYNAMIC_ALIGN:
  //    {
  //      ui->labINSState->setText("动态对准");
  //      break;
  //    }
  //    case INS_STATE_ERROR:
  //    {
  //      ui->labINSState->setText("系统异常");
  //      break;
  //    }
  //    default:
  //      break;
  //  }

  //  ui->labGPSState->setText("数据无效");

  //  /*GPS信息显示*/
  //  for (UINT8 i = 0; i < 7; i++)
  //  {
  //    if (0x1 == ((xw5651_msg.gps_state >> i) & 0x1))
  //    {
  //      switch (i)
  //      {
  //        case 0:
  //        {
  //          ui->labGPSState->setText("数据有效");
  //          break;
  //        }
  //        case 1:
  //        {
  //          ui->labGPSState->setText("位置速度有效");
  //          break;
  //        }
  //        case 2:
  //        {
  //          ui->labGPSState->setText("定向有效");
  //          break;
  //        }
  //        case 3:
  //        {
  //          ui->labGPSState->setText("差分有效");
  //          break;
  //        }
  //        case 4:
  //        {
  //          ui->labGPSState->setText("时间有效");
  //          break;
  //        }
  //        case 5:
  //        {
  //          ui->labGPSState->setText("差分浮点解有效");
  //          break;
  //        }
  //        case 6:
  //        {
  //          ui->labGPSState->setText("差分固定解有效");
  //          break;
  //        }
  //        default:
  //          break;
  //      }
  //    }
  //  }

  QString strText;
  strText = QString("%1").arg(xw5651_msg.altitude, 0, 'f', 3);
  ui->labINSHeight->setText(strText);

  strText = QString("%1").arg(xw5651_msg.longitude, 0, 'f', 7);
  ui->labINSLongitude->setText(strText);

  strText = QString("%1").arg(xw5651_msg.latitude, 0, 'f', 7);
  ui->labINSLatitude->setText(strText);

  lon_task_point_get = xw5651_msg.longitude;
  lat_task_point_get = xw5651_msg.latitude;

  strText = QString("%1").arg(xw5651_msg.angle_heading * 180 / M_PI, 0, 'f', 3);
  ui->labINSCourse->setText(strText);

  strText = QString("%1").arg(xw5651_msg.angle_pitch * 180 / M_PI, 0, 'f', 3);
  ui->labINSPitch->setText(strText);

  strText = QString("%1").arg(xw5651_msg.angle_roll * 180 / M_PI, 0, 'f', 3);
  ui->labINSRoll->setText(strText);
  // //Z              W              Y
  strText = QString("%1").arg(uint(xw5651_msg.main_star_num), 0, 'f', 3);
  ui->labGPSFrontStar->setText(strText);

  //  strText = QString("%1").arg(xw5651_msg.gps_state, 0, 'f', 3);
  //  ui->labGPSState->setText(strText);

  /*20210419修改成km/h显示*/
  strText = QString("%1").arg(xw5651_msg.ground_speed * 3.6, 0, 'f', 2);
  ui->labINSGroudSpeed->setText(strText);

  // 导航状态 目前显示 椭球高程
  // strText = QString("%1").arg(xw5651_msg.ellip_height, 0, 'f', 2);
  // ui->labINSState->setText(strText);

  switch (int(xw5651_msg.satellite_status)) /*导航状态*/
  {
  case 0:
  {
    ui->labINSState->setText("不定位不定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
    break;
  }
  case 1:
  {
    ui->labINSState->setText("单点定位定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 2:
  {
    ui->labINSState->setText("伪距差分定位定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 3:
  {
    ui->labINSState->setText("组合推算");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 4:
  {
    ui->labINSState->setText("RTK固定解");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color: green; color:white}");
    break;
  }
  case 5:
  {
    ui->labINSState->setText("RTK浮点解");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 6:
  {
    ui->labINSState->setText("单点定位不定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 7:
  {
    ui->labINSState->setText("伪距差分定位不定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 8:
  {
    ui->labINSState->setText("RTK稳定解定位不定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }
  case 9:
  {
    ui->labINSState->setText("RTK浮点解定位不定向");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");

    break;
  }

  default:
  {
    ui->labINSState->setText("无此值");
    ui->labINSState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
    break;
  }
  }

  strText = QString("%1").arg(xw5651_msg.acc_x, 0, 'f', 2);
  ui->labINSAcc_x->setText(strText);

  strText = QString("%1").arg(xw5651_msg.acc_y, 0, 'f', 2);
  ui->labINSAcc_y->setText(strText);

  strText = QString("%1").arg(xw5651_msg.acc_z, 0, 'f', 2);
  ui->labINSAcc_z->setText(strText);

  strText = QString("%1").arg(xw5651_msg.gyro_z * 180.0 / M_PI, 0, 'f', 2);
  ui->labINSAngleSpeed->setText(strText);

  strText = QString("%1").arg(xw5651_msg.gyro_z * 180.0 / M_PI, 0, 'f', 2);
  ui->labINSAngleSpeed_2->setText(strText);

  strText = QString("%1").arg(xw5651_msg.gyro_x * 180.0 / M_PI, 0, 'f', 2);
  ui->labINSAngular_x->setText(strText);

  strText = QString("%1").arg(xw5651_msg.gyro_y * 180.0 / M_PI, 0, 'f', 2);
  ui->labINSAngular_y->setText(strText);

  strText = QString("%1").arg(xw5651_msg.nav_uncertainty, 0, 'f', 2);
  ui->labINSNavUncertainty->setText(strText);

  // strText = QString("%1").arg(xw5651_msg.satelliteState, 0, 'f', 2);
  // ui->labINSSatelliteState->setText(strText);

  switch (xw5651_msg.system_state) /*卫星状态*/
  {
  case 0:
  {
    ui->labINSSatelliteState->setText("初始化");
    break;
  }
  case 1:
  {
    ui->labINSSatelliteState->setText("卫星导航");
    break;
  }
  case 2:
  {
    ui->labINSSatelliteState->setText("组合导航");
    break;
  }
  case 3:
  {
    ui->labINSSatelliteState->setText("纯惯性");
    break;
  }
  case 0x11:
  {
    ui->labINSSatelliteState->setText("对准");
    break;
  }

  default:
  {
    ui->labINSSatelliteState->setText("无此值");
    break;
  }
  }

  //  ui->labGPSFrontStar->setText(QString("%1").arg(xw5651_msg.front_star));
  //  ui->labGPSBackStar->setText(QString("%1").arg(xw5651_msg.back_star));

  //  strText = QString("%1").arg(xw5651_msg.gps_base_line, 0, 'f', 3);
  //  ui->labGPSLine->setText(strText);

  m_uiSendInsNo++;
  if (m_uiSendInsNo == m_uiSendInsNoThres)
  {
    PNC2PadINSDataMsgST pad_ins_send_msg;
    memset(&pad_ins_send_msg, 0, sizeof(PNC2PadINSDataMsgST));
    pad_ins_send_msg.stNetHeader.usMsgType = PNC2PAD_INS_MSG;
    pad_ins_send_msg.stNetHeader.usMsgLen = sizeof(PNC2PadINSDataMsgST);
    pad_ins_send_msg.stNetHeader.uiMsgTime = 0;
    pad_ins_send_msg.stNetHeader.uiMsgNo = 0;
    pad_ins_send_msg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
    pad_ins_send_msg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
    pad_ins_send_msg.stNetHeader.usAck = NET_MSG_NOACK;
    pad_ins_send_msg.stINSData.fTimeStampS = 0;  // xw5651_msg.fTimeStampS;
    pad_ins_send_msg.stINSData.fTimeStampNs = 0; // xw5651_msg.fTimeStampNs;
    pad_ins_send_msg.stINSData.fLatitude = xw5651_msg.latitude;
    pad_ins_send_msg.stINSData.fLongitude = xw5651_msg.longitude;
    pad_ins_send_msg.stINSData.fEllipHeight = xw5651_msg.ellip_height;
    pad_ins_send_msg.stINSData.fAlt_EllipHeight = xw5651_msg.altitude;
    pad_ins_send_msg.stINSData.fAzimuth = xw5651_msg.angle_heading * 180 / M_PI;
    pad_ins_send_msg.stINSData.fPitch = xw5651_msg.angle_pitch * 180 / M_PI;
    pad_ins_send_msg.stINSData.fRoll = xw5651_msg.angle_roll * 180 / M_PI;
    pad_ins_send_msg.stINSData.fEastSpeed = xw5651_msg.east_speed;
    pad_ins_send_msg.stINSData.fNorthSpeed = xw5651_msg.north_speed;
    pad_ins_send_msg.stINSData.fSkySpeed = xw5651_msg.up_speed;
    pad_ins_send_msg.stINSData.fGroundSpeed = xw5651_msg.ground_speed;
    pad_ins_send_msg.stINSData.fActX = xw5651_msg.acc_x;
    pad_ins_send_msg.stINSData.fActY = xw5651_msg.acc_y;
    pad_ins_send_msg.stINSData.fActZ = xw5651_msg.acc_z;
    pad_ins_send_msg.stINSData.fGyroX = xw5651_msg.gyro_x;
    pad_ins_send_msg.stINSData.fGyroY = xw5651_msg.gyro_y;
    pad_ins_send_msg.stINSData.fGyroZ = xw5651_msg.gyro_z;
    pad_ins_send_msg.stINSData.ucINSState = xw5651_msg.system_state;
    pad_ins_send_msg.stINSData.ucGPSState = xw5651_msg.satellite_status;

    pad_ins_send_msg.stINSData.ucFrontStarNum = xw5651_msg.main_star_num;
    pad_ins_send_msg.stINSData.ucBackStarNum = xw5651_msg.aux_star_num;

    pad_ins_send_msg.stINSData.stGlobalPostion.fX = vehicle_motion_state_msg_.x;
    pad_ins_send_msg.stINSData.stGlobalPostion.fY = vehicle_motion_state_msg_.y;
    pad_ins_send_msg.stINSData.stGlobalPostion.fZ = vehicle_motion_state_msg_.z;
    pad_ins_send_msg.stINSData.stQuaternion.w = 0;
    pad_ins_send_msg.stINSData.stQuaternion.x = 0;
    pad_ins_send_msg.stINSData.stQuaternion.y = 0;
    pad_ins_send_msg.stINSData.stQuaternion.z = 0;

    emit SendNetMsg(QByteArray((char *)&pad_ins_send_msg, sizeof(PNC2PadINSDataMsgST)), sizeof(PNC2PadINSDataMsgST),
                    QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
    m_uiSendInsNo = 0;
  }
}

// 接收底盘状态信息，并将其转发给pad
void ParentDlg::onVehicleChassisState(VehicleChassisStateST msg)
{
  vehicle_chassis_state_mutex_.lock();
  vehicle_chassis_state_msg_ = msg;
  auto vehicle_chassis_state_msg = msg;
  vehicle_chassis_state_mutex_.unlock();
  // 驾驶模式
  ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");      // eps
  ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");    // drive
  ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}"); // bcm
  ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");     // gear
  ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");    // brake
  ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");      // estop

  if (0 == vehicle_chassis_state_msg.driving_mode)
  {
    ui->labChasWorkMode->setText("人控模式");
    ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }

  else if (vehicle_chassis_state_msg.driving_mode == 1)
  {
    if (vehicle_chassis_state_msg.mode_flag == 0) // auto
    {
      ui->labChasWorkMode->setText("自动驾驶模式");
      ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
      ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
      ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");       //
      ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");     //
      ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");     //
      ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      //
      ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");       //
    }
    else if (vehicle_chassis_state_msg.mode_flag == 1) // remote
    {
      ui->labChasWorkMode->setText("遥控驾驶模式");
      ui->labAutoState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");  //
      ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");       //
      ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");     //
      ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");     //
      ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      //
      ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");       //
    }
    else if (vehicle_chassis_state_msg.mode_flag == 2) // reverse
    {
      ui->labChasWorkMode->setText("人工反向驾驶模式");
      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); //
      ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      //
      ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");    //
      ui->labBrake->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");    //
      ui->labGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");     //
      ui->labEpb->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      //
    }
  }
  else
  {
    //    if (vehicle_chassis_state_msg.adu_mode == 1)
    //    {
    //      ui->labChasWorkMode->setText("纵向控制模式");
    //      ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
    //      ui->labDrive->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");    // drive
    //      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); // bcm
    //    }

    //    if (vehicle_chassis_state_msg.eps_mode == 1)
    //    {
    //      ui->labChasWorkMode->setText("横向控制模式");
    //      ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
    //      ui->labEps->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");      // eps
    //      ui->labBCMState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}"); // bcm
    //    }
  }

  if (vehicle_chassis_state_msg.brake_pedal > 0) /*刹车状态*/
  {
    ui->labChasBrakState->setText("刹车中");
    ui->labChasBrakState->setStyleSheet(
        "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
  }
  else if (vehicle_chassis_state_msg.brake_pedal == 0)
  {
    ui->labChasBrakState->setText("未刹车");
    ui->labChasBrakState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    ui->labChasBrakState->setText("故障");
    ui->labChasBrakState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

  QString strText =
      QString("%1").arg((vehicle_chassis_state_msg.current_velocity * 3.6), 0, 'f', 1); /*底盘车速，20210419修改成km/h显示*/
  ui->labChasVehSpeed->setText(strText);

  strText = QString("%1").arg((vehicle_chassis_state_msg.steering_wheel_angle), 0, 'f', 2); /*底盘前轮转角*/
  ui->labChasTurnAngle->setText(strText);

  strText = QString("%1").arg((vehicle_chassis_state_msg.steering_wheel_angle_speed), 0, 'f', 1); /*底盘前轮转角速度*/
  ui->labChasTurnAngleSpeed->setText(strText);

  ui->labChasGear->setStyleSheet(
      "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");

  switch (vehicle_chassis_state_msg.gear_location) /*变速箱档位*/
  {
  case 0:
  {
    ui->labChasGear->setText("N");
    break;
  }
  case 7:
  {
    ui->labChasGear->setText("R1");
    break;
  }
  case 1:
  {
    ui->labChasGear->setText("D1");
    break;
  }
  case 2:
  {
    ui->labChasGear->setText("P");
    break;
  }
  default:
  {
    break;
  }
  }

  strText = QString("%1").arg(vehicle_chassis_state_msg.throttle_pedal); /*油门开度*/
  ui->labChasOilOpen->setText(strText);

  strText = QString("%1").arg(vehicle_chassis_state_msg.remote_button_status); /*远程按钮反馈*/
  ui->labCHASRemote_button_status->setText(strText);

  //    strText = QString("%1").arg(stChas2VCWorkData.ucTurnArmOpening);//转向开度
  //    ui->labChasTurnArmOpen->setText(strText);

  strText = QString("%1").arg(vehicle_chassis_state_msg.brake_pedal); /*刹车开度*/
  ui->labChasBrakOpen->setText(strText);

  // ui->labEngineWorkState->setStyleSheet(
  //     "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");

  // if (vehicle_chassis_state_msg.current_engine_torque > 0)
  // {
  //   ui->labEngineWorkState->setText("启动完成");
  // }
  // else
  // {
  //   ui->labEngineWorkState->setText("停止完成");
  // }

  // std::cout << vehicle_chassis_state_msg.steer_intervene << std::endl;

  if (vehicle_chassis_state_msg.is_ready)
  {
    ui->labCHASAuto_switch->setText("准备好");
    ui->labCHASAuto_switch->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else
  {
    ui->labCHASAuto_switch->setText("未准备好");
    ui->labCHASAuto_switch->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:red; color:white}");
  }

  if (vehicle_chassis_state_msg.steer_intervene)
  {
    ui->labCHASSteer_intervene->setText("干预");
    ui->labCHASSteer_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    ui->labCHASSteer_intervene->setText("无干预");
    ui->labCHASSteer_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }

  if (vehicle_chassis_state_msg.brake_intervene)
  {
    ui->labCHASBrake_intervene->setText("干预");
    ui->labCHASBrake_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    ui->labCHASBrake_intervene->setText("无干预");
    ui->labCHASBrake_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }

  if (vehicle_chassis_state_msg.estop_intervene)
  {
    ui->labCHASEstop_intervene->setText("干预");
    ui->labCHASEstop_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    ui->labCHASEstop_intervene->setText("无干预");
    ui->labCHASEstop_intervene->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }

  if (vehicle_chassis_state_msg.timeout_status)
  {
    ui->labCHASTimeout_status->setText("超时");
    ui->labCHASTimeout_status->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    ui->labCHASTimeout_status->setText("未超时");
    ui->labCHASTimeout_status->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.FuelCapacity); /*油箱油量*/
  //  ui->labOilRemain->setText(strText);

  //  if (true == vehicle_chassis_state_msg.EmcyBarkeSts) /*底盘紧急制动状态*/
  //  {
  //    ui->labChasEmerencyBrakState->setText("紧急制动中");
  //    ui->labChasEmerencyBrakState->setStyleSheet(
  //        "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  //  }
  //  else
  //  {
  //    ui->labChasEmerencyBrakState->setText("未紧急制动");
  //    ui->labChasEmerencyBrakState->setStyleSheet(
  //        "QLabel{font:11pt '仿宋' bold; background-color:rgb(35,35,35); color:rgb(21,197,212);}");
  //  }

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.AirTankPressure); /**/
  //  ui->labChasBatVolt->setText(strText);

  //  strText = QString("%1").arg(vehicle_chassis_state_msg.Range); /*底盘里程*/
  //  ui->labChasMileage->setText(strText);

  if (vehicle_chassis_state_msg.parking_brake)
  {
    ui->labChasParkState->setText("驻车制动");
    ui->labChasParkState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else
  {
    ui->labChasParkState->setText("解除驻车");
    ui->labChasParkState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

  m_uiSendChassisNo++;
  if (m_uiSendChassisNo == m_uiSendChassisNoThres)
  {
    PadChassisToRemoteControlReportST pad_chassis_send_msg;
    pad_chassis_send_msg.stNetHeader.usMsgType = TANK_TO_REMOTE_CONTROL_STATE_MSG;
    pad_chassis_send_msg.stNetHeader.usMsgLen = sizeof(PadChassisToRemoteControlReportST);
    pad_chassis_send_msg.stNetHeader.uiMsgTime = 0;
    pad_chassis_send_msg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
    pad_chassis_send_msg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
    pad_chassis_send_msg.stNetHeader.usAck = NET_MSG_NOACK;

    pad_chassis_send_msg.fTimeStampS = 0;                                                  // 单位s，格林威治时间1970年01月01日00时00分00秒起至当前时刻的时间
    pad_chassis_send_msg.fTimeStampNs = 0.;                                                // 纳秒
    pad_chassis_send_msg.usErrorState = 0x00;                                              // 底盘故障状态
    pad_chassis_send_msg.usErrorCode = 0;                                                  // 底盘故障编码
    pad_chassis_send_msg.us_total_kilometres = vehicle_chassis_state_msg.total_kilometres; // 里程

    if (vehicle_chassis_state_msg.current_engine_speed > 500) // 发动机状态   转速大于500：启动1  转速小于500：关闭0
    {
      pad_chassis_send_msg.ucEngineState = 1;
    }
    else
    {
      pad_chassis_send_msg.ucEngineState = 0;
    }
    // std::cout<<vehicle_chassis_state_msg.gear_location<<std::endl;
    pad_chassis_send_msg.usParkingBrake = vehicle_chassis_state_msg.parking_brake; // 驻车制动
    pad_chassis_send_msg.usGearLocation = vehicle_chassis_state_msg.gear_location;
    pad_chassis_send_msg.usLeftLight = vehicle_chassis_state_msg.left_light;
    pad_chassis_send_msg.usRightLight = vehicle_chassis_state_msg.right_light;
    pad_chassis_send_msg.usOilPercent = vehicle_chassis_state_msg.remaining_oil;
    pad_chassis_send_msg.usVehicleSpeed = vehicle_chassis_state_msg.current_velocity * 3.6; // 前面单位km
    pad_chassis_send_msg.usEngineSpeed = vehicle_chassis_state_msg.current_engine_speed;

    pad_chassis_send_msg.usDrivingMode = vehicle_chassis_state_msg.driving_mode; // 反馈驾驶模式
    pad_chassis_send_msg.bRecTask = rec_task_;                                   // 收到任务文件反馈
    // std::cout << "rec_task is" << rec_task_ << std::endl;

    emit SendNetMsg(QByteArray((char *)&pad_chassis_send_msg, sizeof(PadChassisToRemoteControlReportST)),
                    sizeof(PadChassisToRemoteControlReportST), QHostAddress(REMOTE_CONTROL_IP),
                    TANK_TASKPOINTS_PORT);

    m_uiSendChassisNo = 0;
  }
}
void ParentDlg::onVehicleMotionState(VehicleMotionStateST msg)
{
  //  vehicle_motion_state_mutex_.lock();
  //  vehicle_motion_state_msg_ = msg;
  //  m_dlgPosMeasure.setVehicleMotionState(vehicle_motion_state_msg_);

  //  ENUCorST stENUVehBackWheel;
  //  memset(&stENUVehBackWheel, 0, sizeof(ENUCorST));
  //  stENUVehBackWheel.x = vehicle_motion_state_msg_.x;  // 后轴位置
  //  stENUVehBackWheel.y = vehicle_motion_state_msg_.y;
  //  stENUVehBackWheel.z = 0.0;

  //  double lf_ = 2.55;
  //  double lr_ = 2.55;
  //  double fVehCourse = vehicle_motion_state_msg_.theta;

  //  fVehCourse = Angle2X180TransAngle2Y360(fVehCourse);
  //  m_fVehCourse = fVehCourse;
  //  ENUCorST stVehCenterENUPos; /*解算车辆质心位置*/
  //  memset(&stVehCenterENUPos, 0, sizeof(ENUCorST));
  //  stVehCenterENUPos.x = stENUVehBackWheel.x + lf_ * sin(fVehCourse * DEGREE_RADIAN);
  //  stVehCenterENUPos.y = stENUVehBackWheel.y + lf_ * cos(fVehCourse * DEGREE_RADIAN);
  //  stVehCenterENUPos.z = stENUVehBackWheel.z;

  //  ENUCorST stFrontWheelENUPos; /*解算车辆前轴位置*/
  //  memset(&stFrontWheelENUPos, 0, sizeof(ENUCorST));
  //  stFrontWheelENUPos.x = stENUVehBackWheel.x + 2 * lf_ * sin(fVehCourse * DEGREE_RADIAN);
  //  stFrontWheelENUPos.y = stENUVehBackWheel.y + 2 * lf_ * cos(fVehCourse * DEGREE_RADIAN);
  //  stFrontWheelENUPos.z = stENUVehBackWheel.z;

  //  memcpy(&m_stENUVehFrontWheel, &(stFrontWheelENUPos), sizeof(ENUCorST)); /*车辆前轮位置*/
  //  memcpy(&m_stENUVehCenterPos, &(stVehCenterENUPos), sizeof(ENUCorST));   /*车辆质心位置*/
  //  memcpy(&m_stENUVehBackWheel, &(stENUVehBackWheel), sizeof(ENUCorST));   /*车辆后轮位置*/

  //  ui->labGridMap->update();

  //  double x = vehicle_motion_state_msg_.x;
  //  ui->labVC2FPHeartMsgNum->setText(QString::number(x, 'f', 2));
  //  double y = vehicle_motion_state_msg_.y;
  //  ui->labVC2FPAckMsgNum->setText(QString::number(y, 'f', 2));
  //  double theta = vehicle_motion_state_msg_.theta;
  //  ui->labVC2FPPathBindRsltMsgNum->setText(QString::number(theta, 'f', 2));
  //  double v_veh = vehicle_motion_state_msg_.v;
  //  ui->labVC2FPCordinateBindRsltMsgNum->setText(QString::number(v_veh, 'f', 2));

  //  double a_veh = vehicle_motion_state_msg_.a;
  //  ui->labVC2FPAbNormalMsgNum->setText(QString::number(a_veh, 'f', 2));

  //  double kappa = vehicle_motion_state_msg_.kappa;
  //  ui->labVC2FPStateMsgNum->setText(QString::number(kappa, 'f', 5));

  //  vehicle_motion_state_mutex_.unlock();
}

ParentDlg::~ParentDlg()
{

  if (m_timerMoveCtrl != Q_NULLPTR)
  {
    m_timerMoveCtrl->stop();
    m_timerMoveCtrl->deleteLater();
    m_timerMoveCtrl = Q_NULLPTR;
  }

  if (m_timerTimeDisp != Q_NULLPTR)
  {
    m_timerTimeDisp->stop();
    m_timerTimeDisp->deleteLater();
    m_timerTimeDisp = Q_NULLPTR;
  }

  //  if (m_timerAnomalyDisp != Q_NULLPTR)
  //  {
  //    m_timerAnomalyDisp->stop();
  //    m_timerAnomalyDisp->deleteLater();
  //    m_timerAnomalyDisp = Q_NULLPTR;
  //  }

  if (m_timerOneKeyStartMotion != Q_NULLPTR)
  {
    m_timerOneKeyStartMotion->stop();
    m_timerOneKeyStartMotion->deleteLater();
    m_timerOneKeyStartMotion = Q_NULLPTR;
  }

  if (m_pNetSendTimer != Q_NULLPTR)
  {
    m_pNetSendTimer->stop();
    m_pNetSendTimer->deleteLater();
    m_pNetSendTimer = Q_NULLPTR;
  }
  //    if(Q_NULLPTR != m_pSpeech)
  //    {
  //        m_pSpeech->deleteLater();
  //        m_pSpeech = Q_NULLPTR;
  //    }

  if (Q_NULLPTR != m_pNetRecv)
  {
    m_pNetRecv->m_bQuit = true; /*退出接收线程*/
    m_pNetRecv->quit();         /*停止以太网线程,退出exec()*/
    m_pNetRecv->wait();
    qDebug() << "以太网接收线程已关闭";
    if (true == m_pNetRecv->isRunning())
    {
      qDebug() << "以太网接收线程仍在运行";
      m_pNetRecv->terminate();
      m_pNetRecv->wait(); // 添加等待计时200ms，防止一直等待导致主程序退不出 20220117 qixianyu
      m_pNetRecv = Q_NULLPTR;
    }
  }

  if (Q_NULLPTR != m_pThreadAstar)
  {
    m_pThreadAstar->quit(); /*停止地图文件读取线程线程,退出exec()*/
    m_pThreadAstar->wait();
    if (true == m_pThreadAstar->isRunning())
    {
      qDebug() << "地图文件读取线程仍在运行";
      m_pThreadAstar->terminate();
      m_pThreadAstar->wait(200); // 添加等待计时200ms，防止一直等待导致主程序退不出 20220117 qixianyu
      qDebug() << "地图文件读取线程已关闭";
    }
    if (Q_NULLPTR != m_pThreadAstar)
    {

      m_pThreadAstar->deleteLater();
      m_pThreadAstar = Q_NULLPTR;
    }
    qDebug() << "m_pThreadAstar已关闭";
  }

  if (Q_NULLPTR != m_pThreadNetSend)
  {
    m_pThreadNetSend->quit(); /*停止以太网线程,退出exec()*/
    m_pThreadNetSend->wait();
    qDebug() << "m_pThreadNetSend已关闭";
  }

  if (Q_NULLPTR != qnode_)
  {

    delete qnode_;
    qDebug() << "qnode已关闭";
  }

  //  if (Q_NULLPTR != m_pRecord)
  //  {
  //    m_pRecord->m_synSem.release(); /*使记录线程可以停止*/
  //    m_pRecord->m_bQuit = true;
  //    m_pRecord->quit();
  //    m_pRecord->wait();
  //  }

  /*关闭组队界面*/
  delete platoondlg_ptr_;

  /*关闭文件更新界面*/
  delete updatefile_ptr_;
  delete ui;
  qDebug() << "~ParentDlg()";
}

void ParentDlg::ReadConfigFile() /*读取配置文件*/
{
  QString strAbsFileName = QFileDialog::getOpenFileName(this, tr("打开文件"), tr("D:/CF/1 OptConfig"),
                                                        tr("Config Files(*.txt)")); /*返回绝对路径+文件名*/
  /*获取文件名*/
  INT32 pos = strAbsFileName.lastIndexOf('/');
  QString strFileName = strAbsFileName.right(strAbsFileName.size() - pos - 1);
  /*获取文件路径*/
  QString strFilePath = strAbsFileName.left(pos + 1);

  if (false == strAbsFileName.isEmpty()) /*打开了文件*/
  {
    QFile objFile;
    objFile.setFileName(strAbsFileName); /*设置文件名称*/
    if (false == objFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      qDebug() << "配置文件打开失败";
    }
    else
    {
      QByteArray array; /*读取信息暂存变量*/
      array.clear();
      while (false == objFile.atEnd()) /*未到文件尾*/
      {
        array = objFile.readLine(); /*读取一行*/
        QString str;
        str.prepend(array);

        INT32 iPos[21] = {0};
        INT32 iPosTemp, iCnt = 0;
        while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
        {
          iPos[iCnt] = iPosTemp;
          iCnt++; /*查找到空格键的次数*/
          if (iCnt > 20)
          {
            break;
          }
        }

        QString strValue;
        if (0 != iPos[1]) /*确认有找到两个空格*/
        {
          strValue = str.right(str.length() - iPos[1] - 1); /*取值*/
        }

        if (true == str.contains("USE_VEH_SIMU"))
        {
          USE_VEH_SIMU = strValue.toUInt();
        }
        else if (true == str.contains("VEHICLE_MASS"))
        {
          VEHICLE_MASS = strValue.toDouble();
        }
        else if (true == str.contains("VEHICLE_WHEEL_BASE"))
        {
          VEHICLE_WHEEL_BASE = strValue.toDouble();
        }
        else if (true == str.contains("CHAS_MOVE_CTRL_PERIOD"))
        {
          CHAS_MOVE_CTRL_PERIOD = strValue.toDouble();
        }
        else if (true == str.contains("VEHICLE_LENGTH"))
        {
          VEHICLE_LENGTH = strValue.toDouble();
        }
        else if (true == str.contains("VEHICLE_WIDTH"))
        {
          VEHICLE_WIDTH = strValue.toDouble();
        }
        else if (true == str.contains("VEHICLE_HEIGHT"))
        {
          VEHICLE_HEIGHT = strValue.toDouble();
        }
        else if (true == str.contains("MAX_LINE_SPEED"))
        {
          MAX_LINE_SPEED = strValue.toDouble();
        }
        else if (true == str.contains("MAX_TURN_ANGLE"))
        {
          MAX_TURN_ANGLE = strValue.toDouble();
        }
        else if (true == str.contains("MIN_TURN_ANGLE"))
        {
          MIN_TURN_ANGLE = strValue.toDouble();
        }
        else if (true == str.contains("TURN_ANGLE_SPEED"))
        {
          TURN_ANGLE_SPEED = strValue.toDouble();
        }
        else if (true == str.contains("GRAVITY_ACC"))
        {
          GRAVITY_ACC = strValue.toDouble();
        }
        else if (true == str.contains("ELECTRIC_FENCE")) // 添加电子围栏显示 20220124 qixianyu
        {
          ELECTRIC_FENCE = strValue.toDouble();
        }
        else
        {
          /*不处理*/
        }
      }
      objFile.close();
      qDebug() << "配置参数文件打开成功";
    }
  }
  else
  {
    QMessageBox::information(this, "critical", tr("未加载配置参数"), QMessageBox::Yes, QMessageBox::Yes);
    qDebug() << "取消了配置参数文件打开操作";
  }
}

void ParentDlg::on_btnWndClose_clicked() /*窗口关闭按钮点击槽函数*/
{
  this->close(); /*关闭窗口*/
}

// void ParentDlg::on_radioMoveByMan_clicked() /*运动控制模式切换到遥控控制*/
//{
//   /*为了安全，每次切换到遥控模式时，把遥控命令值都清零*/
//   //  ui->labVehSpeedCmd->setText("0");       /*遥控运动速度命令值*/
//   //  ui->labTurnAngleCmd->setText("0");      /*遥控转向角度命令值*/
//   //  ui->labTurnAngleSpeedCmd->setText("0"); /*遥控转向角速度命令值*/

//  //  ui->grpPathTrackCmd->hide(); /*隐藏路径跟踪控制区*/
//  //  ui->grpManCtrl->show();      /*显示遥控控制区*/
//}

// void ParentDlg::on_radioMovePathTrack_clicked() /*运动控制模式切换到路径跟踪*/
//{
//   //  ui->grpManCtrl->hide();      /*隐藏遥控控制区*/
//   //  ui->grpPathTrackCmd->show(); /*显示路径跟踪控制区*/
// }

// void ParentDlg::on_radioMoveCourseKeep_clicked() /*运动模式切换到航向不变*/
//{
//   //  ui->grpManCtrl->hide();      /*隐藏遥控控制区*/
//   //  ui->grpPathTrackCmd->show(); /*显示路径跟踪控制区*/
// }

// void ParentDlg::on_sliderVehSpeedCmd_valueChanged(int value) /*遥控速度滚动条值变化槽函数*/
//{
////  ui->labVehSpeedCmd->setText(QString("%1").arg(value / 10.0));
//}

// void ParentDlg::on_sliderTurnAngleCmd_valueChanged(int value) /*遥控转向角滚动条值变化槽函数*/
//{
////  ui->labTurnAngleCmd->setText(QString("%1").arg(value / 10.0));
//}

// void ParentDlg::on_sliderTurnAngleSpeedCmd_valueChanged(int value) /*遥控转向角速度滚动条值变化槽函数*/
//{
////  ui->labTurnAngleSpeedCmd->setText(QString("%1").arg(value / 10.0));
//}

/*界面网格初始化延时函数*/
void ParentDlg::timerUIInitFunc()
{
  ui->tableTrackPathInfo->setColumnWidth(0, ui->tableTrackPathInfo->width() / 4 - 20);
  ui->tableTrackPathInfo->setColumnWidth(1, ui->tableTrackPathInfo->width() / 4);
  ui->tableTrackPathInfo->setColumnWidth(2, ui->tableTrackPathInfo->width() / 4);
  ui->tableTrackPathInfo->setColumnWidth(3, ui->tableTrackPathInfo->width() / 4);

  ui->tablePathBindRsltInfo->setColumnWidth(0, 2 * ui->tableTrackPathInfo->width() / 10);
  ui->tablePathBindRsltInfo->setColumnWidth(1, 4 * ui->tableTrackPathInfo->width() / 10 - 15);
  ui->tablePathBindRsltInfo->setColumnWidth(2, 4 * ui->tableTrackPathInfo->width() / 10 - 15);

  ui->tableMsgDisp->setColumnWidth(0, ui->tableMsgDisp->width() / 3);
  ui->tableMsgDisp->setColumnWidth(1, ui->tableMsgDisp->width() / 3);
  ui->tableMsgDisp->setColumnWidth(2, ui->tableMsgDisp->width() / 3);
  m_timerUIInit->stop(); /*停止定时器*/
}

void ParentDlg::timerTimeDispFunc() /*系统时间显示定时器*/
{
  //  QDateTime curDateTime = QDateTime::currentDateTime();
  //  QString strCurDateTime = QString("%1年%2月%3日%4:%5:%6")
  //                               .arg(curDateTime.date().year(), 4, 10, QChar('0'))
  //                               .arg(curDateTime.date().month(), 2, 10, QChar('0'))
  //                               .arg(curDateTime.date().day(), 2, 10, QChar('0'))
  //                               .arg(curDateTime.time().hour(), 2, 10, QChar('0'))
  //                               .arg(curDateTime.time().minute(), 2, 10, QChar('0'))
  //                               .arg(curDateTime.time().second(), 2, 10, QChar('0'));

  //  m_uiRunTime++; /*已运行时间*/
  //  QString strRunTime = QString("运行时间:%1时%2分 ")
  //                           .arg(m_uiRunTime / 3600, 2, 10, QChar('0'))
  //                           .arg((m_uiRunTime % 3600) / 60, 2, 10, QChar('0'));

  //  strCurDateTime += strRunTime;
  ////  ui->labTimeDisp->setText(strCurDateTime);

  //  /*以心跳报文有无增加来判断电台信号的好坏*/
  //  if (fabs(m_uiRecvHeartMsgNum - m_uiLstRecordRecvHeartMsgNum) >=
  //      4) /*由于心跳报文100ms一帧，加上定时器可能不太准，因此8以上认为电台信号好*/
  //  {
  //    ui->labRadioSignalState->setText("优");
  //    ui->labRadioSignalState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  //  }
  //  else if (fabs(m_uiRecvHeartMsgNum - m_uiLstRecordRecvHeartMsgNum) >= 3)
  //  {
  //    ui->labRadioSignalState->setText("良");
  //    ui->labRadioSignalState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:yellow; color:white}");
  //  }
  //  else if (m_uiRecvHeartMsgNum != m_uiLstRecordRecvHeartMsgNum) /*只要不相等说明能收到数*/
  //  {
  //    ui->labRadioSignalState->setText("差");
  //    ui->labRadioSignalState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:red; color:white}");
  //  }
  //  else
  //  {
  //    ui->labRadioSignalState->setText("断链");
  //    ui->labRadioSignalState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:gray; color:white}");
  //  }

  //  m_uiLstRecordRecvHeartMsgNum = m_uiRecvHeartMsgNum;
}

// void ParentDlg::AnomalyInfoDisp(QString strInfo, UINT8 ucDispType) /*异常信息显示*/
// {
//   ui->labChasCtrlAnomalCode->setText(strInfo); /*显示异常信息*/
//   if (DISP_WARNING == ucDispType)              /*显示的为警告信息*/
//   {
//     if (true == m_timerAnomalyDisp->isActive()) /*定时器已运行，则将定时器关闭，再重开一个*/
//     {
//       m_timerAnomalyDisp->stop();
//     }
//     m_timerAnomalyDisp->start(3000); /*异常信息显示3s就清空*/
//   }
//   else if (DISP_ERROR == ucDispType)
//   {
//     m_timerAnomalyDisp->stop();
//   }
//   else
//   {
//     /*其他类型，暂时保留*/
//   }
// }

// void ParentDlg::timerAnomalyDispFunc() /*异常信息显示定时器处理*/
// {
//   ui->labChasCtrlAnomalCode->setText("");
//   m_timerAnomalyDisp->stop();
// }

/*报文表格显示*/
void ParentDlg::MsgTableDisp(QString strCol2, QString strCol3, UINT8 ucDispType)
{
  if ((strCol3 == m_strTableNewInfo) && (DISP_WARNING == ucDispType)) /*如果警告信息再持续显示，则不处理*/
  {
    return;
  }
  m_strTableNewInfo = strCol3; /*表格最新一行信息*/
  QTime curTime = QTime::currentTime();
  QString strCurTime = QString("%1:%2:%3.%4")
                           .arg(curTime.hour(), 2, 10, QChar('0'))
                           .arg(curTime.minute(), 2, 10, QChar('0'))
                           .arg(curTime.second(), 2, 10, QChar('0'))
                           .arg(curTime.msec(), 3, 10, QChar('0'));

  /*报文表格增加一行*/
  INT32 iRowCnt = ui->tableMsgDisp->rowCount();
  // UINT32 uiRowCnt = 0;
  ui->tableMsgDisp->insertRow(iRowCnt);
  QTableWidgetItem *item0 = new QTableWidgetItem;
  item0->setText(strCurTime);
  ui->tableMsgDisp->setItem(iRowCnt, 0, item0);

  QTableWidgetItem *item1 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
  item1->setText(strCol2);
  ui->tableMsgDisp->setItem(iRowCnt, 1, item1);
  ui->tableMsgDisp->item(iRowCnt, 1)->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter); /*文字内容居中*/

  QTableWidgetItem *item2 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
  item2->setText(strCol3);
  ui->tableMsgDisp->setItem(iRowCnt, 2, item2);

  if ("错误" == strCol2) /*错误标红显示*/
  {
    item0->setTextColor(QColor(179, 0, 0));
    item1->setTextColor(QColor(179, 0, 0));
    item2->setTextColor(QColor(179, 0, 0));
  }

  // m_tableMsgDisp->scrollToTop();

  ui->tableMsgDisp->selectRow(iRowCnt);
  TextToVoice(strCol3); /*播放声音*/
}

/******************************以太网发送定时器******************************/
void ParentDlg::NetSendTimer()
{
  SendHeartMsg(); /*发送心跳报文*/
  //  static int counter = 0;
  //  counter++;
  //  m_uiPadToRemoteControlHeartNo++;
  //  if (counter == 15)
  //  {
  //    int gap = m_uiPadToRemoteControlHeartNo - m_uiRemoteControlToPadHeartNo;
  //    if (gap >= 10)
  //    {
  //      //停止
  //    }
  //    else
  //    {
  //      counter = 0;
  //      m_uiPadToRemoteControlHeartNo = 0;
  //      m_uiRemoteControlToPadHeartNo = 0;driveMode
  //    }
  //  }
}

void ParentDlg::SendHeartMsg() /*无人驾驶台向综合控制设备发送心跳报文*/
{
  PNC2PadHeartMsgST stPNC2PadHeartMsg;
  memset(&stPNC2PadHeartMsg, 0, sizeof(PNC2PadHeartMsgST));

  stPNC2PadHeartMsg.stNetHeader.usMsgType = PNC2PAD_HEART_MSG;
  stPNC2PadHeartMsg.stNetHeader.usMsgLen = sizeof(PNC2PadHeartMsgST); // 100
  stPNC2PadHeartMsg.stNetHeader.uiMsgTime = 0;

  stPNC2PadHeartMsg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
  stPNC2PadHeartMsg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
  stPNC2PadHeartMsg.stNetHeader.usAck = 0x00ff;

  vehicle_chassis_state_mutex_.lock();
  stPNC2PadHeartMsg.stChassis.battery = vehicle_chassis_state_msg_.remaining_electricity;
  stPNC2PadHeartMsg.stChassis.brake = vehicle_chassis_state_msg_.brake_pedal;
  if (0 == vehicle_chassis_state_msg_.driving_mode)
  {
    stPNC2PadHeartMsg.stChassis.driveMode = 0;
  }

  else if (1 == vehicle_chassis_state_msg_.driving_mode)
  {
    if (vehicle_chassis_state_msg_.eps_mode == 1 && vehicle_chassis_state_msg_.adu_mode == 1)
    {
      stPNC2PadHeartMsg.stChassis.driveMode = 1;
    }
    else if (vehicle_chassis_state_msg_.eps_mode == 0 && vehicle_chassis_state_msg_.adu_mode == 1)
    {
      stPNC2PadHeartMsg.stChassis.driveMode = 2;
    }
    else if (vehicle_chassis_state_msg_.eps_mode == 1 && vehicle_chassis_state_msg_.adu_mode == 0)
    {
      stPNC2PadHeartMsg.stChassis.driveMode = 3;
    }

    else
    {
    }
  }

  stPNC2PadHeartMsg.stChassis.throttle = vehicle_chassis_state_msg_.throttle_pedal;
  stPNC2PadHeartMsg.stChassis.velocity = vehicle_chassis_state_msg_.current_velocity;

  // gear
  if (vehicle_chassis_state_msg_.gear_location == 0)
  {
    stPNC2PadHeartMsg.stChassis.gearSta = 0;
  }
  if (vehicle_chassis_state_msg_.gear_location == 7)
  {
    stPNC2PadHeartMsg.stChassis.gearSta = 1;
  }
  if (vehicle_chassis_state_msg_.gear_location == 1)
  {
    stPNC2PadHeartMsg.stChassis.gearSta = 2;
  }

  stPNC2PadHeartMsg.stChassis.parkingBrak = vehicle_chassis_state_msg_.parking_brake;

  vehicle_chassis_state_mutex_.unlock();

  // 车辆位置和姿态赋值
  xw5651_mutex_.lock();
  // 发布四元数 20220930
  Eigen::Vector3d eulerAngle(xw5651_msg_.angle_heading, 0, 0); /*初始化欧拉角*/
  Eigen::Quaterniond temp_q = Eigen::AngleAxisd(eulerAngle[0], ::Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(eulerAngle[1], ::Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(eulerAngle[2], ::Eigen::Vector3d::UnitY()); /*XYZ为东北天坐标系*/

  stPNC2PadHeartMsg.stQuaternion.x = temp_q.x();
  stPNC2PadHeartMsg.stQuaternion.y = temp_q.y();
  stPNC2PadHeartMsg.stQuaternion.z = temp_q.z();
  stPNC2PadHeartMsg.stQuaternion.w = temp_q.w();

  // 发布位置
  //  stPNC2PadHeartMsg.stGlobalPostion.bIsSouth =false;
  //  stPNC2PadHeartMsg.stGlobalPostion.emCoordinate =3;
  //  stPNC2PadHeartMsg.stGlobalPostion.fLat =xw5651_msg_.latitude;
  //  stPNC2PadHeartMsg.stGlobalPostion.fLon =xw5651_msg_.longitude;
  //  stPNC2PadHeartMsg.stGlobalPostion.fHeight =xw5651_msg_.height;
  xw5651_mutex_.unlock();

  QByteArray datagram;
  datagram.clear();
  datagram.append((const char *)&stPNC2PadHeartMsg, sizeof(PNC2PadHeartMsgST));

  emit SendNetMsg(QByteArray((char *)&stPNC2PadHeartMsg, sizeof(PNC2PadHeartMsgST)), sizeof(PNC2PadHeartMsgST),
                  QHostAddress(REMOTE_CONTROL_IP), REMOTE_CONTROL_AUTO_DRIVING_PORT);
}

void ParentDlg::SendPathBindMsg() /*发送路径装订报文*/
{
  // UINT8 ucFrameMultiFlag;  /*多包标识*/
  // UINT16 usFrameNum;       /*多包包数*/
  // UINT32 uiTotalByteNum;   /*总装订字节数*/
  // UINT16 usFrameNo;        /*本包序号*/
  // UINT16 usFrameDataBytes; /*本包装订字节数*/
  // QByteArray arrFrameData; /*当前帧装订的路径数据*/
  // arrFrameData.clear();

  // if (m_listTrackPathCmd.size() <= 20) /*总路径点小于20个，则直接单帧发送*/
  // {
  //   ucFrameMultiFlag = 0x0;                                                   /*多包标识，单包*/
  //   usFrameNum = 0x1;                                                         /*多包包数*/
  //   uiTotalByteNum = m_listTrackPathCmd.size() * sizeof(SmoothPathPointST);   /*总装订字节数*/
  //   usFrameNo = 0x0;                                                          /*本包序号*/
  //   usFrameDataBytes = m_listTrackPathCmd.size() * sizeof(SmoothPathPointST); /*本包装订字节数*/
  //   for (INT32 i = 0; i < m_listTrackPathCmd.size(); i++)
  //   {
  //     arrFrameData.append((const char*)&(m_listTrackPathCmd.at(i)), sizeof(SmoothPathPointST));
  //     /*当前帧装订的路径数据*/
  //   }

  //   FoldAndSendPathBindMsg(ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
  //                          arrFrameData); /*打包并发送路径数据*/
  // }
  // else /*总路径点大于20个,则分包发送*/
  // {
  //   uiTotalByteNum = m_listTrackPathCmd.size() * sizeof(SmoothPathPointST); /*总装订字节数*/
  //   usFrameNum = m_listTrackPathCmd.size() / 20 + ((m_listTrackPathCmd.size() % 20 == 0) ? 0 : 1); /*数据包数*/
  //   usFrameNo = 0;                                                                                 /*包序号*/
  //   usFrameDataBytes = 20 * sizeof(SmoothPathPointST); /*本包装订字节数*/
  //   UINT32 uiSendByteNum = 0;                          /*已发送字节数*/
  //   while ((uiTotalByteNum - uiSendByteNum) > 20 * sizeof(SmoothPathPointST))
  //   {
  //     arrFrameData.clear(); /*每次都要清一次发送缓存*/
  //     usFrameNo++;          /*包序号自增*/
  //     if (0x1 == usFrameNo) /*包序号为1*/
  //     {
  //       ucFrameMultiFlag = 0x1; /*首包标识*/
  //       // emit MsgTableDisp("命令发送", "路径装订起始包");
  //     }
  //     else
  //     {
  //       ucFrameMultiFlag = 0x2; /*中间包*/
  //     }

  //     for (INT32 i = (usFrameNo - 1) * 20; i < ((usFrameNo - 1) * 20 + 20); i++)
  //     {
  //       arrFrameData.append((const char*)&(m_listTrackPathCmd.at(i)),
  //                           sizeof(SmoothPathPointST)); /*当前帧装订的路径数据*/
  //     }

  //     FoldAndSendPathBindMsg(ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
  //                            arrFrameData);            /*打包并发送路径数据*/
  //     uiSendByteNum += 20 * sizeof(SmoothPathPointST); /*已发送字节数+*/
  //   }

  //   arrFrameData.clear();   /*每次都要清一次发送缓存*/
  //   usFrameNo++;            /*包序号自增*/
  //   ucFrameMultiFlag = 0x3; /*结束包*/
  //   emit MsgTableDisp("命令发送", "路径装订结束包");
  //   usFrameDataBytes = (UINT16)(uiTotalByteNum - uiSendByteNum); /*本包装订字节数*/
  //   for (INT32 i = (usFrameNo - 1) * 20; i < m_listTrackPathCmd.size(); i++)
  //   {
  //     arrFrameData.append((char*)&(m_listTrackPathCmd.at(i)), sizeof(SmoothPathPointST)); /*当前帧装订的路径数据*/
  //   }
  //   FoldAndSendPathBindMsg(ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
  //                          arrFrameData); /*打包并发送路径数据*/
  // }
}

// 解析遥控器发送至决策控制单元的自动驾驶报文
void ParentDlg::RecvRemoteControlAutoDrivingMsgProc(QByteArray datagram, INT32 iLen)
{
  // std::cout << "recv AUTO Msg from remote control " << std::endl;
  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));    /*获取报文头内容*/
  UINT8 ucCheckSum = CheckSum((UINT8 *)(datagram.data()), iLen); /*计算校验和*/
  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))                /*校验和正确*/
  {
    if (stNetHeader.usMsgType == PAD2PNC_CTRL_MSG)
    {
      // Pad2ADASHeartMsgST data;
      // memcpy(&data, datagram.data(), sizeof(Pad2ADASHeartMsgST));
      // if (data.ctrlType == 0x04) // 寻迹避障
      // {
      //   emit sendMotionStart(data.movingFlag);
      // }
    }
  }
  else
  {
    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
                          .arg(ucCheckSum, 2, 16, QChar('0'));
    //  m_pRecord->write(strInfo, RECORD_ERROR);
    // std::cout << strInfo.toStdString() << std::endl;
    //  MsgTableDisp("以太网报文校验和错误", strInfo);
  }
}

void ParentDlg::RecvRemoteMsgProc(QByteArray datagram, INT32 iLen) // 运动开始
{
  std::cout << "recv  Msg from remote control " << std::endl;
  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));    /*获取报文头内容*/
  UINT8 ucCheckSum = CheckSum((UINT8 *)(datagram.data()), iLen); /*计算校验和*/
  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))                /*校验和正确*/
  {
    if (stNetHeader.usMsgType == PAD2PNC_CTRL_MSG)
    {
      Pad2ADASHeartMsgST data;
      memcpy(&data, datagram.data(), sizeof(Pad2ADASHeartMsgST));
      if (data.ctrlType == 0x04) // 寻迹避障
      {
        emit sendMotionStart(data.movingFlag);
      }

      if (0xf1 == data.ctrlType) // 人控模式
      {
        emit sendDriveModeCmd(0);
      }
      if (0xf2 == data.ctrlType) // 无人模式
      {
        emit sendDriveModeCmd(1);
      }
      // if (0xf3 == data.ctrlType) // re,mote模式
      // {
      //   emit sendDriveModeCmd(4);
      // }
      if (0xfa == data.ctrlType)
      {
        rec_task_ = false;
        std::cout << "rec_task is" << rec_task_ << std::endl;
      }
      if (0xff == data.ctrlType)
      {
        emit sendTargetThrottleAndBrakePct(0, 30);
      }
      if (0x77 == data.ctrlType)
      {
        emit sendTargetGearCmd(2);
        emit sendTargetThrottleAndBrakePct(0, 0);
      }
    }
  }
  else
  {
    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
                          .arg(ucCheckSum, 2, 16, QChar('0'));
    //  m_pRecord->write(strInfo, RECORD_ERROR);
    // std::cout << strInfo.toStdString() << std::endl;
    //  MsgTableDisp("以太网报文校验和错误", strInfo);
  }
}

void ParentDlg::RecvRemoteDriveMsgProc(QByteArray datagram, INT32 iLen) // 遥控驾驶控制
{

  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));    /*获取报文头内容*/
  UINT8 ucCheckSum = CheckSum((UINT8 *)(datagram.data()), iLen); /*计算校验和*/
  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))                /*校验和正确*/
  {

    if (stNetHeader.usMsgType == REMOTE_DRIVE_CYCLE_MSG)
    {
      RemoteDriveST data;

      memcpy(&data, datagram.data(), sizeof(RemoteDriveST));
      emit sendRemoteDrive(data);
      return;
    }
  }
  else
  {
    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
                          .arg(ucCheckSum, 2, 16, QChar('0'));
    //  m_pRecord->write(strInfo, RECORD_ERROR);
    // std::cout << strInfo.toStdString() << std::endl;
    //  MsgTableDisp("以太网报文校验和错误", strInfo);
  }
}

void ParentDlg::TaskpointsMsgProc(QByteArray datagram, INT32 iLen)
{

  std::cout << "recv taskpoints Msg from remote control " << std::endl;

  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));
  UINT8 ucCheckSum = CheckSum((UINT8 *)(datagram.data()), iLen);
  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))
  {
    if (stNetHeader.usMsgType == PAD2TANK_TASK_MSG)
    {
      TaskPointsST data;
      memcpy(&data, datagram.data(), sizeof(TaskPointsST));

      // task_points_.clear();

      /*首包或单包起始时，清空障碍物存储变量*/
      if ((0x01 == data.multipakgeflag) || (0x0 == data.multipakgeflag))
      {
        task_points_.clear();
      }

      if (0x1 == data.multipakgeflag) /*多包的首包*/
      {
        /*首包中描述的共有几包数和装订路径点总字节数*/
        // m_uiObsClusterFrameNum = data.usFrameNum; /*数据总包数*/
        uint size = data.taskpakgebytenum / sizeof(TaskPointST);
        for (int i = 0; i < size; i++)
        {
          // TaskPointST point =data.taskpoint[i];
          task_points_.push_back(data.taskpoint[i]);
        }
      }
      else if (0x2 == data.multipakgeflag) /*多包的中间包*/
      {
        uint size = data.taskpakgebytenum / sizeof(TaskPointST);
        for (int i = 0; i < size; i++)
        {
          // TaskPointST point =data.taskpoint[i];
          task_points_.push_back(data.taskpoint[i]);
        }
      }
      else if (0x3 == data.multipakgeflag) /*多包的结束包*/
      {
        uint size = data.taskpakgebytenum / sizeof(TaskPointST);
        for (int i = 0; i < size; i++)
        {
          // TaskPointST point =data.taskpoint[i];
          task_points_.push_back(data.taskpoint[i]);
        }
        std::cout << "task  recv finished" << std::endl;
        emit emittaskpoints(task_points_, data.tasktype);
        rec_task_ = true;
      }
      else if (0x0 == data.multipakgeflag)
      {
        std::cout << "danbao``````````````````````````" << std::endl;
        uint size = data.taskpakgebytenum / sizeof(TaskPointST);
        for (int i = 0; i < size; i++)
        {
          // TaskPointST point =data.taskpoint[i];
          task_points_.push_back(data.taskpoint[i]);
        }
        emit emittaskpoints(task_points_, data.tasktype);
        rec_task_ = true;
      }
    }
  }

  else
  {
    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
                          .arg(ucCheckSum, 2, 16, QChar('0'));
    //  m_pRecord->write(strInfo, RECORD_ERROR);
    // std::cout << strInfo.toStdString() << std::endl;
    //  MsgTableDisp("以太网报文校验和错误", strInfo);
  }
}

// 解析遥控器发送至决策控制单元的底盘报文
void ParentDlg::RecvRemoteControlChassisMsgProc(QByteArray datagram, INT32 iLen)
{
  // std::cout << "recv chassis Msg from remote control " << std::endl;
  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST));    /*获取报文头内容*/
  UINT8 ucCheckSum = CheckSum((UINT8 *)(datagram.data()), iLen); /*计算校验和*/
  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))                /*校验和正确*/
  {

    if (stNetHeader.usMsgType == REMOTE_CONTROL_TO_PNC_CHASSIS_EVENT_MSG)
    {
      PAD2PNC_ST data;
      memcpy(&data, datagram.data(), sizeof(PAD2PNC_ST));
      emit sendMotionStart(data.cmd.movingFlag);
      // std::cout << "driving_cmd_mode " << static_cast<int>(driving_cmd_mode_) << std::endl;
      return;
    }
  }
  else
  {
    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
                          .arg(ucCheckSum, 2, 16, QChar('0'));
    //  m_pRecord->write(strInfo, RECORD_ERROR);
    // std::cout << strInfo.toStdString() << std::endl;
    //  MsgTableDisp("以太网报文校验和错误", strInfo);
  }
}

// void ParentDlg::SendRouteAndLocalPathMsg(
//     unsigned char ucPathType, std::vector<GlobalPositionST> listPathPoint) /*发送路径装订结果给战斗操作台进行确认*/
// {
//  unsigned char ucFrameMultiFlag;             /*多包标识*/
//  unsigned short int usFrameNum;              /*多包包数*/
//  unsigned int uiTotalByteNum;                /*总装订字节数*/
//  unsigned short int usFrameNo;               /*本包序号*/
//  unsigned short int usFrameDataBytes;        /*本包装订字节数*/
//  std::vector<GlobalPositionST> arrFrameData; /*当前帧装订的路径数据*/

//  if (listPathPoint.size() <= 20) /*总路径点小于20个，则直接单帧发送*/
//  {
//    ucFrameMultiFlag = 0x0;                                             /*多包标识，单包*/
//    usFrameNum = 0x1;                                                   /*多包包数*/
//    uiTotalByteNum = listPathPoint.size() * sizeof(GlobalPositionST);   /*总装订字节数*/
//    usFrameNo = 0x0;                                                    /*本包序号*/
//    usFrameDataBytes = listPathPoint.size() * sizeof(GlobalPositionST); /*本包装订字节数*/
//    for (int i = 0; i < listPathPoint.size(); i++)
//    {
//      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
//    }

//    FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
//                           arrFrameData); /*打包并发送路径数据*/
//  }
//  else /*总路径点大于20个,则分包发送*/
//  {
//    uiTotalByteNum = listPathPoint.size() * sizeof(GlobalPositionST);                    /*总装订字节数*/
//    usFrameNum = listPathPoint.size() / 20 + ((listPathPoint.size() % 20 == 0) ? 0 : 1); /*数据包数*/
//    usFrameNo = 0;                                                                       /*包序号*/
//    usFrameDataBytes = 20 * sizeof(GlobalPositionST);                                    /*本包装订字节数*/
//    unsigned int uiSendByteNum = 0;                                                      /*已发送字节数*/
//    while ((uiTotalByteNum - uiSendByteNum) > 20 * sizeof(GlobalPositionST))             /*保证肯定剩一包*/
//    {
//      arrFrameData.clear(); /*当前帧装订的路径数据*/
//      usFrameNo++;          /*包序号自增*/
//      if (0x1 == usFrameNo) /*包序号为1*/
//      {
//        ucFrameMultiFlag = 0x1; /*首包标识*/
//      }
//      else
//      {
//        ucFrameMultiFlag = 0x2; /*中间包*/
//      }

//      for (int i = (usFrameNo - 1) * 20; i < ((usFrameNo - 1) * 20 + 20); i++)
//      {
//        arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
//      }

//      FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
//                             arrFrameData);           /*打包并发送路径数据*/
//      uiSendByteNum += 20 * sizeof(GlobalPositionST); /*已发送字节数+*/
//    }

//    arrFrameData.clear();                              /*当前帧装订的路径数据*/
//    usFrameNo++;                                       /*包序号自增*/
//    ucFrameMultiFlag = 0x3;                            /*结束包*/
//    usFrameDataBytes = uiTotalByteNum - uiSendByteNum; /*本包装订字节数*/
//    for (int i = (usFrameNo - 1) * 20; i < listPathPoint.size(); i++)
//    {
//      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
//    }

//    FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes,
//                           arrFrameData); /*打包并发送路径数据*/
//  }
// }

/*打包并且发送跟踪路径数据*/
// void ParentDlg::FoldAndSendPathBindMsg(unsigned char ucPathType, unsigned char ucFrameMultiFlag,
//                                        unsigned short int usFrameNum, unsigned int uiTotalByteNum,
//                                        unsigned short int usFrameNo, unsigned short int usFrameDataBytes,
//                                        std::vector<GlobalPositionST> arrFrameData)
// {
//   PNC2PadRouteAndLocalPathMsgST stMsg;
//   memset(&stMsg, 0, sizeof(PNC2PadRouteAndLocalPathMsgST));

//   stMsg.stNetHeader.usMsgType = PNC2PAD_TRAJECTORY_UP_MSG;
//   stMsg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
//   stMsg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
//   stMsg.stNetHeader.usMsgLen = sizeof(PNC2PadRouteAndLocalPathMsgST); /*报文长度*/
//   stMsg.stNetHeader.uiMsgNo = 0;
//   stMsg.stNetHeader.usAck = NET_MSG_NOACK;

//   stMsg.ucPathType = ucPathType;
//   stMsg.ucMultiFrameFlag = ucFrameMultiFlag;
//   stMsg.usFrameNum = usFrameNum;             /*数据包数*/
//   stMsg.uiTotalByteNum = uiTotalByteNum;     /*装订数据总字节数*/
//   stMsg.usFrameNo = usFrameNo;               /*数据包序号*/
//   stMsg.usFrameDataBytes = usFrameDataBytes; /*当前包内装订数据总字节数*/
//   for (int i = 0; i < arrFrameData.size(); i++)
//   {
//     memcpy(&(stMsg.stGlobalPostion[i]), &(arrFrameData.at(i)), sizeof(GlobalPositionST));
//   }

//   stMsg.stNetEnd.ucCheckSum = CheckSum((unsigned char*)(&stMsg), sizeof(PNC2PadRouteAndLocalPathMsgST));

//   emit SendNetMsg(QByteArray((char*)&stMsg, stMsg.stNetHeader.usMsgLen), stMsg.stNetHeader.usMsgLen,
//                   QHostAddress(REMOTE_CONTROL_IP), REMOTE_CONTROL_AUTO_DRIVING_PORT);
// }

// 解析感知发送至决策控制单元的感知报文
void ParentDlg::RecvPerceptionMsgProc(QByteArray datagram, INT32 iLen)
{
  NetHeaderST header;
  memset(&header, 0, sizeof(NetHeaderST));
  memcpy(&header, datagram, sizeof(NetHeaderST));
  if (header.usMsgType == 0xE030)
  {
    if (datagram.size() != sizeof(Percpt2VCObsClusterMsgST))
    {
      std::cout << "RecvPerceptionMsgProc size is not equal " << std::endl;
      return;
    }
    Percpt2VCObsClusterMsgST stPercpt2VCMsg;
    memset(&stPercpt2VCMsg, 0, sizeof(Percpt2VCObsClusterMsgST));
    memcpy(&stPercpt2VCMsg, datagram, sizeof(Percpt2VCObsClusterMsgST));
    /*首包或单包起始时，清空障碍物存储变量*/
    m_uiRecvObsClusterMsgNum = stPercpt2VCMsg.stNetHeader.uiMsgNo; /*接收到障碍聚类报文信息计数*/

    /*首包或单包起始时，清空障碍物存储变量*/
    if ((0x01 == stPercpt2VCMsg.ucFrameMultiFlag) || (0x0 == stPercpt2VCMsg.ucFrameMultiFlag))
    {
      m_listObsClusterInfo.clear(); /*障碍物聚类信息*/
                                    // 车辆位置和姿态赋值
      xw5651_mutex_.lock();
      perception_vehicle_ins_heading_ = xw5651_msg_.angle_heading;
      xw5651_mutex_.unlock();
      vehicle_motion_state_mutex_.lock();
      perception_vehicle_x_ = vehicle_motion_state_msg_.x;
      perception_vehicle_y_ = vehicle_motion_state_msg_.y;
      vehicle_motion_state_mutex_.unlock();
    }

    if (0x1 == stPercpt2VCMsg.ucFrameMultiFlag) /*多包的首包*/
    {
      /*首包中描述的共有几包数和装订路径点总字节数*/
      m_uiObsClusterFrameNum = stPercpt2VCMsg.usFrameNum; /*数据总包数*/

      /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
      m_uiRecvObsClusterFrameCnt = 1; /*自计接收到的要跟踪的路径数据点的数据帧号*/

      for (int i = 0; i < stPercpt2VCMsg.ucFrameObsNum; i++) /*将当前包内的数据转存到list中*/
      {
        m_listObsClusterInfo.push_back(stPercpt2VCMsg.stObsClusterInfo[i]);
      }
      stPercpt2VCMsg.x = perception_vehicle_x_;
      stPercpt2VCMsg.y = perception_vehicle_y_;
      stPercpt2VCMsg.fVehCourse = perception_vehicle_ins_heading_;
      stPercpt2VCMsg.stNetHeader.uiSrcIP = QHostAddress(PERCETION_IP).toIPv4Address(); // 信息来自感知，所以是感知IP

      emit SendNetMsg(QByteArray((char *)&stPercpt2VCMsg, sizeof(Percpt2VCObsClusterMsgST)),
                      sizeof(Percpt2VCObsClusterMsgST), QHostAddress(REMOTE_CONTROL_IP),
                      REMOTE_CONTROL_AUTO_DRIVING_PORT);
    }
    else if (0x2 == stPercpt2VCMsg.ucFrameMultiFlag) /*多包的中间包*/
    {
      /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
      m_uiRecvObsClusterFrameCnt++; /*自计接收到的要跟踪的路径数据点的数据包数*/

      for (int i = 0; i < (stPercpt2VCMsg.ucFrameObsNum); i++) /*将当前包内的数据转存到list中*/
      {
        m_listObsClusterInfo.push_back(stPercpt2VCMsg.stObsClusterInfo[i]);
      }
      stPercpt2VCMsg.x = perception_vehicle_x_;
      stPercpt2VCMsg.y = perception_vehicle_y_;
      stPercpt2VCMsg.fVehCourse = perception_vehicle_ins_heading_;
      stPercpt2VCMsg.stNetHeader.uiSrcIP = QHostAddress(PERCETION_IP).toIPv4Address();

      emit SendNetMsg(QByteArray((char *)&stPercpt2VCMsg, sizeof(Percpt2VCObsClusterMsgST)),
                      sizeof(Percpt2VCObsClusterMsgST), QHostAddress(REMOTE_CONTROL_IP),
                      REMOTE_CONTROL_AUTO_DRIVING_PORT);
    }
    else if (0x3 == stPercpt2VCMsg.ucFrameMultiFlag) /*多包的结束包*/
    {
      /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
      m_uiRecvObsClusterFrameCnt++; /*自计接收到的要跟踪的路径数据点的数据帧号*/

      if (m_uiRecvObsClusterFrameCnt != m_uiObsClusterFrameNum) /*自计包数不等于发送的总包数*/
      {
        // 此处存在BUG UDP不保证顺序到达
        std::cout << "决策规划单元:接收sibianxing障碍聚类报文包数错误" << std::endl;
      }
      else /*接收报文正确*/
      {
        for (int i = 0; i < (stPercpt2VCMsg.ucFrameObsNum); i++) /*将当前包内的数据转存到list中*/
        {
          m_listObsClusterInfo.push_back(stPercpt2VCMsg.stObsClusterInfo[i]);
        }
        stPercpt2VCMsg.x = perception_vehicle_x_;
        stPercpt2VCMsg.y = perception_vehicle_y_;
        stPercpt2VCMsg.fVehCourse = perception_vehicle_ins_heading_;
        stPercpt2VCMsg.stNetHeader.uiSrcIP = QHostAddress(PERCETION_IP).toIPv4Address();
        emit SendNetMsg(QByteArray((char *)&stPercpt2VCMsg, sizeof(Percpt2VCObsClusterMsgST)),
                        sizeof(Percpt2VCObsClusterMsgST), QHostAddress(REMOTE_CONTROL_IP),
                        REMOTE_CONTROL_AUTO_DRIVING_PORT);
        // std::cout << "Rec Obj size is " << m_listObsClusterInfo.size() << std::endl;
        /////////////////////////////////////
      }
    }
    else
    {
      /*无效报文*/
    }
  }
  else if (header.usMsgType == 0xE080)
  {
    // std::cout << "rec multi objs pkg" << std::endl;
    // 判断是否为首包
    unsigned char ucFrameMultiFlag = datagram.at(24);
    if (ucFrameMultiFlag == 0x01)
    {
      obs_.clear();
      obs_vertex_one_dim_.clear();
      obs_vertex_number_.clear();
      // std::cout << " rec first pkg" << std::endl;
    }
    if (0x1 == ucFrameMultiFlag) /*多包的首包*/
    {
      ObsPolyFirstPackageST data;
      memset(&data, 0, sizeof(ObsPolyFirstPackageST));
      memcpy(&data, datagram, sizeof(ObsPolyFirstPackageST));

      /*首包中描述的共有几包数和装订路径点总字节数*/
      m_uiObsFrameNum = data.usFrameNum;
      m_uiRecvObsFrameCnt = 1;

      for (int i = 0; i < data.usObsNo; i++) /*将当前包内的数据转存到list中*/
      {
        obs_vertex_number_.push_back(data.ucObsIndexNo[i]);
        // 包含xy的总点数
        // std::cout << "obs " << i<< " point number include (x y) " << static_cast<unsigned int>(data.ucObsIndexNo[i])
        // << std::endl;  //包含x和y的点数
      }
    }
    else if (0x2 == ucFrameMultiFlag) /*多包的中间包*/
    {
      ObsPolyMiddleANdLastPackageST data;
      memset(&data, 0, sizeof(ObsPolyMiddleANdLastPackageST));
      memcpy(&data, datagram, sizeof(ObsPolyMiddleANdLastPackageST));

      m_uiRecvObsFrameCnt++; /*自计接收到的要跟踪的路径数据点的数据包数*/
      // 此包中的顶点个数
      for (int i = 0; i < data.usCurObsVertexNo; i += 2)
      {
        // 转换为顶点个数
        obs_vertex_one_dim_.push_back({data.usObsVertex[i] / 100.0f, data.usObsVertex[i + 1] / 100.0f});
      }
    }
    else if (0x3 == ucFrameMultiFlag) /*多包的结束包*/
    {
      /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
      m_uiRecvObsFrameCnt++;                      /*自计接收到的要跟踪的路径数据点的数据帧号*/
      if (m_uiObsFrameNum != m_uiRecvObsFrameCnt) /*自计包数不等于发送的总包数*/
      {
        // 此处存在BUG UDP不保证顺序到达
        std::cout << "决策规划单元:接收障碍聚类报文包数错误" << std::endl;
      }
      else /*接收报文正确*/
      {
        ObsPolyMiddleANdLastPackageST data;
        memset(&data, 0, sizeof(ObsPolyMiddleANdLastPackageST));
        memcpy(&data, datagram, sizeof(ObsPolyMiddleANdLastPackageST));
        for (int i = 0; i < data.usCurObsVertexNo; i += 2)
        {
          obs_vertex_one_dim_.push_back({data.usObsVertex[i] / 100.0f, data.usObsVertex[i + 1] / 100.0f});
        }
        // std::cout << "rec last pkg" << std::endl;

        // std::cout << "obs number " << obs_vertex_number_.size() << std::endl;
        // std::cout << "obs veterx total number is " << obs_vertex_one_dim_.size() << std::endl;
        // 发布ros消息
        int sum = 0;
        for (int j = 0; j < obs_vertex_number_.size(); ++j)
        {
          int start = sum;
          if (obs_vertex_number_.at(j) % 2 != 0)
          {
            std::cout << "error " << std::endl;
            // exit(1);
          }
          int end = sum + obs_vertex_number_.at(j) / 2;
          // std::cout << "obs vertex number " << obs_vertex_number_.at(j) / 2 << std::endl;
          // std::cout << "start " << start << std::endl;
          // std::cout << "end " << end << std::endl;

          std::vector<std::pair<float, float>> one_obs;
          for (int i = start; i < end; ++i)
          {
            one_obs.push_back(obs_vertex_one_dim_.at(i));
          }
          obs_.push_back(one_obs);
          sum += obs_vertex_number_.at(j) / 2;
        }
        // std::cout << " pad to ros pub " << std::endl;
        // std::cout << obs_.size() << std::endl;
        emit sendObstacle(obs_);
      }
    }
    else
    {
      /*无效报文*/
    }
  }
  else
  {
    std::cout << "no such percept msg" << std::endl;
  }
}
void ParentDlg::SendMapCordtBindMsg() /*发送地图坐标系装订报文*/
{
  //  if (0xFFFFFFFF > m_uiSendMapCordtBindMsgNum) /*发送地图坐标系装订报文计数*/
  //  {
  //    m_uiSendMapCordtBindMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendMapCordtBindMsgNum = 0;
  //  }
  //  ui->labFP2VCCordinateBindMsgNum->setText(QString("%1").arg(m_uiSendMapCordtBindMsgNum));

  //  FP2VCCorditBindMsgST stFP2VCMsg; /*坐标系装订报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCCorditBindMsgST));

  //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_COORDINATE_BIND_MSG;
  //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCCorditBindMsgST);
  //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_ACK;

  //  memcpy(&stFP2VCMsg.stMapCordit, &g_stMapCorditCmd, sizeof(MapCorditST));

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stFP2VCMsg, sizeof(FP2VCCorditBindMsgST));
  //  emit SendNetMsg(datagram, sizeof(FP2VCCorditBindMsgST), QHostAddress(VEHICLE_CTRL_IP), VEHICLE_CTRL_PORT);

  //  emit MsgTableDisp("命令发送", "坐标系装订");
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 坐标系装订报文", RECORD_PROCESS);
}

void ParentDlg::SendPathAskMsg() /*发送跟踪路径查询报文*/
{
  //  if (0xFFFFFFFF > m_uiSendPathAskMsgNum) /*发送跟踪路径查询报文计数*/
  //  {
  //    m_uiSendPathAskMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendPathAskMsgNum = 0;
  //  }
  //  ui->labFP2VCPathAskMsgNum->setText(QString("%1").arg(m_uiSendPathAskMsgNum));

  //  FP2VCCmdMsgST stFP2VCMsg; /*底盘路径查询报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCCmdMsgST));

  //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_MOVEPATH_ASK_MSG;
  //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCCmdMsgST);
  //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_ACK;

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stFP2VCMsg, sizeof(FP2VCCmdMsgST));
  //  emit SendNetMsg(datagram, sizeof(FP2VCCmdMsgST), QHostAddress(VEHICLE_CTRL_IP), VEHICLE_CTRL_PORT);

  //  emit MsgTableDisp("命令发送", "跟踪路径查询");
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 跟踪路径查询报文", RECORD_PROCESS);
}

void ParentDlg::SendMapCordtAskMsg() /*发送运动坐标系查询报文*/
{
  //  if (0xFFFFFFFF > m_uiSendMapCordtAskMsgNum) /*发送运动坐标系查询报文计数*/
  //  {
  //    m_uiSendMapCordtAskMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendMapCordtAskMsgNum = 0;
  //  }
  //  ui->labFP2VCCordinateAskMsgNum->setText(QString("%1").arg(m_uiSendMapCordtAskMsgNum));

  //  FP2VCCmdMsgST stFP2VCMsg; /*底盘运动坐标系查询报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCCmdMsgST));

  //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_COORDINATE_ASK_MSG;
  //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCCmdMsgST);
  //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_ACK;

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stFP2VCMsg, sizeof(FP2VCCmdMsgST));
  //  emit SendNetMsg(datagram, sizeof(FP2VCCmdMsgST), QHostAddress(VEHICLE_CTRL_IP), VEHICLE_CTRL_PORT);

  //  emit MsgTableDisp("命令发送", "坐标系查询");
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 坐标系查询报文", RECORD_PROCESS);
}

void ParentDlg::SendAckMsg(UINT32 uiFrameNo) /*发送接收确认报文*/
{
  //  if (0xFFFFFFFF > m_uiSendAckMsgNum) /*发送接收确认报文计数*/
  //  {
  //    m_uiSendAckMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendAckMsgNum = 0;
  //  }
  //  ui->labFP2VCAckMsgNum->setText(QString("%1").arg(m_uiSendAckMsgNum));

  //  FP2VCAckMsgST stNetAckMsg; /*接收确认报文*/
  //  memset(&stNetAckMsg, 0, sizeof(FP2VCAckMsgST));

  //  stNetAckMsg.stNetHeader.usMsgType = FP2VC_ACK_MSG; /*接收确认报文*/
  //  stNetAckMsg.stNetHeader.usMsgLen = sizeof(FP2VCAckMsgST);
  //  stNetAckMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stNetAckMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stNetAckMsg.stNetHeader.usAck = NET_MSG_NOACK;

  //  stNetAckMsg.uiMsgNo = uiFrameNo;

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stNetAckMsg, sizeof(FP2VCAckMsgST));

  //  emit SendNetMsg(datagram, sizeof(FP2VCAckMsgST), QHostAddress(VEHICLE_CTRL_IP), VEHICLE_CTRL_PORT);
  //  /*发送以太网报文*/

  //  emit MsgTableDisp("命令发送", "回复接收确认");
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 接收确认报文", RECORD_PROCESS);
}

void ParentDlg::SendChasCtrlMsg(FP2VCChasCtrlMsgST stFP2VCMsg) /*发送底盘设备控制报文*/
{
  //  if (0xFFFFFFFF > m_uiSendChasCtrlMsgNum) /*发送底盘设备控制报文计数*/
  //  {
  //    m_uiSendChasCtrlMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendChasCtrlMsgNum = 0;
  //  }
  //  ui->labFP2VCChasCtrlMsgNum->setText(QString("%1").arg(m_uiSendChasCtrlMsgNum));

  //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_CHASEQCTRL_MSG; /*底盘设备控制报文*/
  //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCChasCtrlMsgST);
  //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_NOACK;

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stFP2VCMsg, sizeof(FP2VCChasCtrlMsgST));

  //  emit SendNetMsg(datagram, sizeof(FP2VCChasCtrlMsgST), QHostAddress(VEHICLE_CTRL_IP),
  //                  VEHICLE_CTRL_PORT); /*发送以太网报文*/

  //  if (CMD_CHAS_MOVE != stFP2VCMsg.stVC2ChasCmd.ucCmdType) /*运动控制是周期命令，不显示在表格中*/
  //  {
  //    emit MsgTableDisp("命令发送", "底盘控制报文");
  //  }ss
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 底盘控制报文", RECORD_PROCESS);
}
// void ParentDlg::ADCTrajectorySubscripeFunc(const planning_msgs::TrajectoryPointArray::ConstPtr msg)/*局部规划轨迹数据订阅*/
//         {
//             std::vector<GlobalPositionST> listPoint;
//             listPoint.clear();
//             GlobalPositionST stPoint;
//             memset(&stPoint, 0, sizeof(GlobalPositionST));
//             for(size_t i=0; i<msg->points.size(); i++)
//             {
//                 stPoint.emCoordinate = 3;/*UTM坐标系*/
//                 stPoint.bIsSouth = 0;    /*北半球*/
//                 stPoint.ucUTMZoneID = m_stPNC2PadHeartMsg.stGlobalPostion.ucUTMZoneID;
//                 stPoint.fX = msg->points.at(i).x;
//                 stPoint.fY = msg->points.at(i).y;
//                 stPoint.fZ = msg->points.at(i).z;
//                 listPoint.push_back(stPoint);
//             }
//             // int PATH_TYPE_LOCAL_PLAN =0x01; //TODO 临时赋值
//             SendRouteAndLocalPathMsg(PATH_TYPE_LOCAL_PLAN, listPoint);/*向远程操控终端发送局部路径*/
//         }

void ParentDlg::onTrajectory(std::vector<GlobalPositionST> listPoint)
{
//  std::cout << " on  qt traj  0" << std::endl;
//  fflush(stdout);
  std::vector<GlobalPositionST> list_bridge = listPoint;
  SendRouteAndLocalPathMsg(PATH_TYPE_LOCAL_PLAN, list_bridge); /*向远程操控终端发送局部路径*/

//  std::cout << " on  qt traj 1 " << std::endl;
 // fflush(stdout);
}

void ParentDlg::onGlobalPath84(std::vector<localPositionST> listPoint)
{
  static int cnt = 50;
  if (cnt == 50)
  {
    std::vector<localPositionST> list_bridge = listPoint;
    SendRouteAndGlobalPathMsg(PATH_TYPE_GLOBAL_ROUTE, list_bridge); /*向远程操控终端发送全局路径*/
    cnt = 0;
  }
  cnt++;
}

void ParentDlg::onObstacle(std::vector<ObstacleST> listPoint)
{
  std::cout << " on qt obs 0" << std::endl;

  fflush(stdout);
  m_uiSendObstacleNo++;
  int usFrameNum = 0;
  if (m_uiSendObstacleNo == m_uiSendDilixinxiNoThres)
  {
    ObstacleBindMsg obstacle_send_msg;
    std::vector<ObstacleST> arrFrameData; /*当前帧装订的路径数据*/
    memset(&obstacle_send_msg, 0, sizeof(ObstacleBindMsg));
    obstacle_send_msg.stNetHeader.usMsgType = TANK_Obstacle_MSG;
    obstacle_send_msg.stNetHeader.usMsgLen = sizeof(ObstacleBindMsg);
    obstacle_send_msg.stNetHeader.uiMsgTime = 0;
    obstacle_send_msg.stNetHeader.uiMsgNo = 0;
    obstacle_send_msg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
    obstacle_send_msg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
    obstacle_send_msg.stNetHeader.usAck = NET_MSG_NOACK;
    obstacle_send_msg.usFrameNum = listPoint.size() / 10 + ((listPoint.size() % 10 == 0) ? 0 : 1);
    if (listPoint.size() <= 10) /*总障碍物个数小于等于10个,则直接单帧发送*/
    {
      obstacle_send_msg.ucFrameMultiFlag = 0x0; /*多包标识,单包*/
      obstacle_send_msg.usFrameNo = 0x0;        /*本包序号*/
      for (size_t i = 0; i < listPoint.size(); i++)
      {
        arrFrameData.push_back(listPoint.at(i)); /*当前帧装订的路径数据*/
      }
      int usFrameNum = 0x1;
      SendObstacleMsg(obstacle_send_msg.ucFrameMultiFlag, usFrameNum, obstacle_send_msg.usFrameNo, arrFrameData); /*打包并发送路径数据*/
    }
    else /*总障碍物个数大于10个,则分包发送*/
    {
      usFrameNum = 0;
      obstacle_send_msg.usFrameNo = 0;
      int uiSendByteNum = 0;                          /*已发送障碍物个数*/
      while ((listPoint.size() - uiSendByteNum) > 10) /*保证肯定剩一包*/
      {
        arrFrameData.clear();                   /*当前帧装订的路径数据*/
        obstacle_send_msg.usFrameNo++;          /*包序号自增*/
        if (0x1 == obstacle_send_msg.usFrameNo) /*包序号为1*/
        {
          obstacle_send_msg.ucFrameMultiFlag = 0x1; /*首包标识*/
        }
        else
        {
          obstacle_send_msg.ucFrameMultiFlag = 0x2; /*中间包*/
        }

        for (int i = (obstacle_send_msg.usFrameNo - 1) * 10; i < (obstacle_send_msg.usFrameNo) * 10; i++)
        {
          arrFrameData.push_back(listPoint.at(i)); /*当前帧装订的路径数据*/
        }
        usFrameNum++;
        SendObstacleMsg(obstacle_send_msg.ucFrameMultiFlag, usFrameNum, obstacle_send_msg.usFrameNo, arrFrameData);
        uiSendByteNum += 10;
      }

      arrFrameData.clear(); /*当前帧装订的路径数据*/
      usFrameNum++;
      obstacle_send_msg.usFrameNo++;            /*包序号自增*/
      obstacle_send_msg.ucFrameMultiFlag = 0x3; /*结束包*/
      for (size_t i = (obstacle_send_msg.usFrameNo - 1) * 10; i < listPoint.size(); i++)
      {
        arrFrameData.push_back(listPoint.at(i)); /*当前帧装订的路径数据*/
      }
      SendObstacleMsg(obstacle_send_msg.ucFrameMultiFlag, usFrameNum, obstacle_send_msg.usFrameNo, arrFrameData);
    }
    m_uiSendObstacleNo = 0;
  }

  std::cout << " on qt obs  1" << std::endl;
  fflush(stdout);
}

void ParentDlg::SendObstacleMsg(uint8 ucFrameMultiFlag, unsigned short int usFrameNum, unsigned short int usFrameNo, std::vector<ObstacleST> arrFrameData)
{
  ObstacleBindMsg stMsg;
  memset(&stMsg, 0, sizeof(ObstacleBindMsg));

  if (0xFFFFFFFF <= m_uiSendMsgNo) /*发送报文序号*/
  {
    m_uiSendMsgNo = 1;
  }
  else
  {
    m_uiSendMsgNo++;
  }

  stMsg.stNetHeader.usMsgType = TANK_Obstacle_MSG;
  stMsg.stNetHeader.usMsgLen = sizeof(ObstacleBindMsg); /*报文长度*/
  stMsg.stNetHeader.uiMsgNo = m_uiSendMsgNo;
  stMsg.stNetHeader.usAck = NET_MSG_NOACK;

  stMsg.ucFrameMultiFlag = ucFrameMultiFlag;
  stMsg.usFrameNum = usFrameNum; /*障碍物个数*/
  for (size_t i = 0; i < arrFrameData.size(); i++)
  {
    memcpy(&(stMsg.stObstacleData[i]), &(arrFrameData.at(i)), sizeof(ObstacleST));
  }

  stMsg.stNetEnd.ucCheckSum = CheckSum((UINT8 *)(&stMsg), sizeof(ObstacleBindMsg));
  stMsg.ObstacleNum = arrFrameData.size();

  QByteArray datagram = QByteArray((const char *)&stMsg, stMsg.stNetHeader.usMsgLen);
  INT32 iLen = stMsg.stNetHeader.usMsgLen;

  emit SendNetMsg(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  // m_pNetSend->SendMsgListAdd(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  //   SendNetMsg((char *)&stMsg, stMsg.stNetHeader.usMsgLen, FLAGS_PAD_IP_GROUP, FLAGS_PAD_PORT); /*发送以太网报文*/
}

void ParentDlg::onDilixinxi(DilixinxiST listPoint)
{
  m_uiSendDilixinxiNo++;
  if (m_uiSendDilixinxiNo == m_uiSendDilixinxiNoThres)
  {
    // PNC2PadINSDataMsgST pad_ins_send_msg;
    FP2DilixinxiBindMsgST tank_dilixinxi_send_msg;
    memset(&tank_dilixinxi_send_msg, 0, sizeof(FP2DilixinxiBindMsgST));
    tank_dilixinxi_send_msg.stNetHeader.usMsgType = TANK_Dilixinxi_MSG;
    tank_dilixinxi_send_msg.stNetHeader.usMsgLen = sizeof(FP2DilixinxiBindMsgST);
    tank_dilixinxi_send_msg.stNetHeader.uiMsgTime = 0;
    tank_dilixinxi_send_msg.stNetHeader.uiMsgNo = 0;
    tank_dilixinxi_send_msg.stNetHeader.uiDestIP = QHostAddress(REMOTE_CONTROL_IP).toIPv4Address();
    tank_dilixinxi_send_msg.stNetHeader.uiSrcIP = QHostAddress(PNC_IP).toIPv4Address();
    tank_dilixinxi_send_msg.stNetHeader.usAck = NET_MSG_NOACK;
    tank_dilixinxi_send_msg.stDilixinxi = listPoint;

    emit SendNetMsg(QByteArray((char *)&tank_dilixinxi_send_msg, sizeof(FP2DilixinxiBindMsgST)), sizeof(FP2DilixinxiBindMsgST),
                    QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
    m_uiSendDilixinxiNo = 0;
  }
}

void ParentDlg::SendRouteAndLocalPathMsg(unsigned char ucPathType, std::vector<GlobalPositionST> listPathPoint)
{
  unsigned char ucFrameMultiFlag;             /*多包标识*/
  unsigned short int usFrameNum;              /*多包包数*/
  unsigned int uiTotalByteNum;                /*总装订字节数*/
  unsigned short int usFrameNo;               /*本包序号*/
  unsigned short int usFrameDataBytes;        /*本包装订字节数*/
  std::vector<GlobalPositionST> arrFrameData; /*当前帧装订的路径数据*/

  if (listPathPoint.size() <= 20) /*总路径点小于20个,则直接单帧发送*/
  {
    ucFrameMultiFlag = 0x0;                                             /*多包标识,单包*/
    usFrameNum = 0x1;                                                   /*多包包数*/
    uiTotalByteNum = listPathPoint.size() * sizeof(GlobalPositionST);   /*总装订字节数*/
    usFrameNo = 0x0;                                                    /*本包序号*/
    usFrameDataBytes = listPathPoint.size() * sizeof(GlobalPositionST); /*本包装订字节数*/
    for (size_t i = 0; i < listPathPoint.size(); i++)
    {
      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
    }

    FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
  }
  else /*总路径点大于20个,则分包发送*/
  {
    uiTotalByteNum = listPathPoint.size() * sizeof(GlobalPositionST);                    /*总装订字节数*/
    usFrameNum = listPathPoint.size() / 20 + ((listPathPoint.size() % 20 == 0) ? 0 : 1); /*数据包数*/
    usFrameNo = 0;                                                                       /*包序号*/
    usFrameDataBytes = 20 * sizeof(GlobalPositionST);                                    /*本包装订字节数*/
    unsigned int uiSendByteNum = 0;                                                      /*已发送字节数*/
    while ((uiTotalByteNum - uiSendByteNum) > 20 * sizeof(GlobalPositionST))             /*保证肯定剩一包*/
    {
      arrFrameData.clear(); /*当前帧装订的路径数据*/
      usFrameNo++;          /*包序号自增*/
      if (0x1 == usFrameNo) /*包序号为1*/
      {
        ucFrameMultiFlag = 0x1; /*首包标识*/
      }
      else
      {
        ucFrameMultiFlag = 0x2; /*中间包*/
      }

      for (int i = (usFrameNo - 1) * 20; i < ((usFrameNo - 1) * 20 + 20); i++)
      {
        arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
      }

      FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
      uiSendByteNum += 20 * sizeof(GlobalPositionST);                                                                              /*已发送字节数+*/
    }

    arrFrameData.clear();                              /*当前帧装订的路径数据*/
    usFrameNo++;                                       /*包序号自增*/
    ucFrameMultiFlag = 0x3;                            /*结束包*/
    usFrameDataBytes = uiTotalByteNum - uiSendByteNum; /*本包装订字节数*/
    for (size_t i = (usFrameNo - 1) * 20; i < listPathPoint.size(); i++)
    {
      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
    }

    FoldAndSendPathBindMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
  }
}

void ParentDlg::FoldAndSendPathBindMsg(unsigned char ucPathType, unsigned char ucFrameMultiFlag, unsigned short int usFrameNum, unsigned int uiTotalByteNum, unsigned short int usFrameNo, unsigned short int usFrameDataBytes, std::vector<GlobalPositionST> arrFrameData)
{
  PNC2PadRouteAndLocalPathMsgST stMsg;
  memset(&stMsg, 0, sizeof(PNC2PadRouteAndLocalPathMsgST));

  if (0xFFFFFFFF <= m_uiSendMsgNo) /*发送报文序号*/
  {
    m_uiSendMsgNo = 1;
  }
  else
  {
    m_uiSendMsgNo++;
  }

  if (ucPathType == 0x01)
  {
    stMsg.stNetHeader.usMsgType = PNC2PAD_GLOBAL_UP_MSG;
  }
  else if (ucPathType == 0x02)
  {
    stMsg.stNetHeader.usMsgType = PNC2PAD_TRAJECTORY_UP_MSG;
  }

  stMsg.stNetHeader.uiSrcIP = inet_addr(FLAGS_PNC_IP.data());
  stMsg.stNetHeader.uiDestIP = inet_addr(FLAGS_PAD_IP_OWN.data());
  stMsg.stNetHeader.usMsgLen = sizeof(PNC2PadRouteAndLocalPathMsgST); /*报文长度*/
  stMsg.stNetHeader.uiMsgNo = m_uiSendMsgNo;
  stMsg.stNetHeader.usAck = NET_MSG_NOACK;

  stMsg.ucPathType = ucPathType;
  stMsg.ucMultiFrameFlag = ucFrameMultiFlag;
  stMsg.usFrameNum = usFrameNum;             /*数据包数*/
  stMsg.uiTotalByteNum = uiTotalByteNum;     /*装订数据总字节数*/
  stMsg.usFrameNo = usFrameNo;               /*数据包序号*/
  stMsg.usFrameDataBytes = usFrameDataBytes; /*当前包内装订数据总字节数*/
  for (size_t i = 0; i < arrFrameData.size(); i++)
  {
    memcpy(&(stMsg.stGlobalPostion[i]), &(arrFrameData.at(i)), sizeof(GlobalPositionST));
  }

  stMsg.stNetEnd.ucCheckSum = CheckSum((UINT8 *)(&stMsg), sizeof(PNC2PadRouteAndLocalPathMsgST));

  QByteArray datagram = QByteArray((const char *)&stMsg, stMsg.stNetHeader.usMsgLen);
  INT32 iLen = stMsg.stNetHeader.usMsgLen;

  emit SendNetMsg(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  // m_pNetSend->SendMsgListAdd(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  //  SendNetMsg((char *)&stMsg, stMsg.stNetHeader.usMsgLen, FLAGS_PAD_IP_GROUP, FLAGS_PAD_PORT); /*发送以太网报文*/
}

////////////////////////////////

void ParentDlg::SendRouteAndGlobalPathMsg(unsigned char ucPathType, std::vector<localPositionST> listPathPoint)
{
  unsigned char ucFrameMultiFlag;            /*多包标识*/
  unsigned short int usFrameNum;             /*多包包数*/
  unsigned int uiTotalByteNum;               /*总装订字节数*/
  unsigned short int usFrameNo;              /*本包序号*/
  unsigned short int usFrameDataBytes;       /*本包装订字节数*/
  std::vector<localPositionST> arrFrameData; /*当前帧装订的路径数据*/

  if (listPathPoint.size() <= 20) /*总路径点小于20个,则直接单帧发送*/
  {
    ucFrameMultiFlag = 0x0;                                            /*多包标识,单包*/
    usFrameNum = 0x1;                                                  /*多包包数*/
    uiTotalByteNum = listPathPoint.size() * sizeof(localPositionST);   /*总装订字节数*/
    usFrameNo = 0x0;                                                   /*本包序号*/
    usFrameDataBytes = listPathPoint.size() * sizeof(localPositionST); /*本包装订字节数*/
    for (size_t i = 0; i < listPathPoint.size(); i++)
    {
      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
    }

    FoldAndSendPathBindGlobalMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
  }
  else /*总路径点大于20个,则分包发送*/
  {
    uiTotalByteNum = listPathPoint.size() * sizeof(localPositionST);                     /*总装订字节数*/
    usFrameNum = listPathPoint.size() / 20 + ((listPathPoint.size() % 20 == 0) ? 0 : 1); /*数据包数*/
    usFrameNo = 0;                                                                       /*包序号*/
    usFrameDataBytes = 20 * sizeof(localPositionST);                                     /*本包装订字节数*/
    unsigned int uiSendByteNum = 0;                                                      /*已发送字节数*/
    while ((uiTotalByteNum - uiSendByteNum) > 20 * sizeof(localPositionST))              /*保证肯定剩一包*/
    {
      arrFrameData.clear(); /*当前帧装订的路径数据*/
      usFrameNo++;          /*包序号自增*/
      if (0x1 == usFrameNo) /*包序号为1*/
      {
        ucFrameMultiFlag = 0x1; /*首包标识*/
      }
      else
      {
        ucFrameMultiFlag = 0x2; /*中间包*/
      }

      for (int i = (usFrameNo - 1) * 20; i < ((usFrameNo - 1) * 20 + 20); i++)
      {
        arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
      }

      FoldAndSendPathBindGlobalMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
      uiSendByteNum += 20 * sizeof(localPositionST);                                                                                     /*已发送字节数+*/
    }

    arrFrameData.clear();                              /*当前帧装订的路径数据*/
    usFrameNo++;                                       /*包序号自增*/
    ucFrameMultiFlag = 0x3;                            /*结束包*/
    usFrameDataBytes = uiTotalByteNum - uiSendByteNum; /*本包装订字节数*/
    for (size_t i = (usFrameNo - 1) * 20; i < listPathPoint.size(); i++)
    {
      arrFrameData.push_back(listPathPoint.at(i)); /*当前帧装订的路径数据*/
    }

    FoldAndSendPathBindGlobalMsg(ucPathType, ucFrameMultiFlag, usFrameNum, uiTotalByteNum, usFrameNo, usFrameDataBytes, arrFrameData); /*打包并发送路径数据*/
  }
}

void ParentDlg::FoldAndSendPathBindGlobalMsg(unsigned char ucPathType, unsigned char ucFrameMultiFlag, unsigned short int usFrameNum, unsigned int uiTotalByteNum, unsigned short int usFrameNo, unsigned short int usFrameDataBytes, std::vector<localPositionST> arrFrameData)
{
  PNC2PadRouteAndGlobalPathMsgST stMsg;
  memset(&stMsg, 0, sizeof(PNC2PadRouteAndGlobalPathMsgST));

  if (0xFFFFFFFF <= m_uiSendMsgNo) /*发送报文序号*/
  {
    m_uiSendMsgNo = 1;
  }
  else
  {
    m_uiSendMsgNo++;
  }

  if (ucPathType == 0x01)
  {
    stMsg.stNetHeader.usMsgType = PNC2PAD_GLOBAL_UP_MSG;
  }
  else if (ucPathType == 0x02)
  {
    stMsg.stNetHeader.usMsgType = PNC2PAD_TRAJECTORY_UP_MSG;
  }

  stMsg.stNetHeader.uiSrcIP = inet_addr(FLAGS_PNC_IP.data());
  stMsg.stNetHeader.uiDestIP = inet_addr(FLAGS_PAD_IP_OWN.data());
  stMsg.stNetHeader.usMsgLen = sizeof(PNC2PadRouteAndGlobalPathMsgST); /*报文长度*/
  stMsg.stNetHeader.uiMsgNo = m_uiSendMsgNo;
  stMsg.stNetHeader.usAck = NET_MSG_NOACK;

  stMsg.ucPathType = ucPathType;
  stMsg.ucMultiFrameFlag = ucFrameMultiFlag;
  stMsg.usFrameNum = usFrameNum;             /*数据包数*/
  stMsg.uiTotalByteNum = uiTotalByteNum;     /*装订数据总字节数*/
  stMsg.usFrameNo = usFrameNo;               /*数据包序号*/
  stMsg.usFrameDataBytes = usFrameDataBytes; /*当前包内装订数据总字节数*/
  for (size_t i = 0; i < arrFrameData.size(); i++)
  {
    memcpy(&(stMsg.stGlobalPostion[i]), &(arrFrameData.at(i)), sizeof(localPositionST));
  }

  stMsg.stNetEnd.ucCheckSum = CheckSum((UINT8 *)(&stMsg), sizeof(PNC2PadRouteAndGlobalPathMsgST));

  QByteArray datagram = QByteArray((const char *)&stMsg, stMsg.stNetHeader.usMsgLen);
  INT32 iLen = stMsg.stNetHeader.usMsgLen;

  emit SendNetMsg(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  // m_pNetSend->SendMsgListAdd(datagram, iLen, QHostAddress(REMOTE_CONTROL_IP), TANK_TASKPOINTS_PORT);
  //   SendNetMsg((char *)&stMsg, stMsg.stNetHeader.usMsgLen, FLAGS_PAD_IP_GROUP, FLAGS_PAD_PORT); /*发送以太网报文*/
}

void ParentDlg::SendMoveCtrlMsg(FP2VCChasMoveCtrlMsgST stFP2VCMsg) /*发送运动控制报文*/
{
  //  if (0xFFFFFFFF > m_uiSendMoveCtrlMsgNum) /*发送运动控制报文计数*/
  //  {
  //    m_uiSendMoveCtrlMsgNum++;
  //  }
  //  else
  //  {
  //    m_uiSendMoveCtrlMsgNum = 0;
  //  }
  //  ui->labFP2VCMoveCtrlMsgNum->setText(QString("%1").arg(m_uiSendMoveCtrlMsgNum));

  //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_MOVECTRL_MSG; /*底盘运动控制报文*/
  //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCChasMoveCtrlMsgST);
  //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
  //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_ACK;

  //  QByteArray datagram;
  //  datagram.clear();
  //  datagram.append((const char*)&stFP2VCMsg, sizeof(FP2VCChasMoveCtrlMsgST));

  //  emit SendNetMsg(datagram, sizeof(FP2VCChasMoveCtrlMsgST), QHostAddress(VEHICLE_CTRL_IP),
  //                  VEHICLE_CTRL_PORT); /*发送以太网报文*/

  //  emit MsgTableDisp("命令发送", "底盘运动控制报文");
  //  emit WriteRecord("命令发送 无人驾驶台 综合控制设备 底盘运动控制报文", RECORD_PROCESS);
}

/******************************和校验算法******************************/
UINT8 ParentDlg::CheckSum(const UINT8 *pucData, INT32 iLen)
{
  UINT8 ucTemp = 0;
  for (int i = 0; i < (iLen - 1); i++)
  {
    ucTemp ^= (pucData[i]); /*全部字节异或*/
  }
  return ucTemp;
}

/******************************接收网络报文处理******************************/
void ParentDlg::RecvMsgProc(QByteArray datagram, INT32 iLen)
{
  // std::cout << "recv Msg proc " << std::endl;

  //  NetHeaderST stNetHeader; /*报文头*/
  //  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  //  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST)); /*获取报文头内容*/

  //  if (NET_MSG_ACK == stNetHeader.usAck) /*需要接收确认*/
  //  {
  //    SendAckMsg(stNetHeader.uiMsgNo);
  //  }

  //  UINT8 ucCheckSum = CheckSum((UINT8*)(datagram.data()), iLen); /*计算校验和*/
  //  if (ucCheckSum == (UINT8)datagram.at(iLen - 1))               /*校验和正确*/
  //  {
  //    switch (stNetHeader.usMsgType)
  //    {
  //      case VC2FP_HEART_MSG: /*心跳报文*/
  //      {
  //        RecvHeartMsgProc(datagram, iLen);
  //        break;
  //      }
  //      case VC2FP_ACK_MSG: /*接收确认报文*/
  //      {
  //        if (0xFFFFFFFF <= m_uiRecvAckMsgNum) /*接收报文数*/
  //        {
  //          m_uiRecvAckMsgNum = 1;
  //        }
  //        else
  //        {
  //          m_uiRecvAckMsgNum++;
  //        }
  //        ui->labVC2FPAckMsgNum->setText(QString("%1").arg(m_uiRecvAckMsgNum));
  //        emit AckMsg(datagram, iLen);
  //        break;
  //      }
  //      case VC2FP_ABNORMAL_MSG: /*异常报文*/
  //      {
  //        RecvAbNormalMsgProc(datagram, iLen);
  //        break;
  //      }
  //      case VC2FP_CANTRANS_MSG: /*CAN透传报文透传*/
  //      {
  //        RecvCanTransMsgProc(datagram, iLen);
  //        break;
  //      }
  //      case VC2FP_MOVEPATH_BIND_ACK_MSG: /*运动路径装订结果报文*/
  //      {
  //        RecvPathBindRsltMsgProc(datagram, iLen);
  //        break;
  //      }
  //      case VC2FP_COORDINATE_BIND_ACK_MSG: /*运动坐标系装订结果报文*/
  //      {
  //        RecvMapCordtBindRlstMsgProc(datagram, iLen);
  //        break;
  //      }
  //      default:
  //        break;
  //    }
  //  }
  //  else
  //  {
  //    QString strInfo = QString("帧号%1H报文校验和%2H错误，计算值%3H")
  //                          .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
  //                          .arg(datagram.at(iLen - 1), 2, 16, QChar('0'))
  //                          .arg(ucCheckSum, 2, 16, QChar('0'));
  //    m_pRecord->write(strInfo, RECORD_ERROR);

  //    MsgTableDisp("以太网报文校验和错误", strInfo);
  //  }
}

/******************************心跳报文处理******************************/
void ParentDlg::RecvHeartMsgProc(QByteArray datagram, INT32 iLen)
{
  //  if (sizeof(VC2FPHeartMsgST) == iLen)
  //  {
  //    if (0xFFFFFFFF > m_uiRecvHeartMsgNum) /*接收综控心跳报文计数*/
  //    {
  //      m_uiRecvHeartMsgNum++;
  //    }
  //    else
  //    {
  //      m_uiRecvHeartMsgNum = 0;
  //    }

  //    ui->labVC2FPHeartMsgNum->setText(QString("%1").arg(m_uiRecvHeartMsgNum));

  //    VC2FPHeartMsgST stVC2FPMsg;
  //    memcpy(&stVC2FPMsg, datagram.data(), sizeof(VC2FPHeartMsgST)); /*心跳报文内容赋值*/

  //    /*各分系统状态显示*/
  //    EqStateDispColor(ui->labVCState, stVC2FPMsg.stEqState.VC);
  //    EqStateDispColor(ui->labChasState, stVC2FPMsg.stEqState.Chassis);
  //    EqStateDispColor(ui->labMemsState, stVC2FPMsg.stEqState.Mems);
  //    //      EqStateDispColor(ui->labPercepState, FP_EQ_STATE_ON_OK); //
  //    感知设备显示设备状态为ok，因为感知设备已经拆除了
  //    //      qixianyu 20220210
  //    EqStateDispColor(ui->labPercepState,
  //                     stVC2FPMsg.stEqState.PercepEq);  //感知设备要装回去，所以恢复原状态 qixianyu 20220216

  //    ChasDataDisp(stVC2FPMsg.stChas2VCWorkData);                    /*底盘数据显示*/
  //    INSDataDisp(stVC2FPMsg.stINS2VCWorkData);                      /*INS数据显示*/
  //    m_fVehCourse = stVC2FPMsg.stINS2VCWorkData.uiVehCourse * 1e-3; /*车辆航向*/

  //    memcpy(&(m_dlgPosMeasure.m_stINS2VCWorkData), &(stVC2FPMsg.stINS2VCWorkData),
  //           sizeof(INS2VCWorkDataST)); /*传递到PosMeasureDlg中，以备使用*/

  //    ChasCmdDataDisp(stVC2FPMsg.stVC2ChasCmd); /*综控发向底盘的命令显示*/
  //    ui->labPercptMsgNum->setText(QString("%1").arg(stVC2FPMsg.uiPercptMsgNum));
  //    CameraDataDisp(stVC2FPMsg.stCameraData); /*摄像头信息显示*/
  //    // RadarDataDisp(stVC2FPMsg.stRadarData);//激光雷达信息显示，暂时没有此功能
  //    m_ucMoveCtrlFlag = stVC2FPMsg.ucMoveCtrlFlag; /*运动控制标识*/
  //    if (0x1 == stVC2FPMsg.ucMoveCtrlFlag)
  //    {
  //      ui->labChasMoveFlag->setText("航向控制");
  //    }
  //    else if (0x2 == stVC2FPMsg.ucMoveCtrlFlag)
  //    {
  //      ui->labChasMoveFlag->setText("路径控制");
  //    }
  //    else if (0x3 == stVC2FPMsg.ucMoveCtrlFlag)
  //    {
  //      ui->labChasMoveFlag->setText("遥控控制");
  //    }
  //    else if (0x4 == stVC2FPMsg.ucMoveCtrlFlag)
  //    {
  //      ui->labChasMoveFlag->setText("刹车控制");
  //    }
  //    else if (0x0 == stVC2FPMsg.ucMoveCtrlFlag)
  //    {
  //      ui->labChasMoveFlag->setText("无控制");
  //    }
  //    else
  //    {
  //      QString str = QString("0x%1").arg(stVC2FPMsg.ucMoveCtrlFlag, 2, 16, QChar('0'));
  //      ui->labChasMoveFlag->setText(str);
  //      ui->labChasMoveFlag->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0);
  //      color:white}");
  //    }

  //    /*车辆在本地东北天坐标系下的位置解算显示*/
  //    m_ucVehENUPosValid = stVC2FPMsg.ucVehPosValid; /*车辆位置是否有效*/
  //    if (0x55 == stVC2FPMsg.ucVehPosValid)
  //    {
  //      ui->labVehPosValid->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  //      ui->labVehPosValid->setText("有效");
  //    }
  //    else
  //    {
  //      ui->labVehPosValid->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0);
  //      color:white}"); ui->labVehPosValid->setText("无效");
  //    }

  //    memcpy(&m_stENUVehFrontWheel, &(stVC2FPMsg.stENUVehFrontWheel), sizeof(ENUCorST)); /*车辆前轮位置*/
  //    memcpy(&m_stENUVehCenterPos, &(stVC2FPMsg.stENUVehCenterPos), sizeof(ENUCorST));   /*车辆质心位置*/
  //    memcpy(&m_stENUVehBackWheel, &(stVC2FPMsg.stENUVehBackWheel), sizeof(ENUCorST));   /*车辆后轮位置*/
  //    if (stVC2FPMsg.iTrackPointIndex < m_listTrackPathBindRslt.size()) /*前置点序号必须在列表内*/
  //    {
  //      memcpy(&m_stENUPathPrependPt, &(m_listTrackPathBindRslt[stVC2FPMsg.iTrackPointIndex]), sizeof(ENUCorST));
  //    }

  //    /*反馈的车辆ENU位置每间隔2米，就记录到车辆轨迹列表中*/
  //    if (2 < sqrt(pow((m_stENUVehCenterPos.x - m_stLstRecordENUVehCenterPos.x), 2) +
  //                 pow((m_stENUVehCenterPos.y - m_stLstRecordENUVehCenterPos.y), 2))) /*车辆相对于之前走了2m*/
  //    {
  //      m_listVehPos.append(m_stENUVehCenterPos); /*存储车辆位置*/
  //      m_stLstRecordENUVehCenterPos = m_stENUVehCenterPos;
  //    }

  //    INT32 iTrackPointIndex = stVC2FPMsg.iTrackPointIndex;

  //    /* 计算最近点*/
  //    if (m_bMoveStart && m_bMoveStartFirstComputeCurrentPoint)
  //    {
  //      double fCurrentX = stVC2FPMsg.stENUVehCenterPos.x;
  //      double fCurrentY = stVC2FPMsg.stENUVehCenterPos.y;
  //      double fMinDist = 1e5;
  //      for (int i = 0; i <= iTrackPointIndex; ++i)
  //      {
  //        double fDX = fCurrentX - m_listTrackPathBindRslt.at(i).stENUPoint.x;
  //        double fDX2 = fDX * fDX;
  //        double fDY = fCurrentY - m_listTrackPathBindRslt.at(i).stENUPoint.y;
  //        double fDY2 = fDY * fDY;
  //        double fDist = sqrt(fDX2 + fDY2);
  //        if (fMinDist > fDist)
  //        {
  //          fMinDist = fDist;
  //          m_iCurrenPointIndex = i;
  //          m_iLastCurrentPointIndex = i;
  //        }
  //      }
  //      /*已经开始运动，初始化完成*/
  //      m_bMoveStartFirstComputeCurrentPoint = false;

  //      /*初始化时间*/
  //      m_usUTCYear = stVC2FPMsg.stINS2VCWorkData.usUTCYear;     /*UTC年*/
  //      m_ucUTCMonth = stVC2FPMsg.stINS2VCWorkData.ucUTCMonth;   /*UTC月*/
  //      m_ucUTCDay = stVC2FPMsg.stINS2VCWorkData.ucUTCDay;       /*UTC日*/
  //      m_ucUTCHour = stVC2FPMsg.stINS2VCWorkData.ucUTCHour;     /*UTC时*/
  //      m_ucUTCMinute = stVC2FPMsg.stINS2VCWorkData.ucUTCMinute; /*UTC分*/
  //      m_ucUTCSecInt = stVC2FPMsg.stINS2VCWorkData.ucUTCSecInt; /*UTC整秒*/
  //      m_ucUTCSecDOt = stVC2FPMsg.stINS2VCWorkData.ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/
  //      QDate qDate;
  //      qDate.setDate(m_usUTCYear, m_ucUTCMonth, m_ucUTCDay);
  //      QTime qTime;
  //      qTime.setHMS(m_ucUTCHour, m_ucUTCMinute, m_ucUTCSecInt, m_ucUTCSecDOt * 10); /*乘以10的目的是变为毫秒*/
  //      QDateTime qDateTime;
  //      qDateTime.setDate(qDate);
  //      qDateTime.setTime(qTime);
  //      m_InitQDateTimeSinceMSecond = qDateTime.currentMSecsSinceEpoch();
  //      m_sEllipseTime = "0.0 s";
  //    }
  //    else if (m_bMoveStart && (false == m_bMoveStartFirstComputeCurrentPoint))
  //    {
  //      double fCurrentX = stVC2FPMsg.stENUVehCenterPos.x;
  //      double fCurrentY = stVC2FPMsg.stENUVehCenterPos.y;
  //      double fMinDist = 1e5;
  //      int iIndex = ((m_iLastCurrentPointIndex - 15) >= 0) ? (m_iLastCurrentPointIndex - 15) : 0;
  //      for (int i = iIndex; i <= iTrackPointIndex; ++i)
  //      {
  //        double fDX = fCurrentX - m_listTrackPathBindRslt.at(i).stENUPoint.x;
  //        double fDX2 = fDX * fDX;
  //        double fDY = fCurrentY - m_listTrackPathBindRslt.at(i).stENUPoint.y;
  //        double fDY2 = fDY * fDY;
  //        double fDist = sqrt(fDX2 + fDY2);
  //        if (fMinDist > fDist)
  //        {
  //          fMinDist = fDist;
  //          m_iCurrenPointIndex = i;
  //        }
  //      }
  //      qDebug() << "当前最近点距离: " << fMinDist << " 最近点序号: " << m_iCurrenPointIndex;

  //      /*初始化时间*/
  //      m_usUTCYear = stVC2FPMsg.stINS2VCWorkData.usUTCYear;     /*UTC年*/
  //      m_ucUTCMonth = stVC2FPMsg.stINS2VCWorkData.ucUTCMonth;   /*UTC月*/
  //      m_ucUTCDay = stVC2FPMsg.stINS2VCWorkData.ucUTCDay;       /*UTC日*/
  //      m_ucUTCHour = stVC2FPMsg.stINS2VCWorkData.ucUTCHour;     /*UTC时*/
  //      m_ucUTCMinute = stVC2FPMsg.stINS2VCWorkData.ucUTCMinute; /*UTC分*/
  //      m_ucUTCSecInt = stVC2FPMsg.stINS2VCWorkData.ucUTCSecInt; /*UTC整秒*/
  //      m_ucUTCSecDOt = stVC2FPMsg.stINS2VCWorkData.ucUTCSecDOt; /*UTC秒小数部分，1LSB = 0.01s*/
  //      QDate qDate;
  //      qDate.setDate(m_usUTCYear, m_ucUTCMonth, m_ucUTCDay);
  //      QTime qTime;
  //      qTime.setHMS(m_ucUTCHour, m_ucUTCMinute, m_ucUTCSecInt, m_ucUTCSecDOt * 10); /*乘以10的目的是变为毫秒*/
  //      QDateTime qDateTime;
  //      qDateTime.setDate(qDate);
  //      qDateTime.setTime(qTime);
  //      qint64 iEllipseTime = qDateTime.currentMSecsSinceEpoch() - m_InitQDateTimeSinceMSecond;
  //      m_sEllipseTime = QString::number(iEllipseTime / 1000.0) + " s";

  //      //由于显示的时间不能实时更新，修改了列表更新显示逻辑，待测试：20220209
  //      //            QTableWidgetItem *newItem = new QTabl
  //      eWidgetItem(tr("%1").arg(iEllipseTime/1000.0));//列表中单位为s
  //      ui->tableTrackPathInfo->item(m_iCurrenPointIndex, 3)
  //          ->setText(tr("%1").arg(iEllipseTime / 1000.0));  //列表中单位为s
  //      //            ui->tableTrackPathInfo->setItem(m_iCurrenPointIndex, 3, newItem);
  //      //            ui->tableTrackPathInfo->update(); //添加更新函数，待确认是否能实时更新列表中的时间
  //      m_iLastCurrentPointIndex = m_iCurrenPointIndex;

  //      if (iEllipseTime >
  //          10000) /*启动到停止的时间需要大于10s，防止提前点了运动开始但是车还没有运动导致的速度为0,qixianyu*/
  //      {
  //        if (0 == stVC2FPMsg.stChas2VCWorkData.usVehSpeed) /*速度为0认为运动停止*/
  //        {
  //          m_bMoveStart = false;
  //        }
  //      }
  //    }
  //    else
  //    {
  //      //不处理
  //    }

  //    if (m_bMoveStart)  // 添加判断，在运动开始后才会自动跟踪列表中路径选择的点 20220117 qixianyu
  //    {
  //      /*车辆在本地东北天坐标系下要跟踪的前置点显示*/
  //      ui->tableTrackPathInfo->selectRow(stVC2FPMsg.iTrackPointIndex); /*显示跟踪点的行数*/
  //      ui->tablePathBindRsltInfo->selectRow(stVC2FPMsg.iTrackPointIndex);
  //    }

  //    ui->labGridMap->update();                                               /*重绘车辆轨迹和前置跟踪点*/
  //    ui->labCANSendChnl->setText(QString::number(stVC2FPMsg.ucCanSendChnl)); /*CAN默认发送通道*/

  //    if (0x55 == stVC2FPMsg.ucReadConfigFileRslt) /*读取文件成功*/
  //    {
  //      ui->labVMCtrlReadFileRslt->setText("成功");
  //      ui->labVMCtrlReadFileRslt->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green;
  //      color:white}");
  //    }
  //    else if (0xAA == stVC2FPMsg.ucReadConfigFileRslt) /*读取文件失败*/
  //    {
  //      ui->labVMCtrlReadFileRslt->setText("失败");
  //      ui->labVMCtrlReadFileRslt->setStyleSheet(
  //          "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  //    }
  //    else
  //    {
  //      QString str = QString("0x%1").arg(stVC2FPMsg.ucReadConfigFileRslt, 2, 16, QChar('0'));
  //      ui->labVMCtrlReadFileRslt->setText(str);
  //      ui->labVMCtrlReadFileRslt->setStyleSheet(
  //          "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  //    }

  //    if (1 == stVC2FPMsg.ucVCSimuFlag) /*仿真状态*/
  //    {
  //      ui->labVMCtrlSimu->setText("仿真");
  //      ui->labVMCtrlSimu->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  //    }
  //    else
  //    {
  //      ui->labVMCtrlSimu->setText("实车");
  //      ui->labVMCtrlSimu->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0);
  //      color:white}");
  //    }
  //  }
  //  else
  //  {
  //    RecvLenErrorProc(datagram, iLen, sizeof(VC2FPHeartMsgST)); /*接收报文长度错误处理*/
  //  }
}

void onLightHornWiperTopicCallback(ChassisLightHornWiper state)
{
}

void ParentDlg::RecvPathBindRsltMsgProc(QByteArray datagram, INT32 iLen) /*接收路径装订结果报文处理*/
{
  //  if (0xFFFFFFFF <= m_uiRecvPathBindRsltMsgNum) /*接收报文数*/
  //  {
  //    m_uiRecvPathBindRsltMsgNum = 1;
  //  }
  //  else
  //  {
  //    m_uiRecvPathBindRsltMsgNum++;
  //  }
  //  ui->labVC2FPPathBindRsltMsgNum->setText(QString("%1").arg(m_uiRecvPathBindRsltMsgNum));

  //  VC2FPPathBindRsltMsgST stVC2FPMsg;
  //  memset(&stVC2FPMsg, 0, sizeof(VC2FPPathBindRsltMsgST));
  //  memcpy(&stVC2FPMsg, datagram.data(), sizeof(VC2FPPathBindRsltMsgST));

  //  if (0x1 == stVC2FPMsg.ucFrameMultiFlag) /*多包的首包*/
  //  {
  //    m_listTrackPathBindRslt.clear(); /*清空综控中跟踪路径点数组，重新存储*/

  //    /*首包中描述的共有几包数和装订路径点总字节数*/
  //    m_uiTrackPathFrameNum = stVC2FPMsg.usFrameNum;       /*数据总包数*/
  //    m_uiTrackPathTotalBytes = stVC2FPMsg.uiTotalByteNum; /*路径点总字节*/

  //    /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
  //    m_uiRecvTrackPathFrameCnt = 1; /*自计接收到的要跟踪的路径数据点的数据帧号*/
  //    m_uiRecvTrackPathTotalBytesCnt = stVC2FPMsg.usFrameDataBytes;
  //    /*自计接收到的要跟踪的路径数据点的数据总字节数*/

  //    for (INT32 i = 0; i < (stVC2FPMsg.usFrameDataBytes / sizeof(SmoothPathPointST));
  //         i++) /*将当前包内的数据转存到list中*/
  //    {
  //      m_listTrackPathBindRslt.append(stVC2FPMsg.stPathData[i]);
  //    }
  //  }
  //  else if (0x2 == stVC2FPMsg.ucFrameMultiFlag) /*多包的中间包*/
  //  {
  //    /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
  //    m_uiRecvTrackPathFrameCnt++; /*自计接收到的要跟踪的路径数据点的数据包数*/
  //    m_uiRecvTrackPathTotalBytesCnt += stVC2FPMsg.usFrameDataBytes;
  //    /*自计接收到的要跟踪的路径数据点的数据总字节数*/

  //    for (INT32 i = 0; i < (stVC2FPMsg.usFrameDataBytes / sizeof(SmoothPathPointST));
  //         i++) /*将当前包内的数据转存到list中*/
  //    {
  //      m_listTrackPathBindRslt.append(stVC2FPMsg.stPathData[i]);
  //    }
  //  }
  //  else if (0x3 == stVC2FPMsg.ucFrameMultiFlag) /*多包的结束包*/
  //  {
  //    /*自计已接收到的包数和已接收到的路径点字节数，用于与首包描述对比，不一致则进行提示*/
  //    m_uiRecvTrackPathFrameCnt++; /*自计接收到的要跟踪的路径数据点的数据帧号*/
  //    m_uiRecvTrackPathTotalBytesCnt += stVC2FPMsg.usFrameDataBytes;
  //    /*自计接收到的要跟踪的路径数据点的数据总字节数*/

  //    if (m_uiRecvTrackPathFrameCnt != m_uiTrackPathFrameNum) /*自计包数不等于发送的总包数*/
  //    {
  //      emit WriteRecord("接收路径装订结果报文包数错误", RECORD_ERROR);
  //      emit MsgTableDisp("路径装订", "接收路径装订结果包数错误");
  //    }
  //    else if (m_uiRecvTrackPathTotalBytesCnt != m_uiTrackPathTotalBytes)
  //    /*自计路径点总字节数不等于发送的总字节数*/
  //    {
  //      emit WriteRecord("接收路径装订结果报文路径点总字节数错误", RECORD_ERROR);
  //      emit MsgTableDisp("路径装订", "接收路径装订结果报文路径点总字节数错误");
  //    }
  //    else /*接收路径装订结果报文正确*/
  //    {
  //      for (INT32 i = 0; i < (stVC2FPMsg.usFrameDataBytes / sizeof(SmoothPathPointST));
  //           i++) /*将当前包内的数据转存到list中*/
  //      {
  //        m_listTrackPathBindRslt.append(stVC2FPMsg.stPathData[i]);
  //      }

  //      BindRsltValidJudge(); /*路径装订结果是否有效标志*/

  //      emit MsgTableDisp("路径装订", "成功接收路径装订结果报文");
  //      PathBindRsltDisp(); /*路径装订结果信息显示*/
  //    }
  //  }
  //  else if (0x0 == stVC2FPMsg.ucFrameMultiFlag) /*单包*/
  //  {
  //    m_listTrackPathBindRslt.clear(); /*清空数组，重新存储*/
  //    for (INT32 i = 0; i < (stVC2FPMsg.usFrameDataBytes / sizeof(SmoothPathPointST));
  //         i++) /*将当前包内的数据转存到list中*/
  //    {
  //      m_listTrackPathBindRslt.append(stVC2FPMsg.stPathData[i]);
  //    }
  //    BindRsltValidJudge(); /*路径装订结果是否有效标志*/
  //    emit MsgTableDisp("路径装订", "成功接收路径装订结果报文");
  //    PathBindRsltDisp(); /*路径装订结果信息显示*/
  //  }
  //  else
  //  {
  //    /*无效路径点装订报文*/
  //  }
}

void ParentDlg::BindRsltValidJudge() /*路径装订结果是否有效标志*/
{
  if (0 < m_listTrackPathCmd.size()) /*装订过路径点*/
  {
    INT32 iPathBindErrNum = 0;
    for (INT32 i = 0; i < m_listTrackPathBindRslt.count(); i++)
    {
      if ((fabs(m_listTrackPathBindRslt.at(i).stENUPoint.x - m_listTrackPathCmd.at(i).stENUPoint.x) > 1e-3) ||
          (fabs(m_listTrackPathBindRslt.at(i).stENUPoint.y - m_listTrackPathCmd.at(i).stENUPoint.y) >
           1e-3)) /*判断装订反馈结果与装订路径的x和y是否相等*/
      {
        iPathBindErrNum++; /*路径点装订错误数*/
      }
    }
    if (0 == iPathBindErrNum)
    {
      m_bTrackPathBindRsltValid = true; /*装订过路径点，那么接收装订结果和装订路径点都相等，即认为装订结果有效*/
    }
  }
  else
  {
    m_bTrackPathBindRsltValid = true; /*没有装订过路径点，仅仅是查询，那么接收成功，即认为装订结果有效*/
  }
}

void ParentDlg::RecvMapCordtBindRlstMsgProc(QByteArray datagram, INT32 iLen) /*地图坐标系装订结果报文处理*/
{
  if (sizeof(VC2FPCorditBindRsltMsgST) == iLen)
  {
    if (0xFFFFFFFF <= m_uiRecvMapCordtBindRlstMsgNum) /*接收报文数*/
    {
      m_uiRecvMapCordtBindRlstMsgNum = 1;
    }
    else
    {
      m_uiRecvMapCordtBindRlstMsgNum++;
    }
    // ui->labVC2FPCordinateBindRsltMsgNum->setText(QString("%1").arg(m_uiRecvMapCordtBindRlstMsgNum));

    VC2FPCorditBindRsltMsgST stVC2FPMsg;
    memset(&stVC2FPMsg, 0, sizeof(VC2FPCorditBindRsltMsgST));
    memcpy(&stVC2FPMsg, datagram.data(), sizeof(VC2FPCorditBindRsltMsgST));

    MapCorditBindRsltDisp(stVC2FPMsg.stMapCordit.stLLHOriginPos, stVC2FPMsg.stMapCordit.stLLHEndPos);
  }
  else
  {
    RecvLenErrorProc(datagram, iLen, sizeof(VC2FPCorditBindRsltMsgST)); /*接收报文长度错误处理*/
  }
}

/******************************异常报文处理******************************/
void ParentDlg::RecvAbNormalMsgProc(QByteArray datagram, INT32 iLen)
{
  if (sizeof(VC2FPAnomalyMsgST) == iLen)
  {
    if (0xFFFFFFFF <= m_uiRecvAbNormlaMsgNum) /*接收报文数*/
    {
      m_uiRecvAbNormlaMsgNum = 1;
    }
    else
    {
      m_uiRecvAbNormlaMsgNum++;
    }
    // ui->labVC2FPAbNormalMsgNum->setText(QString("%1").arg(m_uiRecvAbNormlaMsgNum));

    UINT8 ucDispType = DISP_WARNING; /*显示类型*/
    VC2FPAnomalyMsgST stVC2FPMsg;
    memcpy(&stVC2FPMsg, datagram.data(), sizeof(VC2FPAnomalyMsgST));
    if (0x0406 == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*CAN连续50帧发送失败*/
    {
      m_strErrorInfo = "CAN连续发送失败"; /*异常报警信息*/
      ucDispType = DISP_WARNING;          /*显示类型*/
    }
    else if (0x0501 == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*接收底盘CAN报文异常上报处理*/
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘CAN帧号不连续"; /*异常报警信息*/
      }
      else if (0x2 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘CAN帧号错误"; /*异常报警信息*/
      }
      else if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘CAN校验错误"; /*异常报警信息*/
      }
      else
      {
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else if (0x0503 == stVC2FPMsg.stAnomalyData.usCtrlFrameType)
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令应答帧号错误"; /*异常报警信息*/
      }
      else if (0x2 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令应答超时"; /*异常报警信息*/
      }
      else if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令执行失败"; /*异常报警信息*/
      }
      else
      {
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else if (0x0504 == stVC2FPMsg.stAnomalyData.usCtrlFrameType)
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令应答帧号错误"; /*异常报警信息*/
      }
      else if (0x2 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令执行超时"; /*异常报警信息*/
      }
      else if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘命令执行失败"; /*异常报警信息*/
      }
      else
      {
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else if (0x0507 == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*底盘命令控制限制*/
    {
      if (0x0701 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘驻车制动下不支持"; /*异常报警信息*/
      }
      else if (0x0702 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘紧急制动下不支持"; /*异常报警信息*/
      }
      else if (0x0703 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘已控制了急停不支持"; /*异常报警信息*/
      }
      else if (0x0704 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘设备状态非正常不支持"; /*异常报警信息*/
      }
      else if (0x0705 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "底盘人控模式下不支持"; /*异常报警信息*/
      }

      switch (stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
      case CMD_CHAS_SELF_CHECK:
      {
        m_strErrorInfo += "自检";
        break;
      }
      case CMD_CHAS_GEAR:
      {
        m_strErrorInfo += "档位设置";
        break;
      }
      case CMD_CHAS_MOVE:
      {
        m_strErrorInfo += "运动控制";
        break;
      }
      case CMD_CHAS_VEH_BASIC_CTRL:
      {
        m_strErrorInfo += "车灯笛控制";
        break;
      }
      case CMD_CHAS_WORK_MODE_SET:
      {
        m_strErrorInfo += "被控模式设置";
        break;
      }
      case CMD_CHAS_PARKING_CTRL:
      {
        m_strErrorInfo += "驻车控制";
        break;
      }
      case CMD_CHAS_BRAKING_CTRL:
      {
        m_strErrorInfo += "紧急制动控制";
        break;
      }
      case CMD_CHAS_CLEAR_BRAK_CTRL:
      {
        m_strErrorInfo += "解除紧急制动";
        break;
      }
      case CMD_CHAS_ENGINE_START_CTRL:
      {
        m_strErrorInfo += "发动机启动";
        break;
      }
      case CMD_CHAS_ENGINE_END_CTRL:
      {
        m_strErrorInfo += "发动机停止";
        break;
      }
      default:
      {
        break;
      }
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else if (0x0508 == stVC2FPMsg.stAnomalyData.usCtrlFrameType)
    {
      m_strErrorInfo = "底盘通信中断"; /*异常报警信息*/
      ucDispType = DISP_WARNING;       /*显示类型*/
    }
    else if (FP2VC_MOVECTRL_MSG == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*运动控制*/
    {
      if (0x0255 == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
        {
          m_strErrorInfo = "未设定地图原点和路径终点，不能开启路径跟踪控制"; /*异常报警信息*/
        }
      }

      ucDispType = DISP_ERROR; /*显示类型*/
    }
    else if (FP2VC_CHASEQCTRL_MSG == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*底盘控制*/
    {
      if (CMD_CHAS_GEAR == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        m_strErrorInfo = "档位设置"; /*异常报警信息*/
      }
      else if (CMD_CHAS_VEH_BASIC_CTRL == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        m_strErrorInfo = "车附属件控制"; /*异常报警信息*/
      }
      else if (CMD_CHAS_WORK_MODE_SET == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        m_strErrorInfo = "被控模式设置"; /*异常报警信息*/
      }
      else if (CMD_CHAS_PARKING_CTRL == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        m_strErrorInfo = "驻车控制"; /*异常报警信息*/
      }

      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo += "命令参数不正确";
      }

      if (0xFFFF == stVC2FPMsg.stAnomalyData.usCtrlCmd)
      {
        m_strErrorInfo = "底盘控制命令无效"; /*异常报警信息*/
      }
      ucDispType = DISP_ERROR; /*显示类型*/
    }
    else if (FP2VC_MOVEPATH_BIND_MSG == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*路径装订*/
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "接收到的路径包数与首包标记不一致"; /*异常报警信息*/
      }
      else if (0x2 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "接收到的路径字节数与首包标记不一致"; /*异常报警信息*/
      }
      else if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "路径装订报文多包标识错误"; /*异常报警信息*/
      }
      ucDispType = DISP_ERROR; /*显示类型*/
    }
    else if (FP2VC_MOVEPATH_ASK_MSG == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*路径查询*/
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "综控中无有效路径"; /*异常报警信息*/
      }
      ucDispType = DISP_ERROR; /*显示类型*/
    }
    else if (0x0602 == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*无人架控台确认报文超时*/
    {
      if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "无人驾控台报文校验和错误"; /*异常报警信息*/
      }
      else if (0x4 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "无人驾控台报文长度错误"; /*异常报警信息*/
      }
      else
      {
        /*暂时保留*/
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else if (0x0701 == stVC2FPMsg.stAnomalyData.usCtrlFrameType) /*Mems异常*/
    {
      if (0x1 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "Mems经纬度报文断链"; /*异常报警信息*/
      }
      else if (0x2 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "Mems高度报文断链"; /*异常报警信息*/
      }
      else if (0x3 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "Mems姿态报文断链"; /*异常报警信息*/
      }
      else if (0x4 == stVC2FPMsg.stAnomalyData.usAnomalCode)
      {
        m_strErrorInfo = "Mems数据失去信任"; /*异常报警信息*/
      }
      else
      {
        /*暂时保留*/
      }
      ucDispType = DISP_WARNING; /*显示类型*/
    }
    else
    {
      /*暂无处理*/
    }
    // AnomalyInfoDisp(m_strErrorInfo, ucDispType);
    // MsgTableDisp("错误", m_strErrorInfo, ucDispType);
    //    m_pRecord->write(m_strErrorInfo, RECORD_ERROR);
  }
  else
  {
    RecvLenErrorProc(datagram, iLen, sizeof(VC2FPAnomalyMsgST)); /*接收报文长度错误处理*/
  }
}

void ParentDlg::RecvCanTransMsgProc(QByteArray datagram, INT32 iLen) /*CAN透传报文处理*/
{
  if (sizeof(VC2FPCanTransMsgST) == iLen)
  {
    VC2FPCanTransMsgST stVC2FPMsg;
    memcpy(&stVC2FPMsg, datagram.data(), sizeof(VC2FPCanTransMsgST));

    switch (stVC2FPMsg.stCanFrame.unCANID.stCanID.P3)
    {
    case CANID_P_CMDACK: /*命令应答报文*/
    {
      RecvCmdAckMsgProc(stVC2FPMsg); /*命令应答CAN报文处理*/
      break;
    }
    case CANID_P_STATE: /*状态报文*/
    {
      RecvStateMsgProc(stVC2FPMsg); /*状态CAN报文处理*/
      break;
    }
    default:
      break;
    }
  }
  else
  {
    RecvLenErrorProc(datagram, iLen, sizeof(VC2FPCanTransMsgST)); /*接收报文长度错误处理*/
  }
}

void ParentDlg::RecvCmdAckMsgProc(VC2FPCanTransMsgST stCanMsg) /*命令应答报文处理*/
{
  if (0xFFFFFFFF <= m_uiRecvCmdAckMsgNum) /*接收报文数*/
  {
    m_uiRecvCmdAckMsgNum = 1;
  }
  else
  {
    m_uiRecvCmdAckMsgNum++;
  }
  // ui->labVC2FPCmdAckMsgNum->setText(QString("%1").arg(m_uiRecvCmdAckMsgNum));

  UINT8 ucSrcID =
      (((stCanMsg.stCanFrame.unCANID.stCanID.SA_H2 << 6) | (stCanMsg.stCanFrame.unCANID.stCanID.SA_L6)) & 0xFF);
  UINT8 ucDestID =
      (((stCanMsg.stCanFrame.unCANID.stCanID.DA_H2 << 6) | (stCanMsg.stCanFrame.unCANID.stCanID.DA_L6)) & 0xFF);

  switch (ucSrcID)
  {
  case CANID_ADDR_CHASSIS: /*底盘设备*/
  {
    RecvChasCanCmdAckMsgProc(stCanMsg);
    break;
  }
  default:
    break;
  }
}

void ParentDlg::RecvStateMsgProc(VC2FPCanTransMsgST stCanMsg) /*状态报文处理*/
{
  if (0xFFFFFFFF <= m_uiRecvStateMsgNum) /*接收报文数*/
  {
    m_uiRecvStateMsgNum = 1;
  }
  else
  {
    m_uiRecvStateMsgNum++;
  }
  // ui->labVC2FPStateMsgNum->setText(QString("%1").arg(m_uiRecvStateMsgNum));

  UINT8 ucSrcID =
      (((stCanMsg.stCanFrame.unCANID.stCanID.SA_H2 << 6) | (stCanMsg.stCanFrame.unCANID.stCanID.SA_L6)) & 0xFF);
  UINT8 ucDestID =
      (((stCanMsg.stCanFrame.unCANID.stCanID.DA_H2 << 6) | (stCanMsg.stCanFrame.unCANID.stCanID.DA_L6)) & 0xFF);

  switch (ucSrcID)
  {
  case CANID_ADDR_CHASSIS: /*底盘设备*/
  {
    RecvChasCanStateMsgProc(stCanMsg);
    break;
  }
  default:
    break;
  }
}

void ParentDlg::RecvChasCanCmdAckMsgProc(VC2FPCanTransMsgST stCanMsg) /*接收底盘CAN命令应答报文处理*/
{
  Chas2VCCmdAckMsgST stChas2VCMsg;
  memset(&stChas2VCMsg, 0, sizeof(Chas2VCCmdAckMsgST));
  memcpy(&stChas2VCMsg, stCanMsg.stCanFrame.data, 8);

  QString strTime = QString("%1:%2:%3.%4 ")
                        .arg(stCanMsg.ucHour, 2, 10, QChar('0'))
                        .arg(stCanMsg.ucMinute, 2, 10, QChar('0'))
                        .arg(stCanMsg.ucSecond, 2, 10, QChar('0'))
                        .arg(stCanMsg.usMSecond, 3, 10, QChar('0')); /*命令信息*/

  QString strCmdAckText = strTime; /*命令应答内容解析*/

  QString strCmdType;

  switch (stChas2VCMsg.ucCmd)
  {
  case CMD_CHAS_SELF_CHECK: /*自检应答*/
  {
    strCmdType = "自检";
    break;
  }
  case CMD_CHAS_GEAR: /*档位设置应答*/
  {
    strCmdType = "档位设置";
    VC2ChasGearMsgST stVC2ChasMsg;
    memcpy(&stVC2ChasMsg, &stChas2VCMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stVC2ChasMsg.ucGear)
    {
    case 7:
    {
      strCmdAckText += "D档";
      break;
    }
    case 8:
    {
      strCmdAckText += "R档";
      break;
    }
    case 9:
    {
      strCmdAckText += "N档";
      break;
    }
    default:
    {
      strCmdAckText += QString("车辆档位:0x%1").arg(stVC2ChasMsg.ucGear, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_VEH_BASIC_CTRL:
  {
    strCmdType = "车灯笛控制";
    VC2ChasVehBasicMsgST stVC2ChasMsg;
    memcpy(&stVC2ChasMsg, &stChas2VCMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stVC2ChasMsg.ucLight)
    {
    case 0x1:
    {
      strCmdAckText += "近光灯开";
      break;
    }
    case 0x2:
    {
      strCmdAckText += "近光灯关";
      break;
    }
    case 0x3:
    {
      strCmdAckText += "远光灯开";
      break;
    }
    case 0x4:
    {
      strCmdAckText += "远光灯关";
      break;
    }
    default:
    {
      strCmdAckText += QString("0x%1").arg(stVC2ChasMsg.ucLight, 2, 16, QChar('0'));
      break;
    }
    }

    switch (stVC2ChasMsg.ucHorn)
    {
    case 0x1:
    {
      strCmdAckText += "车笛开";
      break;
    }
    case 0x2:
    {
      strCmdAckText += "车笛关";
      break;
    }
    default:
    {
      strCmdAckText += QString("0x%1").arg(stVC2ChasMsg.ucHorn, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_WORK_MODE_SET:
  {
    strCmdType = "被控模式设置";
    VC2ChasWorkModeMsgST stVC2ChasMsg;
    memcpy(&stVC2ChasMsg, &stChas2VCMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stVC2ChasMsg.ucWorkMode)
    {
    case 0x1:
    {
      strCmdAckText += "人控模式";
      break;
    }
    case 0x2:
    {
      strCmdAckText += "兼容模式";
      break;
    }
    default:
    {
      strCmdAckText += QString("被控模式:0x%1").arg(stVC2ChasMsg.ucWorkMode, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_PARKING_CTRL:
  {
    strCmdType = "驻车控制";
    VC2ChasParkBrakMsgST stVC2ChasMsg;
    memcpy(&stVC2ChasMsg, &stChas2VCMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stVC2ChasMsg.ucParkFlag)
    {
    case 0x55:
    {
      strCmdAckText += "标识:驻车制动";
      break;
    }
    case 0xAA:
    {
      strCmdAckText += "标识:解除驻车";
      break;
    }
    default:
    {
      strCmdAckText += QString("驻车控制标识:0x%1").arg(stVC2ChasMsg.ucParkFlag, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_BRAKING_CTRL:
  {
    strCmdType = "紧急制动";
    break;
  }
  case CMD_CHAS_ENGINE_START_CTRL:
  {
    strCmdType = "发动机启动";
    break;
  }
  case CMD_CHAS_ENGINE_END_CTRL:
  {
    strCmdType = "发动机停止";
    break;
  }
  default:
    break;
  }
  // ui->labChasCmdAckType->setText(strCmdType);
  // ui->labChasCmdAckInfo->setText(strCmdAckText);
  // MsgTableDisp("命令应答", (strCmdType + strCmdAckText));
}

void ParentDlg::RecvChasCanStateMsgProc(VC2FPCanTransMsgST stCanMsg) /*接收底盘CAN状态报文处理*/
{
  Chas2VCCmdStateMsgST stChas2VCStateMsg;
  memset(&stChas2VCStateMsg, 0, sizeof(Chas2VCCmdStateMsgST));
  memcpy(&stChas2VCStateMsg, stCanMsg.stCanFrame.data, 8);

  QString strTime = QString("%1:%2:%3.%4 ")
                        .arg(stCanMsg.ucHour, 2, 10, QChar('0'))
                        .arg(stCanMsg.ucMinute, 2, 10, QChar('0'))
                        .arg(stCanMsg.ucSecond, 2, 10, QChar('0'))
                        .arg(stCanMsg.usMSecond, 3, 10, QChar('0')); /*命令信息*/
  QString strCmdStateText = strTime;                                 /*命令应答内容解析*/

  QString strCmdType;

  switch (stChas2VCStateMsg.ucCmd)
  {
  case CMD_CHAS_SELF_CHECK: /*自检应答*/
  {
    strCmdType = "自检";
    Chas2VCSelfCheckStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8);
    switch (stChas2VCMsg.ucState)
    {
    case STATE_OK:
    {
      strCmdStateText = strCmdStateText + "综合状态:正常" + " ";
      break;
    }
    case STATE_ERROR:
    {
      strCmdStateText = strCmdStateText + "综合状态:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText =
          strCmdStateText + QString("综合状态:0x%1").arg(stChas2VCMsg.ucState, 2, 16, QChar('0')) + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.EMS)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "EMS:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "EMS:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "EMS:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "EMS:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.EBS)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "EBS:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "EBS:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "EBS:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "EBS:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.EPB)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "EPB:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "EPB:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "EPB:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "EPB:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.EPS)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "EPS:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "EPS:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "EPS:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "EPS:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.TCU)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "TCU:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "TCU:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "TCU:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "TCU:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.Ctrler)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "Ctrler:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "Ctrler:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "Ctrler:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "Ctrler:未知" + " ";
      break;
    }
    }
    switch (stChas2VCMsg.stState.BCM)
    {
    case CAN_EQ_STATE_OFF:
    {
      strCmdStateText = strCmdStateText + "BCM:断电" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_OK:
    {
      strCmdStateText = strCmdStateText + "BCM:正常" + " ";
      break;
    }
    case CAN_EQ_STATE_ON_ERROR:
    {
      strCmdStateText = strCmdStateText + "BCM:故障" + " ";
      break;
    }
    default:
    {
      strCmdStateText = strCmdStateText + "BCM:未知" + " ";
      break;
    }
    }
    break;
  }
  case CMD_CHAS_GEAR: /*档位设置应答*/
  {
    strCmdType = "档位设置";
    Chas2VCGearStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stChas2VCMsg.ucGear)
    {
    case 7:
    {
      strCmdStateText += "D档";
      break;
    }
    case 8:
    {
      strCmdStateText += "R档";
      break;
    }
    case 9:
    {
      strCmdStateText += "N档";
      break;
    }
    default:
    {
      strCmdStateText += QString("车辆档位:0x%1").arg(stChas2VCMsg.ucGear, 2, 16, QChar('0'));
      break;
    }
    }
    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_VEH_BASIC_CTRL:
  {
    strCmdType = "车灯笛控制";
    Chas2VCVehBasicStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stChas2VCMsg.ucLight)
    {
    case 0x1:
    {
      strCmdStateText = strCmdStateText + "近光灯开" + " ";
      break;
    }
    case 0x2:
    {
      strCmdStateText = strCmdStateText + "近光灯关" + " ";
      break;
    }
    case 0x3:
    {
      strCmdStateText = strCmdStateText + "远光灯开" + " ";
      break;
    }
    case 0x4:
    {
      strCmdStateText = strCmdStateText + "远光灯关" + " ";
      break;
    }
    default:
    {
      strCmdStateText =
          strCmdStateText + QString("车灯控制:0x%1").arg(stChas2VCMsg.ucLight, 2, 16, QChar('0')) + " ";
      break;
    }
    }

    switch (stChas2VCMsg.ucHorn)
    {
    case 0x1:
    {
      strCmdStateText += "车笛开";
      break;
    }
    case 0x2:
    {
      strCmdStateText += "车笛关";
      break;
    }
    default:
    {
      strCmdStateText += QString("车笛控制:0x%1").arg(stChas2VCMsg.ucHorn, 2, 16, QChar('0'));
      break;
    }
    }

    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_WORK_MODE_SET:
  {
    strCmdType = "被控模式设置";
    Chas2VCWorkModeStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stChas2VCMsg.ucWorkMode)
    {
    case 0x1:
    {
      strCmdStateText = strCmdStateText + "人控模式" + " ";
      break;
    }
    case 0x2:
    {
      strCmdStateText = strCmdStateText + "兼容模式" + " ";
      break;
    }
    default:
    {
      strCmdStateText =
          strCmdStateText + QString("被控模式:0x%1").arg(stChas2VCMsg.ucWorkMode, 2, 16, QChar('0')) + " ";
      break;
    }
    }

    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_PARKING_CTRL:
  {
    strCmdType = "驻车控制";
    Chas2VCParkStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8); /*由于命令应答报文与命令报文一致*/
    switch (stChas2VCMsg.ucParkFlag)
    {
    case 0x55:
    {
      strCmdStateText = strCmdStateText + "标识:驻车制动" + " ";
      break;
    }
    case 0xAA:
    {
      strCmdStateText = strCmdStateText + "标识:解除驻车" + " ";
      break;
    }
    default:
    {
      strCmdStateText =
          strCmdStateText + QString("驻车控制标识:0x%1").arg(stChas2VCMsg.ucParkFlag, 2, 16, QChar('0')) + " ";
      break;
    }
    }

    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_BRAKING_CTRL:
  {
    strCmdType = "紧急制动";
    Chas2VCBrakStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8);
    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_ENGINE_START_CTRL:
  {
    strCmdType = "发动机启动";
    Chas2VCEngineStartStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8);
    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_ENGINE_END_CTRL:
  {
    strCmdType = "发动机停止";
    Chas2VCEngineStopStateMsgST stChas2VCMsg;
    memcpy(&stChas2VCMsg, &stChas2VCStateMsg, 8);
    switch (stChas2VCMsg.ucExeState)
    {
    case 0x55:
    {
      strCmdStateText += "执行成功";
      break;
    }
    case 0xAA:
    {
      strCmdStateText += "执行失败";
      break;
    }
    default:
    {
      strCmdStateText += QString("执行结果:0x%1").arg(stChas2VCMsg.ucExeState, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  default:
    break;
  }
  // ui->labChasStateType->setText(strCmdType);
  // ui->labChasStateInfo->setText(strCmdStateText);
  // MsgTableDisp("状态", (strCmdType + strCmdStateText));
}

void ParentDlg::PathBindRsltDisp() /*路径装订结果信息显示*/
{
  /*首先清空路径装订结果列表*/
  ui->tablePathBindRsltInfo->clearContents();
  ui->tablePathBindRsltInfo->setRowCount(0);
  ui->tablePathBindRsltInfo->scrollToTop();

  INT32 iPathBindErrNum = 0; /*路径点装订错误数*/
  for (INT32 i = 0; i < m_listTrackPathBindRslt.count(); i++)
  {
    /*报文表格增加一行*/
    INT32 iRowCnt = ui->tablePathBindRsltInfo->rowCount();
    ui->tablePathBindRsltInfo->insertRow(iRowCnt);

    QTableWidgetItem *item0 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
    QString strRowNo = QString("%1").arg(iRowCnt + 1);
    item0->setText(strRowNo);

    QTableWidgetItem *item1 = new QTableWidgetItem;
    QString strPathBindRsltENUX = QString("%1").arg(m_listTrackPathBindRslt.at(i).stENUPoint.x, 0, 'f', 2);
    item1->setText(strPathBindRsltENUX);

    QTableWidgetItem *item2 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
    QString strPathBindRsltENUY = QString("%1").arg(m_listTrackPathBindRslt.at(i).stENUPoint.y, 0, 'f', 2);
    item2->setText(strPathBindRsltENUY);

    /*装订反馈结果数据应该与装订数据相等，否则以红色字体标出*/
    if (0 < m_listTrackPathCmd.size()) /*装订过路径点*/
    {
      if ((fabs(m_listTrackPathBindRslt.at(i).stENUPoint.x - m_listTrackPathCmd.at(i).stENUPoint.x) > 1e-3) ||
          (fabs(m_listTrackPathBindRslt.at(i).stENUPoint.y - m_listTrackPathCmd.at(i).stENUPoint.y) > 1e-3))
      {
        item0->setTextColor(QColor(179, 0, 0));
        item1->setTextColor(QColor(179, 0, 0));
        item2->setTextColor(QColor(179, 0, 0));
        iPathBindErrNum++; /*路径点装订错误数*/
      }
    }

    ui->tablePathBindRsltInfo->setItem(iRowCnt, 0, item0);
    ui->tablePathBindRsltInfo->setItem(iRowCnt, 1, item1);
    ui->tablePathBindRsltInfo->setItem(iRowCnt, 2, item2);
  }
  QString strPathBindRsltInfo;
  if (0 == iPathBindErrNum)
  {
    strPathBindRsltInfo = "路径装订结果反馈正确";    /*在信息表格中进行显示*/
    MsgTableDisp("路径点装订", strPathBindRsltInfo); /*在信息表格中进行显示*/
  }
  else
  {
    strPathBindRsltInfo = QString("存在%1个路径点装订错误").arg(iPathBindErrNum);
    MsgTableDisp("错误", strPathBindRsltInfo); /*在信息表格中进行显示*/
  }

  emit WriteRecord(strPathBindRsltInfo, RECORD_ERROR); /*记录该错误*/

  ui->tablePathBindRsltInfo->scrollToTop();
  // ui->tablePathBindRsltInfo->selectRow(iRowCnt);
}

void ParentDlg::TrackPathDisp() /*需要跟踪的路径信息显示*/
{
  /*首先清空路径装订结果列表*/
  ui->tableTrackPathInfo->clearContents();
  ui->tableTrackPathInfo->setRowCount(0);
  ui->tableTrackPathInfo->scrollToTop();

  for (INT32 i = 0; i < m_listTrackPathCmd.count(); i++)
  {
    /*报文表格增加一行*/
    INT32 iRowCnt = ui->tableTrackPathInfo->rowCount();
    ui->tableTrackPathInfo->insertRow(iRowCnt);

    QTableWidgetItem *item0 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
    QString strRowNo = QString("%1").arg(iRowCnt + 1);
    item0->setText(strRowNo);

    QTableWidgetItem *item1 = new QTableWidgetItem;
    QString strPathBindRsltENUX = QString("%1").arg(m_listTrackPathCmd.at(i).stENUPoint.x, 0, 'f', 2);
    item1->setText(strPathBindRsltENUX);

    QTableWidgetItem *item2 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
    QString strPathBindRsltENUY = QString("%1").arg(m_listTrackPathCmd.at(i).stENUPoint.y, 0, 'f', 2);
    item2->setText(strPathBindRsltENUY);

    QTableWidgetItem *item3 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
    item3->setText("0");

    ui->tableTrackPathInfo->setItem(iRowCnt, 0, item0);
    ui->tableTrackPathInfo->setItem(iRowCnt, 1, item1);
    ui->tableTrackPathInfo->setItem(iRowCnt, 2, item2);
    ui->tableTrackPathInfo->setItem(iRowCnt, 3, item3);
  }
  ui->tableTrackPathInfo->scrollToTop();
}

void ParentDlg::PathCollectDisp(QList<SmoothPathPointST> listENUPos) /*采集到的路径点进行显示*/
{
  if (1 == listENUPos.size()) /*需重新显示路径列表*/
  {
    ui->tableTrackPathInfo->clearContents(); /*清空原表格中的信息*/
    ui->tableTrackPathInfo->setRowCount(0);
    ui->tableTrackPathInfo->scrollToTop();
  }

  /*报文表格增加一行*/
  INT32 iRowCnt = ui->tableTrackPathInfo->rowCount();
  ui->tableTrackPathInfo->insertRow(iRowCnt);

  QTableWidgetItem *item0 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
  QString strRowNo = QString("%1").arg(iRowCnt + 1);
  item0->setText(strRowNo);

  QTableWidgetItem *item1 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
  QString strPathBindRsltENUX = QString("%1").arg(listENUPos.last().stENUPoint.x, 0, 'f', 2);
  item1->setText(strPathBindRsltENUX);

  QTableWidgetItem *item2 = new QTableWidgetItem; /*不能加delete，否则无显示内容，应该是自动释放内存*/
  QString strPathBindRsltENUY = QString("%1").arg(listENUPos.last().stENUPoint.y, 0, 'f', 2);
  item2->setText(strPathBindRsltENUY);

  ui->tableTrackPathInfo->setItem(iRowCnt, 0, item0);
  ui->tableTrackPathInfo->setItem(iRowCnt, 1, item1);
  ui->tableTrackPathInfo->setItem(iRowCnt, 2, item2);

  /*采集到的循迹点就是装订路径，赋值到路径列表中*/
  m_listTrackPathCmd = listENUPos;
  ui->labGridMap->update(); /*重绘路径点*/
}

void ParentDlg::MapCorditBindRsltDisp(LongLatHeightST stOrigin, ENUCorST stEnd) /*地图坐标系装订结果信息显示*/
{
  if (true == g_bSetMapOrigin) /*如果已设定了地图原点，那么要比对查询到的值与装订的值是否一致*/
  {
    if (fabs(g_stMapCorditCmd.stLLHOriginPos.fLongitude - stOrigin.fLongitude) > 1e-7)
    {
      ui->labOriginLongitudeBindRslt->setStyleSheet("QLabel{color:rgb(179,0,0)}");
      MsgTableDisp("错误", "原点经度装订反馈不一致");
    }
    else
    {
      ui->labOriginLongitudeBindRslt->setStyleSheet(
          "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
      MsgTableDisp("坐标系装订", "原点经度装订反馈正确");
    }

    if (fabs(g_stMapCorditCmd.stLLHOriginPos.fLatitude - stOrigin.fLatitude) > 1e-7)
    {
      ui->labOriginLatitudeBindRslt->setStyleSheet("QLabel{color:rgb(179,0,0)}");
      MsgTableDisp("错误", "原点纬度装订反馈不一致");
    }
    else
    {
      ui->labOriginLatitudeBindRslt->setStyleSheet(
          "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
      MsgTableDisp("坐标系装订", "原点纬度装订反馈正确");
    }

    if (fabs(g_stMapCorditCmd.stLLHOriginPos.fLatitude - stOrigin.fLatitude) > 1e-3)
    {
      ui->labOriginLatitudeBindRslt->setStyleSheet("QLabel{color:rgb(179,0,0)}");
      MsgTableDisp("错误", "原点高度装订反馈不一致");
    }
    else
    {
      ui->labOriginLatitudeBindRslt->setStyleSheet(
          "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
      MsgTableDisp("坐标系装订", "原点高度装订反馈正确");
    }
  }
  ui->labOriginLatitudeBindRslt->setText(QString("%1").arg(stOrigin.fLatitude, 0, 'f', 7));
  ui->labOriginLongitudeBindRslt->setText(QString("%1").arg(stOrigin.fLongitude, 0, 'f', 7));
  ui->labOriginHeightBindRslt->setText(QString("%1").arg(stOrigin.fHeight, 0, 'f', 3));

  if (false == ui->labPathEndENUXBind->text().isEmpty()) /*装订了路径终点X值*/
  {
    if (fabs(ui->labPathEndENUXBind->text().toDouble() - stEnd.x) > 1e-3)
    {
      ui->labPathEndENUXBindRslt->setStyleSheet("QLabel{color:rgb(179,0,0)}");
      MsgTableDisp("错误", "路径终点X装订反馈不一致");
    }
    else
    {
      ui->labPathEndENUXBindRslt->setStyleSheet(
          "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
      MsgTableDisp("坐标系装订", "路径终点X装订反馈正确");
    }
  }

  if (false == ui->labPathEndENUYBind->text().isEmpty()) /*装订了路径终点Y值*/
  {
    if (fabs(ui->labPathEndENUYBind->text().toDouble() - stEnd.y) > 1e-3)
    {
      ui->labPathEndENUYBindRslt->setStyleSheet("QLabel{color:rgb(179,0,0)}");
      MsgTableDisp("错误", "路径终点Y装订反馈不一致");
    }
    else
    {
      ui->labPathEndENUYBindRslt->setStyleSheet(
          "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
      MsgTableDisp("坐标系装订", "路径终点Y装订反馈正确");
    }
  }

  ui->labPathEndENUXBindRslt->setText(QString("%1").arg(stEnd.x, 0, 'f', 2));
  ui->labPathEndENUYBindRslt->setText(QString("%1").arg(stEnd.y, 0, 'f', 2));
}

void ParentDlg::EqStateDispColor(QLabel *label, UINT8 ucColor) /*设备状态文本背景颜色设置*/
{
  switch (ucColor)
  {
  case FP_EQ_STATE_OFF: /*设备断电*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:black; color:gray}");
    break;
  }
  case FP_EQ_STATE_ON_UNKONOWN: /*设备加电但状态未知*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:gray; color:white}");
    break;
  }
  case FP_EQ_STATE_ON_OK: /*设备正常*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
    break;
  }
  case FP_EQ_STATE_ON_ERROR: /*设备故障*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:rgb(179,0,0); color:white}");
    break;
  }
  default:
  {
    break;
  }
  }
}

void ParentDlg::ChasDetailStateDispColor(QLabel *label, UINT8 ucColor) /*底盘设备状态文本背景颜色设置*/
{
  switch (ucColor)
  {
  case CHAS_EQ_STATE_OFF: /*设备断电*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:black; color:gray}");
    break;
  }
  case CHAS_EQ_STATE_ON_UNKONOWN: /*设备加电但状态未知*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:gray; color:white}");
    break;
  }
  case CHAS_EQ_STATE_ON_OK: /*设备正常*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
    break;
  }
  case CHAS_EQ_STATE_ON_ERROR: /*设备故障*/
  {
    label->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:rgb(179,0,0); color:white}");
    break;
  }
  default:
  {
    break;
  }
  }
}

void ParentDlg::CameraDataDisp(CameraDataST stCameraData) /*摄像头信息显示*/
{
  //  if (0x55 == stCameraData.ucCameraOn)
  //  {
  //    ui->labCameraOn->setText("是");
  //  }
  //  else if (0xAA == stCameraData.ucCameraOn)
  //  {
  //    ui->labCameraOn->setText("否");
  //  }
  //  else
  //  {
  //    ui->labCameraOn->setText(QString("0x%1").arg(stCameraData.ucCameraOn, 2, 16, QChar('0')));
  //  }

  //  if (0x55 == stCameraData.ucCameraInfoValid)
  //  {
  //    ui->labCameraInfoValid->setText("有效");
  //  }
  //  else if (0xAA == stCameraData.ucCameraInfoValid)
  //  {
  //    ui->labCameraInfoValid->setText("无效");
  //  }
  //  else
  //  {
  //    ui->labCameraInfoValid->setText(QString("0x%1").arg(stCameraData.ucCameraInfoValid, 2, 16, QChar('0')));
  //  }

  //  ui->labDistToCenterLine->setText(
  //      QString("%1").arg(stCameraData.sCameraToCenterLineDist * 0.05, 0, 'f', 2)); /*摄像头中心距左车道线距离*/
  //  ui->labCameraAngle->setText(
  //      QString("%1").arg(stCameraData.sCamera_Line_Angle * 0.01, 0, 'f', 2)); /*摄像头纵轴与车道线夹角*/
}

void ParentDlg::RadarDataDisp(RadarDataST stRadarData) /*激光雷达信息显示*/
{
  //  if (0x55 == stRadarData.ucRadarOn)
  //  {
  //    ui->labRadarOn->setText("是");
  //  }
  //  else if (0xAA == stRadarData.ucRadarOn)
  //  {
  //    ui->labRadarOn->setText("否");
  //  }
  //  else
  //  {
  //    ui->labRadarOn->setText(QString("0x%1").arg(stRadarData.ucRadarOn, 2, 16, QChar('0')));
  //  }

  //  if (0x55 == stRadarData.ucRadarInfoValid)
  //  {
  //    ui->labRadarInfoValid->setText("有效");
  //  }
  //  else if (0xAA == stRadarData.ucRadarInfoValid)
  //  {
  //    ui->labRadarInfoValid->setText("无效");
  //  }
  //  else
  //  {
  //    ui->labRadarInfoValid->setText(QString("0x%1").arg(stRadarData.ucRadarInfoValid, 2, 16, QChar('0')));
  //  }

  //  ui->labRadarENUPosX->setText(
  //      QString("%1").arg(stRadarData.sRadarENUX * 0.05, 0, 'f', 2)); /*激光雷达在本地东北天坐标系中的X坐标*/
  //  ui->labRadarENUPosY->setText(
  //      QString("%1").arg(stRadarData.sRadarENUY * 0.05, 0, 'f', 2)); /*激光雷达在本地东北天坐标系中的Y坐标*/
  //  ui->labRadarNorthAngle->setText(
  //      QString("%1").arg(stRadarData.sRadar_North_Angle * 0.01, 0, 'f', 2)); /*激光雷达纵轴与真北的夹角*/
}

void ParentDlg::ChasDataDisp(Chas2VCWorkDataST stChas2VCWorkData) /*底盘数据显示处理*/
{
  // ui->labChas2VCWorkMsgNum->setText(QString("%1").arg(stChas2VCWorkData.uiRecvWorkDataCnt));
  /*底盘分设备状态显示*/
  if (0x55 == stChas2VCWorkData.ucParkState) /*底盘驻车制动状态*/
  {
    ui->labChasParkState->setText("驻车制动");
    ui->labChasParkState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else if (0xAA == stChas2VCWorkData.ucParkState)
  {
    ui->labChasParkState->setText("解除驻车");
    ui->labChasParkState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    QString str = QString("0x%1").arg(stChas2VCWorkData.ucParkState, 2, 16, QChar('0'));
    ui->labChasParkState->setText(str);
    ui->labChasParkState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

  if (0x1 == stChas2VCWorkData.ucWorkMode) /*底盘被控模式*/
  {
    ui->labChasWorkMode->setText("人控模式");
    ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:green; color:white}");
  }
  else if (0x2 == stChas2VCWorkData.ucWorkMode)
  {
    ui->labChasWorkMode->setText("兼容模式");
    ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    QString str = QString("0x%1").arg(stChas2VCWorkData.ucWorkMode, 2, 16, QChar('0'));
    ui->labChasWorkMode->setText(str);
    ui->labChasWorkMode->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

  // if (0x1 == stChas2VCWorkData.ucChasJudgeCtrlerState) /*底盘端判定上装状态*/
  // {
  //   ui->labChasJudgeVCState->setText("正常");
  //   ui->labChasJudgeVCState->setStyleSheet(
  //       "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
  // }
  // else if (0x2 == stChas2VCWorkData.ucChasJudgeCtrlerState)
  // {
  //   ui->labChasJudgeVCState->setText("故障");
  //   ui->labChasJudgeVCState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  // }
  // else
  // {
  //   QString str = QString("0x%1").arg(stChas2VCWorkData.ucChasJudgeCtrlerState, 2, 16, QChar('0'));
  //   ui->labChasJudgeVCState->setText(str);
  //   ui->labChasJudgeVCState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  // }

  if (0x1 == stChas2VCWorkData.ucBrakState) /*刹车状态*/
  {
    ui->labChasBrakState->setText("未刹车");
    ui->labChasBrakState->setStyleSheet(
        "QLabel{font:11pt '仿宋' bold; background-color:rgb(35, 35, 35);color:rgb(21,197,212);}");
  }
  else if (0x2 == stChas2VCWorkData.ucBrakState)
  {
    ui->labChasBrakState->setText("刹车中");
    ui->labChasBrakState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }
  else
  {
    QString str = QString("0x%1").arg(stChas2VCWorkData.ucBrakState, 2, 16, QChar('0'));
    ui->labChasBrakState->setText(str);
    ui->labChasBrakState->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  }

  QString strText =
      QString("%1").arg((stChas2VCWorkData.usVehSpeed * 0.1 * 3.6), 0, 'f', 1); /*底盘车速，20210419修改成km/h显示*/
  ui->labChasVehSpeed->setText(strText);

  strText = QString("%1").arg((stChas2VCWorkData.sTurnAngle * 0.01), 0, 'f', 2); /*底盘前轮转角*/
  ui->labChasTurnAngle->setText(strText);

  strText = QString("%1").arg((stChas2VCWorkData.ucTurnAngleSpeed * 0.1), 0, 'f', 1); /*底盘前轮转角速度*/
  ui->labChasTurnAngleSpeed->setText(strText);

  ui->labChasGear->setStyleSheet(
      "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");
  switch (stChas2VCWorkData.ucTransGear) /*变速箱档位*/
  {
  case 0:
  {
    ui->labChasGear->setText("N");
    break;
  }
  case 1:
  {
    ui->labChasGear->setText("D");
    break;
  }
  case 2:
  {
    ui->labChasGear->setText("P");
    break;
  }
  case 3:
  {
    ui->labChasGear->setText("D3");
    break;
  }
  case 4:
  {
    ui->labChasGear->setText("D4");
    break;
  }
  case 5:
  {
    ui->labChasGear->setText("D5");
    break;
  }
  case 6:
  {
    ui->labChasGear->setText("D6");
    break;
  }
  case 7:
  {
    ui->labChasGear->setText("R");
    break;
  }
  default:
  {
    ui->labChasGear->setStyleSheet("QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
    ui->labChasGear->setText(QString("0x%1").arg(stChas2VCWorkData.ucTransGear, 2, 16, QChar('0')));
    break;
  }
  }

  // strText = QString("%1").arg(stChas2VCWorkData.usEngineRotSpeed); /*发动机转速*/
  // ui->labChasEngineRotSpeed->setText(strText);

  // strText = QString("%1").arg((stChas2VCWorkData.sEngineTorque - 125)); /*发动机扭矩*/
  // ui->labChasEngineTorque->setText(strText);

  strText = QString("%1").arg(stChas2VCWorkData.ucOilOpening); /*油门开度*/
  ui->labChasOilOpen->setText(strText);

  //    strText = QString("%1").arg(stChas2VCWorkData.ucTurnArmOpening);//转向开度
  //    ui->labChasTurnArmOpen->setText(strText);

  strText = QString("%1").arg(stChas2VCWorkData.ucBrakOpening); /*刹车开度*/
  ui->labChasBrakOpen->setText(strText);

  // ui->labEngineWorkState->setStyleSheet(
  //     "QLabel{font:11pt '仿宋' bold; color:rgb(21,197,212);background-color:rgb(35,35,35);}");
  // switch (stChas2VCWorkData.ucEMSWorkState) /*发动机状态*/
  // {
  //   case 1:
  //   {
  //     ui->labEngineWorkState->setText("启动中");
  //     break;
  //   }
  //   case 2:
  //   {
  //     ui->labEngineWorkState->setText("启动完成");
  //     break;
  //   }
  //   case 3:
  //   {
  //     ui->labEngineWorkState->setText("停止中");
  //     break;
  //   }
  //   case 4:
  //   {
  //     ui->labEngineWorkState->setText("停止完成");
  //     break;
  //   }
  //   default:
  //   {
  //     ui->labEngineWorkState->setStyleSheet(
  //         "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  //     ui->labEngineWorkState->setText(QString("0x%1").arg(stChas2VCWorkData.ucEMSWorkState, 2, 16, QChar('0')));
  //     break;
  //   }
  // }

  // strText = QString("%1").arg(stChas2VCWorkData.ucOilRemain); /*油箱油量*/
  // ui->labOilRemain->setText(strText);

  // if (0x55 == stChas2VCWorkData.ucEmergBrakState) /*底盘紧急制动状态*/
  // {
  //   ui->labChasEmerencyBrakState->setText("紧急制动中");
  //   ui->labChasEmerencyBrakState->setStyleSheet(
  //       "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  // }
  // else if (0xAA == stChas2VCWorkData.ucEmergBrakState)
  // {
  //   ui->labChasEmerencyBrakState->setText("未紧急制动");
  //   ui->labChasEmerencyBrakState->setStyleSheet(
  //       "QLabel{font:11pt '仿宋' bold; background-color:rgb(35,35,35); color:rgb(21,197,212);}");
  // }
  // else
  // {
  //   QString str = QString("0x%1").arg(stChas2VCWorkData.ucEmergBrakState, 2, 16, QChar('0'));
  //   ui->labChasEmerencyBrakState->setText(str);
  //   ui->labChasEmerencyBrakState->setStyleSheet(
  //       "QLabel{font:11pt '仿宋' bold; background-color:rgb(179,0,0); color:white}");
  // }

  // strText = QString("%1").arg(stChas2VCWorkData.usBatVolt * 0.1); /*底盘蓄电池电压*/
  // ui->labChasBatVolt->setText(strText);

  strText = QString("%1").arg((stChas2VCWorkData.uiMileage * 0.125), 0, 'f', 3); /*底盘里程*/
  ui->labChasMileage->setText(strText);
}

void ParentDlg::INSDataDisp(INS2VCWorkDataST stINS2VCWorkData) /*INS数据显示处理*/
{
  /*导航信息显示*/
  QString strUTCTime = QString("%1年%2月%3日%4时%5分%6.%7秒")
                           .arg(stINS2VCWorkData.usUTCYear)
                           .arg(stINS2VCWorkData.ucUTCMonth)
                           .arg(stINS2VCWorkData.ucUTCDay)
                           .arg(stINS2VCWorkData.ucUTCHour)
                           .arg(stINS2VCWorkData.ucUTCMinute)
                           .arg(stINS2VCWorkData.ucUTCSecInt)
                           .arg(stINS2VCWorkData.ucUTCSecDOt);
  ui->labINSUTCTime->setText(strUTCTime);

  switch (stINS2VCWorkData.ucINSState)
  {
  case INS_STATE_INIT:
  {
    ui->labINSState->setText("初始化");
    break;
  }
  case INS_STATE_COARSE_ALIGN:
  {
    ui->labINSState->setText("粗对准");
    break;
  }
  case INS_STATE_FINE_ALIGN:
  {
    ui->labINSState->setText("精对准");
    break;
  }
  case INS_STATE_ONE_ANTENA_LOCATE:
  {
    ui->labINSState->setText("单天线定位");
    break;
  }
  case INS_STATE_DOUBLE_ANTENA_LOCATE:
  {
    ui->labINSState->setText("双天线定位");
    break;
  }
  case INS_STATE_ONE_ANTENA_RTK:
  {
    ui->labINSState->setText("单天线差分");
    break;
  }
  case INS_STATE_ONLY_IMU:
  {
    ui->labINSState->setText("纯惯");
    break;
  }
  case INS_STATE_ZERO_SPEED_REVISE:
  {
    ui->labINSState->setText("零速校正");
    break;
  }
  case INS_STATE_DOUBLE_ANTENA_RTK:
  {
    ui->labINSState->setText("双天线差分");
    break;
  }
  case INS_STATE_DYNAMIC_ALIGN:
  {
    ui->labINSState->setText("动态对准");
    break;
  }
  case INS_STATE_ERROR:
  {
    ui->labINSState->setText("系统异常");
    break;
  }
  default:
    break;
  }

  // ui->labGPSState->setText("数据无效");

  /*GPS信息显示*/
  // for (UINT8 i = 0; i < 7; i++)
  // {
  //   if (0x1 == ((stINS2VCWorkData.ucGPSState >> i) & 0x1))
  //   {
  //     switch (i)
  //     {
  //       case 0:
  //       {
  //         ui->labGPSState->setText("数据有效");
  //         break;
  //       }
  //       case 1:
  //       {
  //         ui->labGPSState->setText("位置速度有效");
  //         break;
  //       }
  //       case 2:
  //       {
  //         ui->labGPSState->setText("定向有效");
  //         break;
  //       }
  //       case 3:
  //       {
  //         ui->labGPSState->setText("差分有效");
  //         break;
  //       }
  //       case 4:
  //       {
  //         ui->labGPSState->setText("时间有效");
  //         break;
  //       }
  //       case 5:
  //       {
  //         ui->labGPSState->setText("差分浮点解有效");
  //         break;
  //       }
  //       case 6:
  //       {
  //         ui->labGPSState->setText("差分固定解有效");
  //         break;
  //       }
  //       default:
  //         break;
  //     }
  //   }
  // }

  QString strText;
  double fValue = stINS2VCWorkData.iHeight * 1e-3;
  strText = QString("%1").arg(fValue, 0, 'f', 3);
  ui->labINSHeight->setText(strText);

  fValue = (float)stINS2VCWorkData.iLongitude * 1e-7;
  strText = QString("%1").arg(fValue, 0, 'f', 7);
  ui->labINSLongitude->setText(strText);

  fValue = (float)stINS2VCWorkData.iLatitude * 1e-7;
  strText = QString("%1").arg(fValue, 0, 'f', 7);
  ui->labINSLatitude->setText(strText);

  fValue = (float)stINS2VCWorkData.uiVehCourse * 1e-3;
  strText = QString("%1").arg(fValue, 0, 'f', 3);
  ui->labINSCourse->setText(strText);

  fValue = (float)stINS2VCWorkData.iVehPitch * 1e-3;
  strText = QString("%1").arg(fValue, 0, 'f', 3);
  ui->labINSPitch->setText(strText);

  fValue = (float)stINS2VCWorkData.iVehRoll * 1e-3;
  strText = QString("%1").arg(fValue, 0, 'f', 3);
  ui->labINSRoll->setText(strText);

  fValue = (float)stINS2VCWorkData.usGroundSpeed * 1e-2 * 3.6; /*20210419修改成km/h显示*/
  strText = QString("%1").arg(fValue, 0, 'f', 2);
  ui->labINSGroudSpeed->setText(strText);

  fValue = (float)stINS2VCWorkData.sAngleSpeed * 1e-2;
  strText = QString("%1").arg(fValue, 0, 'f', 2);
  ui->labINSAngleSpeed->setText(strText);

  ui->labGPSFrontStar->setText(QString("%1").arg(uint(stINS2VCWorkData.ucFrontStar)));
  ui->labGPSBackStar->setText(QString("%1").arg(uint(stINS2VCWorkData.ucBackStar)));

  // fValue = (float)stINS2VCWorkData.sLine * 1e-3;
  // strText = QString("%1").arg(fValue, 0, 'f', 3);
  // ui->labGPSLine->setText(strText);
}

void ParentDlg::ChasCmdDataDisp(VC2ChasCmdST stVC2ChasCmd) /*综控发给底盘的命令数据显示处理*/
{
  bool bMsgTableDisp = true; /*在数据表格中显示*/
  QString strCmdInfo = QString("%1:%2:%3.%4 ")
                           .arg(stVC2ChasCmd.ucHour, 2, 10, QChar('0'))
                           .arg(stVC2ChasCmd.ucMinute, 2, 10, QChar('0'))
                           .arg(stVC2ChasCmd.ucSecond, 2, 10, QChar('0'))
                           .arg(stVC2ChasCmd.usMSecond, 3, 10, QChar('0')); /*命令信息*/
  QString strCmdType;
  switch (stVC2ChasCmd.ucCmdType)
  {
  case CMD_CHAS_SELF_CHECK:
  {
    strCmdType = "自检";
    break;
  }
  case CMD_CHAS_GEAR:
  {
    strCmdType = "档位设置";

    strCmdInfo += ""; /*命令信息*/
    switch (stVC2ChasCmd.ucGear)
    {
    case 7:
    {
      strCmdInfo += "D档"; /*命令信息*/
      break;
    }
    case 8:
    {
      strCmdInfo += "R档"; /*命令信息*/
      break;
    }
    case 9:
    {
      strCmdInfo += "N档"; /*命令信息*/
      break;
    }
    default:
    {
      strCmdInfo += "无效"; /*命令信息*/
      break;
    }
    }
    break;
  }
  case CMD_CHAS_MOVE:
  {
    strCmdType = "运动控制";

    strCmdInfo += "车速 "; /*命令信息*/

    QString strText =
        QString("%1").arg((stVC2ChasCmd.usVehSpeed * 0.1 * 3.6), 0, 'f', 1); /*底盘车速，20210419修改为km/h显示*/
    strCmdInfo = strCmdInfo + strText + "km/h";
    strCmdInfo += " ";
    strCmdInfo += "转角 ";                            /*命令信息*/
    m_fVehTurnAngle = stVC2ChasCmd.sTurnAngle * 0.01; /*车辆前轮转角赋值*/

    strText = QString("%1").arg((stVC2ChasCmd.sTurnAngle * 0.01), 0, 'f', 2); /*底盘前轮转角*/
    strCmdInfo = strCmdInfo + strText + "°";
    strCmdInfo += " ";
    strCmdInfo += "转角速度 ";                                                     /*命令信息*/
    strText = QString("%1").arg((stVC2ChasCmd.ucTurnAngleSpeed * 0.1), 0, 'f', 1); /*底盘前轮转角速度*/
    strCmdInfo = strCmdInfo + strText + "°/s";
    bMsgTableDisp = false; /*在数据表格中显示*/
    break;
  }
  case CMD_CHAS_VEH_BASIC_CTRL:
  {
    strCmdType = "车灯笛控制";

    switch (stVC2ChasCmd.ucLight)
    {
    case 0x1:
    {
      strCmdInfo = strCmdInfo + "近光灯开" + " ";
      break;
    }
    case 0x2:
    {
      strCmdInfo = strCmdInfo + "近光灯关" + " ";
      break;
    }
    case 0x3:
    {
      strCmdInfo = strCmdInfo + "远光灯开" + " ";
      break;
    }
    case 0x4:
    {
      strCmdInfo = strCmdInfo + "远光灯关" + " ";
      break;
    }
    default:
    {
      strCmdInfo = strCmdInfo + QString("车灯控制:0x%1").arg(stVC2ChasCmd.ucLight, 2, 16, QChar('0')) + " ";
      break;
    }
    }

    switch (stVC2ChasCmd.ucHorn)
    {
    case 0x1:
    {
      strCmdInfo += "车笛开";
      break;
    }
    case 0x2:
    {
      strCmdInfo += "车笛关";
      break;
    }
    default:
    {
      strCmdInfo += QString("车笛控制:0x%1").arg(stVC2ChasCmd.ucHorn, 2, 16, QChar('0'));
      break;
    }
    }
    break;
  }
  case CMD_CHAS_WORK_MODE_SET:
  {
    strCmdType = "被控模式设置";

    if (0x1 == stVC2ChasCmd.ucWorkMode) /*底盘被控模式*/
    {
      strCmdInfo += "人控模式";
    }
    else if (0x2 == stVC2ChasCmd.ucWorkMode)
    {
      strCmdInfo += "兼容模式";
    }
    else
    {
      strCmdInfo += QString("被控模式:0x%1").arg(stVC2ChasCmd.ucWorkMode, 2, 16, QChar('0'));
    }
    break;
  }
  case CMD_CHAS_PARKING_CTRL:
  {
    strCmdType = "驻车控制";

    if (0x55 == stVC2ChasCmd.ucPark) /*底盘驻车制动状态*/
    {
      strCmdInfo += "标识:驻车制动";
    }
    else if (0xAA == stVC2ChasCmd.ucPark)
    {
      strCmdInfo += "标识:解除驻车";
    }
    else
    {
      strCmdInfo += QString("驻车控制标识:0x%1").arg(stVC2ChasCmd.ucPark, 2, 16, QChar('0'));
    }
    break;
  }
  case CMD_CHAS_BRAKING_CTRL:
  {
    strCmdType = "紧急制动控制";
    break;
  }
  case CMD_CHAS_CLEAR_BRAK_CTRL:
  {
    strCmdType = "解除紧急制动";
    break;
  }
  case CMD_CHAS_ENGINE_START_CTRL:
  {
    strCmdType = "发动机启动";
    break;
  }
  case CMD_CHAS_ENGINE_END_CTRL:
  {
    strCmdType = "发动机停止";
    break;
  }
  default:
  {
    bMsgTableDisp = false;
    strCmdType = "无效";
    break;
  }
  }
  // if ("无效" != strCmdType)
  // {
  //   ui->labChasCmdType->setText(strCmdType);
  //   m_strChasCmdText = strCmdInfo;
  //   ui->labChasCmdInfo->setText(strCmdInfo);
  //   if (true == bMsgTableDisp)
  //   {
  //     MsgTableDisp("发送命令", (strCmdType + strCmdInfo));
  //   }
  // }
}

/******************************接收报文长度错误处理******************************/
void ParentDlg::RecvLenErrorProc(QByteArray datagram, INT32 iLen, UINT32 uiRightLen)
{
  Q_UNUSED(iLen);

  NetHeaderST stNetHeader; /*报文头*/
  memset(&stNetHeader, 0, sizeof(NetHeaderST));
  memcpy(&stNetHeader, datagram.data(), sizeof(NetHeaderST)); /*获取报文头内容*/

  QString strInfo = QString("帧号%1H报文长度%2错误，应为%3")
                        .arg(stNetHeader.uiMsgNo, 8, 16, QChar('0'))
                        .arg(stNetHeader.usMsgLen)
                        .arg(uiRightLen);
  //  m_pRecord->write(strInfo, RECORD_ERROR);

  strInfo = QString("帧标识%1H报文长度错误").arg(stNetHeader.usMsgType, 4, 16, QChar('0'));
  MsgTableDisp("错误", strInfo);
}

void ParentDlg::ReadMapFileRslt(bool bRslt) /*读取地图文件结果槽函数*/
{
  if (true == bRslt)
  {
    MsgTableDisp("控制流程", "地图打开成功");
    QMessageBox::information(Q_NULLPTR, "提示", "地图文件打开成功");

    m_bMapInfoValid = true; /*地图信息有效标志，每次点击打开文件按钮，将其置为false*/
    ;
    m_fMapMeterPerPixel = m_pAStar->m_fResolution / m_fPixelPerGrid; /*当前地图缩放级数,即1个像素等于多少米*/
    ui->labMapMeterPerPixel->setText(QString("%1m").arg(m_fMapMeterPerPixel, 0, 'f', 2));

    m_bMapInfoRepaint = true; /*地图需重新绘制*/
    ui->labGridMap->update(); /*重绘地图区域*/
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "错误", "地图文件打开失败");
  }
}

bool ParentDlg::eventFilter(QObject *watched, QEvent *event)
{
  if ((watched == ui->labGridMap) && (event->type() == QEvent::Paint))
  {
    if ((0 == m_pAStar->m_iMapColNum) || (0 == m_pAStar->m_iMapRowNum) || (1e-5 > m_fPixelPerGrid))
    {
      return false;
    }

    if (true == m_bMapInfoRepaint) /*地图内信息重绘*/
    {
      DrawChessBoard();
    }

    DrawAxis();             /*地图有效，就可以绘制坐标轴*/
    DrawStartAndEndPoint(); /*有路径终点信息，则画路径终点*/
    DrawPath();             /*有路径信息，则绘制路径*/
    DrawVehPos();           /*绘制车辆当前位置*/
    DrawPathPrependPoint();
    /*绘制车辆前置点*/
    if (true == m_bPointSelect)
    {
      DrawSelectPoint();
    }
  }
  return QWidget::eventFilter(watched, event);
}

void ParentDlg::mouseMoveEvent(QMouseEvent *e) /*鼠标移动重载函数*/
{
  if ((e->pos().x() > ui->scrollArea->x()) && (e->pos().x() < (ui->scrollArea->x() + ui->scrollArea->width())))
  {
    if ((e->pos().y() > ui->scrollArea->y()) && (e->pos().y() < (ui->scrollArea->y() + ui->scrollArea->height())))
    {
      if (false == m_bMousePressed)
      {
        return;
      }
      QPoint curPt = e->pos();
      INT32 iDist = m_PressPosition.y() - curPt.y();
      ui->scrollArea->verticalScrollBar()->setValue(ui->scrollArea->verticalScrollBar()->value() + iDist);

      iDist = m_PressPosition.x() - curPt.x();
      ui->scrollArea->horizontalScrollBar()->setValue(ui->scrollArea->horizontalScrollBar()->value() + iDist);
      m_PressPosition = curPt;
    }
  }
}

void ParentDlg::mousePressEvent(QMouseEvent *e) /*鼠标按下重载函数*/
{
  m_bMousePressed = true;
  m_PressPosition = e->pos();
}

void ParentDlg::mouseReleaseEvent(QMouseEvent *e) /*鼠标释放重载函数*/
{
  Q_UNUSED(e);
  m_bMousePressed = false;

  m_PressPosition.setX(0);
  m_PressPosition.setY(0);
}

void ParentDlg::MapOriginSet(LongLatHeightST stLLHOrigin) /*设定了地图原点信号*/
{
  /*地图量测对话框进行了原点设置，在主界面上显示原点信息*/
  QString strText = QString("%1").arg(stLLHOrigin.fLongitude, 0, 'f', 7);
  ui->editOriginLongitude->setText(strText);

  strText = QString("%1").arg(stLLHOrigin.fLatitude, 0, 'f', 7);
  ui->editOriginLatitude->setText(strText);

  strText = QString("%1").arg(stLLHOrigin.fHeight, 0, 'f', 3);
  ui->editOriginHeight->setText(strText);
}

void ParentDlg::DrawChessBoard() /*绘制一张白板,清除之前的图*/
{
  INT32 iMapColPixel, iMapRowPixel; /*地图在宽和高上占的屏幕像素*/
  if (1 <= m_fPixelPerGrid)         /*每个栅格占1个以上像素，地图放大*/
  {
    iMapColPixel = m_pAStar->m_iMapColNum * m_fPixelPerGrid;
    iMapRowPixel = m_pAStar->m_iMapRowNum * m_fPixelPerGrid;
  }
  else /*1个像素中含有多个栅格，地图缩小*/
  {
    iMapColPixel = (INT32)(m_pAStar->m_iMapColNum * m_fPixelPerGrid) + 1;
    iMapRowPixel = (INT32)(m_pAStar->m_iMapRowNum * m_fPixelPerGrid) + 1;
  }

  QPainter painter(ui->labGridMap);
  painter.fillRect(0, 0, iMapColPixel, iMapRowPixel, QBrush(QColor(0xff1C3245)));
}

void ParentDlg::DrawAxis() /*画坐标轴*/
{
  if (true == m_bMapInfoValid) /*地图信息有效，则绘制坐标轴*/
  {
    GridMapIndexST stOriginGridIndex;   /*原点在地图上的栅格索引位置*/
    INT32 iOriginPXPosX, iOriginPXPosY; /*原点在lab上的像素点位置*/

    stOriginGridIndex.iGridColNo = fabs(0 - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stOriginGridIndex.iGridRowNo = fabs(0 - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    iOriginPXPosX = stOriginGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iOriginPXPosY = stOriginGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

    QPainter painter(ui->labGridMap);
    painter.setPen(Qt::white);
    painter.drawLine(QPoint(0, iOriginPXPosY), QPoint(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid, iOriginPXPosY));
    painter.drawLine(QPoint(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid, iOriginPXPosY),
                     QPoint(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid - 5, iOriginPXPosY - 5));
    painter.drawLine(QPoint(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid, iOriginPXPosY),
                     QPoint(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid - 5, iOriginPXPosY + 5));
    painter.drawLine(QPoint(iOriginPXPosX, 0), QPoint(iOriginPXPosX, 0 + m_pAStar->m_iMapRowNum * m_fPixelPerGrid));
    painter.drawLine(QPoint(iOriginPXPosX, 0), QPoint(iOriginPXPosX - 5, 0 + 5));
    painter.drawLine(QPoint(iOriginPXPosX, 0), QPoint(iOriginPXPosX + 5, 0 + 5));

    QString strText = QString("%1").arg(m_pAStar->m_fMapENU_XMax);
    painter.drawText(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid - 5, iOriginPXPosY + 15, strText);

    strText = "米";
    painter.drawText(0 + m_pAStar->m_iMapColNum * m_fPixelPerGrid, iOriginPXPosY + 5, strText);

    strText = QString("%1").arg(m_pAStar->m_fMapENU_XMin);
    painter.drawText(0, iOriginPXPosY + 15, strText);

    strText = QString("%1").arg(m_pAStar->m_fMapENU_YMax);
    painter.drawText(QPoint(iOriginPXPosX, 0 + 15), strText);

    strText = "米";
    painter.drawText(QPoint(iOriginPXPosX - 15, 0 + 15), strText);

    strText = QString("%1").arg(m_pAStar->m_fMapENU_YMin);
    painter.drawText(QPoint(iOriginPXPosX, 0 + m_pAStar->m_iMapRowNum * m_fPixelPerGrid), strText);

    strText = "原点";
    painter.drawText(iOriginPXPosX, iOriginPXPosY + 15, strText);
  }
  else
  {
    /*从没有打开过地图，或者又在重新等待地图打开中，则暂不绘制坐标轴*/
  }
}

void ParentDlg::DrawStartAndEndPoint() /*绘制路径起点和终点*/
{
  if (true == g_bSetMapOrigin) /*设定了地图终点后再绘制终点图标*/
  {
    GridMapIndexST stStartGridIndex, stEndGridIndex; /*起点在地图上的栅格索引位置*/
    stStartGridIndex.iGridColNo = fabs(m_stENUVehCenterPos.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stStartGridIndex.iGridRowNo = fabs(m_stENUVehCenterPos.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    stEndGridIndex.iGridColNo =
        fabs(g_stMapCorditCmd.stLLHEndPos.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stEndGridIndex.iGridRowNo =
        fabs(g_stMapCorditCmd.stLLHEndPos.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    /*在地图上绘制起终点*/
    INT32 iEndPXPosX, iEndPXPosY; /*路径起终点在地图上的像素点位置*/
    iEndPXPosX = stEndGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iEndPXPosY = stEndGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    QPoint pxPathEnd = QPoint(iEndPXPosX, iEndPXPosY);

    QPainter painter(ui->labGridMap);
    painter.setPen(Qt::white);
    painter.setBrush(Qt::blue);
    painter.drawEllipse(pxPathEnd.x() - 3, pxPathEnd.y() - 3, 6, 6); /*终点绘制*/
    QString strText = "终点";
    painter.drawText(QPoint(pxPathEnd.x(), pxPathEnd.y()), strText);
  }
  else
  {
    /*未设定终点，不进行绘制*/
  }
}

void ParentDlg::DrawPath() /*绘制路径*/
{
  QPainter painter(ui->labGridMap);
  painter.setBrush(Qt::green);
  painter.setPen(Qt::green);
  if (0 < m_listTrackPathCmd.size())
  {
    for (INT32 i = 0; i < m_listTrackPathCmd.size(); i++)
    {
      GridMapIndexST stGridIndex;
      stGridIndex.iGridColNo =
          fabs(m_listTrackPathCmd.at(i).stENUPoint.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
      stGridIndex.iGridRowNo =
          fabs(m_listTrackPathCmd.at(i).stENUPoint.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
      INT32 iPXPosX, iPXPosY; /*路径起终点在地图上的像素点位置*/
      iPXPosX = stGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      iPXPosY = stGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      painter.drawEllipse(iPXPosX - 2, iPXPosY - 2, 4, 4); /*终点绘制*/
    }

    // 绘制左边界
    {
      QPen pen;
      pen.setWidthF(2);
      pen.setColor(Qt::red);
      painter.setBrush(Qt::red);
      painter.setPen(pen);
      GridMapIndexST stLineFirstIndex;
      stLineFirstIndex.iGridColNo =
          fabs(m_vectorTrackPathLeftBoundary.front().x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
      stLineFirstIndex.iGridRowNo =
          fabs(m_vectorTrackPathLeftBoundary.front().y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
      INT32 iPXFirstPosX, iPXFirstPosY;
      iPXFirstPosX = stLineFirstIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      iPXFirstPosY = stLineFirstIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      for (INT32 i = 1; i < m_vectorTrackPathLeftBoundary.size(); i++)
      {
        GridMapIndexST stLineSecondIndex;
        stLineSecondIndex.iGridColNo =
            fabs(m_vectorTrackPathLeftBoundary.at(i).x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
        stLineSecondIndex.iGridRowNo =
            fabs(m_vectorTrackPathLeftBoundary.at(i).y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
        INT32 iPXSecondPosX, iPXSecondPosY;
        iPXSecondPosX = stLineSecondIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
        iPXSecondPosY = stLineSecondIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
        painter.drawLine(iPXFirstPosX, iPXFirstPosY, iPXSecondPosX, iPXSecondPosY);

        iPXFirstPosX = iPXSecondPosX;
        iPXFirstPosY = iPXSecondPosY;
      }
    }

    // 绘制右边界
    {
      QPen pen;
      pen.setWidthF(2);
      pen.setColor(Qt::blue);
      painter.setBrush(Qt::blue);
      painter.setPen(pen);

      GridMapIndexST stLineFirstIndex;
      stLineFirstIndex.iGridColNo =
          fabs(m_vectorTrackPathRightBoundary.front().x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
      stLineFirstIndex.iGridRowNo =
          fabs(m_vectorTrackPathRightBoundary.front().y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
      INT32 iPXFirstPosX, iPXFirstPosY;
      iPXFirstPosX = stLineFirstIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      iPXFirstPosY = stLineFirstIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      for (INT32 i = 1; i < m_vectorTrackPathRightBoundary.size(); i++)
      {
        GridMapIndexST stLineSecondIndex;
        stLineSecondIndex.iGridColNo =
            fabs(m_vectorTrackPathRightBoundary.at(i).x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
        stLineSecondIndex.iGridRowNo =
            fabs(m_vectorTrackPathRightBoundary.at(i).y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
        INT32 iPXSecondPosX, iPXSecondPosY;
        iPXSecondPosX = stLineSecondIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
        iPXSecondPosY = stLineSecondIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
        painter.drawLine(iPXFirstPosX, iPXFirstPosY, iPXSecondPosX, iPXSecondPosY);

        iPXFirstPosX = iPXSecondPosX;
        iPXFirstPosY = iPXSecondPosY;
      }
    }
  }
}

void ParentDlg::DrawPathPrependPoint() /*绘制路径前置点*/
{
  // if (true == m_bMapInfoValid) /*地图信息有效，则进行绘制*/
  // {
  QPainter painter(ui->labGridMap);
  painter.setPen(Qt::white);

  GridMapIndexST stGridIndex;
  stGridIndex.iGridColNo = fabs(m_stENUPathPrependPt.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
  stGridIndex.iGridRowNo = fabs(m_stENUPathPrependPt.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;

  /*在地图上绘制起终点*/
  INT32 iPXPosX, iPXPosY; /*路径起终点在地图上的像素点位置*/
  iPXPosX = stGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
  iPXPosY = stGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

  painter.setBrush(Qt::red);
  painter.drawEllipse(iPXPosX - 3, iPXPosY - 3, 6, 6); /*终点绘制*/
  // QString strText = "前置点";
  // painter.drawText(QPoint(iPXPosX, iPXPosY), strText);
  QString strText = "index:" + QString::number(control_error_msg_.pp_look_ahead_point_index) +
                    " pre_dist:" + QString::number(control_error_msg_.pp_look_ahead_dist, 10, 2) +
                    " lat_e:" + QString::number(control_error_msg_.lat_error, 10, 2) +
                    " thro:" + QString::number(control_error_msg_.throttle_cmd, 10, 2) +
                    " deacc:" + QString::number(control_error_msg_.de_acc_cmd, 10, 2);
  painter.drawText(QPoint(iPXPosX, iPXPosY), strText);
  // }
  // else
  // {
  //   /*地图信息无效，则不绘制*/
  // }
}

void ParentDlg::DrawVehPos() /*绘制车辆当前位置*/
{
  if (true == m_bMapInfoValid) /*地图信息有效，则计算车辆在地图中的位置信息*/
  {
    QPainter painter(ui->labGridMap);
    painter.setPen(QPen(QBrush(Qt::white), 2));
    vehicle_motion_state_mutex_.lock();
    /*计算车辆前轮中心在labGridMap控件中的x和y像素位置*/
    GridMapIndexST stGridVehFrontWheelIndex;
    stGridVehFrontWheelIndex.iGridColNo =
        fabs(m_stENUVehFrontWheel.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stGridVehFrontWheelIndex.iGridRowNo =
        fabs(m_stENUVehFrontWheel.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    INT32 iPXVehFrontWheelPosX, iPXVehFrontWheelPosY;
    iPXVehFrontWheelPosX = stGridVehFrontWheelIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iPXVehFrontWheelPosY = stGridVehFrontWheelIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

    /*计算车辆后轮中心在labGridMap控件中的x和y像素位置*/
    GridMapIndexST stGridVehBackWheelIndex;
    stGridVehBackWheelIndex.iGridColNo =
        fabs(m_stENUVehBackWheel.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stGridVehBackWheelIndex.iGridRowNo =
        fabs(m_stENUVehBackWheel.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    INT32 iPXVehBackWheelPosX, iPXVehBackWheelPosY;
    iPXVehBackWheelPosX = stGridVehBackWheelIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iPXVehBackWheelPosY = stGridVehBackWheelIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    vehicle_motion_state_mutex_.unlock();

    painter.setBrush(Qt::black);
    painter.drawEllipse(iPXVehFrontWheelPosX - 3, iPXVehFrontWheelPosY - 3, 6, 6); /*车辆前轮中心点绘制*/
    painter.setBrush(Qt::yellow);
    painter.drawEllipse(iPXVehBackWheelPosX - 3, iPXVehBackWheelPosY - 3, 6, 6); /*车辆后轮中心点绘制*/

    painter.setBrush(Qt::red);
    painter.resetTransform(); /*重置变换*/
    painter.translate(iPXVehFrontWheelPosX - (iPXVehFrontWheelPosX - iPXVehBackWheelPosX) / 2,
                      iPXVehFrontWheelPosY - (iPXVehFrontWheelPosY - iPXVehBackWheelPosY) / 2); /*坐标原点定到矩形中心*/
    float fRotateAngle;
    if (m_fVehCourse <= 90)
    {
      fRotateAngle = m_fVehCourse - 90;
    }
    else if (m_fVehCourse <= 180)
    {
      fRotateAngle = m_fVehCourse - 90;
    }
    else if (m_fVehCourse <= 270)
    {
      fRotateAngle = m_fVehCourse - 90;
    }
    else
    {
      fRotateAngle = m_fVehCourse - 450;
    }
    painter.rotate(fRotateAngle); /*旋转坐标系*/
    if (1e-5 < m_fMapMeterPerPixel)
    {
      painter.fillRect(-VEHICLE_WHEEL_BASE / 2.0 / m_fMapMeterPerPixel, -VEHICLE_WIDTH / 2.0 / m_fMapMeterPerPixel,
                       VEHICLE_WHEEL_BASE / m_fMapMeterPerPixel, VEHICLE_WIDTH / m_fMapMeterPerPixel,
                       QBrush(QColor(146, 251, 91, 128)));

      /*左转为负，右转为正*/
      /*绘制车辆左前轮*/
      float fWheelR = 1.0; /*车轮半径，看起来效果明显*/
      INT32 iLFWheelX1 = VEHICLE_WHEEL_BASE / 2.0 / m_fMapMeterPerPixel +
                         fWheelR / m_fMapMeterPerPixel * cos((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iLFWheelY1 = -VEHICLE_WIDTH / 2.0 / m_fMapMeterPerPixel +
                         fWheelR / m_fMapMeterPerPixel * sin((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iLFWheelX2 = VEHICLE_WHEEL_BASE / 2.0 / m_fMapMeterPerPixel -
                         fWheelR / m_fMapMeterPerPixel * cos((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iLFWheelY2 = -VEHICLE_WIDTH / 2.0 / m_fMapMeterPerPixel -
                         fWheelR / m_fMapMeterPerPixel * sin((m_fVehTurnAngle)*DEGREE_RADIAN);
      painter.drawLine(QPoint(iLFWheelX1, iLFWheelY1), QPoint(iLFWheelX2, iLFWheelY2));
      /*绘制车辆右前轮*/
      INT32 iRFWheelX1 = VEHICLE_WHEEL_BASE / 2.0 / m_fMapMeterPerPixel +
                         fWheelR / m_fMapMeterPerPixel * cos((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iRFWheelY1 = VEHICLE_WIDTH / 2.0 / m_fMapMeterPerPixel +
                         fWheelR / m_fMapMeterPerPixel * sin((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iRFWheelX2 = VEHICLE_WHEEL_BASE / 2.0 / m_fMapMeterPerPixel -
                         fWheelR / m_fMapMeterPerPixel * cos((m_fVehTurnAngle)*DEGREE_RADIAN);
      INT32 iRFWheelY2 = VEHICLE_WIDTH / 2.0 / m_fMapMeterPerPixel -
                         fWheelR / m_fMapMeterPerPixel * sin((m_fVehTurnAngle)*DEGREE_RADIAN);
      painter.drawLine(QPoint(iRFWheelX1, iRFWheelY1), QPoint(iRFWheelX2, iRFWheelY2));
    }
    painter.resetTransform(); /*恢复translate和rotate对painter作的修改*/

    // QString strText = "车辆点";
    // painter.drawText(QPoint(iPXVehBackWheelPosX, iPXVehBackWheelPosY), strText);
    painter.drawText(QPoint(iPXVehBackWheelPosX - 30, iPXVehBackWheelPosY - 20), m_strChasCmdText);

    painter.drawText(QPoint(iPXVehBackWheelPosX - 30, iPXVehBackWheelPosY + 40), m_sEllipseTime);

    // 绘制车辆后轴坐标点
    QString strVehPos = QString("(%1,%2)").arg(m_stENUVehBackWheel.x, 0, 'f', 2).arg(m_stENUVehBackWheel.y, 0, 'f', 2);
    painter.drawText(QPoint(iPXVehBackWheelPosX - 30, iPXVehBackWheelPosY + 20), strVehPos);

    painter.setPen(QPen(QBrush(Qt::red), 2)); /*绘制车辆运动轨迹*/
    for (INT32 i = 0; i < m_listVehPos.size() - 1; i++)
    {
      GridMapIndexST stGridVehBackWheelIndex;
      stGridVehBackWheelIndex.iGridColNo =
          fabs(m_listVehPos.at(i).x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
      stGridVehBackWheelIndex.iGridRowNo =
          fabs(m_listVehPos.at(i).y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
      INT32 iPXVehBackWheelPosX, iPXVehBackWheelPosY;
      iPXVehBackWheelPosX = stGridVehBackWheelIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      iPXVehBackWheelPosY = stGridVehBackWheelIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

      GridMapIndexST stLstGridVehBackWheelIndex;
      stLstGridVehBackWheelIndex.iGridColNo =
          fabs(m_listVehPos.at(i + 1).x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
      stLstGridVehBackWheelIndex.iGridRowNo =
          fabs(m_listVehPos.at(i + 1).y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
      INT32 iLstPXVehBackWheelPosX, iLstPXVehBackWheelPosY;
      iLstPXVehBackWheelPosX = stLstGridVehBackWheelIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
      iLstPXVehBackWheelPosY = stLstGridVehBackWheelIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

      painter.drawLine(QPoint(iPXVehBackWheelPosX, iPXVehBackWheelPosY),
                       QPoint(iLstPXVehBackWheelPosX, iLstPXVehBackWheelPosY));
    }
    if (is_lock_third_person_view_)
    {
      on_btnGoToVehPos_clicked();
    }
  }
  else
  {
    /*地图信息无效，则不知道车辆点在地图的什么位置，无法绘制*/
  }
}

void ParentDlg::on_btnGridMapFileOpen_clicked() /*地图打开按钮单击槽函数*/
{
  MsgTableDisp("操作输入", "地图文件打开");

  QString strFileName = "D:/CF/2 MapFile";
  QString strMapName = QFileDialog::getOpenFileName(this, tr("地图文件打开"), strFileName,
                                                    tr("Map Files(*.txt)")); /*返回绝对路径+文件名*/
  /*获取文件名*/
  INT32 pos = strMapName.lastIndexOf('/');
  QString strfileName = strMapName.right(strMapName.size() - pos - 1);
  /*获取文件路径*/
  QString strfilePath = strMapName.left(pos + 1);

  if (false == strMapName.isEmpty()) /*打开了文件*/
  {
    m_bMapInfoValid = false; /*地图信息有效标志，每次点击打开文件按钮，将其置为false*/
    ;
    emit ReadMapFile(strfilePath, strfileName);

    ui->btnPathPlan->setText("路径规划"); /*重新显示路径规划按钮*/
    m_listVehPos.clear();                 /*清除车辆位置信息*/
  }
  else
  {
    MsgTableDisp("操作输入", "取消了地图文件打开操作");
  }
}

void ParentDlg::on_btnPathPlan_clicked() /*路径规划按钮点击槽函数*/
{
  if (0 == ui->tableTrackPathInfo->rowCount())
  {
    QMessageBox::information(Q_NULLPTR, "提示", "路径为空，首先打开路径文件");
    return;
  }

  QTableWidgetItem *itemStartX = ui->tableTrackPathInfo->item(0, 1);
  double fStartX = itemStartX->text().toDouble();
  QTableWidgetItem *itemStartY = ui->tableTrackPathInfo->item(0, 2);
  double fStartY = itemStartY->text().toDouble();

  int iLastRowIndex = ui->tableTrackPathInfo->rowCount() - 1;
  QTableWidgetItem *itemEndX = ui->tableTrackPathInfo->item(iLastRowIndex, 1);
  double fEndX = itemEndX->text().toDouble();
  QTableWidgetItem *itemEndY = ui->tableTrackPathInfo->item(iLastRowIndex, 2);
  double fEndY = itemEndY->text().toDouble();

  double dx2 = (fStartX - fEndX) * (fStartX - fEndX);
  double dy2 = (fStartY - fEndY) * (fStartY - fEndY);
  double dist = sqrt(dx2 + dy2);
  QString text = tr("路径长度为: ") + QString::number(dist) + " m";
  QMessageBox::information(Q_NULLPTR, "提示", text);

  for (int i = 0; i < ui->tableTrackPathInfo->rowCount(); ++i)
  {
    QTableWidgetItem *newItem = new QTableWidgetItem(tr("%1").arg(0)); // 列表中单位为s
    ui->tableTrackPathInfo->setItem(i, 3, newItem);
  }
  ui->tableTrackPathInfo->update();
  //    MsgTableDisp("操作输入", "本软件中无功能，仅先保留接口");
  //    QMessageBox::information(Q_NULLPTR, "提示", "本软件中无功能，仅先保留接口");
}

void ParentDlg::on_btnMapAndPathBind_clicked() /*地图坐标系和路径装订按钮单击槽函数*/
{
  if (m_listTrackPathCmd.empty())
  {
    QMessageBox::information(Q_NULLPTR, "提示", "无有效路径，不支持进行路径装订");
    MsgTableDisp("控制流程", "无有效路径，不支持进行路径装订");
    return;
  }
  MsgTableDisp("操作输入", "地图坐标系和路径装订");
  emit sendReferencePath(m_listTrackPathCmd);

  //  if (CheckStartCommunicationStateEnable())
  //  {
  //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
  //    return;
  //  }

  //  MsgTableDisp("操作输入", "地图坐标系和路径装订");

  //  if (true == g_bSetMapOrigin)
  //  {
  //    if ((fabs(g_stMapCorditCmd.stLLHEndPos.x) < 1e-2) && (fabs(g_stMapCorditCmd.stLLHEndPos.y) < 1e-2))
  //    {
  //      QMessageBox::information(Q_NULLPTR, "提示", "未完成终点设置，不支持进行坐标系装订");
  //      MsgTableDisp("控制流程", "未完成终点设置，不支持进行坐标系装订");
  //      return;
  //    }
  //    else
  //    {
  //      if ((fabs(m_stENUVehCenterPos.x - g_stMapCorditCmd.stLLHEndPos.x) < 1) &&
  //          (fabs(m_stENUVehCenterPos.y - g_stMapCorditCmd.stLLHEndPos.y) < 1))
  //      {
  //        QMessageBox::information(Q_NULLPTR, "提示", "距离终点距离太近，不支持进行坐标系装订");
  //        MsgTableDisp("控制流程", "距离终点距离太近，不支持进行坐标系装订");
  //        return;
  //      }
  //      else
  //      {
  //        if (0 == m_ucMoveCtrlFlag) /*未处于运动控制中*/
  //        {
  //          SendMapCordtBindMsg(); /*发送地图坐标系装订报文*/
  //        }
  //        else
  //        {
  //          QMessageBox::information(Q_NULLPTR, "提示", "底盘运动控制中，不支持进行坐标系装订");
  //          MsgTableDisp("控制流程", "底盘运动控制中，不支持进行坐标系装订");
  //          return;
  //        }
  //      }
  //    }
  //  }
  //  else
  //  {
  //    QMessageBox::information(Q_NULLPTR, "提示", "未完成地图原点设置，不支持进行坐标系装订");
  //    MsgTableDisp("控制流程", "未完成地图原点设置，不支持进行坐标系装订");
  //    return;
  //  }

  //  if (0 < m_listTrackPathCmd.size())
  //  {
  //    SendPathBindMsg(); /*发送路径装订报文*/
  //  }
  //  else
  //  {
  //    QMessageBox::information(Q_NULLPTR, "提示", "无有效路径，不支持进行路径装订");
  //    MsgTableDisp("控制流程", "无有效路径，不支持进行路径装订");
  //  }
}

void ParentDlg::on_btnMapAndPathAsk_clicked() /*地图坐标系和路径查询按钮单击槽函数*/
{
  emit sendReferencePathCheckCmd();
}

void ParentDlg::onReferencePathFeedback(QList<SmoothPathPointST> msg)
{
  // std::cout << "parent dlg " << msg.size() << std::endl;
  m_listTrackPathBindRslt = msg;
  PathBindRsltDisp();
}
// /*打包并且发送跟踪路径数据*/
// void ParentDlg::FoldAndSendPathBindMsg(UINT8 ucFrameMultiFlag, UINT16 usFrameNum, UINT32 uiTotalByteNum,
//                                        UINT16 usFrameNo, UINT16 usFrameDataBytes, QByteArray arrFrameData)
// {
//   //  if (0xFFFFFFFF <= m_uiSendPathBindMsgNum) /*发送报文数*/
//   //  {
//   //    m_uiSendPathBindMsgNum = 1;
//   //  }
//   //  else
//   //  {
//   //    m_uiSendPathBindMsgNum++;
//   //  }
//   ////  ui->labFP2VCPathBindMsgNum->setText(QString("%1").arg(m_uiSendPathBindMsgNum));

//   //  FP2VCPathBindMsgST stFP2VCMsg;
//   //  memset(&stFP2VCMsg, 0, sizeof(FP2VCPathBindMsgST));

//   //  stFP2VCMsg.stNetHeader.usMsgType = FP2VC_MOVEPATH_BIND_MSG;
//   //  stFP2VCMsg.stNetHeader.uiSrcIP = QHostAddress(FIGHT_OPT_IP).toIPv4Address();
//   //  stFP2VCMsg.stNetHeader.uiDestIP = QHostAddress(VEHICLE_CTRL_IP).toIPv4Address();
//   //  stFP2VCMsg.stNetHeader.usMsgLen = sizeof(FP2VCPathBindMsgST); /*报文长度*/
//   //  stFP2VCMsg.stNetHeader.usAck = NET_MSG_NOACK;

//   //  stFP2VCMsg.ucFrameMultiFlag = ucFrameMultiFlag;
//   //  stFP2VCMsg.usFrameNum = usFrameNum;                                   /*数据包数*/
//   //  stFP2VCMsg.uiTotalByteNum = uiTotalByteNum;                           /*装订数据总字节数*/
//   //  stFP2VCMsg.usFrameNo = usFrameNo;                                     /*数据包序号*/
//   //  stFP2VCMsg.usFrameDataBytes = usFrameDataBytes;                       /*当前包内装订数据总字节数*/
//   //  memcpy(stFP2VCMsg.stPathData, arrFrameData.data(), usFrameDataBytes); /*拷贝路径数据*/

//   //  QByteArray arrMsgData; /*报文数据*/
//   //  arrMsgData.clear();
//   //  // arrMsgData.append((char *)&stFP2VCMsg, stFP2VCMsg.stNetHeader.usMsgLen-sizeof(NetEndST));
//   //  arrMsgData.append((char*)&(stFP2VCMsg), sizeof(FP2VCPathBindMsgST));

//   //  emit SendNetMsg(arrMsgData, stFP2VCMsg.stNetHeader.usMsgLen, QHostAddress(VEHICLE_CTRL_IP),
//   //                  VEHICLE_CTRL_PORT); /*发送以太网报文*/
// }

void ParentDlg::on_btnPathFileOpen_clicked() /*路径文件打开按钮单击槽函数*/
{
  MsgTableDisp("操作输入", "路径文件打开");

  QString strMapName = QFileDialog::getOpenFileName(this, tr("打开文件"), "D:/CF/3 PathFile",
                                                    tr("Path Files(*.txt)")); /*返回绝对路径+文件名*/
  /*获取文件名*/
  INT32 pos = strMapName.lastIndexOf('/');
  QString strfileName = strMapName.right(strMapName.size() - pos - 1);
  /*获取文件路径*/
  QString strfilePath = strMapName.left(pos + 1);

  if (false == strMapName.isEmpty()) /*打开了文件*/
  {
    QFile objPathFile;
    objPathFile.setFileName(strMapName); /*设置文件名称*/
    if (false == objPathFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      MsgTableDisp("控制流程", "路径文件打开失败");
    }
    else
    {
      m_listTrackPathCmd.clear(); /*清空跟踪路径命令列表*/
      m_PNC2PADPath.clear();
      QByteArray array; /*读取信息暂存变量*/
      array.clear();
      while (false == objPathFile.atEnd()) /*未到文件尾*/
      {
        // array = objPathFile.readLine(); /*读取一行*/
        // QString str;
        // str.prepend(array);
        // if (true == str.contains("原点"))
        // {
        //   INT32 iPos[21] = { 0 };
        //   INT32 iPosTemp, iCnt = 0;
        //   while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
        //   {
        //     iPos[iCnt] = iPosTemp;
        //     iCnt++; /*查找到TAB键的次数*/
        //     if (iCnt > 20)
        //     {
        //       break;
        //     }
        //   }

        //   QString strLongOrigin = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1); /*取经度值*/
        //   QString strLatOrigin = str.mid(iPos[3] + 1, iPos[4] - iPos[3] - 1);  /*取纬度值*/
        //   QString strHeightOrigin = str.right(str.length() - iPos[5] - 1);     /*取高度值*/
        //   strHeightOrigin = strHeightOrigin.left(strHeightOrigin.length() - 1);

        //   /*将原点经纬度信息显示到界面中*/
        //   ui->editOriginLongitude->setText(strLongOrigin);
        //   ui->editOriginLatitude->setText(strLatOrigin);
        //   ui->editOriginHeight->setText(strHeightOrigin);
        // }
        // else if (true == str.contains("路径终点"))
        // {
        //   INT32 iPos[21] = { 0 };
        //   INT32 iPosTemp, iCnt = 0;
        //   while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
        //   {
        //     iPos[iCnt] = iPosTemp;
        //     iCnt++; /*查找到TAB键的次数*/
        //     if (iCnt > 20)
        //     {
        //       break;
        //     }
        //   }

        //   QString strLLHEndPosX = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1);
        //   QString strLLHEndPosY = str.right(str.length() - iPos[3] - 1);

        //   ui->editPathENUEndPointX->setText(strLLHEndPosX);
        //   ui->editPathENUEndPointY->setText(strLLHEndPosY);
        // }
        // else if (true == str.contains("路径点"))
        // {
        //   INT32 iPos[21] = { 0 };
        //   INT32 iPosTemp, iCnt = 0;
        //   while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
        //   {
        //     iPos[iCnt] = iPosTemp;
        //     iCnt++; /*查找到TAB键的次数*/
        //     if (iCnt > 20)
        //     {
        //       break;
        //     }
        //   }

        //   QString strPathENUX = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1);
        //   QString strPathENUY = str.mid(iPos[3] + 1, iPos[4] - iPos[3] - 1);
        //   QString strAngle2Y = str.mid(iPos[5] + 1, iPos[6] - iPos[5] - 1);
        //   QString strCurve = str.mid(iPos[7] + 1, iPos[8] - iPos[7] - 1);
        //   QString strCurveDiff = str.right(str.length() - iPos[9] - 1);

        //   //          QString strAngle2Y = "0";
        //   //          QString strCurve = "0";
        //   //          QString strCurveDiff = "0";
        //   ENUCorST stPathENU;
        //   memset(&stPathENU, 0, sizeof(ENUCorST));
        //   stPathENU.x = strPathENUX.toDouble();
        //   stPathENU.y = strPathENUY.toDouble();

        //   /*增加存储平滑后的路径信息数据*/
        //   SmoothPathPointST stSmoothPathPoint;
        //   memset(&stSmoothPathPoint, 0, sizeof(SmoothPathPointST));
        //   stSmoothPathPoint.stENUPoint = stPathENU;
        //   stSmoothPathPoint.fAngle2X_Rad = strAngle2Y.toDouble();
        //   stSmoothPathPoint.fCurve = strCurve.toDouble();
        //   stSmoothPathPoint.fCurveDiff = strCurveDiff.toDouble();
        //   m_listTrackPathCmd.append(stSmoothPathPoint);

        //   GlobalPositionST stGlobalPoint;
        //   stGlobalPoint.bIsSouth = true;
        //   stGlobalPoint.fX = stPathENU.x;
        //   stGlobalPoint.fY = stPathENU.y;
        //   stGlobalPoint.fZ = 0;
        //   m_PNC2PADPath.push_back(stGlobalPoint);
        // }
        // else
        // {
        //   /*不处理*/
        // }
      }
      objPathFile.close();

      GenerateRoadBoundary(); // 生成道路边界点 qixianyu 20220124

      MsgTableDisp("控制流程", "路径文件打开成功");
      TrackPathDisp();          /*需要跟踪的路径信息显示*/
      ui->labGridMap->update(); /*重绘地图区域*/

      /*测试*/
      //      SendRouteAndLocalPathMsg(PATH_TYPE_LOCAL_PLAN, m_PNC2PADPath);
      //      pnc2pad_move_data_mutex_.lock();
      //      memset(&m_stPNC2PadMoveDataMsg, 0, sizeof(PNC2PadMoveDataMsgST));

      //      m_stPNC2PadMoveDataMsg.stMoveEvaluateData.fPathHorizErr = 1;
      //      m_stPNC2PadMoveDataMsg.stMoveCaltData.stMoveCaltData.fFrontAxleAngle = 10;
      //      m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fX = m_listTrackPathCmd.at(5).stENUPoint.x + 2;
      //      m_stPNC2PadMoveDataMsg.stMoveCaltData.stGlobalPostion.fY = m_listTrackPathCmd.at(5).stENUPoint.y + 2;
      //      SendPNCMoveDataMsg(m_stPNC2PadMoveDataMsg);
      //      pnc2pad_move_data_mutex_.unlock();
    }
  }
  else
  {
    MsgTableDisp("操作输入", "取消了路径文件打开操作");
  }
}

void ParentDlg::GenerateRoadBoundary()
{
  if (m_listTrackPathCmd.size() < 2) // 路径少于两个点，计算不了角度，直接返回
  {
    return;
  }

  m_vectorTrackPathLeftBoundary.clear();
  m_vectorTrackPathRightBoundary.clear();

  double angle = 0;
  ENUCorST stLeftBoudaryPoint;
  memset(&stLeftBoudaryPoint, 0, sizeof(ENUCorST));
  ENUCorST stRightBoundaryPoint;
  memset(&stRightBoundaryPoint, 0, sizeof(ENUCorST));

  for (int i = 0; i < m_listTrackPathCmd.size() - 1; ++i)
  {
    double x = m_listTrackPathCmd.at(i).stENUPoint.x;
    double y = m_listTrackPathCmd.at(i).stENUPoint.y;
    double dx = m_listTrackPathCmd.at(i + 1).stENUPoint.x - x;
    double dy = m_listTrackPathCmd.at(i + 1).stENUPoint.y - y;

    angle = atan2(dy, dx);
    stLeftBoudaryPoint.x = x + cos(angle + M_PI / 2) * ELECTRIC_FENCE / 2.0;
    stLeftBoudaryPoint.y = y + sin(angle + M_PI / 2) * ELECTRIC_FENCE / 2.0;

    stRightBoundaryPoint.x = x + cos(angle - M_PI / 2) * ELECTRIC_FENCE / 2.0;
    stRightBoundaryPoint.y = y + sin(angle - M_PI / 2) * ELECTRIC_FENCE / 2.0;

    m_vectorTrackPathLeftBoundary.append(stLeftBoudaryPoint);
    m_vectorTrackPathRightBoundary.append(stRightBoundaryPoint);
  }

  // 添加最后一个点的边界
  stLeftBoudaryPoint.x = m_listTrackPathCmd.last().stENUPoint.x + cos(angle + M_PI / 2) * ELECTRIC_FENCE / 2.0;
  stLeftBoudaryPoint.y = m_listTrackPathCmd.last().stENUPoint.y + sin(angle + M_PI / 2) * ELECTRIC_FENCE / 2.0;
  stRightBoundaryPoint.x = m_listTrackPathCmd.last().stENUPoint.x + cos(angle - M_PI / 2) * ELECTRIC_FENCE / 2.0;
  stRightBoundaryPoint.y = m_listTrackPathCmd.last().stENUPoint.y + sin(angle - M_PI / 2) * ELECTRIC_FENCE / 2.0;

  m_vectorTrackPathLeftBoundary.append(stLeftBoudaryPoint);
  m_vectorTrackPathRightBoundary.append(stRightBoundaryPoint);
}

bool ParentDlg::CheckStartCommunicationStateEnable()
{
  bool ret = ui->btnStartCommunication->isEnabled();
  return ret;
}

void ParentDlg::on_btnPosMeasure_clicked() /*位置量测按钮点击槽函数*/
{
  //    m_dlgPosMeasure.setGeometry(50, 50, 1024, 768);
  m_dlgPosMeasure.init();
  m_dlgPosMeasure.show(); /*显示路径结果窗口*/
  m_dlgPosMeasure.raise();
}

void ParentDlg::on_sliderMapZoom_valueChanged(int value) /*地图缩放处理*/
{
  m_fPixelPerGrid = m_fMapZoomArray[value - 1];
  m_fMapMeterPerPixel = m_pAStar->m_fResolution / m_fMapZoomArray[value - 1]; /*当前地图缩放级数,即1个像素等于多少米*/
  ui->labMapMeterPerPixel->setText(QString("%1m").arg(m_fMapMeterPerPixel, 0, 'f', 2));
  ui->labMapPixelPerGrid->setText(QString("%1P/G").arg(m_fPixelPerGrid, 0, 'f', 2));
  m_bMapInfoRepaint = true;
  ui->labGridMap->update(); /*地图缩放，全体重绘*/
}

// void ParentDlg::on_btnEngineStart_clicked() /*发动机启动按钮单击槽函数*/
//{
//   //  if (CheckStartCommunicationStateEnable())
//   //  {
//   //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
//   //    return;
//   //  }

//  MsgTableDisp("操作输入", "发动机启动");

//  emit sendEngineCmd(true);
//  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/

//  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

//  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_ENGINE_START_CTRL; /*发动机启动控制*/

//  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
//}

// void ParentDlg::on_btnEngineStop_clicked() /*发动机停止按钮单击槽函数*/
//{
//   // if (CheckStartCommunicationStateEnable())
//   // {
//   //   QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
//   //   return;
//   // }

//  MsgTableDisp("操作输入", "发动机停止");
//  emit sendEngineCmd(false);
//  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
//  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

//  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_ENGINE_END_CTRL; /*发动机停止控制*/

//  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
//}

void ParentDlg::on_btnGearSet_clicked() /*档位设置按钮单击槽函数*/
{
  //  if (CheckStartCommunicationStateEnable())
  //  {
  //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
  //    return;
  //  }

  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_GEAR; /*档位设置*/

  int index = ui->comBoxGear->currentIndex();

  if (1 == index)
  {
    MsgTableDisp("操作输入", "D档位设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_D;
    emit sendTargetGearCmd(1);
  }
  else if (2 == index)
  {
    MsgTableDisp("操作输入", "R档位设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_R;
    emit sendTargetGearCmd(7);
  }
  else if (0 == index)
  {
    MsgTableDisp("操作输入", "N档位设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_N;
    emit sendTargetGearCmd(0);
  }
  else if (3 == index)
  {
    MsgTableDisp("操作输入", "P档位设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_N;
    emit sendTargetGearCmd(2);
  }
  else
  {
    MsgTableDisp("操作输入", "无效档位设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucGear = 0;
  }

  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
}

// void ParentDlg::on_btnParkCtrl_clicked() /*驻车制动按钮单击槽函数*/
//{
//   //  if (CheckStartCommunicationStateEnable())
//   //  {
//   //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
//   //    return;
//   //  }

//  MsgTableDisp("操作输入", "驻车制动控制");
//  emit sendParkBrakeCmd(true);
//}

// void ParentDlg::on_btnReleasePark_clicked() /*解除驻车按钮单击槽函数*/
//{
//   //  if (CheckStartCommunicationStateEnable())
//   //  {
//   //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
//   //    return;
//   //  }

//  MsgTableDisp("操作输入", "解除驻车控制");

//  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
//  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

//  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_PARKING_CTRL; /*驻车制动控制*/
//  //  stFP2VCMsg.stVC2ChasCmd.ucPark = CHAS_RELEASE_PARK;        /*解除驻车标识*/

//  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
//  emit sendParkBrakeCmd(false);
//}

void ParentDlg::on_btnChasWorkModeSet_clicked() /*底盘被控模式按钮单击槽函数*/
{
  //  if (CheckStartCommunicationStateEnable())
  //  {
  //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
  //    return;
  //  }

  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_WORK_MODE_SET; /*被控模式设置*/
  int index = ui->comBoxDrivingMode->currentIndex();

  if (0 == index) //
  {
    MsgTableDisp("操作输入", "人控模式设置");
    emit sendDriveModeCmd(0);
    //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = CHAS_WORK_MODE_MAN;
  }
  else if (1 == index)
  {
    MsgTableDisp("操作输入", "无人驾驶模式模式设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = CHAS_WORK_MODE_AUTO_DRIVE;
    emit sendDriveModeCmd(1);
  }
  else if (2 == index)
  {
    MsgTableDisp("操作输入", "遥控模式模式设置");
    emit sendDriveModeCmd(2);
  }
  else if (3 == index)
  {
    MsgTableDisp("操作输入", "人工反向驾驶模式设置");
    //    stFP2VCMsg.stVC2ChasCmd.ucWorkMode = 0;
    emit sendDriveModeCmd(3);
  }
  else if (4 == index)
  {
    MsgTableDisp("操作输入", "横向控制模式设置");

    emit sendDriveModeCmd(4);
  }
  else if (5 == index)
  {
    MsgTableDisp("操作输入", "纵向控制模式设置");

    emit sendDriveModeCmd(5);
  }
  else
  {
    MsgTableDisp("操作输入", "无效模式设置");
  }
}

void ParentDlg::on_btnMoveStart_clicked() /*运动开始按钮单击槽函数*/
{
  // if (is_move_forward_)
  // {
  //   emit sendMotionStart(2);
  //   MsgTableDisp("操作输入", "路径跟踪运动开始控制");
  // }
  // else
  // {
  emit sendMotionStart(1);
  MsgTableDisp("操作输入", "路径跟踪运动开始控制");
  // }
  emit WriteRecord("路径跟踪运动开始控制", RECORD_DATA);

  //  if (CheckStartCommunicationStateEnable())
  //  {
  //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
  //    return;
  //  }

  //  FP2VCChasMoveCtrlMsgST stFP2VCMsg; /*底盘运动控制报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasMoveCtrlMsgST));

  //  if (true == ui->radioMoveCourseKeep->isChecked())
  //  {
  //    MsgTableDisp("操作输入", "暂不支持航向不变运动控制");
  //    return; /*暂时不支持航向不变控制*/
  //  }
  //  else if (true == ui->radioMovePathTrack->isChecked())
  //  {
  //    MsgTableDisp("操作输入", "路径跟踪运动开始控制");
  //    if (false == m_bTrackPathBindRsltValid)
  //    {
  //      MsgTableDisp("控制流程", "请首先确认综控路径有效性");
  //      return;
  //    }
  //    stFP2VCMsg.ucMoveCtrlMode = 0x2; /*路径跟踪*/
  //  }
  //  else if (true == ui->radioMoveByMan->isChecked())
  //  {
  //    MsgTableDisp("操作输入", "遥控运动开始控制");
  //    stFP2VCMsg.ucMoveCtrlMode = 0x3;          /*遥控控制*/
  //    if (false == m_timerMoveCtrl->isActive()) /*运动控制定时器未开启*/
  //    {
  //      m_timerMoveCtrl->start(100);
  //      MsgTableDisp("控制流程", "开启遥控运动控制定时器");
  //    }
  //  }
  //  else
  //  {
  //    MsgTableDisp("操作输入", "无效模式运动开始控制");
  //    stFP2VCMsg.ucMoveCtrlMode = 0;
  //  }

  //  stFP2VCMsg.ucMoveFlag = 0x55; /*运动开始*/ stFP2VCMsg.usVehSpeed =
  //  ui->comboCourseVehSpeed->currentText().toUShort() * 1000.0 / 3600.0 * 10.0; /*巡航速度*/

  //  SendMoveCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
  //  m_bMoveStart = true;
  //  m_bMoveStartFirstComputeCurrentPoint = true;
  //  m_iCurrenPointIndex = 0;
  //  m_iLastCurrentPointIndex = 0;
}

void ParentDlg::on_btnMoveStop_clicked() /*运动停止按钮单击槽函数*/
{
  emit sendMotionStart(false);
  MsgTableDisp("操作输入", "路径跟踪模式运动停止控制");
}

void ParentDlg::on_btnParkBrak_clicked() /*紧急制动按钮单击槽函数*/
{
  //  if (CheckStartCommunicationStateEnable())
  //  {
  //    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("请首先与下位机建立通信"));
  //    return;
  //  }

  MsgTableDisp("操作输入", "紧急制动控制");

  emit sendEmcyBrakeCmd(true);
  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_BRAKING_CTRL; /*紧急制动控制*/

  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
}

// void ParentDlg::on_btnCorditBindAreaDispSwitch_clicked() /*坐标系装订编辑区和显示区相互切换*/
//{
//   if (false == m_bCorditBindAreaDisp)
//   {
//     ui->grpMapCorditBindDisp->hide();
//     ui->grpMapCorditBindEdit->show();
//   }
//   else
//   {
//     ui->grpMapCorditBindEdit->hide();
//     ui->grpMapCorditBindDisp->show();
//   }
//   m_bCorditBindAreaDisp = !m_bCorditBindAreaDisp;
// }

void ParentDlg::on_btnMapCordtEnter_clicked() /*坐标系数据确认输入按钮单击槽函数*/
{
  ui->labOriginHeightBind->setText(ui->editOriginHeight->text());
  ui->labOriginLatitudeBind->setText(ui->editOriginLatitude->text());
  ui->labOriginLongitudeBind->setText(ui->editOriginLongitude->text());

  ui->labPathEndENUXBind->setText(ui->editPathENUEndPointX->text());
  ui->labPathEndENUYBind->setText(ui->editPathENUEndPointY->text());

  /*确认输入后，把地图坐标系数据确定下来*/
  // g_stMapCorditCmd.stLLHEndPos.x = ui->editPathENUEndPointX->text().toFloat();
  // g_stMapCorditCmd.stLLHEndPos.y = ui->editPathENUEndPointY->text().toFloat();

  // g_stMapCorditCmd.stLLHOriginPos.fLatitude = ui->editOriginLatitude->text().toDouble();
  // g_stMapCorditCmd.stLLHOriginPos.fLongitude = ui->editOriginLongitude->text().toDouble();
  // g_stMapCorditCmd.stLLHOriginPos.fHeight = ui->editOriginHeight->text().toFloat();
  g_bSetMapOrigin = true; /*置已设定地图原点标志*/

  //  on_btnCorditBindAreaDispSwitch_clicked();
}

// void ParentDlg::on_btnTurnAngleCmdReset_clicked() /*转向角滑动条清零*/
//{
////  ui->sliderTurnAngleCmd->setValue(0); /*转向角滑动条清零*/
//}

void ParentDlg::onTrackPathTableVSliderMove(INT32 iValue) /*规划路径列表竖滚条滚动槽函数*/
{
  ui->tableTrackPathInfo->verticalScrollBar()->setValue(iValue);
}

void ParentDlg::onPathBindRsltTableVSliderMove(INT32 iValue) /*路径装订结果反馈列表竖滚条滚动槽函数*/
{
  ui->tablePathBindRsltInfo->verticalScrollBar()->setValue(iValue);
}

void ParentDlg::timerMoveCtrlFunc() /*运动控制定时器处理*/
{
  //    if(100 < m_ucTimerCnt)//计时器计数
  //    {
  //        emit TextToVoice("车辆运动控制中");
  //        m_ucTimerCnt = 0;
  //    }
  //    else
  //    {
  //        m_ucTimerCnt++;
  //    }

  //  FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
  //  memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));

  //  stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_MOVE;                   /*底盘运动控制*/
  //  stFP2VCMsg.stVC2ChasCmd.usVehSpeed = ui->sliderVehSpeedCmd->value();
  //  /*滑动条直接是乘10倍的效果，因此此处直接赋值*/ stFP2VCMsg.stVC2ChasCmd.sTurnAngle =
  //      (ui->sliderTurnAngleCmd->value()) * 10;
  //      /*滑动条直接是乘10倍的效果，因此此处只要再乘10，就相当于原值乘100*/
  //  stFP2VCMsg.stVC2ChasCmd.ucTurnAngleSpeed =
  //      ui->sliderTurnAngleSpeedCmd->value(); /*滑动条直接是乘10倍的效果，因此此处直接赋值*/

  //  SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
}

// void ParentDlg::timerTaskPointFunc() /*任务下发定时器处理*/
// {
//   taskPoints_msgs::taskPoints task_pub;
//   task_pub = taskpointbridgemsg;
//   emit sendTaskPoints(task_pub);
// }

void ParentDlg::on_btnGoToVehPos_clicked() /*将车辆位置挪至地图的中心*/
{
  QSize MapSize = ui->scrollArea->size(); /*首先获取滚动区域所占大小*/
  qDebug() << "MapSize.width()" << MapSize.width() << " "
           << "MapSize.height()" << MapSize.height();

  if (true == m_bMapInfoValid) /*地图信息有效*/
  {
    /*计算车辆后轮中心在labGridMap控件中的x和y像素位置*/
    GridMapIndexST stGridVehCenterPosIndex;
    stGridVehCenterPosIndex.iGridColNo =
        fabs(m_stENUVehCenterPos.x - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
    stGridVehCenterPosIndex.iGridRowNo =
        fabs(m_stENUVehCenterPos.y - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
    INT32 iPXVehCenterPosPosX, iPXVehCenterPosPosY; /*质心在Map中的像素位置*/
    iPXVehCenterPosPosX = stGridVehCenterPosIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
    iPXVehCenterPosPosY = stGridVehCenterPosIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;

    ui->scrollArea->horizontalScrollBar()->setValue(iPXVehCenterPosPosX - MapSize.width() / 2);
    ui->scrollArea->verticalScrollBar()->setValue(iPXVehCenterPosPosY - MapSize.height() / 2);
  }
}

void ParentDlg::on_btnTabelInfoClear_clicked() /*信息显示列表清空*/
{
  ui->tableMsgDisp->clearContents();
  ui->tableMsgDisp->setRowCount(0);
  ui->tableMsgDisp->scrollToTop();
}

void ParentDlg::TextToVoice(QString strVoice) /*文本转声音*/
{
  //    m_pSpeech->stop();
  //    m_pSpeech->say(strVoice);
}

void ParentDlg::on_btnClearPath_clicked() /*车辆轨迹列表清空*/
{
  m_listVehPos.clear();
  m_bPointSelect = false;
}

void ParentDlg::on_btConfigDlg_clicked() /*配置文件操作对话框*/
{
  m_dlgConfig.init();
  m_dlgConfig.show();
  m_dlgConfig.raise();
}

void ParentDlg::on_btnMapFileGeneration_clicked() /*地图文件生成*/
{
  double fLat = ui->labINSLatitude->text().toDouble();
  double fLon = ui->labINSLongitude->text().toDouble();
  double fHeight = ui->labINSHeight->text().toDouble();
  m_dlgMapFileGenration.setMapOrigin(fLat, fLon, fHeight);
  m_dlgMapFileGenration.show();
  m_dlgMapFileGenration.raise();
}

void ParentDlg::DrawSelectPoint()
{
  QPainter painter(ui->labGridMap);
  painter.setBrush(Qt::blue);
  painter.setPen(Qt::blue);

  GridMapIndexST stGridIndex;
  stGridIndex.iGridColNo = fabs(m_fSelectPointX - m_pAStar->m_fMapENU_XMin) / m_pAStar->m_fResolution;
  stGridIndex.iGridRowNo = fabs(m_fSelectPointY - m_pAStar->m_fMapENU_YMax) / m_pAStar->m_fResolution;
  INT32 iPXPosX, iPXPosY;
  iPXPosX = stGridIndex.iGridColNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
  iPXPosY = stGridIndex.iGridRowNo * m_fPixelPerGrid + m_fPixelPerGrid / 2;
  painter.drawEllipse(iPXPosX - 2, iPXPosY - 2, 4, 4);
}

/*单击在界面中绘制被选点 qixianyu*/
void ParentDlg::on_tableTrackPathInfo_cellPressed(int row, int column)
{
  Q_UNUSED(column);
  QTableWidgetItem *itemX = ui->tableTrackPathInfo->item(row, 1);
  m_fSelectPointX = itemX->text().toDouble();
  QTableWidgetItem *itemY = ui->tableTrackPathInfo->item(row, 2);
  m_fSelectPointY = itemY->text().toDouble();
  m_bPointSelect = true;
  qDebug() << m_fSelectPointX << " " << m_fSelectPointY;
  ui->labGridMap->update();
}

/*添加一键开始运动功能 20220117 qixianyu*/
// void ParentDlg::on_btnOneKeyStartMotion_clicked()
//{
//   is_move_forward_ = !is_move_forward_;
//   if (is_move_forward_ == true)
//   {
//     ui->btnOneKeyStartMotion->setText("向前运动");
//   }
//   else
//   {
//     ui->btnOneKeyStartMotion->setText("后退运动");
//   }
// }

/*添加一键开始运动功能的定时器回调函数 20220117 qixianyu*/
void ParentDlg::onOneKeyStartMotionTimerCallBack()
{
  m_iOneKeyStartMotionCounter++;
  if (1 == m_iOneKeyStartMotionCounter) // 1. 挂D档位 100ms
  {
    FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
    memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));
    stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_GEAR; /*档位设置*/
    MsgTableDisp("操作输入", "D档位设置");
    stFP2VCMsg.stVC2ChasCmd.ucGear = GEAR_D;
    SendChasCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
    return;
  }

  if (3 == m_iOneKeyStartMotionCounter) // 2. 解除驻车 300ms
  {
    MsgTableDisp("操作输入", "解除驻车控制");
    FP2VCChasCtrlMsgST stFP2VCMsg; /*底盘设备控制报文*/
    memset(&stFP2VCMsg, 0, sizeof(FP2VCChasCtrlMsgST));
    stFP2VCMsg.stVC2ChasCmd.ucCmdType = CMD_CHAS_PARKING_CTRL; /*驻车制动控制*/
    stFP2VCMsg.stVC2ChasCmd.ucPark = CHAS_RELEASE_PARK;        /*解除驻车标识*/
    SendChasCtrlMsg(stFP2VCMsg);                               /*发送底盘设备控制报文*/
    return;
  }

  if (30 == m_iOneKeyStartMotionCounter) // 3. 发送运动开始指令 3s
  {
    FP2VCChasMoveCtrlMsgST stFP2VCMsg; /*底盘运动控制报文*/
    memset(&stFP2VCMsg, 0, sizeof(FP2VCChasMoveCtrlMsgST));
    MsgTableDisp("操作输入", "路径跟踪运动开始控制");
    stFP2VCMsg.ucMoveCtrlMode = 0x2;                                                                    /*路径跟踪*/
    stFP2VCMsg.ucMoveFlag = 0x55;                                                                       /*运动开始*/
    stFP2VCMsg.usVehSpeed = ui->comboCourseVehSpeed->currentText().toUShort() * 1000.0 / 3600.0 * 10.0; /*巡航速度*/

    SendMoveCtrlMsg(stFP2VCMsg); /*发送底盘设备控制报文*/
    m_bMoveStart = true;
    m_bMoveStartFirstComputeCurrentPoint = true;
    m_iCurrenPointIndex = 0;
    m_iLastCurrentPointIndex = 0;

    // 停止定时器，并恢复初始状态
    m_timerOneKeyStartMotion->stop(); // 100ms 定时器
    m_iOneKeyStartMotionCounter = 0;
  }
}

// 建立与下位机的通信连接 qixianyu 20220210
void ParentDlg::on_btnStartCommunication_clicked()
{
  emit NetSendInit();  /*网络发送Socket初始化*/
  m_pNetRecv->start(); /*运行run函数，绑定接收端口，开始Net接收*/

  m_pThreadNetSend->start();
  m_pNetSendTimer->start(500); /*开启心跳以太网报文发送定时器*/
  std::cout << "已建立通信连接" << std::endl;

  ui->btnStartCommunication->setEnabled(false);
}

// 终止与下位机的通信连接 qixianyu 20220210
void ParentDlg::on_btnStopCommunication_clicked()
{
  QMessageBox::information(Q_NULLPTR, tr("提示"), tr("当前不支持终止通信连接"));
  return;
}

// void ParentDlg::on_btnEmcyReset_clicked()
//{
//   MsgTableDisp("操作输入", "解除紧急制动");
//   emit sendEmcyBrakeCmd(false);
// }

// void ParentDlg::on_btnLeftLight_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "左转向打开");
//     emit sendLeftLightCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "左转向关闭");
//     emit sendLeftLightCmd(is_open);
//     is_open = true;
//   }
// }

// void ParentDlg::on_btnRightLight_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "右转向打开");
//     emit sendRightLightCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "右转向关闭");
//     emit sendRightLightCmd(is_open);
//     is_open = true;
//   }
// }

// void ParentDlg::on_btnDoubleLight_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "双闪打开");
//     emit sendEmcyFlasherCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "双闪关闭");
//     emit sendEmcyFlasherCmd(is_open);
//     is_open = true;
//   }
// }

// void ParentDlg::on_btnHighBeam_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "远光打开");
//     emit sendHighBeamCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "远光关闭");
//     emit sendHighBeamCmd(is_open);
//     is_open = true;
//   }
// }

// void ParentDlg::on_btnLowBeam_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "近光打开");
//     emit sendLowBeamCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "近光关闭");
//     emit sendLowBeamCmd(is_open);
//     is_open = true;
//   }
// }

// void ParentDlg::on_btnHorn_clicked()
// {
//   static bool is_open = true;
//   if (is_open)
//   {
//     MsgTableDisp("操作输入", "喇叭打开");
//     emit sendHonkCmd(is_open);
//     is_open = false;
//   }
//   else
//   {
//     MsgTableDisp("操作输入", "喇叭关闭");
//     emit sendHonkCmd(is_open);
//     is_open = true;
//   }
// }

void ParentDlg::on_btnLockThirdPersonView_clicked()
{
  is_lock_third_person_view_ = true;
}

void ParentDlg::on_btnUnLockThirdPersonView_clicked()
{
  is_lock_third_person_view_ = false;
}

void ParentDlg::on_btnResetVehicleState_clicked()
{
  emit sendVehicleResetCmd();
}
// 左侧为正，右侧为负数，还需要满足阈值条件
void ParentDlg::on_btnSendSteerWheelCmd_clicked()
{
  double steer_value = ui->editSteerWheelValue->text().toDouble();
  double steer_speed_value = ui->editSteerWheelSpeedValue->text().toDouble();

  // std::cout << "steer vlaue is " << steer_value << std::endl;
  if (steer_value > 1000 || steer_value < -1000)
  {
    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("角度超出阈值"));
    return;
  }

  if (steer_speed_value > 250 || steer_speed_value < 0)
  {
    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("角速度超出阈值"));
    return;
  }

  emit sendTargetSteeringAngleAndAngleSpeedCmd(steer_value, steer_speed_value);
}

void ParentDlg::on_btnSendThrottleCmd_clicked()
{
  double throttle_value = ui->editThrottleValue->text().toDouble();
  double brake_value = ui->editBrakeValue->text().toDouble();

  std::cout << "throttle_value vlaue is " << throttle_value << std::endl;
  if (throttle_value > 100 || throttle_value < 0)
  {
    QMessageBox::information(Q_NULLPTR, tr("提示"), tr("超出阈值10%"));
    return;
  }

  if (brake_value > 0)
  {
    emit sendTargetThrottleAndBrakePct(0, brake_value);
  }
  else if (throttle_value > 0)
  {
    emit sendTargetThrottleAndBrakePct(throttle_value, 0);
  }
  else
  {
    emit sendTargetThrottleAndBrakePct(0, 0);
  }
}

// void ParentDlg::on_btnSendStopCmd_clicked()
//{
//   emit sendTargetThrottleAndBrakePct(0, -4.0);
// }

// void ParentDlg::on_btnComputeFrontAngle_clicked()
//{
//  auto circle_path = m_listTrackPathCmd;
//  // double r = 10;
//  // for (int i = 0; i < 200; ++i)
//  // {
//  //   double x = i * 0.05;
//  //   double y = sqrt(r * r - x * x);
//  //   SmoothPathPointST point;
//  //   point.stENUPoint.x = x;
//  //   point.stENUPoint.y = y;
//  //   circle_path.push_back(point);
//  // }

// if (circle_path.empty())
// {
//   QMessageBox::information(this, "critical", tr("圆拟合路径点数为0"), QMessageBox::Yes, QMessageBox::Yes);
//   return;
// }

// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// for (int i = 0; i < circle_path.size(); ++i)
// {
//   pcl::PointXYZ pcl_point;
//   pcl_point.x = circle_path.at(i).stENUPoint.x;
//   pcl_point.y = circle_path.at(i).stENUPoint.y;
//   cloud->points.push_back(pcl_point);
// }
// cloud->width = circle_path.size();
// cloud->height = 1;

// pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_circle2D(
//     new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(cloud));  //选择拟合点云与几何模型
// pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);  //创建随机采样一致性对象
// ransac.setDistanceThreshold(0.1);  //设置距离阈值，与模型距离小于0.01的点作为内点
// ransac.setMaxIterations(10000);    //设置最大迭代次数
// ransac.computeModel();             //执行模型估计

// // std::vector<int> inliers;    //存储内点索引的向量
// // ransac.getInliers(inliers);  //提取内点对应的索引

// /// 根据索引提取内点
// // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>);

// // pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_circle);

// /// 输出模型参数(x-x0)^2 + (y-y0)^2 = r^2;
// Eigen::VectorXf coefficient;
// ransac.getModelCoefficients(coefficient);
// std::cout << "二维圆方程为：\n"
//           << "(x0 " << coefficient[0] << "( y0 " << coefficient[1] << " r " << coefficient[2] << std::endl;
// double wheel_base = 2.7;
// if (wheel_base < 0.5)
// {
//   QMessageBox::information(this, "critical", tr("轴距参数错误"), QMessageBox::Yes, QMessageBox::Yes);
//   return;
// }
// double delta = atan(VEHICLE_WHEEL_BASE / coefficient[2]);
// double steer_value = ui->editSteerWheelValue->text().toDouble();
// if (steer_value < 0)
// {
//   ui->editFrontWheelValue->setText(QString::number(-delta * 180 / M_PI, 'f', 6));
// }
// else
// {
//   ui->editFrontWheelValue->setText(QString::number(delta * 180 / M_PI, 'f', 6));
// }
//}

// void ParentDlg::on_btnConfirmMode_clicked()
// {
// }

// void ParentDlg::on_btnPlanMode_clicked()
//{
//   //  m_uiPlanMode++;
//   //  //当前只支持寻迹和避障模式
//   //  if (2 == m_uiPlanMode)
//   //  {
//   //    m_uiPlanMode = 0;
//   //  }

//  //  if (m_uiPlanMode == 0)
//  //  {
//  //    ui->btnPlanMode->setText("巡迹模式");
//  //  }
//  //  else
//  //  {
//  //    ui->btnPlanMode->setText("避障模式");
//  //  }

//  //  emit sendPlanningMode(m_uiPlanMode);
//}
void ParentDlg::SendNetMsg(char data[], int iLen, std::string strDestIP, unsigned short int usDestPort) /*发送以太网报文*/
{
  struct sockaddr_in client_sockaddr;
  memset(&client_sockaddr, 0, sizeof(client_sockaddr));
  client_sockaddr.sin_family = AF_INET;
  client_sockaddr.sin_addr.s_addr = (inet_addr(strDestIP.c_str()));
  client_sockaddr.sin_port = htons(usDestPort);

  socklen_t cliLen = sizeof(client_sockaddr);

  char senddata[1000];
  memset(senddata, 0, 1000);
  memcpy(senddata, data, iLen);
  int iSendLen = sendto(m_sockClient, senddata, iLen, 0, (struct sockaddr *)(&client_sockaddr), cliLen);
  if (-1 == iSendLen)
  {
    perror("sendto failed");
  }
}

void ParentDlg::on_btntaskdist_clicked()
{
  uint8 taskdistclick;
  taskdistclick = 0x00;
  emit sendTaskdist(taskdistclick);
}

void ParentDlg::on_btntaskstart_clicked()
{
  uint8 taskdistclick;
  taskdistclick = 0x01;
  emit sendTaskstart(taskdistclick);
}

void ParentDlg::on_btntasksuspend_clicked()
{
  uint8 taskdistclick;
  taskdistclick = 0x02;
  emit sendTasksuspend(taskdistclick);
}

void ParentDlg::on_btntaskstop_clicked()
{
  uint8 taskdistclick;
  taskdistclick = 0x03;
  emit sendTaskstop(taskdistclick);
}

void ParentDlg::on_taskpointget_clicked()
{
  uint8 taskpointgetclicked = 0;
  double lon, lat;
  lon = lon_task_point_get;
  lat = lat_task_point_get;

  emit sendTaskPointGet(lon, lat, taskpointgetclicked);
}

void ParentDlg::onHeartBeatTimerCallBack()
{
  //  int thres = 5000;
  //  QTime cur = QTime::currentTime();
  //  {
  //    int elapsed = heart_beat_ref_time_.msecsTo(cur); // ms
  //    if (elapsed > thres)
  //    {
  //      ui->labReferenceLine->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //    }
  //    else
  //    {
  //      ui->labReferenceLine->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //    }
  //  }
  //  {
  //    int elapsed = heart_beat_planning_time_.msecsTo(cur); // ms
  //    if (elapsed > thres)
  //    {
  //      ui->labPlanning->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //    }
  //    else
  //    {
  //      ui->labPlanning->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //    }
  //  }
  //  {
  //    int elapsed = heart_beat_control_time_.msecsTo(cur); // ms
  //    if (elapsed > thres)
  //    {
  //      ui->labControl->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //    }
  //    else
  //    {
  //      ui->labControl->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //    }
  //  }

  //  {
  //    int elapsed = heart_beat_estop_.msecsTo(cur); // ms
  //    // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //    if (elapsed > thres)
  //    {
  //      // std::cout <<" red"<<std::endl;
  //      ui->labESopState->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //    }
  //    else
  //    {
  //      ui->labESopState->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //      // std::cout <<" green"<<std::endl;
  //    }
  //  }

  //  {
  //    int elapsed = heart_beat_lidar_pos_.msecsTo(cur); // ms

  //    // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //    if (elapsed > thres)
  //    {
  //      // std::cout <<" red"<<std::endl;
  //      ui->labPositive->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //      emit sendMotionStart(0);
  //    }
  //    else
  //    {
  //      ui->labPositive->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //      // std::cout <<" green"<<std::endl;
  //    }
  //  }
  //  // {
  //  //   int elapsed = heart_beat_lidar_neg_.msecsTo(cur); // ms
  //  //   // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //  //   if (elapsed > thres)
  //  //   {
  //  //     // std::cout <<" red"<<std::endl;
  //  //     ui->labNegtive->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //  //     emit sendMotionStart(0);
  //  //   }
  //  //   else
  //  //   {
  //  //     ui->labNegtive->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //  //     // std::cout <<" green"<<std::endl;
  //  //   }
  //  // }

  //  {
  //    int elapsed = heart_beat_radar_.msecsTo(cur); // ms
  //    // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //    if (elapsed > thres)
  //    {
  //      // std::cout <<" red"<<std::endl;
  //      ui->labRadar->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //      emit sendMotionStart(0);
  //    }
  //    else
  //    {
  //      ui->labRadar->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //      // std::cout <<" green"<<std::endl;
  //    }
  //  }

  //  // {
  //  //   int elapsed = heart_beat_camera_sense_.msecsTo(cur); // ms
  //  //   // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //  //   if (elapsed > thres)
  //  //   {
  //  //     // std::cout <<" red"<<std::endl;
  //  //     ui->labCameraSence->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //  //     emit sendMotionStart(0);
  //  //   }
  //  //   else
  //  //   {
  //  //     ui->labCameraSence->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //  //     // std::cout <<" green"<<std::endl;
  //  //   }
  //  // }

  //  // {
  //  //   int elapsed = heart_beat_camera_segment_.msecsTo(cur); // ms
  //  //   // std::cout <<" heart_beat_estop_  elapsed"<<elapsed <<" ms"<<std::endl;
  //  //   if (elapsed > thres)
  //  //   {
  //  //     // std::cout <<" red"<<std::endl;
  //  //     ui->labCameraSeg->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:red; color:white}");
  //  //     emit sendMotionStart(0);
  //  //   }
  //  //   else
  //  //   {
  //  //     ui->labCameraSeg->setStyleSheet("QLabel{font:'楷体' 12pt bold; background-color:green; color:white}");
  //  //     // std::cout <<" green"<<std::endl;
  //  //   }
  //  // }
}

void ParentDlg::onHeartBeat(HeartBeatST heart_beat)
{
  switch (heart_beat.flag)
  {
  case 0:
  {
    heart_beat_ref_time_ = QTime::currentTime();
  }
  break;
  case 1:
  {
    heart_beat_planning_time_ = QTime::currentTime();
  }
  break;

  case 2:
  {
    heart_beat_control_time_ = QTime::currentTime();
  }
  break;

  case 100:
  {
    heart_beat_estop_ = QTime::currentTime();
  }
  break;
  case 20:
  {
    heart_beat_lidar_pos_ = QTime::currentTime();
  }
  break;
    // case 21:
    // {
    //   heart_beat_lidar_neg_ = QTime::currentTime();
    // }
    // break;

  case 22:
  {
    heart_beat_radar_ = QTime::currentTime();
  }
  break;
    // case 23:
    // {
    //   heart_beat_camera_sense_ = QTime::currentTime();
    // }
    // break;
    // case 24:
    // {
    //   heart_beat_camera_segment_ = QTime::currentTime();
    // }
    // break;

  default:
    break;
  }
}

void ParentDlg::on_btnTest_clicked()
{
  // 使用QDialog::raise()时，偶发界面阻塞卡死现象。
  // QDialog::raise()函数是置于顶部的作用，但是如果使用不当，会导致界面线程卡死（弹窗是在界面线程弹出的），因为QDialog::raise()会阻塞当前线程，直到对话框置于顶部。

  if (!is_vehicle_test_dlg_pressed_)
  {
    chassis_light_horn_wiper_dlg_ptr_ = new ChassisLightHornWiperDialog(this);
    connect(qnode_, SIGNAL(emitLightHornWiperState(ChassisLightHornWiper)), chassis_light_horn_wiper_dlg_ptr_, SLOT(onLightHornWiperTopicCallback(ChassisLightHornWiper)));

    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendLeftLightCmd(bool)), qnode_, SLOT(onSendLeftLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendRightLightCmd(bool)), qnode_, SLOT(onSendRightLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendEmcyFlasherCmd(bool)), qnode_, SLOT(onSendEmcyFlasherCmd(bool)));

    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendLowBeamCmd(bool)), qnode_, SLOT(onSendLowBeamCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendHighBeamCmd(bool)), qnode_, SLOT(onSendHighBeamCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendHornCmd(bool)), qnode_, SLOT(onSendHonkCmd(bool)));

    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendBrakeLightCmd(bool)), qnode_, SLOT(onSendBrakeLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendFrontFoggyLightCmd(bool)), qnode_, SLOT(onSendFrontFoggyLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendRearFoggyLightCmd(bool)), qnode_, SLOT(onSendRearFoggyLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendPositionLightCmd(bool)), qnode_, SLOT(onSendPositionLightCmd(bool)));

    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendReverseLightCmd(bool)), qnode_, SLOT(onSendReverseLightCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendWiperCmd(bool)), qnode_, SLOT(onSendWiperCmd(bool)));
    connect(chassis_light_horn_wiper_dlg_ptr_, SIGNAL(sendHeadLightCmd(bool)), qnode_, SLOT(onsendHeadLightCmd(bool)));


    connect(qnode_, SIGNAL(emitLightHornWiperTopicCallback(ChassisLightHornWiper)), chassis_light_horn_wiper_dlg_ptr_, SLOT(onLightHornWiperTopicCallback(ChassisLightHornWiper)));


    QTimer::singleShot(1, chassis_light_horn_wiper_dlg_ptr_, &QDialog::raise);
    chassis_light_horn_wiper_dlg_ptr_->show();
    //    vehicle_test_dlg_ptr_->setWindowFlags(vehicle_test_dlg_ptr_->windowFlags()|Qt::WindowStaysOnTopHint);
    is_vehicle_test_dlg_pressed_ = true;
  }
  else
  {
    if (chassis_light_horn_wiper_dlg_ptr_ != Q_NULLPTR)
    {
      disconnect(qnode_, SIGNAL(emitLightHornWiperState(ChassisLightHornWiper)), chassis_light_horn_wiper_dlg_ptr_, SLOT(onLightHornWiperTopicCallback(ChassisLightHornWiper)));
      chassis_light_horn_wiper_dlg_ptr_->hide();
      delete chassis_light_horn_wiper_dlg_ptr_;
      chassis_light_horn_wiper_dlg_ptr_ = nullptr;
    }
    is_vehicle_test_dlg_pressed_ = false;
  }
}

void ParentDlg::on_btnParkingBrakeOn_clicked()
{
  MsgTableDisp("操作输入", "驻车制动");
  emit sendParkBrakeCmd(true);
}

void ParentDlg::on_btnParkingBrakeOff_clicked()
{
  MsgTableDisp("操作输入", "解除驻车");
  emit sendParkBrakeCmd(false);
}

void ParentDlg::on_btnWireControl_clicked()
{
  // 使用QDialog::raise()时，偶发界面阻塞卡死现象。
  // QDialog::raise()函数是置于顶部的作用，但是如果使用不当，会导致界面线程卡死（弹窗是在界面线程弹出的），因为QDialog::raise()会阻塞当前线程，直到对话框置于顶部。

  if (!is_vehicle_wire_control_pressed_)
  {
    vehicle_wire_control_ptr_ = new VehicleWireControl(this);

    connect(vehicle_wire_control_ptr_, SIGNAL(sendEngineCmd(bool)), qnode_, SLOT(onSendEngineCmd(bool)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendLowVoltageCmd(bool)), qnode_, SLOT(onSendLowVoltageCmd(bool)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendHighVoltageCmd(bool)), qnode_, SLOT(onSendHighVoltageCmd(bool)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendDriveModeCmd(uint8)), qnode_, SLOT(onSendDriveModeCmd(uint8)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendTargetGearCmd(uint8)), qnode_, SLOT(onSendTargetGearCmd(uint8)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendTargetThrottleAndBrakePct(float, float)), qnode_,
            SLOT(onsendTargetThrottleAndBrakePct(float, float)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendTargetSteeringAngleAndAngleSpeedCmd(float, float)), qnode_,
            SLOT(onSendTargetSteeringAngleAndAngleSpeedCmd(float, float)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendParkBrakeCmd(bool)), qnode_, SLOT(onSendParkBrakeCmd(bool)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendMotionStart(uint8)), qnode_, SLOT(onSendMotionStart(uint8)));
    connect(vehicle_wire_control_ptr_, SIGNAL(sendVehicleResetCmd()), qnode_, SLOT(onSendVehicleResetCmd()));

    connect(vehicle_wire_control_ptr_, SIGNAL(sendEmcyBrakeCmd(bool)), qnode_, SLOT(onSendEmcyBrakeCmd(bool)));

    connect(qnode_, SIGNAL(emitVehicleChassisState(VehicleChassisStateST)), vehicle_wire_control_ptr_,
            SLOT(onVehicleChassisState(VehicleChassisStateST)));

    QTimer::singleShot(1, vehicle_wire_control_ptr_, &QDialog::raise);
    vehicle_wire_control_ptr_->show();
    is_vehicle_wire_control_pressed_ = true;
  }
  else
  {
    if (vehicle_wire_control_ptr_ != Q_NULLPTR)
    {
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendEngineCmd(bool)), qnode_, SLOT(onSendEngineCmd(bool)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendLowVoltageCmd(bool)), qnode_, SLOT(onSendLowVoltageCmd(bool)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendHighVoltageCmd(bool)), qnode_, SLOT(onSendHighVoltageCmd(bool)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendDriveModeCmd(uint8)), qnode_, SLOT(onSendDriveModeCmd(uint8)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendTargetGearCmd(uint8)), qnode_, SLOT(onSendTargetGearCmd(uint8)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendTargetThrottleAndBrakePct(float, float)), qnode_,
                 SLOT(onsendTargetThrottleAndBrakePct(float, float)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendTargetSteeringAngleAndAngleSpeedCmd(float, float)), qnode_,
                 SLOT(onSendTargetSteeringAngleAndAngleSpeedCmd(float, float)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendParkBrakeCmd(bool)), qnode_, SLOT(onSendParkBrakeCmd(bool)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendMotionStart(uint8)), qnode_, SLOT(onSendMotionStart(uint8)));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendVehicleResetCmd()), qnode_, SLOT(onSendVehicleResetCmd()));
      disconnect(vehicle_wire_control_ptr_, SIGNAL(sendEmcyBrakeCmd(bool)), qnode_, SLOT(onSendEmcyBrakeCmd(bool)));

      vehicle_wire_control_ptr_->hide();
      delete vehicle_wire_control_ptr_;
      vehicle_wire_control_ptr_ = nullptr;
    }
    is_vehicle_wire_control_pressed_ = false;
  }
}

void ParentDlg::on_btnSwarm_clicked()
{

  if (!is_swarm_pressed_)
  {
    QTimer::singleShot(1, platoondlg_ptr_, &QDialog::raise);
    platoondlg_ptr_->show();
    is_swarm_pressed_ = true;
  }
  else
  {
    if (platoondlg_ptr_ != Q_NULLPTR)
    {
      platoondlg_ptr_->hide();
    }
    is_swarm_pressed_ = false;
  }
}

void ParentDlg::on_btnUpdate_clicked()
{
    if (!is_update_pressed_)
    {
      connect(updatefile_ptr_, SIGNAL(closedialog()), this, SLOT(close_update()));
      QTimer::singleShot(1, updatefile_ptr_, &QDialog::raise);
      updatefile_ptr_->show();
      is_update_pressed_ = true;
    }
    else
    {
      if (updatefile_ptr_ != Q_NULLPTR)
      {
        disconnect(updatefile_ptr_, SIGNAL(closedialog()), this, SLOT(close_update()));
        updatefile_ptr_->hide();
      }
      is_update_pressed_ = false;
    }
}

void ParentDlg::close_update()
{
    if (updatefile_ptr_ != Q_NULLPTR)
    {
      disconnect(updatefile_ptr_, SIGNAL(closedialog()), this, SLOT(close_update()));
      updatefile_ptr_->hide();
    }
    is_update_pressed_ = false;
}

void ParentDlg::on_btnRecoad_clicked()
{
    QString showtext = ui->btnRecoad->text();
    if(showtext == QString::fromLocal8Bit("轨迹录制开始")){
        ui->btnRecoad->setText("轨迹录制停止");
        emit(sendRecordCmd(1));
    }
    else if(showtext == QString::fromLocal8Bit("轨迹录制停止")){
        ui->btnRecoad->setText("轨迹录制开始");
        emit(sendRecordCmd(0));
    }
}

void ParentDlg::on_btnbackword_clicked()
{
    QString showtext = ui->btnbackword->text();
    if(showtext == QString::fromLocal8Bit("倒车开始")){
        ui->btnbackword->setText("倒车停止");
        emit(sendBackWordCmd(1));
    }
    else if(showtext == QString::fromLocal8Bit("倒车停止")){
        ui->btnbackword->setText("倒车开始");
        emit(sendBackWordCmd(0));
    }
}
