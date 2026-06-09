#ifndef POSMEASUREDLG_H
#define POSMEASUREDLG_H

#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>
#include "pad/allheader.h"
#include "pad/llh2enu.h"
#include "pad/alldeclare.h"
#include <mutex>
#include "pad/alltypes.h"
namespace Ui
{
class PosMeasureDlg;
}

class PosMeasureDlg : public QDialog
{
  Q_OBJECT

public:
  explicit PosMeasureDlg(QWidget* parent = nullptr);
  ~PosMeasureDlg();

  bool createFile(const QString filePath, const QString fileName); /*创建记录文件*/

  QTimer* m_timerPathCollect; /*路径采集定时器*/

  INS2VCWorkDataST m_stINS2VCWorkData; /*INS工作数据*/

  LongLatHeightST m_stLLHMapVertex[4]; /*地图顶点经纬度信息存储变量*/
  ENUCorST m_stENUMapVertex[4];        /*地图顶点ENU信息存储变量*/
  bool m_bMapVertexSet[4];             /*地图顶点已设置标志*/

  QFile m_objVertexFile; /*顶点信息文件*/
  QFile m_objPathFile;   /*路径信息文件*/

  QList<SmoothPathPointST> m_listPathENU;        /*采集路径数据点暂存*/
  UINT32 m_uiCurrentPointID;                     /*对应当前列表中的序号，从1开始*/
  QVector<SmoothPathPointST> m_listTrackPathCmd; /*要跟踪的路径数据点*/

  void init();
  void setVehicleMotionState(const VehicleMotionStateST& msg)
  {
    vehicle_motion_state_msg_mutex_.lock();
    vehicle_motion_state_msg_ = msg;
    vehicle_motion_state_msg_mutex_.unlock();
  }

  void setXW5651(const XW5651ST& msg)
  {
    xw5651_mutex_.lock();
    xw5651_msg_ = msg;
    xw5651_mutex_.unlock();
  }
  
signals:
  void MapOriginSet(LongLatHeightST stLLHOrigin);            /*设定了地图原点信号*/
  void PathCollectDisp(QList<SmoothPathPointST> listENUPos); /*采集到的路径点进行显示*/

private slots:
  void on_btnOriginSet_clicked();       /*设置原点按钮单击槽函数*/
  void on_btnMapVertex1Set_clicked();   /*地图顶点第1点设置按钮单击槽函数*/
  void on_btnMapVertex2Set_clicked();   /*地图顶点第2点设置按钮单击槽函数*/
  void on_btnMapVertex3Set_clicked();   /*地图顶点第3点设置按钮单击槽函数*/
  void on_btnMapVertex4Set_clicked();   /*地图顶点第4点设置按钮单击槽函数*/
  void on_btnCurPointMeasure_clicked(); /*当前点量测按钮单击槽函数*/
  void on_btnVertexInfoSave_clicked();  /*顶点信息存储按钮单击槽函数*/
  void on_btnPathPointSave_clicked();   /*路径点采集按钮单击槽函数*/
  void on_btnPathPointClear_clicked();  /*清空已采集路径点按钮单击槽函数*/
  void on_btnPathFileSave_clicked();    /*路径文件存储按钮单击槽函数*/

  void timerPathCollectFunc(); /*路径采集定时器*/

  void on_btnPathFileOpen_clicked();

  void on_btnPathFileEditSave_clicked();

  void on_btnCutDistanceToEnd_clicked();

private:
  Ui::PosMeasureDlg* ui;
  std::mutex vehicle_motion_state_msg_mutex_;
  VehicleMotionStateST vehicle_motion_state_msg_;

  std::mutex xw5651_mutex_;
  XW5651ST xw5651_msg_;

  std::mutex vehicle_chassis_state_mutex_;
  VehicleChassisStateST vehicle_chassis_state_msg_;
};

#endif  // POSMEASUREDLG_H
