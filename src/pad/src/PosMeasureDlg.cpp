#include "pad/PosMeasureDlg.h"
#include "ui_PosMeasureDlg.h"
#include <QMessageBox>
#include <QList>
#include "pad/alltypes.h"
#include <cmath>
#include <iostream>
PosMeasureDlg::PosMeasureDlg(QWidget* parent) : QDialog(parent), ui(new Ui::PosMeasureDlg)
{
  ui->setupUi(this);

  QPalette p = this->palette();
  p.setColor(QPalette::Window, QColor(35, 35, 35));
  this->setPalette(p); /*设置窗口背景颜色*/

  m_timerPathCollect = new QTimer;
  m_timerPathCollect->setTimerType(Qt::PreciseTimer); /*高精度定时器*/
  connect(m_timerPathCollect, SIGNAL(timeout()), this, SLOT(timerPathCollectFunc()), Qt::DirectConnection);
  m_uiCurrentPointID = 0;

  ui->labCutDistanceToEnd->setEnabled(true);

  ui->btnMapVertex3Set->setEnabled(false);
  ui->btnMapVertex4Set->setEnabled(false);
  ui->btnVertexInfoSave->setEnabled(false);
}

PosMeasureDlg::~PosMeasureDlg()
{
  m_timerPathCollect->stop();
  if (m_timerPathCollect != Q_NULLPTR)
  {
    delete m_timerPathCollect;
    m_timerPathCollect = Q_NULLPTR;
  }

  delete ui;
  qDebug() << "~PosMeasureDlg()";
}

void PosMeasureDlg::init()
{
  ui->btnPathFileEditSave->setEnabled(false);
  ui->btnCutDistanceToEnd->setEnabled(false);
  ui->btnMapVertex2Set->setEnabled(false);
}
void PosMeasureDlg::timerPathCollectFunc() /*路径采集定时器*/
{
  std::cout << "timer start" << std::endl;
  vehicle_motion_state_msg_mutex_.lock();
  auto vehicle_motion_state_msg = vehicle_motion_state_msg_;
  vehicle_motion_state_msg_mutex_.unlock();

  //   xw5651_mutex_.lock();
  //   auto xw5651_msg = xw5651_msg_;
  //   xw5651_mutex_.unlock();

  //   double mass_fl = VEHICLE_MASS / 4.0; /*左前轮承载重量*/
  //   double mass_fr = VEHICLE_MASS / 4.0; /*右前轮承载重量*/
  //   double mass_rl = VEHICLE_MASS / 4.0; /*左后轮承载重量*/
  //   double mass_rr = VEHICLE_MASS / 4.0; /*右后轮承载重量*/

  //   double mass_front = mass_fl + mass_fr;
  //   double mass_rear = mass_rl + mass_rr;
  //   double mass_ = mass_front + mass_rear;

  //   double lf_ = VEHICLE_WHEEL_BASE * (1.0 - mass_front / mass_);
  //   double lr_ = VEHICLE_WHEEL_BASE * (1.0 - mass_rear / mass_);

  //   LongLatHeightST stLLHPoint;

  //   stLLHPoint.fLatitude = xw5651_msg.latitude;
  //   stLLHPoint.fLongitude = xw5651_msg.longitude;
  //   stLLHPoint.fHeight = xw5651_msg.height;

  //   stLLHPoint.fLatitude = xw5651_msg.gps_latitude;
  //   stLLHPoint.fLongitude = xw5651_msg.gps_longtitude;
  //   stLLHPoint.fHeight = xw5651_msg.gps_height;
  //   static bool is_first = true;
  //   if (is_first)
  //   {
  //     g_stMapCorditCmd.stLLHOriginPos.fLatitude = xw5651_msg.gps_latitude;
  //     g_stMapCorditCmd.stLLHOriginPos.fLongitude = xw5651_msg.gps_longtitude;
  //     g_stMapCorditCmd.stLLHOriginPos.fHeight = xw5651_msg.gps_height;
  //     is_first = false;
  //   }
  //   ENUCorST stFrontWheelENUPos = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos); /*将当前点转成东北天坐标系*/
  //   double fVehCourse = m_stINS2VCWorkData.uiVehCourse * 1e-3;                          /*当前车辆航向*/
  /*根据车头位置解算质心位置*/
  //   ENUCorST stVehCenterENUPos; /*解算车辆质心位置*/
  //   memset(&stVehCenterENUPos, 0, sizeof(ENUCorST));
  //   stVehCenterENUPos.x = stFrontWheelENUPos.x - lf_ * sin(fVehCourse * DEGREE_RADIAN);
  //   stVehCenterENUPos.y = stFrontWheelENUPos.y - lf_ * cos(fVehCourse * DEGREE_RADIAN);
  //   stVehCenterENUPos.z = stFrontWheelENUPos.z;

  SmoothPathPointST stTransVehCenterENUPos;
  memset(&stTransVehCenterENUPos, 0, sizeof(SmoothPathPointST));

  //   stTransVehCenterENUPos.stENUPoint.x = vehicle_motion_state_msg.x;
  //   stTransVehCenterENUPos.stENUPoint.y = vehicle_motion_state_msg.y;

  stTransVehCenterENUPos.stENUPoint.x = vehicle_motion_state_msg_.x;
  stTransVehCenterENUPos.stENUPoint.y = vehicle_motion_state_msg_.y;

  if (m_listPathENU.empty())
  {
    m_listPathENU.append(stTransVehCenterENUPos); /*存储路径点*/
    emit PathCollectDisp(m_listPathENU);          /*显示采集到的路径点*/
  }
  else
  {
    double fDist = sqrt(pow((stTransVehCenterENUPos.stENUPoint.x - m_listPathENU.last().stENUPoint.x), 2) +
                        pow((stTransVehCenterENUPos.stENUPoint.y - m_listPathENU.last().stENUPoint.y), 2));
    if (fDist >=
        0.5) /*每隔1米存储一个路径点,20210413考虑弯道情况，由2米更改为1米,20210817考虑其他车应用情况，由1米更改为0.5米*/
    {
      m_listPathENU.append(stTransVehCenterENUPos); /*存储路径点*/
      emit PathCollectDisp(m_listPathENU);          /*显示采集到的路径点*/
    }
  }
}

void PosMeasureDlg::on_btnOriginSet_clicked() /*设置原点按钮单击槽函数*/
{
  QMessageBox::information(Q_NULLPTR, "提示", "原点只供显示，请打开路径文件");
}

void PosMeasureDlg::on_btnMapVertex1Set_clicked() /*地图顶点第1点设置按钮单击槽函数*/
{
  QMessageBox::information(Q_NULLPTR, "提示", "车在路径上会自动寻找起点，不需要设置");
}

void PosMeasureDlg::on_btnMapVertex2Set_clicked() /*地图顶点第2点设置按钮单击槽函数*/
{
  if (0xB == m_stINS2VCWorkData.ucINSState) /*双天线差分状态*/
  {
    if (true == g_bSetMapOrigin) /*地图原点已设置*/
    {
      LongLatHeightST stLLHPoint;
      stLLHPoint.fLatitude = m_stINS2VCWorkData.iLatitude * 1e-7;
      stLLHPoint.fLongitude = m_stINS2VCWorkData.iLongitude * 1e-7;
      stLLHPoint.fHeight = m_stINS2VCWorkData.iHeight * 1e-3f;

      ENUCorST stENUCurPos = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos); /*将当前点转成东北天坐标系*/
      ui->labMapVertex2Longtitude->setText(QString("%1").arg(stLLHPoint.fLongitude, 0, 'f', 7));
      ui->labMapVertex2Latitude->setText(QString("%1").arg(stLLHPoint.fLatitude, 0, 'f', 7));
      ui->labMapVertex2Height->setText(QString("%1").arg(stLLHPoint.fHeight, 0, 'f', 3));
      ui->labMapVertex2ENUX->setText(QString("%1").arg(stENUCurPos.x));
      ui->labMapVertex2ENUY->setText(QString("%1").arg(stENUCurPos.y));
    }
    else
    {
      QMessageBox::information(Q_NULLPTR, "提示", "地图原点未设置，不支持进行位置量测");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "MEMS未处于双天线差分，不支持进行位置量测");
  }
}

void PosMeasureDlg::on_btnMapVertex3Set_clicked() /*地图顶点第3点设置按钮单击槽函数*/
{
  if (0xB == m_stINS2VCWorkData.ucINSState) /*双天线差分状态*/
  {
    if (true == g_bSetMapOrigin)
    {
      LongLatHeightST stLLHPoint;
      stLLHPoint.fLatitude = m_stINS2VCWorkData.iLatitude * 1e-7;
      stLLHPoint.fLongitude = m_stINS2VCWorkData.iLongitude * 1e-7;
      stLLHPoint.fHeight = m_stINS2VCWorkData.iHeight * 1e-3f;

      m_stLLHMapVertex[2] = stLLHPoint;
      m_stENUMapVertex[2] = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos); /*将当前点转成东北天坐标系*/

      ui->labMapVertex3Longtitude->setText(QString("%1").arg(stLLHPoint.fLongitude, 0, 'f', 7));
      ui->labMapVertex3Latitude->setText(QString("%1").arg(stLLHPoint.fLatitude, 0, 'f', 7));
      ui->labMapVertex3Height->setText(QString("%1").arg(stLLHPoint.fHeight, 0, 'f', 3));
      ui->labMapVertex3ENUX->setText(QString("%1").arg(m_stENUMapVertex[2].x));
      ui->labMapVertex3ENUY->setText(QString("%1").arg(m_stENUMapVertex[2].y));

      m_bMapVertexSet[2] = true;
    }
    else
    {
      QMessageBox::information(Q_NULLPTR, "提示", "地图原点未设置，不支持进行位置量测");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "MEMS未处于双天线差分，不支持进行位置量测");
  }
}

void PosMeasureDlg::on_btnMapVertex4Set_clicked() /*地图顶点第4点设置按钮单击槽函数*/
{
  if (0xB == m_stINS2VCWorkData.ucINSState) /*双天线差分状态*/
  {
    if (true == g_bSetMapOrigin)
    {
      LongLatHeightST stLLHPoint;
      stLLHPoint.fLatitude = m_stINS2VCWorkData.iLatitude * 1e-7;
      stLLHPoint.fLongitude = m_stINS2VCWorkData.iLongitude * 1e-7;
      stLLHPoint.fHeight = m_stINS2VCWorkData.iHeight * 1e-3f;

      m_stLLHMapVertex[3] = stLLHPoint;
      m_stENUMapVertex[3] = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos); /*将当前点转成东北天坐标系*/

      ui->labMapVertex4Longtitude->setText(QString("%1").arg(stLLHPoint.fLongitude, 0, 'f', 7));
      ui->labMapVertex4Latitude->setText(QString("%1").arg(stLLHPoint.fLatitude, 0, 'f', 7));
      ui->labMapVertex4Height->setText(QString("%1").arg(stLLHPoint.fHeight, 0, 'f', 3));
      ui->labMapVertex4ENUX->setText(QString("%1").arg(m_stENUMapVertex[3].x));
      ui->labMapVertex4ENUY->setText(QString("%1").arg(m_stENUMapVertex[3].y));

      m_bMapVertexSet[3] = true;
    }
    else
    {
      QMessageBox::information(Q_NULLPTR, "提示", "地图原点未设置，不支持进行位置量测");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "MEMS未处于双天线差分，不支持进行位置量测");
  }
}

void PosMeasureDlg::on_btnCurPointMeasure_clicked() /*当前点量测按钮单击槽函数*/
{
  if (0xB == m_stINS2VCWorkData.ucINSState) /*双天线差分状态*/
  {
    if (true == g_bSetMapOrigin) /*地图原点已设置*/
    {
      LongLatHeightST stLLHPoint;
      stLLHPoint.fLatitude = m_stINS2VCWorkData.iLatitude * 1e-7;
      stLLHPoint.fLongitude = m_stINS2VCWorkData.iLongitude * 1e-7;
      stLLHPoint.fHeight = m_stINS2VCWorkData.iHeight * 1e-3f;

      ENUCorST stENUCurPos = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos); /*将当前点转成东北天坐标系*/

      ui->labCurPointLongtitude->setText(QString("%1").arg(stLLHPoint.fLongitude, 0, 'f', 7));
      ui->labCurPointLatitude->setText(QString("%1").arg(stLLHPoint.fLatitude, 0, 'f', 7));
      ui->labCurPointHeight->setText(QString("%1").arg(stLLHPoint.fHeight, 0, 'f', 3));
      ui->labCurPointENUX->setText(QString("%1").arg(stENUCurPos.x));
      ui->labCurPointENUY->setText(QString("%1").arg(stENUCurPos.y));
    }
    else
    {
      QMessageBox::information(Q_NULLPTR, "提示", "地图原点未设置，不支持进行位置量测");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "MEMS未处于双天线差分，不支持进行位置量测");
  }
}

void PosMeasureDlg::on_btnVertexInfoSave_clicked() /*顶点信息存储按钮单击槽函数*/
{
  /*4个顶点和原点已设置，才可以进行顶点信息存储*/
  if ((true == g_bSetMapOrigin) && (true == m_bMapVertexSet[0]) && (true == m_bMapVertexSet[1]) &&
      (true == m_bMapVertexSet[2]) && (true == m_bMapVertexSet[3]))
  {
    QDateTime dateTime = QDateTime::currentDateTime();
    QString strTime =
        "D:/VertexFile/" + dateTime.toString("yyyyMMdd hhmmss") + ".txt"; /*window中文件名不能有冒号，会创建文件失败*/
    QString strName = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("文件另存为"), strTime,
                                                   tr("Vertex Files(*.txt)")); /*返回绝对路径+文件名*/

    /*获取文件名*/
    INT32 pos = strName.lastIndexOf('/');
    QString strfileName = strName.right(strName.size() - pos - 1);
    qDebug() << "strfileName:" << strfileName;
    /*获取文件路径*/
    QString strfilePath = strName.left(pos + 1);
    qDebug() << "strfilePath:" << strfilePath;

    if (true == createFile(strfilePath, strfileName)) /*创建记录文件*/
    {
      /*写地图文件*/
      QTextStream text_stream(&m_objVertexFile);
      QString str = QString("原点 经度 %1 纬度 %2 高度 %3\n")
                        .arg(g_stMapCorditCmd.stLLHOriginPos.fLongitude, 0, 'f', 7)
                        .arg(g_stMapCorditCmd.stLLHOriginPos.fLatitude, 0, 'f', 7)
                        .arg(g_stMapCorditCmd.stLLHOriginPos.fHeight, 0, 'f', 3);
      text_stream << str;

      str = QString("第1点 经度 %1 纬度 %2 高度 %3 X %4 Y %5\n")
                .arg(m_stLLHMapVertex[0].fLongitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[0].fLatitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[0].fHeight, 0, 'f', 3)
                .arg(m_stENUMapVertex[0].x)
                .arg(m_stENUMapVertex[0].y);
      text_stream << str;

      str = QString("第2点 经度 %1 纬度 %2 高度 %3 X %4 Y %5\n")
                .arg(m_stLLHMapVertex[1].fLongitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[1].fLatitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[1].fHeight, 0, 'f', 3)
                .arg(m_stENUMapVertex[1].x)
                .arg(m_stENUMapVertex[1].y);
      text_stream << str;

      str = QString("第3点 经度 %1 纬度 %2 高度 %3 X %4 Y %5")
                .arg(m_stLLHMapVertex[2].fLongitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[2].fLatitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[2].fHeight, 0, 'f', 3)
                .arg(m_stENUMapVertex[2].x)
                .arg(m_stENUMapVertex[2].y);
      text_stream << str;

      str = QString("第4点 经度 %1 纬度 %2 高度 %3 X %4 Y %5")
                .arg(m_stLLHMapVertex[3].fLongitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[3].fLatitude, 0, 'f', 7)
                .arg(m_stLLHMapVertex[3].fHeight, 0, 'f', 3)
                .arg(m_stENUMapVertex[3].x)
                .arg(m_stENUMapVertex[3].y);
      text_stream << str;

      m_objVertexFile.flush();
      m_objVertexFile.close();
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "需首先完成4个顶点和原点的设置");
  }
}

bool PosMeasureDlg::createFile(const QString filePath, const QString fileName) /*创建记录文件*/
{
  QDir tempDir;
  QString currentDir = QDir::currentPath(); /*临时保存程序当前路径*/
  if (false == tempDir.exists(filePath))    /*不存在记录文件的存储路径则创建*/
  {
    if (false == tempDir.mkpath(filePath))
    {
      return false;
    }
  }
  if (false == QDir::setCurrent(filePath)) /*打开filePath路径*/
  {
    return false;
  }

  if (true == tempDir.exists(fileName)) /*查询路径下文件是否存在*/
  {
    qDebug() << "文件存在";
    return true;
  }
  m_objVertexFile.setFileName(fileName); /*在当前路径下创建文件*/
  if (false == m_objVertexFile.open(QIODevice::ReadWrite | QIODevice::Text))
  {
    QMessageBox::information(Q_NULLPTR, "提示", "创建顶点信息文件失败");
    return false;
  }
  QDir::setCurrent(currentDir); /*将程序当前路径设为原来的路径*/
  return true;
}

void PosMeasureDlg::on_btnPathPointSave_clicked() /*路径点采集按钮单击槽函数*/
{
  if (INS_STATE_DOUBLE_ANTENA_RTK == xw5651_msg_.combined_navigation_state) /*双天线差分状态*/
  {
    if (true) /*地图原点已设置*/
    {
      if (false == m_timerPathCollect->isActive())
      {
        // double mass_fl = VEHICLE_MASS / 4.0; /*左前轮承载重量*/
        // double mass_fr = VEHICLE_MASS / 4.0; /*右前轮承载重量*/
        // double mass_rl = VEHICLE_MASS / 4.0; /*左后轮承载重量*/
        // double mass_rr = VEHICLE_MASS / 4.0; /*右后轮承载重量*/

        // double mass_front = mass_fl + mass_fr;
        // double mass_rear = mass_rl + mass_rr;
        // double mass_ = mass_front + mass_rear;

        // double lf_ = VEHICLE_WHEEL_BASE * (1.0 - mass_front / mass_);
        // double lr_ = VEHICLE_WHEEL_BASE * (1.0 - mass_rear / mass_);

        // LongLatHeightST stLLHPoint;
        // stLLHPoint.fLatitude = m_stINS2VCWorkData.iLatitude * 1e-7;
        // stLLHPoint.fLongitude = m_stINS2VCWorkData.iLongitude * 1e-7;
        // stLLHPoint.fHeight = m_stINS2VCWorkData.iHeight * 1e-3f;

        // ENUCorST stFrontWheelENUPos = LLH2ENU(stLLHPoint, g_stMapCorditCmd.stLLHOriginPos);
        // /*将当前点转成东北天坐标系*/ double fVehCourse = m_stINS2VCWorkData.uiVehCourse * 1e-3; /*当前车辆航向*/
        // /*根据车头位置解算质心位置*/
        // ENUCorST stVehCenterENUPos; /*解算车辆质心位置*/
        // memset(&stVehCenterENUPos, 0, sizeof(ENUCorST));
        // stVehCenterENUPos.x = stFrontWheelENUPos.x - lf_ * sin(fVehCourse * DEGREE_RADIAN);
        // stVehCenterENUPos.y = stFrontWheelENUPos.y - lf_ * cos(fVehCourse * DEGREE_RADIAN);
        // stVehCenterENUPos.z = stFrontWheelENUPos.z;

        // SmoothPathPointST stTransVehCenterENUPos;
        // memset(&stTransVehCenterENUPos, 0, sizeof(SmoothPathPointST));
        // stTransVehCenterENUPos.stENUPoint.x = stVehCenterENUPos;

        m_listPathENU.clear(); /*每次点击开始路径点采集都先清空已采集路径点列表*/
        // m_listPathENU.append(stTransVehCenterENUPos); /*存储第一个点*/
        //         emit PathCollectDisp(m_listPathENU);          /*显示采集到的路径点*/
        m_timerPathCollect->start(50); /*开启50ms定时器进行路径存储*/
      }
    }
    else
    {
      QMessageBox::information(Q_NULLPTR, "提示", "地图原点未设置，不支持进行路径点采集");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "MEMS未处于双天线差分，不支持进行路径点采集");
  }
}

void PosMeasureDlg::on_btnPathPointClear_clicked() /*清空已采集路径点按钮单击槽函数*/
{
  m_listPathENU.clear(); /*清空已采集的路径点*/
  m_timerPathCollect->stop();
}

void PosMeasureDlg::on_btnPathFileSave_clicked() /*路径文件存储按钮单击槽函数*/
{
  if (0 != m_listPathENU.size()) /*已采集好了路径数据*/
  {
    m_timerPathCollect->stop();
    QDateTime dateTime = QDateTime::currentDateTime();
    QString strTime =
        "D:/PathFile/" + dateTime.toString("yyyyMMdd hhmmss") + ".txt"; /*window中文件名不能有冒号，会创建文件失败*/
    QString strName = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("文件另存为"), strTime,
                                                   tr("Vertex Files(*.txt)")); /*返回绝对路径+文件名*/

    /*获取文件名*/
    INT32 pos = strName.lastIndexOf('/');
    QString strfileName = strName.right(strName.size() - pos - 1);
    qDebug() << "strfileName:" << strfileName;
    /*获取文件路径*/
    QString strfilePath = strName.left(pos + 1);
    qDebug() << "strfilePath:" << strfilePath;

    if (true == createFile(strfilePath, strfileName)) /*创建记录文件*/
    {
      QTextStream text_stream(&m_objVertexFile);
      text_stream.setCodec("utf-8");
      QString strInfo = QString("原点 经度 %1 纬度 %2 高度 %3\n")
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fLongitude, 0, 'f', 7)
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fLatitude, 0, 'f', 7)
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fHeight, 0, 'f', 3);
      text_stream << strInfo;

      // compute the path length
      double sum = 0.0;
      if (m_listPathENU.size() == 1)
      {
        sum = 0.0;
      }
      else
      {
        for (int i = 1; i < m_listPathENU.size(); ++i)
        {
          double dx = m_listPathENU.at(i).stENUPoint.x - m_listPathENU.at(i - 1).stENUPoint.x;
          double dy = m_listPathENU.at(i).stENUPoint.y - m_listPathENU.at(i - 1).stENUPoint.y;
          double dist = std::sqrt(dx * dx + dy * dy);
          sum += dist;
        }
      }

      if (sum > 10)
      {
        float fDistToEnd = 0; /*取路径最后一个点的前方10m处为终点*/
        INT32 iEnd;
        for (iEnd = m_listPathENU.size() - 1; iEnd > 1; iEnd--)
        {
          fDistToEnd += sqrt(pow((m_listPathENU.at(iEnd).stENUPoint.x - m_listPathENU.at(iEnd - 1).stENUPoint.x), 2) +
                             pow((m_listPathENU.at(iEnd).stENUPoint.y - m_listPathENU.at(iEnd - 1).stENUPoint.y), 2));
          if (fDistToEnd > 10)
          {
            break;
          }
        }
        strInfo = QString("路径终点 X %1 Y %2\n")
                      .arg(m_listPathENU.at(iEnd - 1).stENUPoint.x, 0, 'f', 2)
                      .arg(m_listPathENU.at(iEnd - 1).stENUPoint.y, 0, 'f', 2);
        text_stream << strInfo;
      }
      else
      {
        strInfo = QString("路径终点 X %1 Y %2\n")
                      .arg(m_listPathENU.back().stENUPoint.x, 0, 'f', 2)
                      .arg(m_listPathENU.back().stENUPoint.y, 0, 'f', 2);
        text_stream << strInfo;
      }

      for (INT32 i = 0; i < m_listPathENU.size(); i++)
      {
        QString strInfo = QString("路径点%1 X %2 Y %3\n")
                              .arg(i + 1)
                              .arg(m_listPathENU.at(i).stENUPoint.x, 0, 'f', 2)
                              .arg(m_listPathENU.at(i).stENUPoint.y, 0, 'f', 2);
        text_stream << strInfo;
      }
      m_objVertexFile.flush();
      m_objVertexFile.close();
      QMessageBox::information(Q_NULLPTR, "提示", "路径点存储完毕");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "需首先完成路径采集");
  }
}

void PosMeasureDlg::on_btnPathFileOpen_clicked() /*打开路径文件*/
{
  QString strMapName = QFileDialog::getOpenFileName(this, tr("打开路径文件"), "D:/CF/3 PathFile/",
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
      QMessageBox::critical(this, "critical", tr("路径文件打开失败"), QMessageBox::Yes, QMessageBox::Yes);
    }
    else
    {
      m_listTrackPathCmd.clear(); /*清空跟踪路径命令列表*/

      QByteArray array; /*读取信息暂存变量*/
      array.clear();
      while (false == objPathFile.atEnd()) /*未到文件尾*/
      {
        array = objPathFile.readLine(); /*读取一行*/
        QString str;
        str.prepend(array);
        if (true == str.contains("原点"))
        {
          INT32 iPos[21] = { 0 };
          INT32 iPosTemp, iCnt = 0;
          while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
          {
            iPos[iCnt] = iPosTemp;
            iCnt++; /*查找到TAB键的次数*/
            if (iCnt > 20)
            {
              break;
            }
          }

          QString strLongOrigin = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1); /*取经度值*/
          QString strLatOrigin = str.mid(iPos[3] + 1, iPos[4] - iPos[3] - 1);  /*取纬度值*/
          QString strHeightOrigin = str.right(str.length() - iPos[5] - 1);     /*取高度值*/
          strHeightOrigin = strHeightOrigin.left(strHeightOrigin.length() - 1);

          /*将原点经纬度信息显示到界面中*/
          ui->labOriginLongtitude->setText(strLongOrigin);
          ui->labOriginLatitude->setText(strLatOrigin);
          ui->labOriginHeight->setText(strHeightOrigin);
        }
        else if (true == str.contains("路径终点"))
        {
          INT32 iPos[21] = { 0 };
          INT32 iPosTemp, iCnt = 0;
          while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
          {
            iPos[iCnt] = iPosTemp;
            iCnt++; /*查找到TAB键的次数*/
            if (iCnt > 20)
            {
              break;
            }
          }

          QString strLLHEndPosX = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1);
          QString strLLHEndPosY = str.right(str.length() - iPos[3] - 1);

          ui->labMapVertex3ENUX->setText(strLLHEndPosX);
          ui->labMapVertex3ENUY->setText(strLLHEndPosY);
        }
        else if (true == str.contains("路径点"))
        {
          INT32 iPos[21] = { 0 };
          INT32 iPosTemp, iCnt = 0;
          while (-1 != (iPosTemp = str.indexOf(" ", ((0 == iCnt) ? 0 : iPos[iCnt - 1] + 1)))) /*查找空格键*/
          {
            iPos[iCnt] = iPosTemp;
            iCnt++; /*查找到TAB键的次数*/
            if (iCnt > 20)
            {
              break;
            }
          }

          QString strPathENUX = str.mid(iPos[1] + 1, iPos[2] - iPos[1] - 1);
          QString strPathENUY = str.mid(iPos[3] + 1, iPos[4] - iPos[3] - 1);
          QString strAngle2Y = str.mid(iPos[5] + 1, iPos[6] - iPos[5] - 1);
          QString strCurve = str.mid(iPos[7] + 1, iPos[8] - iPos[7] - 1);
          QString strCurveDiff = str.right(str.length() - iPos[9] - 1);
          ENUCorST stPathENU;
          memset(&stPathENU, 0, sizeof(ENUCorST));
          stPathENU.x = strPathENUX.toDouble();
          stPathENU.y = strPathENUY.toDouble();

          /*增加存储平滑后的路径信息数据*/
          SmoothPathPointST stSmoothPathPoint;
          memset(&stSmoothPathPoint, 0, sizeof(SmoothPathPointST));
          stSmoothPathPoint.stENUPoint = stPathENU;
          stSmoothPathPoint.fAngle2X_Rad = strAngle2Y.toDouble();
          stSmoothPathPoint.fCurve = strCurve.toDouble();
          stSmoothPathPoint.fCurveDiff = strCurveDiff.toDouble();
          m_listTrackPathCmd.append(stSmoothPathPoint);
        }
        else
        {
          /*不处理*/
        }
      }
      objPathFile.close();

      //车辆停止点
      ui->labMapVertex2ENUX->setText(QString::number(m_listTrackPathCmd.last().stENUPoint.x));
      ui->labMapVertex2ENUY->setText(QString::number(m_listTrackPathCmd.last().stENUPoint.y));

      ui->btnPathFileEditSave->setEnabled(true);
      ui->btnCutDistanceToEnd->setEnabled(true);
      ui->btnMapVertex2Set->setEnabled(true);
    }
  }
  else
  {
    QMessageBox::critical(this, "critical", tr("取消了路径文件打开操作"), QMessageBox::Yes, QMessageBox::Yes);
  }
}

void PosMeasureDlg::on_btnPathFileEditSave_clicked() /*保存编辑的路径*/
{
  if (0 != m_listTrackPathCmd.size()) /*已采集好了路径数据*/
  {
    //首先把路径按照终点裁剪；
    float fMinDist = 10000.0;
    float fEndX = ui->labMapVertex2ENUX->text().toFloat();
    float fEndY = ui->labMapVertex2ENUY->text().toFloat();
    int iCount = 0;

    for (int i = 0; i < m_listTrackPathCmd.size(); ++i)
    {
      float fDist = sqrt(pow((m_listTrackPathCmd.at(i).stENUPoint.x - fEndX), 2) +
                         pow((m_listTrackPathCmd.at(i).stENUPoint.y - fEndY), 2));
      if (fDist < fMinDist)
      {
        iCount = i;
        fMinDist = fDist;
      }
    }

    m_listTrackPathCmd.resize(iCount + 1);

    QDateTime dateTime = QDateTime::currentDateTime();
    QString strTime = "D:/CF/3 PathFile/" + dateTime.toString("yyyyMMdd hhmmss") +
                      ".txt"; /*window中文件名不能有冒号，会创建文件失败*/
    QString strName = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("文件另存为"), strTime,
                                                   tr("Smooth Path Files(*.txt)")); /*返回绝对路径+文件名*/

    /*获取文件名*/
    INT32 pos = strName.lastIndexOf('/');
    QString strfileName = strName.right(strName.size() - pos - 1);
    qDebug() << "strfileName:" << strfileName;
    /*获取文件路径*/
    QString strfilePath = strName.left(pos + 1);
    qDebug() << "strfilePath:" << strfilePath;

    if (true == createFile(strfilePath, strfileName)) /*创建记录文件*/
    {
      QTextStream text_stream(&m_objVertexFile);

      text_stream.setCodec("utf-8");

      QString strInfo = QString("原点 经度 %1 纬度 %2 高度 %3\n")
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fLongitude, 0, 'f', 7)
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fLatitude, 0, 'f', 7)
                            .arg(g_stMapCorditCmd.stLLHOriginPos.fHeight, 0, 'f', 3);
      text_stream << strInfo;

      float fDistToEnd = 0; /*取路径最后一个点的前方10m处为终点*/
      INT32 iEnd;
      for (iEnd = m_listTrackPathCmd.size() - 1; iEnd > 1; iEnd--)
      {
        fDistToEnd +=
            sqrt(pow((m_listTrackPathCmd.at(iEnd).stENUPoint.x - m_listTrackPathCmd.at(iEnd - 1).stENUPoint.x), 2) +
                 pow((m_listTrackPathCmd.at(iEnd).stENUPoint.y - m_listTrackPathCmd.at(iEnd - 1).stENUPoint.y), 2));
        if (fDistToEnd > 10)
        {
          break;
        }
      }

      strInfo = QString("路径终点 X %1 Y %2\n")
                    .arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.x, 0, 'f', 2)
                    .arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.y, 0, 'f', 2);

      ui->labMapVertex2ENUX->setText(QString("%1").arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.x, 0, 'f', 2));
      ui->labMapVertex2ENUY->setText(QString("%1").arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.y, 0, 'f', 2));

      text_stream << strInfo;

      for (INT32 i = 0; i < m_listTrackPathCmd.size(); i++)
      {
        /*1217修改路径保存序号*/
        QString strInfo = QString("路径点%1 X %2 Y %3 Heading %4 Kappa %5 Dkappa %6\n")
                              .arg(i + 1)
                              .arg(m_listTrackPathCmd.at(i).stENUPoint.x, 0, 'f', 2)
                              .arg(m_listTrackPathCmd.at(i).stENUPoint.y, 0, 'f', 2)
                              .arg(m_listTrackPathCmd.at(i).fAngle2X_Rad, 0, 'f', 4)
                              .arg(m_listTrackPathCmd.at(i).fCurve, 0, 'f', 4)
                              .arg(m_listTrackPathCmd.at(i).fCurveDiff, 0, 'f', 4);
        text_stream << strInfo;
      }

      ui->labMapVertex3ENUX->setText(QString::number(m_listTrackPathCmd.last().stENUPoint.x));
      ui->labMapVertex3ENUY->setText(QString::number(m_listTrackPathCmd.last().stENUPoint.y));
      m_objVertexFile.flush();
      m_objVertexFile.close();
      QMessageBox::information(Q_NULLPTR, "提示", "编辑后的路径点存储完毕");
    }
  }
  else
  {
    QMessageBox::information(Q_NULLPTR, "提示", "首先需要打开路径");
  }
}

void PosMeasureDlg::on_btnCutDistanceToEnd_clicked()
{
  if (m_listTrackPathCmd.empty())
  {
    QMessageBox::information(Q_NULLPTR, "提示", "路径为空，首先需要打开路径");
    return;
  }

  double fDist = ui->labCutDistanceToEnd->text().toDouble();

  if (fDist <= 0)
  {
    QMessageBox::information(Q_NULLPTR, "提示", "输入值要大于0");
    return;
  }

  double fSum = 0;
  int iStopPointIndex = -1;
  for (int i = m_listTrackPathCmd.size() - 1; i >= 1; --i)
  {
    fSum += sqrt(pow((m_listTrackPathCmd.at(i).stENUPoint.x - m_listTrackPathCmd.at(i - 1).stENUPoint.x), 2) +
                 pow((m_listTrackPathCmd.at(i).stENUPoint.y - m_listTrackPathCmd.at(i - 1).stENUPoint.y), 2));

    if (fSum >= fDist)
    {
      iStopPointIndex = i;
      break;
    }
    if (1 == i)
    {
      iStopPointIndex = i;
      break;
    }
  }

  if (iStopPointIndex < 10)
  {
    QMessageBox::information(Q_NULLPTR, "提示", "路径点过少，不支持裁剪");
    return;
  }

  ui->labMapVertex2ENUX->setText(QString::number(m_listTrackPathCmd.at(iStopPointIndex - 1).stENUPoint.x));
  ui->labMapVertex2ENUY->setText(QString::number(m_listTrackPathCmd.at(iStopPointIndex - 1).stENUPoint.y));

  float fDistToEnd = 0; /*取路径最后一个点的前方10m处为终点*/
  INT32 iEnd;
  for (iEnd = iStopPointIndex; iEnd > 1; iEnd--)
  {
    fDistToEnd +=
        sqrt(pow((m_listTrackPathCmd.at(iEnd).stENUPoint.x - m_listTrackPathCmd.at(iEnd - 1).stENUPoint.x), 2) +
             pow((m_listTrackPathCmd.at(iEnd).stENUPoint.y - m_listTrackPathCmd.at(iEnd - 1).stENUPoint.y), 2));
    if (fDistToEnd > 10)
    {
      break;
    }
  }

  ui->labMapVertex3ENUX->setText(QString("%1").arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.x, 0, 'f', 2));
  ui->labMapVertex3ENUY->setText(QString("%1").arg(m_listTrackPathCmd.at(iEnd - 1).stENUPoint.y, 0, 'f', 2));
}
