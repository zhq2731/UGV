#include "pad/parentdlg.h"
#include <QApplication>
#include "pad/alltypes.h"
#include "pad/qnode.h"
#include <vector>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  qRegisterMetaType<CANFrameST>("CANFrameST"); /*注册自定义类型*/
  qRegisterMetaType<UINT8>("UINT8");
  qRegisterMetaType<UINT16>("UINT16");
  qRegisterMetaType<UINT32>("UINT32");
  qRegisterMetaType<INT8>("INT8");
  qRegisterMetaType<INT16>("INT16");
  qRegisterMetaType<INT32>("INT32");
  qRegisterMetaType<QByteArray>("QByteArray");
  qRegisterMetaType<QHostAddress>("QHostAddress");
  qRegisterMetaType<GridMapIndexST>("GridMapIndexST");
  qRegisterMetaType<QList<GridMapIndexST *>>("QList<GridMapIndexST *>");
  qRegisterMetaType<QList<ENUCorST>>("QList<ENUCorST>");
  qRegisterMetaType<QList<SmoothPathPointST>>("QList<SmoothPathPointST>");
  qRegisterMetaType<QList<LocalPathPointST>>("QList<LocalPathPointST>");
  qRegisterMetaType<ENUCorST>("ENUCorST");
  qRegisterMetaType<SmoothPathPointST>("SmoothPathPointST");
  qRegisterMetaType<LocalPathPointST>("LocalPathPointST");
  qRegisterMetaType<LongLatHeightST>("LongLatHeightST");
  qRegisterMetaType<XW5651ST>("XW5651ST");
  qRegisterMetaType<VehicleChassisStateST>("VehicleChassisStateST");
  qRegisterMetaType<DilixinxiST>("DilixinxiST");
  qRegisterMetaType<FP2DilixinxiBindMsgST>("FP2DilixinxiBindMsgST");
  qRegisterMetaType<VehicleMotionStateST>("VehicleMotionStateST");
  qRegisterMetaType<std::pair<float, float>>("PairFST");
  qRegisterMetaType<std::vector<std::pair<float, float>>>("VPairFST");
  qRegisterMetaType<std::vector<std::vector<std::pair<float, float>>>>("VVPairFST");
  qRegisterMetaType<GNSS_IMU_ST>("GNSS_IMU_ST");
  qRegisterMetaType<GlobalPositionST>("GlobalPositionST");
  qRegisterMetaType<std::vector<GlobalPositionST>>("std::vector<GlobalPositionST>");
  qRegisterMetaType<std::vector<TaskPointST>>("std::vector<TaskPointST>");
  qRegisterMetaType<TaskPointST>("TaskPointST");
  qRegisterMetaType<std::vector<localPositionST>>("std::vector<localPositionST>");
  qRegisterMetaType<localPositionST>("localPositionST");
  qRegisterMetaType<RemoteDriveST>("RemoteDriveST");
  qRegisterMetaType<ObstacleST>("ObstacleST");
  qRegisterMetaType<std::vector<ObstacleST>>("std::vector<ObstacleST>");
  qRegisterMetaType<HeartBeatST>("HeartBeatST");
  qRegisterMetaType<ChassisLightHornWiper>("ChassisLightHornWiper");
  qRegisterMetaType<platoon_msgs::PlatoonMember::ConstPtr>("platoon_msgs::PlatoonMember::ConstPtr");
  qRegisterMetaType<platoon_msgs::PlatoonMission::ConstPtr>("platoon_msgs::PlatoonMission::ConstPtr");

  ParentDlg w(argc, argv);

  w.show();

  return a.exec();
}
