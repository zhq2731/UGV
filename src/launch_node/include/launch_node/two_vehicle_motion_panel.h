#pragma once

#include <QPushButton>
#include <QWidget>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/panel.h>

#include "driver_msgs/MotionStartCmd.h"

namespace launch_node
{

class TwoVehicleMotionPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit TwoVehicleMotionPanel(QWidget* parent = nullptr);

private Q_SLOTS:
  void startVehicle1();
  void stopVehicle1();
  void startVehicle2();
  void stopVehicle2();
  void startBoth();
  void stopBoth();

private:
  void publishMotion(const ros::Publisher& publisher, unsigned char motion_start);

  ros::NodeHandle nh_;
  ros::Publisher vehicle_1_motion_pub_;
  ros::Publisher vehicle_2_motion_pub_;
};

}  // namespace launch_node
