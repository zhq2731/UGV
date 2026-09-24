#pragma once

#include <QPushButton>
#include <QWidget>
#include <array>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/panel.h>

#include "driver_msgs/MotionStartCmd.h"
#include "std_msgs/Empty.h"

namespace launch_node
{

class ThreeVehicleMotionPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit ThreeVehicleMotionPanel(QWidget* parent = nullptr);

private Q_SLOTS:
  void startVehicle1();
  void stopVehicle1();
  void confirmNextSegmentVehicle1();
  void startVehicle2();
  void stopVehicle2();
  void confirmNextSegmentVehicle2();
  void startVehicle3();
  void stopVehicle3();
  void confirmNextSegmentVehicle3();
  void startAll();
  void stopAll();
  void startTaskScheduler();

private:
  void publishMotion(std::size_t vehicle_index, unsigned char motion_start);
  void publishNextSegmentConfirm(std::size_t vehicle_index);

  ros::NodeHandle nh_;
  std::array<ros::Publisher, 3> motion_publishers_;
  std::array<ros::Publisher, 3> next_segment_publishers_;
  ros::Publisher task_scheduler_start_pub_;
};

}  // namespace launch_node
