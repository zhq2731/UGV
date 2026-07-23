#include "launch_node/single_vehicle_motion_panel.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

#include "driver_msgs/MotionStartCmd.h"

namespace launch_node
{

SingleVehicleMotionPanel::SingleVehicleMotionPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  motion_pub_ = nh_.advertise<driver_msgs::MotionStartCmd>("/chassis_motion_start_cmd", 1);

  auto* start_button = new QPushButton("开始运动", this);
  auto* stop_button = new QPushButton("停止运动", this);
  start_button->setStyleSheet("QPushButton { background-color: #2e7d32; color: white; padding: 6px; }");
  stop_button->setStyleSheet("QPushButton { background-color: #c62828; color: white; padding: 6px; }");

  auto* buttons = new QHBoxLayout();
  buttons->addWidget(start_button);
  buttons->addWidget(stop_button);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->addLayout(buttons);
  setLayout(layout);

  connect(start_button, SIGNAL(clicked()), this, SLOT(startMotion()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(stopMotion()));
}

void SingleVehicleMotionPanel::publishMotion(unsigned char motion_start)
{
  driver_msgs::MotionStartCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.motion_start = motion_start;
  motion_pub_.publish(msg);
}

void SingleVehicleMotionPanel::startMotion()
{
  publishMotion(1);
}

void SingleVehicleMotionPanel::stopMotion()
{
  publishMotion(0);
}

}  // namespace launch_node

PLUGINLIB_EXPORT_CLASS(launch_node::SingleVehicleMotionPanel, rviz::Panel)
