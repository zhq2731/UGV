#include "launch_node/two_vehicle_motion_panel.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace launch_node
{

namespace
{

void compactButton(QPushButton* button)
{
  button->setMinimumWidth(36);
  button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void compactGroup(QGroupBox* group, QGridLayout* layout)
{
  group->setMinimumWidth(0);
  group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  layout->setContentsMargins(4, 6, 4, 4);
  layout->setHorizontalSpacing(4);
  layout->setVerticalSpacing(4);
}

}  // namespace

TwoVehicleMotionPanel::TwoVehicleMotionPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  vehicle_1_motion_pub_ =
      nh_.advertise<driver_msgs::MotionStartCmd>("/vehicle_1/chassis_motion_start_cmd", 1);
  vehicle_2_motion_pub_ =
      nh_.advertise<driver_msgs::MotionStartCmd>("/vehicle_2/chassis_motion_start_cmd", 1);

  setMinimumWidth(0);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

  auto* vehicle_1_group = new QGroupBox("V1", this);
  auto* vehicle_1_layout = new QGridLayout(vehicle_1_group);
  auto* vehicle_1_start = new QPushButton("Go", vehicle_1_group);
  auto* vehicle_1_stop = new QPushButton("Stop", vehicle_1_group);
  compactButton(vehicle_1_start);
  compactButton(vehicle_1_stop);
  compactGroup(vehicle_1_group, vehicle_1_layout);
  vehicle_1_layout->addWidget(vehicle_1_start, 0, 0);
  vehicle_1_layout->addWidget(vehicle_1_stop, 1, 0);

  auto* vehicle_2_group = new QGroupBox("V2", this);
  auto* vehicle_2_layout = new QGridLayout(vehicle_2_group);
  auto* vehicle_2_start = new QPushButton("Go", vehicle_2_group);
  auto* vehicle_2_stop = new QPushButton("Stop", vehicle_2_group);
  compactButton(vehicle_2_start);
  compactButton(vehicle_2_stop);
  compactGroup(vehicle_2_group, vehicle_2_layout);
  vehicle_2_layout->addWidget(vehicle_2_start, 0, 0);
  vehicle_2_layout->addWidget(vehicle_2_stop, 1, 0);

  auto* both_group = new QGroupBox("All", this);
  auto* both_layout = new QGridLayout(both_group);
  auto* both_start = new QPushButton("Go", both_group);
  auto* both_stop = new QPushButton("Stop", both_group);
  compactButton(both_start);
  compactButton(both_stop);
  compactGroup(both_group, both_layout);
  both_layout->addWidget(both_start, 0, 0);
  both_layout->addWidget(both_stop, 1, 0);

  auto* root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(2, 2, 2, 2);
  root_layout->setSpacing(4);
  root_layout->addWidget(vehicle_1_group);
  root_layout->addWidget(vehicle_2_group);
  root_layout->addWidget(both_group);
  root_layout->addStretch();
  setLayout(root_layout);

  connect(vehicle_1_start, SIGNAL(clicked()), this, SLOT(startVehicle1()));
  connect(vehicle_1_stop, SIGNAL(clicked()), this, SLOT(stopVehicle1()));
  connect(vehicle_2_start, SIGNAL(clicked()), this, SLOT(startVehicle2()));
  connect(vehicle_2_stop, SIGNAL(clicked()), this, SLOT(stopVehicle2()));
  connect(both_start, SIGNAL(clicked()), this, SLOT(startBoth()));
  connect(both_stop, SIGNAL(clicked()), this, SLOT(stopBoth()));
}

void TwoVehicleMotionPanel::publishMotion(const ros::Publisher& publisher, unsigned char motion_start)
{
  driver_msgs::MotionStartCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.motion_start = motion_start;
  publisher.publish(msg);
}

void TwoVehicleMotionPanel::startVehicle1()
{
  publishMotion(vehicle_1_motion_pub_, 1);
}

void TwoVehicleMotionPanel::stopVehicle1()
{
  publishMotion(vehicle_1_motion_pub_, 0);
}

void TwoVehicleMotionPanel::startVehicle2()
{
  publishMotion(vehicle_2_motion_pub_, 1);
}

void TwoVehicleMotionPanel::stopVehicle2()
{
  publishMotion(vehicle_2_motion_pub_, 0);
}

void TwoVehicleMotionPanel::startBoth()
{
  startVehicle1();
  startVehicle2();
}

void TwoVehicleMotionPanel::stopBoth()
{
  stopVehicle1();
  stopVehicle2();
}

}  // namespace launch_node

PLUGINLIB_EXPORT_CLASS(launch_node::TwoVehicleMotionPanel, rviz::Panel)
