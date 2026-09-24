#include "launch_node/three_vehicle_motion_panel.h"

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

QGroupBox* makeVehicleGroup(QWidget* parent,
                            const QString& title,
                            QPushButton** start,
                            QPushButton** stop,
                            QPushButton** next)
{
  auto* group = new QGroupBox(title, parent);
  auto* layout = new QGridLayout(group);
  layout->setContentsMargins(4, 6, 4, 4);
  layout->setHorizontalSpacing(4);
  layout->setVerticalSpacing(4);

  *start = new QPushButton("Go", group);
  *stop = new QPushButton("Stop", group);
  *next = new QPushButton("Next", group);
  compactButton(*start);
  compactButton(*stop);
  compactButton(*next);
  layout->addWidget(*start, 0, 0);
  layout->addWidget(*stop, 1, 0);
  layout->addWidget(*next, 2, 0);
  return group;
}

}  // namespace

ThreeVehicleMotionPanel::ThreeVehicleMotionPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  for (std::size_t i = 0; i < motion_publishers_.size(); ++i) {
    const std::string vehicle_ns = "/vehicle_" + std::to_string(i + 1);
    motion_publishers_[i] = nh_.advertise<driver_msgs::MotionStartCmd>(
        vehicle_ns + "/chassis_motion_start_cmd", 1);
    next_segment_publishers_[i] = nh_.advertise<std_msgs::Empty>(
        vehicle_ns + "/next_route_segment", 1);
  }
  task_scheduler_start_pub_ =
      nh_.advertise<std_msgs::Empty>("/task_scheduler/start", 1);

  QPushButton* vehicle_1_start = nullptr;
  QPushButton* vehicle_1_stop = nullptr;
  QPushButton* vehicle_1_next = nullptr;
  QPushButton* vehicle_2_start = nullptr;
  QPushButton* vehicle_2_stop = nullptr;
  QPushButton* vehicle_2_next = nullptr;
  QPushButton* vehicle_3_start = nullptr;
  QPushButton* vehicle_3_stop = nullptr;
  QPushButton* vehicle_3_next = nullptr;

  auto* root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(2, 2, 2, 2);
  root_layout->setSpacing(4);

  auto* task_group = new QGroupBox("Task", this);
  auto* task_layout = new QGridLayout(task_group);
  auto* task_start = new QPushButton("Start", task_group);
  compactButton(task_start);
  task_layout->addWidget(task_start, 0, 0);
  root_layout->addWidget(task_group);

  root_layout->addWidget(makeVehicleGroup(
      this, "V1 Blue", &vehicle_1_start, &vehicle_1_stop, &vehicle_1_next));
  root_layout->addWidget(makeVehicleGroup(
      this, "V2 Yellow", &vehicle_2_start, &vehicle_2_stop, &vehicle_2_next));
  root_layout->addWidget(makeVehicleGroup(
      this, "V3 Red", &vehicle_3_start, &vehicle_3_stop, &vehicle_3_next));

  auto* all_group = new QGroupBox("All", this);
  auto* all_layout = new QGridLayout(all_group);
  auto* all_start = new QPushButton("Go", all_group);
  auto* all_stop = new QPushButton("Stop", all_group);
  compactButton(all_start);
  compactButton(all_stop);
  all_layout->addWidget(all_start, 0, 0);
  all_layout->addWidget(all_stop, 1, 0);
  root_layout->addWidget(all_group);
  root_layout->addStretch();
  setLayout(root_layout);

  connect(vehicle_1_start, SIGNAL(clicked()), this, SLOT(startVehicle1()));
  connect(vehicle_1_stop, SIGNAL(clicked()), this, SLOT(stopVehicle1()));
  connect(vehicle_1_next, SIGNAL(clicked()), this, SLOT(confirmNextSegmentVehicle1()));
  connect(vehicle_2_start, SIGNAL(clicked()), this, SLOT(startVehicle2()));
  connect(vehicle_2_stop, SIGNAL(clicked()), this, SLOT(stopVehicle2()));
  connect(vehicle_2_next, SIGNAL(clicked()), this, SLOT(confirmNextSegmentVehicle2()));
  connect(vehicle_3_start, SIGNAL(clicked()), this, SLOT(startVehicle3()));
  connect(vehicle_3_stop, SIGNAL(clicked()), this, SLOT(stopVehicle3()));
  connect(vehicle_3_next, SIGNAL(clicked()), this, SLOT(confirmNextSegmentVehicle3()));
  connect(all_start, SIGNAL(clicked()), this, SLOT(startAll()));
  connect(all_stop, SIGNAL(clicked()), this, SLOT(stopAll()));
  connect(task_start, SIGNAL(clicked()), this, SLOT(startTaskScheduler()));
}

void ThreeVehicleMotionPanel::publishMotion(
    std::size_t vehicle_index, unsigned char motion_start)
{
  driver_msgs::MotionStartCmd msg;
  msg.header.stamp = ros::Time::now();
  msg.motion_start = motion_start;
  motion_publishers_.at(vehicle_index).publish(msg);
}

void ThreeVehicleMotionPanel::publishNextSegmentConfirm(std::size_t vehicle_index)
{
  std_msgs::Empty msg;
  next_segment_publishers_.at(vehicle_index).publish(msg);
}

void ThreeVehicleMotionPanel::startVehicle1() { publishMotion(0, 1); }
void ThreeVehicleMotionPanel::stopVehicle1() { publishMotion(0, 0); }
void ThreeVehicleMotionPanel::confirmNextSegmentVehicle1() { publishNextSegmentConfirm(0); }
void ThreeVehicleMotionPanel::startVehicle2() { publishMotion(1, 1); }
void ThreeVehicleMotionPanel::stopVehicle2() { publishMotion(1, 0); }
void ThreeVehicleMotionPanel::confirmNextSegmentVehicle2() { publishNextSegmentConfirm(1); }
void ThreeVehicleMotionPanel::startVehicle3() { publishMotion(2, 1); }
void ThreeVehicleMotionPanel::stopVehicle3() { publishMotion(2, 0); }
void ThreeVehicleMotionPanel::confirmNextSegmentVehicle3() { publishNextSegmentConfirm(2); }

void ThreeVehicleMotionPanel::startAll()
{
  startVehicle1();
  startVehicle2();
  startVehicle3();
}

void ThreeVehicleMotionPanel::stopAll()
{
  stopVehicle1();
  stopVehicle2();
  stopVehicle3();
}

void ThreeVehicleMotionPanel::startTaskScheduler()
{
  std_msgs::Empty msg;
  task_scheduler_start_pub_.publish(msg);
}

}  // namespace launch_node

PLUGINLIB_EXPORT_CLASS(launch_node::ThreeVehicleMotionPanel, rviz::Panel)
