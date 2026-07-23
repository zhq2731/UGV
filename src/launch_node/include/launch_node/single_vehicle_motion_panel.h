#pragma once

#include <QWidget>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <rviz/panel.h>

namespace launch_node
{

/**
 * @brief 在 RViz 中提供单车泊车仿真的开始与停止按钮
 *
 * 面板只负责发布 /chassis_motion_start_cmd，不参与规划或控制状态判断。
 */
class SingleVehicleMotionPanel : public rviz::Panel
{
  Q_OBJECT

public:
  explicit SingleVehicleMotionPanel(QWidget* parent = nullptr);

private Q_SLOTS:
  /** @brief 发布 motion_start=1，允许仿真车执行控制命令。 */
  void startMotion();
  /** @brief 发布 motion_start=0，停止仿真车的控制积分。 */
  void stopMotion();

private:
  /** @brief 组装并发布运动开关消息。 */
  void publishMotion(unsigned char motion_start);

  ros::NodeHandle nh_;
  ros::Publisher motion_pub_;
};

}  // namespace launch_node
