#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <driver_msgs/ChassisReport.h>
#include <driver_msgs/DriveCmd.h>
#include <driver_msgs/GearCmd.h>
#include <driver_msgs/MotionStartCmd.h>
#include <driver_msgs/ParkingBrakeCmd.h>
#include <driver_msgs/SteeringWheelCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <localization_msgs/Localization.h>
#include <nav_msgs/OccupancyGrid.h>
#include <planning_msgs/TrajectoryPointArray.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace {

constexpr std::size_t kChatterFieldCount = 18;
constexpr int8_t kPlannerFreeCell = 25;

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

geometry_msgs::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

double yawFromQuaternion(const geometry_msgs::Quaternion &quaternion)
{
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z +
             quaternion.x * quaternion.y),
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z));
}

bool isValidVehicleId(const double value)
{
  return std::isfinite(value) && value >= 0.0 &&
         std::fabs(value - std::round(value)) < 1.0e-6;
}

}  // namespace

/**
 * @brief 将比赛 Lite 标准话题适配到现有开放空间规划器。
 *
 * 该节点不生成轨迹或控制量，只完成两侧协议转换：状态、目标、地图和轨迹
 * 从Lite接入现有规划控制链路，控制器输出则汇总为Lite的单条车辆控制消息。
 * 状态和目标均以后轴中心为参考点，保持全局x、y、yaw不变；Lite全局地图
 * 则转换成以当前车辆为原点的局部栅格，满足开放空间规划器的输入约定。
 */
class LiteParkingBridge
{
public:
  LiteParkingBridge() : private_nh_("~")
  {
    private_nh_.param("vehicle_id", vehicle_id_, 0);
    private_nh_.param("map_crop_margin", map_crop_margin_, 20.0);
    private_nh_.param(
        "max_front_tire_angle_rad", max_front_tire_angle_rad_,
        32.0 * M_PI / 180.0);
    private_nh_.param("steering_ratio", steering_ratio_, 1.0);
    private_nh_.param(
        "enable_vehicle_control", enable_vehicle_control_, true);
    private_nh_.param("auto_motion_start", auto_motion_start_, true);
    private_nh_.param("control_publish_rate", control_publish_rate_, 50.0);
    private_nh_.param("control_command_timeout", control_command_timeout_, 0.5);
    max_front_tire_angle_rad_ =
        std::max(1.0e-3, std::fabs(max_front_tire_angle_rad_));
    control_publish_rate_ = std::max(1.0, control_publish_rate_);
    control_command_timeout_ = std::max(0.05, control_command_timeout_);

    localization_pub_ =
        nh_.advertise<localization_msgs::Localization>("odomData", 10);
    chassis_pub_ = nh_.advertise<driver_msgs::ChassisReport>("chassis", 10);
    planner_goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    planner_map_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("/free_space_map", 1, true);
    lite_trajectory_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("/SIM_trajectory", 1, true);
    motion_start_pub_ =
        nh_.advertise<driver_msgs::MotionStartCmd>(
            "chassis_motion_start_cmd", 1, false);
    if (enable_vehicle_control_) {
      lite_control_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>(
              "/vehicle_control_cmd", 10);
    }

    chatter_sub_ = nh_.subscribe(
        "/chatter", 10, &LiteParkingBridge::chatterCallback, this);
    goal_sub_ = nh_.subscribe(
        "/goal_pose", 10, &LiteParkingBridge::goalCallback, this);
    map_sub_ = nh_.subscribe(
        "/map", 1, &LiteParkingBridge::mapCallback, this);
    trajectory_sub_ = nh_.subscribe(
        "trajectory", 1, &LiteParkingBridge::trajectoryCallback, this);
    if (enable_vehicle_control_) {
      steering_command_sub_ = nh_.subscribe(
          "auto_chassis_steeringwheel_cmd", 1,
          &LiteParkingBridge::steeringCommandCallback, this);
      drive_command_sub_ = nh_.subscribe(
          "auto_chassis_drive_cmd", 1,
          &LiteParkingBridge::driveCommandCallback, this);
      gear_command_sub_ = nh_.subscribe(
          "auto_chassis_gear_cmd", 1,
          &LiteParkingBridge::gearCommandCallback, this);
      parking_brake_command_sub_ = nh_.subscribe(
          "auto_chassis_parking_brake_cmd", 1,
          &LiteParkingBridge::parkingBrakeCommandCallback, this);
      control_timer_ = nh_.createTimer(
          ros::Duration(1.0 / control_publish_rate_),
          &LiteParkingBridge::controlTimerCallback, this);
    }

    ROS_INFO(
        "[lite_bridge] ready: vehicle_id=%d, map_crop_margin=%.1f m, control=%s",
        vehicle_id_, map_crop_margin_,
        enable_vehicle_control_ ? "enabled" : "disabled");
  }

private:
  void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr &message)
  {
    if (message->data.size() % kChatterFieldCount != 0) {
      ROS_WARN_THROTTLE(
          1.0, "[lite_bridge] ignore chatter: data length is not a multiple of 18");
      return;
    }

    for (std::size_t offset = 0; offset < message->data.size();
         offset += kChatterFieldCount) {
      if (!isValidVehicleId(message->data[offset]) ||
          static_cast<int>(std::lround(message->data[offset])) != vehicle_id_) {
        continue;
      }
      for (std::size_t index = 0; index < kChatterFieldCount; ++index) {
        if (!std::isfinite(message->data[offset + index])) {
          ROS_WARN_THROTTLE(
              1.0, "[lite_bridge] ignore non-finite vehicle state");
          return;
        }
      }

      // Lite 历史协议按 y、x 排列；转换后恢复为规划器使用的 x、y。
      vehicle_x_ = message->data[offset + 2];
      vehicle_y_ = message->data[offset + 1];
      const double vehicle_z = message->data[offset + 3];
      const double yaw = message->data[offset + 7];
      vehicle_yaw_ = yaw;
      const double velocity_x = message->data[offset + 9];
      const double velocity_y = message->data[offset + 10];
      const double normalized_steer =
          clamp(message->data[offset + 13], -1.0, 1.0);
      actual_normalized_steer_ = normalized_steer;
      const bool reverse = message->data[offset + 15] >= 0.5 ||
                           message->data[offset + 17] < 0.0;
      const int gear = static_cast<int>(std::lround(message->data[offset + 17]));
      const bool first_vehicle_state = !has_vehicle_state_;
      has_vehicle_state_ = true;

      publishLocalization(vehicle_x_, vehicle_y_, vehicle_z, yaw);
      publishChassis(
          std::hypot(velocity_x, velocity_y), normalized_steer, reverse, gear,
          message->data[offset + 12], message->data[offset + 14]);
      // 兼容桥接节点晚于已锁存目标启动的情况，首帧状态到达后补发规划地图。
      if (first_vehicle_state) {
        publishPlanningMap();
      }
      return;
    }

    ROS_WARN_THROTTLE(
        2.0, "[lite_bridge] vehicle_id=%d is absent from chatter", vehicle_id_);
  }

  void publishLocalization(
    const double x, const double y, const double z, const double yaw)
  {
    localization_msgs::Localization localization;
    localization.location.header.stamp = ros::Time::now();
    localization.location.header.frame_id = "map";
    localization.location.pose.pose.position.x = x;
    localization.location.pose.pose.position.y = y;
    localization.location.pose.pose.position.z = z;
    localization.location.pose.pose.orientation = quaternionFromYaw(yaw);
    localization.is_valid = true;
    localization.original_ins.nav_uncertainty = 0.0;
    localization_pub_.publish(localization);
  }

  void publishChassis(
      const double speed, const double normalized_steer, const bool reverse,
      const int gear, const double throttle, const double brake)
  {
    driver_msgs::ChassisReport chassis;
    chassis.header.stamp = ros::Time::now();
    chassis.driving_mode = 1;
    chassis.current_velocity = speed;
    // Lite静止时原始gear会回到0；前进档反馈需结合最近一次已发出的档位命令，
    // 否则控制状态机会一直停留在SHIFT_GEAR。倒档仍以Lite的reverse反馈为准。
    chassis.gear_location =
        reverse ? 7 :
        (has_applied_gear_command_ && applied_gear_ == 1 ? 1 :
        (gear > 0 ? 1 : 0));

    // Lite 发布归一化前轮转角；现有规划器从方向盘角和转向比还原前轮角。
    const double front_tire_angle =
        normalized_steer * max_front_tire_angle_rad_;
    chassis.front_wheel_angle = front_tire_angle * 180.0 / M_PI;
    chassis.steering_wheel_angle =
        chassis.front_wheel_angle * steering_ratio_;
    chassis.throttle_pedal = clamp(throttle, 0.0, 1.0) * 100.0;
    chassis.brake_pedal = clamp(brake, 0.0, 1.0) * 100.0;
    chassis_pub_.publish(chassis);
  }

  void goalCallback(const std_msgs::Float64MultiArray::ConstPtr &message)
  {
    if (message->data.size() != 5 || !isValidVehicleId(message->data[0])) {
      ROS_WARN("[lite_bridge] ignore goal_pose: expected [id,x,y,z,yaw]");
      return;
    }
    if (static_cast<int>(std::lround(message->data[0])) != vehicle_id_) {
      return;
    }
    if (!std::all_of(
            message->data.begin(), message->data.end(),
            [](const double value) { return std::isfinite(value); })) {
      ROS_WARN("[lite_bridge] ignore goal_pose containing NaN or Inf");
      return;
    }

    goal_x_ = message->data[1];
    goal_y_ = message->data[2];
    has_goal_ = true;
    motion_started_for_goal_ = false;

    // 目标改变时先更新局部规划地图，再提交目标，避免沿用上一任务的裁剪区域。
    publishPlanningMap();

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.position.z = message->data[3];
    goal.pose.orientation = quaternionFromYaw(message->data[4]);
    planner_goal_pub_.publish(goal);

    ROS_INFO(
        "[lite_bridge] goal: vehicle=%d, x=%.3f, y=%.3f, yaw=%.3f rad",
        vehicle_id_, goal_x_, goal_y_, message->data[4]);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &message)
  {
    if (message->info.resolution <= 0.0 || message->info.width == 0 ||
        message->info.height == 0 ||
        message->data.size() !=
            static_cast<std::size_t>(message->info.width) *
                message->info.height) {
      ROS_WARN_THROTTLE(1.0, "[lite_bridge] ignore invalid occupancy grid");
      return;
    }
    latest_map_ = *message;
    has_map_ = true;
    publishPlanningMap();
  }

  void publishPlanningMap()
  {
    if (!has_map_ || !has_vehicle_state_ || !has_goal_) {
      return;
    }

    const double resolution = latest_map_.info.resolution;
    const double margin = std::max(0.0, map_crop_margin_);
    const double cos_vehicle_yaw = std::cos(vehicle_yaw_);
    const double sin_vehicle_yaw = std::sin(vehicle_yaw_);
    const double goal_dx = goal_x_ - vehicle_x_;
    const double goal_dy = goal_y_ - vehicle_y_;
    const double goal_local_x =
        cos_vehicle_yaw * goal_dx + sin_vehicle_yaw * goal_dy;
    const double goal_local_y =
        -sin_vehicle_yaw * goal_dx + cos_vehicle_yaw * goal_dy;

    // 规划器始终以当前车辆后轴中心为(0,0,0)，因此输出地图范围也必须使用
    // 车辆局部坐标，不能沿用Lite全局地图中的原点。
    const double local_min_x =
        std::floor((std::min(0.0, goal_local_x) - margin) / resolution) *
        resolution;
    const double local_max_x =
        std::ceil((std::max(0.0, goal_local_x) + margin) / resolution) *
        resolution;
    const double local_min_y =
        std::floor((std::min(0.0, goal_local_y) - margin) / resolution) *
        resolution;
    const double local_max_y =
        std::ceil((std::max(0.0, goal_local_y) + margin) / resolution) *
        resolution;

    nav_msgs::OccupancyGrid planning_map;
    planning_map.header.stamp = ros::Time::now();
    planning_map.header.frame_id = "base_link";
    planning_map.info.map_load_time = planning_map.header.stamp;
    planning_map.info.resolution = resolution;
    planning_map.info.width = static_cast<unsigned int>(
        std::ceil((local_max_x - local_min_x) / resolution));
    planning_map.info.height = static_cast<unsigned int>(
        std::ceil((local_max_y - local_min_y) / resolution));
    planning_map.info.origin.position.x = local_min_x;
    planning_map.info.origin.position.y = local_min_y;
    planning_map.info.origin.orientation.w = 1.0;
    planning_map.data.resize(
        static_cast<std::size_t>(planning_map.info.width) *
        planning_map.info.height, -1);

    const double source_origin_x = latest_map_.info.origin.position.x;
    const double source_origin_y = latest_map_.info.origin.position.y;
    const double source_origin_yaw =
        yawFromQuaternion(latest_map_.info.origin.orientation);
    const double cos_source_yaw = std::cos(source_origin_yaw);
    const double sin_source_yaw = std::sin(source_origin_yaw);

    // 对每个局部格中心反投影到Lite全局地图。规划器采用“仅值25可通行”
    // 的安全语义，因此源地图之外、未知格和占用格都不会被误判为自由区。
    for (unsigned int row = 0; row < planning_map.info.height; ++row) {
      for (unsigned int column = 0; column < planning_map.info.width; ++column) {
        const double local_x =
            local_min_x + (static_cast<double>(column) + 0.5) * resolution;
        const double local_y =
            local_min_y + (static_cast<double>(row) + 0.5) * resolution;
        const double world_x =
            vehicle_x_ + cos_vehicle_yaw * local_x -
            sin_vehicle_yaw * local_y;
        const double world_y =
            vehicle_y_ + sin_vehicle_yaw * local_x +
            cos_vehicle_yaw * local_y;
        const double source_dx = world_x - source_origin_x;
        const double source_dy = world_y - source_origin_y;
        const long source_column = static_cast<long>(std::floor(
            (cos_source_yaw * source_dx + sin_source_yaw * source_dy) /
            resolution));
        const long source_row = static_cast<long>(std::floor(
            (-sin_source_yaw * source_dx + cos_source_yaw * source_dy) /
            resolution));
        if (source_column < 0 || source_row < 0 ||
            source_column >= static_cast<long>(latest_map_.info.width) ||
            source_row >= static_cast<long>(latest_map_.info.height)) {
          continue;
        }

        const std::size_t source_index =
            static_cast<std::size_t>(source_row) * latest_map_.info.width +
            static_cast<std::size_t>(source_column);
        const std::size_t target_index =
            static_cast<std::size_t>(row) * planning_map.info.width + column;
        const int8_t source_value = latest_map_.data[source_index];
        planning_map.data[target_index] =
            source_value == 0 ? kPlannerFreeCell : source_value;
      }
    }
    planner_map_pub_.publish(planning_map);

    ROS_INFO(
        "[lite_bridge] local planning map: %u x %u cells, "
        "goal=(%.2f, %.2f), resolution=%.2f m",
        planning_map.info.width, planning_map.info.height,
        goal_local_x, goal_local_y, resolution);
  }

  void trajectoryCallback(
      const planning_msgs::TrajectoryPointArray::ConstPtr &message)
  {
    std_msgs::Float64MultiArray lite_trajectory;
    lite_trajectory.data.reserve(1 + 4 * message->points.size());
    lite_trajectory.data.push_back(static_cast<double>(vehicle_id_));
    for (const auto &point : message->points) {
      lite_trajectory.data.push_back(point.x);
      lite_trajectory.data.push_back(point.y);
      lite_trajectory.data.push_back(point.z);
      lite_trajectory.data.push_back(point.theta);
    }
    lite_trajectory_pub_.publish(lite_trajectory);
    if (auto_motion_start_ && has_goal_ && !message->points.empty() &&
        !motion_started_for_goal_) {
      // 新目标会先触发控制器任务重置；首条有效轨迹到达后再自动允许运动，
      // 避免规划失败或旧锁存目标导致车辆提前进入执行状态。
      driver_msgs::MotionStartCmd start;
      start.motion_start = 1;
      motion_start_pub_.publish(start);
      motion_started_for_goal_ = true;
      ROS_INFO("[lite_bridge] parking motion automatically enabled");
    }
    ROS_INFO(
        "[lite_bridge] trajectory displayed: vehicle=%d, points=%zu",
        vehicle_id_, message->points.size());
  }

  void steeringCommandCallback(
      const driver_msgs::SteeringWheelCmd::ConstPtr &message)
  {
    if (!std::isfinite(message->steering_wheel_angle)) {
      ROS_WARN_THROTTLE(1.0, "[lite_bridge] ignore non-finite steering command");
      return;
    }
    // 现有控制话题使用方向盘角度；先除转向比得到前轮角，再归一化到Lite的[-1,1]。
    const double safe_steering_ratio =
        std::max(1.0e-3, std::fabs(steering_ratio_));
    const double front_tire_angle =
        message->steering_wheel_angle / safe_steering_ratio * M_PI / 180.0;
    steering_command_ =
        clamp(front_tire_angle / max_front_tire_angle_rad_, -1.0, 1.0);
    last_steering_command_time_ = ros::Time::now();
    has_steering_command_ = true;
  }

  void driveCommandCallback(const driver_msgs::DriveCmd::ConstPtr &message)
  {
    if (!std::isfinite(message->throttle_pedal) ||
        !std::isfinite(message->brake_pedal)) {
      ROS_WARN_THROTTLE(1.0, "[lite_bridge] ignore non-finite drive command");
      return;
    }
    throttle_command_ = clamp(message->throttle_pedal / 100.0, 0.0, 1.0);
    brake_command_ = clamp(message->brake_pedal / 100.0, 0.0, 1.0);
    // 防御性保证油门和制动互斥；控制器正常输出本身已满足该约束。
    if (brake_command_ > 1.0e-6) {
      throttle_command_ = 0.0;
    }
    last_drive_command_time_ = ros::Time::now();
    has_drive_command_ = true;
  }

  void gearCommandCallback(const driver_msgs::GearCmd::ConstPtr &message)
  {
    if (message->gear_location != 0 && message->gear_location != 1 &&
        message->gear_location != 7) {
      ROS_WARN_THROTTLE(
          1.0, "[lite_bridge] ignore unsupported gear command: %u",
          message->gear_location);
      return;
    }
    requested_gear_ = message->gear_location;
    has_requested_gear_command_ = true;
  }

  void parkingBrakeCommandCallback(
      const driver_msgs::ParkingBrakeCmd::ConstPtr &message)
  {
    hand_brake_command_ = message->parking_brake != 0;
  }

  void controlTimerCallback(const ros::TimerEvent &)
  {
    std_msgs::Float64MultiArray command;
    command.data.resize(6, 0.0);
    command.data[0] = static_cast<double>(vehicle_id_);

    const ros::Time now = ros::Time::now();
    const bool drive_command_fresh =
        has_drive_command_ &&
        (now - last_drive_command_time_).toSec() <= control_command_timeout_;
    const bool steering_command_fresh =
        has_steering_command_ &&
        (now - last_steering_command_time_).toSec() <= control_command_timeout_;
    if (drive_command_fresh) {
      command.data[1] = throttle_command_;
      command.data[3] = brake_command_;
    } else {
      // 控制节点退出、阻塞或尚未启动时持续发送零油门和最大行车制动。
      command.data[1] = 0.0;
      command.data[3] = 1.0;
      ROS_WARN_THROTTLE(
          2.0, "[lite_bridge] drive command timeout; apply full brake");
    }
    // 转向命令超时后保持当前实测角，避免安全制动时突然回轮。
    command.data[2] =
        steering_command_fresh ? steering_command_ : actual_normalized_steer_;

    if (has_requested_gear_command_) {
      applied_gear_ = requested_gear_;
      has_applied_gear_command_ = true;
    }
    command.data[4] =
        has_applied_gear_command_ && applied_gear_ == 7 ? 1.0 : 0.0;
    command.data[5] = hand_brake_command_ ? 1.0 : 0.0;
    lite_control_pub_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber chatter_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber steering_command_sub_;
  ros::Subscriber drive_command_sub_;
  ros::Subscriber gear_command_sub_;
  ros::Subscriber parking_brake_command_sub_;
  ros::Publisher localization_pub_;
  ros::Publisher chassis_pub_;
  ros::Publisher planner_goal_pub_;
  ros::Publisher planner_map_pub_;
  ros::Publisher lite_trajectory_pub_;
  ros::Publisher lite_control_pub_;
  ros::Publisher motion_start_pub_;
  ros::Timer control_timer_;

  int vehicle_id_{0};
  double map_crop_margin_{20.0};
  double max_front_tire_angle_rad_{32.0 * M_PI / 180.0};
  double steering_ratio_{1.0};
  double control_publish_rate_{50.0};
  double control_command_timeout_{0.5};
  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double vehicle_yaw_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double actual_normalized_steer_{0.0};
  double steering_command_{0.0};
  double throttle_command_{0.0};
  double brake_command_{1.0};
  uint8_t requested_gear_{0};
  uint8_t applied_gear_{0};
  bool enable_vehicle_control_{true};
  bool auto_motion_start_{true};
  bool has_vehicle_state_{false};
  bool has_goal_{false};
  bool has_map_{false};
  bool has_steering_command_{false};
  bool has_drive_command_{false};
  bool has_requested_gear_command_{false};
  bool has_applied_gear_command_{false};
  bool hand_brake_command_{false};
  bool motion_started_for_goal_{false};
  ros::Time last_steering_command_time_;
  ros::Time last_drive_command_time_;
  nav_msgs::OccupancyGrid latest_map_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lite_parking_bridge");
  LiteParkingBridge bridge;
  ros::spin();
  return 0;
}
