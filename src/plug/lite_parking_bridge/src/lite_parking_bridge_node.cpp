#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
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
 * Lite/CARLA 的状态、目标和展示轨迹以 actor 中心为参考点，规划器和
 * 控制器以后轴中心为参考点。桥接节点负责两种参考点之间的平移，并把
 * actor 中心局部栅格的坐标表示转换成后轴中心局部坐标系。
 */
class LiteParkingBridge
{
public:
  LiteParkingBridge() : private_nh_("~")
  {
    private_nh_.param("vehicle_id", vehicle_id_, 0);
    private_nh_.param<std::string>(
        "local_grid_topic", local_grid_topic_,
        "/segmenter/points_freeGridMap");
    private_nh_.param(
        "max_front_tire_angle_rad", max_front_tire_angle_rad_,
        35.0 * M_PI / 180.0);
    private_nh_.param(
        "carla_max_inner_wheel_angle_rad",
        carla_max_inner_wheel_angle_rad_, 70.0 * M_PI / 180.0);
    private_nh_.param(
        "carla_wheel_base_m", carla_wheel_base_m_, 4.0752487613);
    private_nh_.param(
        "carla_front_track_m", carla_front_track_m_, 1.9454631016);
    private_nh_.param<std::string>(
        "steering_calibration_model", steering_calibration_model_,
        "ackermann");
    private_nh_.param(
        "max_normalized_steer", max_normalized_steer_, 1.0);
    private_nh_.param(
        "normalized_to_tire_linear", normalized_to_tire_linear_, 0.0);
    private_nh_.param(
        "normalized_to_tire_cubic", normalized_to_tire_cubic_, 0.0);
    private_nh_.param(
        "normalized_to_tire_quintic", normalized_to_tire_quintic_, 0.0);
    private_nh_.param(
        "tire_to_normalized_linear", tire_to_normalized_linear_, 0.0);
    private_nh_.param(
        "tire_to_normalized_cubic", tire_to_normalized_cubic_, 0.0);
    private_nh_.param(
        "tire_to_normalized_quintic", tire_to_normalized_quintic_, 0.0);
    private_nh_.param(
        "actor_center_to_rear_axle_m", actor_center_to_rear_axle_m_, 0.0);
    private_nh_.param(
        "local_grid_x_shift_to_rear_axle_m",
        local_grid_x_shift_to_rear_axle_m_, actor_center_to_rear_axle_m_);
    private_nh_.param<std::string>(
        "rear_axle_frame_id", rear_axle_frame_id_, "rear_axle");
    private_nh_.param("steering_ratio", steering_ratio_, 1.0);
    private_nh_.param(
        "enable_vehicle_control", enable_vehicle_control_, true);
    private_nh_.param("auto_motion_start", auto_motion_start_, true);
    private_nh_.param("control_publish_rate", control_publish_rate_, 50.0);
    private_nh_.param("control_command_timeout", control_command_timeout_, 0.5);
    max_front_tire_angle_rad_ =
        std::max(1.0e-3, std::fabs(max_front_tire_angle_rad_));
    carla_max_inner_wheel_angle_rad_ = std::max(
        1.0e-3, std::min(
            std::fabs(carla_max_inner_wheel_angle_rad_),
            0.5 * M_PI - 1.0e-3));
    carla_wheel_base_m_ = std::max(1.0e-3, carla_wheel_base_m_);
    carla_front_track_m_ = std::max(0.0, carla_front_track_m_);
    max_normalized_steer_ = clamp(
        std::fabs(max_normalized_steer_), 1.0e-3, 1.0);
    if (steering_calibration_model_ != "ackermann" &&
        steering_calibration_model_ != "empirical_odd_polynomial") {
      ROS_WARN(
          "[lite_bridge] unsupported steering_calibration_model=%s; "
          "fall back to ackermann",
          steering_calibration_model_.c_str());
      steering_calibration_model_ = "ackermann";
    }
    if (steering_calibration_model_ == "empirical_odd_polynomial" &&
        (!std::isfinite(normalized_to_tire_linear_) ||
         !std::isfinite(normalized_to_tire_cubic_) ||
         !std::isfinite(normalized_to_tire_quintic_) ||
         !std::isfinite(tire_to_normalized_linear_) ||
         !std::isfinite(tire_to_normalized_cubic_) ||
         !std::isfinite(tire_to_normalized_quintic_))) {
      ROS_WARN(
          "[lite_bridge] non-finite empirical steering coefficients; "
          "fall back to ackermann");
      steering_calibration_model_ = "ackermann";
      max_normalized_steer_ = 1.0;
    }
    if (steering_calibration_model_ == "empirical_odd_polynomial") {
      const double command = max_normalized_steer_;
      const double command_squared = command * command;
      const double calibrated_angle_at_limit =
          normalized_to_tire_linear_ * command +
          normalized_to_tire_cubic_ * command * command_squared +
          normalized_to_tire_quintic_ * command * command_squared *
              command_squared;
      const double angle = max_front_tire_angle_rad_;
      const double angle_squared = angle * angle;
      const double calibrated_command_at_limit =
          tire_to_normalized_linear_ * angle +
          tire_to_normalized_cubic_ * angle * angle_squared +
          tire_to_normalized_quintic_ * angle * angle_squared *
              angle_squared;
      if (calibrated_angle_at_limit <= 1.0e-3 ||
          calibrated_command_at_limit <= 1.0e-3) {
        ROS_WARN(
            "[lite_bridge] empirical steering model does not provide a "
            "positive usable range; fall back to ackermann");
        steering_calibration_model_ = "ackermann";
        max_normalized_steer_ = 1.0;
      }
    }
    actor_center_to_rear_axle_m_ =
        std::max(0.0, actor_center_to_rear_axle_m_);
    if (!std::isfinite(local_grid_x_shift_to_rear_axle_m_)) {
      ROS_WARN(
          "[lite_bridge] invalid local-grid x shift; use actor-to-rear "
          "offset %.4f m",
          actor_center_to_rear_axle_m_);
      local_grid_x_shift_to_rear_axle_m_ = actor_center_to_rear_axle_m_;
    }
    if (rear_axle_frame_id_.empty()) {
      rear_axle_frame_id_ = "rear_axle";
    }
    const double carla_max_equivalent_angle =
        equivalentFrontTireAngleFromNormalizedSteer(1.0);
    if (max_front_tire_angle_rad_ > carla_max_equivalent_angle) {
      ROS_WARN(
          "[lite_bridge] requested tire-angle limit %.2f deg exceeds "
          "CARLA equivalent maximum %.2f deg; clamp to CARLA maximum",
          max_front_tire_angle_rad_ * 180.0 / M_PI,
          carla_max_equivalent_angle * 180.0 / M_PI);
      max_front_tire_angle_rad_ = carla_max_equivalent_angle;
    }
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
        local_grid_topic_, 1, &LiteParkingBridge::localGridCallback, this);
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
        "[lite_bridge] ready: vehicle_id=%d, local_grid_topic=%s, control=%s, "
        "steering_model=%s, equivalent_steer_limit=%.2f deg, "
        "CARLA_command_limit=%.4f, "
        "actor_center_to_rear_axle=%.4f m, local_grid_x_shift=%.4f m, "
        "planner_frame=%s",
        vehicle_id_, local_grid_topic_.c_str(),
        enable_vehicle_control_ ? "enabled" : "disabled",
        steering_calibration_model_.c_str(),
        max_front_tire_angle_rad_ * 180.0 / M_PI,
        std::fabs(normalizedSteerFromEquivalentFrontTireAngle(
            max_front_tire_angle_rad_)),
        actor_center_to_rear_axle_m_, local_grid_x_shift_to_rear_axle_m_,
        rear_axle_frame_id_.c_str());
  }

private:
  void actorCenterToRearAxle(
      const double actor_x, const double actor_y, const double yaw,
      double *rear_x, double *rear_y) const
  {
    *rear_x = actor_x - actor_center_to_rear_axle_m_ * std::cos(yaw);
    *rear_y = actor_y - actor_center_to_rear_axle_m_ * std::sin(yaw);
  }

  void rearAxleToActorCenter(
      const double rear_x, const double rear_y, const double yaw,
      double *actor_x, double *actor_y) const
  {
    *actor_x = rear_x + actor_center_to_rear_axle_m_ * std::cos(yaw);
    *actor_y = rear_y + actor_center_to_rear_axle_m_ * std::sin(yaw);
  }

  bool usesEmpiricalSteeringCalibration() const
  {
    return steering_calibration_model_ == "empirical_odd_polynomial";
  }

  // 规划控制器始终使用等效自行车前轮角。Cybertruck默认使用低速定圆
  // 实测奇次多项式；Ackermann名义模型只作为其他车型和异常配置的回退。
  double equivalentFrontTireAngleFromNormalizedSteer(
      const double normalized_steer) const
  {
    const double command = clamp(
        normalized_steer, -max_normalized_steer_, max_normalized_steer_);
    if (std::fabs(command) <= 1.0e-9) {
      return 0.0;
    }
    if (usesEmpiricalSteeringCalibration()) {
      const double command_squared = command * command;
      const double command_cubed = command * command_squared;
      const double command_quintic =
          command_cubed * command_squared;
      return clamp(
          normalized_to_tire_linear_ * command +
              normalized_to_tire_cubic_ * command_cubed +
              normalized_to_tire_quintic_ * command_quintic,
          -max_front_tire_angle_rad_, max_front_tire_angle_rad_);
    }
    const double inner_wheel_angle =
        std::fabs(command) * carla_max_inner_wheel_angle_rad_;
    const double inner_wheel_radius =
        carla_wheel_base_m_ / std::tan(inner_wheel_angle);
    const double center_radius =
        inner_wheel_radius + 0.5 * carla_front_track_m_;
    return std::copysign(
        std::atan2(carla_wheel_base_m_, center_radius), command);
  }

  double normalizedSteerFromEquivalentFrontTireAngle(
      const double equivalent_angle) const
  {
    const double limited_angle = clamp(
        equivalent_angle, -max_front_tire_angle_rad_,
        max_front_tire_angle_rad_);
    if (std::fabs(limited_angle) <= 1.0e-9) {
      return 0.0;
    }
    if (usesEmpiricalSteeringCalibration()) {
      const double angle_squared = limited_angle * limited_angle;
      const double angle_cubed = limited_angle * angle_squared;
      const double angle_quintic = angle_cubed * angle_squared;
      return clamp(
          tire_to_normalized_linear_ * limited_angle +
              tire_to_normalized_cubic_ * angle_cubed +
              tire_to_normalized_quintic_ * angle_quintic,
          -max_normalized_steer_, max_normalized_steer_);
    }
    const double center_radius =
        carla_wheel_base_m_ / std::tan(std::fabs(limited_angle));
    const double inner_wheel_radius = std::max(
        1.0e-6, center_radius - 0.5 * carla_front_track_m_);
    const double inner_wheel_angle =
        std::atan2(carla_wheel_base_m_, inner_wheel_radius);
    return clamp(std::copysign(
        clamp(
            inner_wheel_angle / carla_max_inner_wheel_angle_rad_,
            0.0, 1.0),
        limited_angle), -max_normalized_steer_, max_normalized_steer_);
  }

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
      const double vehicle_x = message->data[offset + 2];
      const double vehicle_y = message->data[offset + 1];
      const double vehicle_z = message->data[offset + 3];
      const double yaw = message->data[offset + 7];
      const double velocity_x = message->data[offset + 9];
      const double velocity_y = message->data[offset + 10];
      const double raw_normalized_steer =
          clamp(message->data[offset + 13], -1.0, 1.0);
      if (std::fabs(raw_normalized_steer) > max_normalized_steer_ + 1.0e-3) {
        ROS_WARN_THROTTLE(
            1.0,
            "[lite_bridge] measured normalized steer %.4f exceeds calibrated "
            "limit %.4f; clamp feedback to the valid calibration range",
            raw_normalized_steer, max_normalized_steer_);
      }
      const double normalized_steer = clamp(
          raw_normalized_steer,
          -max_normalized_steer_, max_normalized_steer_);
      actual_normalized_steer_ = normalized_steer;
      const bool reverse = message->data[offset + 15] >= 0.5 ||
                           message->data[offset + 17] < 0.0;
      const int gear = static_cast<int>(std::lround(message->data[offset + 17]));

      double rear_axle_x = 0.0;
      double rear_axle_y = 0.0;
      actorCenterToRearAxle(
          vehicle_x, vehicle_y, yaw, &rear_axle_x, &rear_axle_y);
      publishLocalization(rear_axle_x, rear_axle_y, vehicle_z, yaw);
      publishChassis(
          std::hypot(velocity_x, velocity_y), normalized_steer, reverse, gear,
          message->data[offset + 12], message->data[offset + 14]);
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

    // Lite发布CARLA归一化转向；换算成自行车模型使用的等效前轮角反馈。
    const double front_tire_angle =
        equivalentFrontTireAngleFromNormalizedSteer(normalized_steer);
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

    has_goal_ = true;
    motion_started_for_goal_ = false;

    const double goal_yaw = message->data[4];
    double rear_axle_goal_x = 0.0;
    double rear_axle_goal_y = 0.0;
    actorCenterToRearAxle(
        message->data[1], message->data[2], goal_yaw,
        &rear_axle_goal_x, &rear_axle_goal_y);

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = rear_axle_goal_x;
    goal.pose.position.y = rear_axle_goal_y;
    goal.pose.position.z = message->data[3];
    goal.pose.orientation = quaternionFromYaw(goal_yaw);
    planner_goal_pub_.publish(goal);

    ROS_INFO(
        "[lite_bridge] goal actor=(%.3f, %.3f), rear_axle=(%.3f, %.3f), "
        "yaw=%.3f rad, vehicle=%d",
        message->data[1], message->data[2], rear_axle_goal_x,
        rear_axle_goal_y, goal_yaw, vehicle_id_);
  }

  void localGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &message)
  {
    if (message->info.resolution <= 0.0 || message->info.width == 0 ||
        message->info.height == 0 ||
        message->data.size() !=
            static_cast<std::size_t>(message->info.width) *
                message->info.height) {
      ROS_WARN_THROTTLE(1.0, "[lite_bridge] ignore invalid occupancy grid");
      return;
    }
    const geometry_msgs::Quaternion &orientation =
        message->info.origin.orientation;
    const double orientation_norm = std::sqrt(
        orientation.x * orientation.x + orientation.y * orientation.y +
        orientation.z * orientation.z + orientation.w * orientation.w);
    const bool orientation_unset = orientation_norm < 1.0e-6;
    const bool orientation_identity =
        std::fabs(orientation.x) < 1.0e-6 &&
        std::fabs(orientation.y) < 1.0e-6 &&
        std::fabs(orientation.z) < 1.0e-6 &&
        std::fabs(std::fabs(orientation.w) - 1.0) < 1.0e-6;
    if (!orientation_unset && !orientation_identity) {
      // OpenSpacePlanner 当前忽略 OccupancyGrid 的 origin.orientation。
      // 拒绝旋转栅格比静默生成错误的碰撞关系更安全。
      ROS_ERROR_THROTTLE(
          2.0,
          "[lite_bridge] reject rotated local grid: planner only supports "
          "axis-aligned grids in the vehicle frame");
      return;
    }

    // 同一物理栅格由 actor 中心坐标改写为后轴中心坐标：
    // p_rear = p_actor + (actor_center_to_rear_axle, 0)。无需重采样数据。
    nav_msgs::OccupancyGrid planner_map = *message;
    planner_map.header.frame_id = rear_axle_frame_id_;
    planner_map.info.origin.position.x +=
        local_grid_x_shift_to_rear_axle_m_;
    if (orientation_unset) {
      planner_map.info.origin.orientation.x = 0.0;
      planner_map.info.origin.orientation.y = 0.0;
      planner_map.info.origin.orientation.z = 0.0;
      planner_map.info.origin.orientation.w = 1.0;
    }
    planner_map_pub_.publish(planner_map);
    ROS_INFO_THROTTLE(
        2.0,
        "[lite_bridge] local grid converted: source_frame=%s, "
        "planner_frame=%s, origin_x %.3f -> %.3f m, %u x %u cells, "
        "resolution=%.2f m",
        message->header.frame_id.c_str(), planner_map.header.frame_id.c_str(),
        message->info.origin.position.x, planner_map.info.origin.position.x,
        message->info.width, message->info.height, message->info.resolution);
  }

  void trajectoryCallback(
      const planning_msgs::TrajectoryPointArray::ConstPtr &message)
  {
    std_msgs::Float64MultiArray lite_trajectory;
    lite_trajectory.data.reserve(1 + 4 * message->points.size());
    lite_trajectory.data.push_back(static_cast<double>(vehicle_id_));
    for (const auto &point : message->points) {
      double actor_center_x = 0.0;
      double actor_center_y = 0.0;
      rearAxleToActorCenter(
          point.x, point.y, point.theta, &actor_center_x, &actor_center_y);
      lite_trajectory.data.push_back(actor_center_x);
      lite_trajectory.data.push_back(actor_center_y);
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
    // 先从方向盘角恢复规划器的等效前轮角，再换算为CARLA内侧轮归一化命令。
    // 换算函数内部将等效转角严格限制在配置的正负35度内。
    const double safe_steering_ratio =
        std::max(1.0e-3, std::fabs(steering_ratio_));
    const double front_tire_angle =
        message->steering_wheel_angle / safe_steering_ratio * M_PI / 180.0;
    steering_command_ =
        normalizedSteerFromEquivalentFrontTireAngle(front_tire_angle);
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
        steering_command_fresh ? steering_command_ :
        normalizedSteerFromEquivalentFrontTireAngle(
            equivalentFrontTireAngleFromNormalizedSteer(
                actual_normalized_steer_));

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
  std::string local_grid_topic_;
  std::string steering_calibration_model_{"ackermann"};
  double max_front_tire_angle_rad_{35.0 * M_PI / 180.0};
  double carla_max_inner_wheel_angle_rad_{70.0 * M_PI / 180.0};
  double carla_wheel_base_m_{4.0752487613};
  double carla_front_track_m_{1.9454631016};
  double max_normalized_steer_{1.0};
  double normalized_to_tire_linear_{0.0};
  double normalized_to_tire_cubic_{0.0};
  double normalized_to_tire_quintic_{0.0};
  double tire_to_normalized_linear_{0.0};
  double tire_to_normalized_cubic_{0.0};
  double tire_to_normalized_quintic_{0.0};
  double actor_center_to_rear_axle_m_{0.0};
  double local_grid_x_shift_to_rear_axle_m_{0.0};
  double steering_ratio_{1.0};
  double control_publish_rate_{50.0};
  double control_command_timeout_{0.5};
  double actual_normalized_steer_{0.0};
  double steering_command_{0.0};
  double throttle_command_{0.0};
  double brake_command_{1.0};
  uint8_t requested_gear_{0};
  uint8_t applied_gear_{0};
  bool enable_vehicle_control_{true};
  bool auto_motion_start_{true};
  bool has_goal_{false};
  bool has_steering_command_{false};
  bool has_drive_command_{false};
  bool has_requested_gear_command_{false};
  bool has_applied_gear_command_{false};
  bool hand_brake_command_{false};
  bool motion_started_for_goal_{false};
  std::string rear_axle_frame_id_{"rear_axle"};
  ros::Time last_steering_command_time_;
  ros::Time last_drive_command_time_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lite_parking_bridge");
  LiteParkingBridge bridge;
  ros::spin();
  return 0;
}
