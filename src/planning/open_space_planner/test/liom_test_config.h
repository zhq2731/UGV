#pragma once

#include "hybrid_a_star/hybrid_a_star.h"
#include <yaml-cpp/yaml.h>

// 检查程序和可视化共用同一份 UGV 车辆参数读取，避免示例参数不一致。
inline OpenSpace_config LoadVehicle(const std::string &path) {
  const auto node = YAML::LoadFile(path);
  OpenSpace_config c{};
  c.wheel_base = node["wheel_base"].as<double>();
  c.car_length = node["car_length"].as<double>();
  c.car_width = node["car_width"].as<double>();
  c.rear_overhang = node["rear_overhang"].as<double>();
  c.steering_angle = node["steering_angle"].as<double>();
  c.trajectory_resample_resolution = node["trajectory_resample_resolution"].as<double>();
  c.forward_speed = node["forward_speed"].as<double>();
  c.reverse_speed = node["reverse_speed"].as<double>();
  c.max_acceleration = node["max_acceleration"].as<double>();
  c.max_deceleration = node["max_deceleration"].as<double>();
  c.max_lateral_acceleration = node["max_lateral_acceleration"].as<double>();
  c.max_front_tire_steering_rate = node["max_front_tire_steering_rate"].as<double>();
  c.search_collision_margin = node["search_collision_margin"].as<double>();
  c.output_collision_margin = node["output_collision_margin"].as<double>(c.output_collision_margin);
  c.steering_angle_discrete_num = node["steering_angle_discrete_num"].as<int>();
  c.segment_length = node["segment_length"].as<double>();
  c.segment_length_discrete_num = node["segment_length_discrete_num"].as<int>();
  c.steering_penalty = node["steering_penalty"].as<double>();
  c.steering_change_penalty = node["steering_change_penalty"].as<double>();
  c.reversing_penalty = node["reversing_penalty"].as<double>();
  c.gear_switch_penalty = node["gear_switch_penalty"].as<double>();
  c.shot_distance = node["shot_distance"].as<double>();
  return c;
}
