#include "hybrid_a_star/openSpacePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <ros/package.h>
#include "yaml-cpp/yaml.h"

namespace {

double Mod2Pi(const double &x)
{
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI)
    {
        v += 2.0 * M_PI;
    }
    else if (v > M_PI)
    {
        v -= 2.0 * M_PI;
    }

    return v;
}

/**
 * @brief 按前进/倒车方向把完整搜索路径切成若干单档位轨迹段
 *
 * 相邻段在换挡点处共享端点，保证控制器停车后能从实际位姿重新规划。
 */
std::vector<HybridAStarType::VectorVec4d> path_split(
    const HybridAStarType::VectorVec3d &path)
{
	std::vector<HybridAStarType::VectorVec4d> path_split_all;
	if (path.empty()) {
		return path_split_all;
	}
	if (path.size() == 1) {
		HybridAStarType::VectorVec4d single_path;
		HybridAStarType::Vec4d point;
		point << path.front().x(), path.front().y(), path.front().z(), 1.0;
		single_path.emplace_back(point);
		path_split_all.emplace_back(single_path);
		return path_split_all;
	}
    HybridAStarType::VectorVec4d path_split;
    double heading_angle = path[0].z();
    const HybridAStarType::Vec2d init_tracking_vector(path[1].x() - path[0].x(), path[1].y() - path[0].y());
    double tracking_angle = std::atan2(init_tracking_vector.y(), init_tracking_vector.x());
    // true 是前进，false 是倒车。
    bool current_gear =
        std::fabs(Mod2Pi(tracking_angle - heading_angle)) < M_PI_2;
	for (unsigned int i = 0; i + 1 < path.size(); i++)
    {
        heading_angle = path[i].z();
        const HybridAStarType::Vec2d tracking_vector(path[i + 1].x() - path[i].x(), path[i + 1].y() - path[i].y());
        double current_tracking_angle =
            std::atan2(tracking_vector.y(), tracking_vector.x());
        bool gear = std::fabs(
            Mod2Pi(current_tracking_angle - heading_angle)) < M_PI_2;
        HybridAStarType::Vec4d point;
        point << path[i].x(), path[i].y(), path[i].z(),
            current_gear ? 1.0 : 0.0;
        if (gear != current_gear)
        {
            path_split.emplace_back(point);
            path_split_all.emplace_back(path_split);
            path_split.clear();
            current_gear = gear;
            point.w() = current_gear ? 1.0 : 0.0;
        }
		path_split.emplace_back(point);
	}
	HybridAStarType::Vec4d last_point;
	last_point << path.back().x(), path.back().y(), path.back().z(), current_gear ? 1.0 : 0.0;
	path_split.emplace_back(last_point);
	path_split_all.emplace_back(path_split);
    return path_split_all;
}

planning_msgs::TrajectoryPoint add_2d(planning_msgs::TrajectoryPoint point1, planning_msgs::TrajectoryPoint point2)
{
    planning_msgs::TrajectoryPoint point;
    point.x = point1.x + point2.x;
    point.y = point1.y + point2.y;
    return point;
}

planning_msgs::TrajectoryPoint scaled_2d(planning_msgs::TrajectoryPoint point1, double scale)
{
    planning_msgs::TrajectoryPoint point;
    point.x = point1.x * scale;
    point.y = point1.y * scale;
    return point;
}

std::pair<size_t, size_t> findNearestIndexPair(
    const std::vector<double> &accumulated_lengths, const double target_length)
{
    // List size
    const auto N = accumulated_lengths.size();

    // Front
    if (target_length < accumulated_lengths.at(1))
    {
        return std::make_pair(0, 1);
    }

    // Back
    if (target_length > accumulated_lengths.at(N - 2))
    {
        return std::make_pair(N - 2, N - 1);
    }

    // Middle
    for (size_t i = 1; i < N; ++i)
    {
        if (
            accumulated_lengths.at(i - 1) <= target_length &&
            target_length <= accumulated_lengths.at(i))
        {
            return std::make_pair(i - 1, i);
        }
    }

    // Throw an exception because this never happens
    throw std::runtime_error(
        "findNearestIndexPair(): No nearest point found.");
}

double dot2D(planning_msgs::TrajectoryPoint point1, planning_msgs::TrajectoryPoint point2)
{
    return point1.x * point2.x + point1.y * point2.y;
}

planning_msgs::TrajectoryPoint minus_2d(planning_msgs::TrajectoryPoint point1, planning_msgs::TrajectoryPoint point2)
{
    planning_msgs::TrajectoryPoint point;
    point.x = point1.x - point2.x;
    point.y = point1.y - point2.y;
    return point;
}

double length(planning_msgs::TrajectoryPoint point1, planning_msgs::TrajectoryPoint point2)
{
    planning_msgs::TrajectoryPoint point = minus_2d(point1, point2);
    return std::sqrt(dot2D(point, point));
}

void resamplePoints(double resolution, std::vector<planning_msgs::TrajectoryPoint> &pts)
{
    double acculate_s = 0;
    std::vector<double> accumulated_lengths;
    accumulated_lengths.push_back(0.0);
    for (unsigned int i = 1; i < pts.size(); i++)
    {
        acculate_s += length(pts[i], pts[i - 1]);
        accumulated_lengths.push_back(acculate_s);
    }
    const int32_t num_segments =
        std::max(static_cast<int32_t>(ceil(acculate_s / resolution)), 1);

    std::vector<planning_msgs::TrajectoryPoint> resampled_points;
    for (auto i = 0; i <= num_segments; ++i)
    {
        // Find two nearest points
        const double target_length = (static_cast<double>(i) / num_segments) * acculate_s;
        const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

        // Apply linear interpolation
        const planning_msgs::TrajectoryPoint back_point = pts[index_pair.first];
        const planning_msgs::TrajectoryPoint front_point = pts[index_pair.second];

        const auto direction_vector = minus_2d(front_point, back_point);

        const auto back_length = accumulated_lengths.at(index_pair.first);
        const auto front_length = accumulated_lengths.at(index_pair.second);
        const auto segment_length = front_length - back_length;
        auto target_point = add_2d(back_point, scaled_2d(direction_vector, (target_length - back_length) / segment_length));
        target_point.theta = back_point.theta + ((target_length - back_length) / segment_length) * (front_point.theta - back_point.theta);
        resampled_points.push_back(target_point);
    }
    pts = std::move(resampled_points);
}

// 计算kappa 相关函数
// 累计s
void caculateAccumulated_s_os(planning_msgs::TrajectoryPointArray &trajectory)
{
    double s = 0.0;
    trajectory.points[0].s = s;

    for (std::size_t i = 1; i < trajectory.points.size(); ++i)
    {
        s += length(trajectory.points[i], trajectory.points[i - 1]);
        trajectory.points[i].s = s;
    }
}

// 计算kappa
void caculateKappaos(
    const size_t curvature_smoothing_num, planning_msgs::TrajectoryPointArray &traj)
{

    /* calculate curvature by circle fitting from three points */
    planning_msgs::TrajectoryPoint p1, p2, p3;
    const size_t max_smoothing_num =
        static_cast<size_t>(std::floor(0.5 * (static_cast<double>(traj.points.size() - 1))));
    const size_t L = std::min(curvature_smoothing_num, max_smoothing_num);
    for (size_t i = L; i < traj.points.size() - L; ++i)
    {
        const size_t curr_idx = i;
        const size_t prev_idx = curr_idx - L;
        const size_t next_idx = curr_idx + L;
        p1.x = traj.points[prev_idx].x;
        p2.x = traj.points[curr_idx].x;
        p3.x = traj.points[next_idx].x;
        p1.y = traj.points[prev_idx].y;
        p2.y = traj.points[curr_idx].y;
        p3.y = traj.points[next_idx].y;
        const double den = std::max(
            length(p1, p2) * length(p2, p3) * length(p3, p1),
            std::numeric_limits<double>::epsilon());
        const double curvature =
            2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
        traj.points.at(curr_idx).kappa = curvature;
    }

    /* first and last curvature is copied from next value */
    for (size_t i = 0; i < std::min(L, traj.points.size()); ++i)
    {
        traj.points.at(i).kappa = traj.points.at(std::min(L, traj.points.size() - 1)).kappa;
        traj.points.at(traj.points.size() - i - 1).kappa =
            traj.points.at(std::max(traj.points.size() - L - 1, size_t(0))).kappa;
    }
}

// 计算dkappa

void caculateDkappaos(planning_msgs::TrajectoryPointArray &trajectory)
{
    // Dkappa calculation
    for (std::size_t i = 0; i < trajectory.points.size(); ++i)
    {
        planning_msgs::TrajectoryPoint p1, p2, p3;
        double dkappa = 0.0;
        if (i == 0)
        {
            dkappa = (trajectory.points[i + 1].kappa - trajectory.points[i].kappa) /
                     (trajectory.points[i + 1].s - trajectory.points[i].s);
        }
        else if (i == trajectory.points.size() - 1)
        {
            dkappa = (trajectory.points[i].kappa - trajectory.points[i - 1].kappa) /
                     (trajectory.points[i].s - trajectory.points[i - 1].s);
        }
        else
        {
            dkappa = (trajectory.points[i + 1].kappa - trajectory.points[i - 1].kappa) /
                     (trajectory.points[i + 1].s - trajectory.points[i - 1].s);
        }
        trajectory.points[i].dkappa = dkappa;
    }
}

}  // namespace

OpenSpacePlanner::OpenSpacePlanner(displayCallback callBack_):BasePlanner(callBack_){
	debug_marker_pub_ = debug_nh_.advertise<visualization_msgs::MarkerArray>("/open_space_debug_marker", 1);
	execution_status_sub_ = debug_nh_.subscribe(
		"open_space_execution_status", 1,
		&OpenSpacePlanner::executionStatusCallback, this);
	std::string param_node_dir;
    param_node_dir = ros::package::getPath("open_space_planner");

    std::string openspace_config_yaml_file = param_node_dir + std::string("/param/") + std::string("openspace.yaml");
    YAML::Node config_;

    // 加载配置参数
    config_ = YAML::LoadFile(openspace_config_yaml_file);

    openspace_config.car_length = config_["car_length"].as<double>();
    openspace_config.car_width = config_["car_width"].as<double>();
    openspace_config.reverse_brake = config_["reverse_brake"].as<double>();
    openspace_config.reverse_speed = config_["reverse_speed"].as<double>();
    openspace_config.safe_reverse_dis = config_["safe_reverse_dis"].as<double>();
    openspace_config.steering_angle = config_["steering_angle"].as<double>();
    openspace_config.steering_angle_discrete_num = config_["steering_angle_discrete_num"].as<int>();
    openspace_config.wheel_base = config_["wheel_base"].as<double>();
    openspace_config.segment_length = config_["segment_length"].as<double>();
    openspace_config.segment_length_discrete_num = config_["segment_length_discrete_num"].as<int>();
    openspace_config.steering_penalty = config_["steering_penalty"].as<double>();
    openspace_config.steering_change_penalty = config_["steering_change_penalty"].as<double>();
    openspace_config.reversing_penalty = config_["reversing_penalty"].as<double>();
    openspace_config.shot_distance = config_["shot_distance"].as<double>();
    openspace_config.segment_end_position_tolerance =
        config_["segment_end_position_tolerance"].as<double>();
    openspace_config.segment_end_heading_tolerance =
        config_["segment_end_heading_tolerance"].as<double>();
    openspace_config.stop_speed_tolerance = config_["stop_speed_tolerance"].as<double>();
    openspace_config.goal_position_tolerance = config_["goal_position_tolerance"].as<double>();
    openspace_config.goal_heading_tolerance = config_["goal_heading_tolerance"].as<double>();

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        openspace_config.steering_angle,
        openspace_config.steering_angle_discrete_num,
        openspace_config.segment_length,
        openspace_config.segment_length_discrete_num,
        openspace_config.wheel_base,
        openspace_config.steering_penalty,
        openspace_config.reversing_penalty,
        openspace_config.steering_change_penalty,
        openspace_config.shot_distance);

}
void  OpenSpacePlanner::setInputData(InputData const & input_data) {
	current_vehicle_state_ptr_ = std::make_shared<VehicleState>(input_data.vehicleState);
	goal_vehicle_state_ptr_ = std::make_shared<VehicleState>(input_data.goalState);
	current_costmap_ptr_ = boost::make_shared<nav_msgs::OccupancyGrid>(input_data.occGrid);
}

void OpenSpacePlanner::executionStatusCallback(
	const planning_msgs::OpenSpaceExecutionStatus::ConstPtr &msg)
{
	// 执行反馈只属于当前已提交段；其他状态下到达的延迟消息直接丢弃。
	if (parking_state_ != ParkingState::TRACKING_COMMITTED_SEGMENT) {
		return;
	}
	latest_execution_status_ = *msg;
	has_execution_status_ = true;
}

const char *OpenSpacePlanner::parkingStateName(ParkingState state) const
{
	switch (state) {
	case ParkingState::WAITING_FOR_INPUT:
		return "WAITING_FOR_INPUT";
	case ParkingState::PLANNING:
		return "PLANNING";
	case ParkingState::TRACKING_COMMITTED_SEGMENT:
		return "TRACKING_COMMITTED_SEGMENT";
	case ParkingState::STOP_AT_CUSP:
		return "STOP_AT_CUSP";
	case ParkingState::EMERGENCY_BRAKING:
		return "EMERGENCY_BRAKING";
	case ParkingState::REPLAN_AFTER_SEGMENT_END_MISS:
		return "REPLAN_AFTER_SEGMENT_END_MISS";
	case ParkingState::GOAL_REACHED:
		return "GOAL_REACHED";
	}
	return "UNKNOWN";
}

void OpenSpacePlanner::setParkingState(ParkingState state)
{
	if (parking_state_ == state) {
		return;
	}
	ROS_INFO_STREAM("[open_space] state " << parkingStateName(parking_state_)
										<< " -> " << parkingStateName(state));
	parking_state_ = state;
}

void OpenSpacePlanner::resetPlanningState()
{
	// 终止当前搜索和已提交轨迹，避免新任务继续沿用上一任务的换挡点、
	// 预期档位、控制反馈或目标到达状态。
	kinodynamic_astar_searcher_ptr_->Reset();
	final_trajectory_ = planning_msgs::TrajectoryPointArray();
	committed_segment_ = planning_msgs::TrajectoryPointArray();
	committed_goal_state_ = VehicleState{};
	current_costmap_ptr_.reset();
	current_vehicle_state_ptr_.reset();
	goal_vehicle_state_ptr_.reset();
	has_committed_goal_ = false;
	has_committed_segment_ = false;
	has_expected_next_direction_ = false;
	expected_next_forward_ = true;
	constrain_replan_start_direction_ = false;
	latest_execution_status_ = planning_msgs::OpenSpaceExecutionStatus();
	has_execution_status_ = false;
	setParkingState(ParkingState::WAITING_FOR_INPUT);

	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker clear;
	clear.action = visualization_msgs::Marker::DELETEALL;
	markers.markers.push_back(clear);
	debug_marker_pub_.publish(markers);
}

bool OpenSpacePlanner::isGoalChanged(const VehicleState &goal) const
{
	if (!has_committed_goal_) {
		return false;
	}
	const double position_error = std::hypot(goal.x - committed_goal_state_.x,
												 goal.y - committed_goal_state_.y);
	return position_error > openspace_config.goal_position_tolerance ||
			std::fabs(Mod2Pi(goal.heading - committed_goal_state_.heading)) >
				openspace_config.goal_heading_tolerance;
}

bool OpenSpacePlanner::isGoalReached() const
{
	if (current_vehicle_state_ptr_ == nullptr || goal_vehicle_state_ptr_ == nullptr) {
		return false;
	}
	const double position_error = std::hypot(current_vehicle_state_ptr_->x - goal_vehicle_state_ptr_->x,
												 current_vehicle_state_ptr_->y - goal_vehicle_state_ptr_->y);
	const double heading_error = std::fabs(
		Mod2Pi(current_vehicle_state_ptr_->heading - goal_vehicle_state_ptr_->heading));
	return position_error <= openspace_config.goal_position_tolerance &&
			heading_error <= openspace_config.goal_heading_tolerance &&
			std::fabs(current_vehicle_state_ptr_->v) <= openspace_config.stop_speed_tolerance;
}

bool OpenSpacePlanner::isCommittedSegmentFinished() const
{
	if (!has_committed_segment_ || committed_segment_.points.empty() ||
		current_vehicle_state_ptr_ == nullptr) {
		return false;
	}
	const auto &end_point = committed_segment_.points.back();
	const double position_error = std::hypot(current_vehicle_state_ptr_->x - end_point.x,
												 current_vehicle_state_ptr_->y - end_point.y);
	const double heading_error = std::fabs(
		Mod2Pi(current_vehicle_state_ptr_->heading - end_point.theta));
	const bool finished =
		position_error <= openspace_config.segment_end_position_tolerance &&
		heading_error <= openspace_config.segment_end_heading_tolerance &&
		std::fabs(current_vehicle_state_ptr_->v) <= openspace_config.stop_speed_tolerance;

	// 换挡点判定数值仅作诊断，需要时可将 ROS 日志级别调为 DEBUG 查看。
	ROS_DEBUG_THROTTLE(0.5,
		"[open_space] segment end check: pos_err=%.3f/%.3f m, heading_err=%.3f/%.3f rad, "
		"speed=%.3f/%.3f mps, finished=%s",
		position_error, openspace_config.segment_end_position_tolerance,
		heading_error, openspace_config.segment_end_heading_tolerance,
		current_vehicle_state_ptr_->v, openspace_config.stop_speed_tolerance,
		finished ? "true" : "false");
	return finished;
}

bool OpenSpacePlanner::initializeCurrentMap()
{
	if (current_costmap_ptr_ == nullptr) {
		return false;
	}

	// 栅格地图由感知侧在自车局部坐标系发布；只有射线确认的 25 可通行。
	map_resolution_ = current_costmap_ptr_->info.resolution;
	if (map_resolution_ <= 0.0 || current_costmap_ptr_->info.width == 0 ||
		current_costmap_ptr_->info.height == 0) {
		return false;
	}

	kinodynamic_astar_searcher_ptr_->Init(
		current_costmap_ptr_->info.origin.position.x,
		current_costmap_ptr_->info.origin.position.x + current_costmap_ptr_->info.width * map_resolution_,
		current_costmap_ptr_->info.origin.position.y,
		current_costmap_ptr_->info.origin.position.y + current_costmap_ptr_->info.height * map_resolution_,
		1.0, map_resolution_,
		openspace_config.car_length,
		openspace_config.car_width,
		openspace_config.wheel_base);

	for (unsigned int w = 0; w < current_costmap_ptr_->info.width; ++w) {
		for (unsigned int h = 0; h < current_costmap_ptr_->info.height; ++h) {
			if (current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w] != 25) {
				kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
			}
		}
	}
	return true;
}

bool OpenSpacePlanner::isCommittedSegmentCollisionFree() const
{
	if (!has_committed_segment_ || committed_segment_.points.empty() ||
		current_vehicle_state_ptr_ == nullptr) {
		return false;
	}

	// 已执行过的轨迹点会自然落在当前局部地图的后方，不能拿它们判定未来轨迹失效。
	size_t nearest_index = 0;
	double nearest_distance = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < committed_segment_.points.size(); ++i) {
		const auto &point = committed_segment_.points[i];
		const double distance = std::hypot(point.x - current_vehicle_state_ptr_->x,
		                                  point.y - current_vehicle_state_ptr_->y);
		if (distance < nearest_distance) {
			nearest_distance = distance;
			nearest_index = i;
		}
	}

	const double cos_yaw = std::cos(current_vehicle_state_ptr_->heading);
	const double sin_yaw = std::sin(current_vehicle_state_ptr_->heading);
	const auto to_local_and_check = [&](const planning_msgs::TrajectoryPoint &point) {
		const double dx = point.x - current_vehicle_state_ptr_->x;
		const double dy = point.y - current_vehicle_state_ptr_->y;
		const double local_x = cos_yaw * dx + sin_yaw * dy;
		const double local_y = -sin_yaw * dx + cos_yaw * dy;
		const double local_heading = Mod2Pi(point.theta - current_vehicle_state_ptr_->heading);
		return kinodynamic_astar_searcher_ptr_->IsStateCollisionFree(
			local_x, local_y, local_heading);
	};

	// 以不大于一个栅格的间隔检查，避免仅检查 0.5 m 轨迹离散点而漏掉小障碍物。
	const double sample_step = std::max(0.05, map_resolution_);
	for (size_t i = nearest_index; i < committed_segment_.points.size(); ++i) {
		if (i > nearest_index) {
			const auto &previous = committed_segment_.points[i - 1];
			const auto &current = committed_segment_.points[i];
			const double distance = std::hypot(current.x - previous.x, current.y - previous.y);
			const size_t samples = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / sample_step)));
			for (size_t sample = 1; sample < samples; ++sample) {
				const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
				planning_msgs::TrajectoryPoint interpolated = previous;
				interpolated.x = previous.x + ratio * (current.x - previous.x);
				interpolated.y = previous.y + ratio * (current.y - previous.y);
				interpolated.theta = Mod2Pi(previous.theta +
					ratio * Mod2Pi(current.theta - previous.theta));
				if (!to_local_and_check(interpolated)) {
					return false;
				}
			}
		}
		if (!to_local_and_check(committed_segment_.points[i])) {
			return false;
		}
	}
	return true;
}

planning_msgs::TrajectoryPointArray OpenSpacePlanner::makeEmergencyStopTrajectory(double cur_time) const
{
	planning_msgs::TrajectoryPointArray stop_trajectory;
	if (current_vehicle_state_ptr_ == nullptr) {
		return stop_trajectory;
	}

	planning_msgs::TrajectoryPoint current_point;
	current_point.x = current_vehicle_state_ptr_->x;
	current_point.y = current_vehicle_state_ptr_->y;
	current_point.theta = current_vehicle_state_ptr_->heading;
	current_point.v = current_vehicle_state_ptr_->v;
	current_point.a = current_point.v >= 0.0 ? -std::fabs(openspace_config.reverse_brake)
		: std::fabs(openspace_config.reverse_brake);
	current_point.relative_time = 0.0;
	stop_trajectory.points.push_back(current_point);

	// 不向未知/碰撞方向再延伸路径：第二点原地零速，跟踪器据此立即制动并保持停车。
	planning_msgs::TrajectoryPoint stopped_point = current_point;
	stopped_point.v = 0.0;
	stopped_point.a = 0.0;
	stopped_point.relative_time = 0.1;
	stop_trajectory.points.push_back(stopped_point);
	stopped_point.relative_time = 1.0;
	stop_trajectory.points.push_back(stopped_point);

	stop_trajectory.is_forward_shift = committed_segment_.is_forward_shift;
	stop_trajectory.header.stamp = ros::Time(cur_time);
	stop_trajectory.header.frame_id = "map";
	return stop_trajectory;
}

bool OpenSpacePlanner::beginEmergencyBraking(double cur_time, const char *reason)
{
	final_trajectory_ = makeEmergencyStopTrajectory(cur_time);
	committed_segment_.points.clear();
	has_committed_segment_ = false;
	// 紧急制动已经中断原轨迹，原轨迹的末端执行反馈不再有效。
	has_execution_status_ = false;
	// 紧急停车发生在原轨迹段中部，旧路径给出的下一档位已不再可靠。
	has_expected_next_direction_ = false;
	constrain_replan_start_direction_ = false;
	setParkingState(ParkingState::EMERGENCY_BRAKING);
	ROS_WARN_STREAM("[open_space] committed segment invalid (" << reason
					<< "), publishing emergency stop trajectory");
	if (callBack != nullptr) {
		callBack(&final_trajectory_, nullptr, nullptr, nullptr, nullptr);
	}
	return !final_trajectory_.points.empty();
}

bool  OpenSpacePlanner::implement(double cur_time,const DiscretizedTrajectory prev_trajectory) {
	(void)prev_trajectory;

	if (current_vehicle_state_ptr_ == nullptr || goal_vehicle_state_ptr_ == nullptr) {
		setParkingState(ParkingState::WAITING_FOR_INPUT);
		return false;
	}

	// 新目标立即取消旧段；当前周期会基于实际车辆位姿提交新的第一段。
	if (parking_state_ != ParkingState::EMERGENCY_BRAKING &&
		isGoalChanged(*goal_vehicle_state_ptr_)) {
		has_committed_segment_ = false;
		committed_segment_.points.clear();
		has_expected_next_direction_ = false;
		constrain_replan_start_direction_ = false;
		has_execution_status_ = false;
		setParkingState(ParkingState::PLANNING);
	}

	// 已提交段执行期间先用最新的点云栅格复核剩余轨迹；失效时必须替换旧轨迹为停车轨迹。
	if (parking_state_ == ParkingState::TRACKING_COMMITTED_SEGMENT) {
		if (!initializeCurrentMap()) {
			return beginEmergencyBraking(cur_time, "current map unavailable");
		}
		if (!isCommittedSegmentCollisionFree()) {
			return beginEmergencyBraking(cur_time, "current map collision or unknown area");
		}
		if (isGoalReached()) {
			has_committed_segment_ = false;
			committed_segment_.points.clear();
			has_expected_next_direction_ = false;
			constrain_replan_start_direction_ = false;
			has_execution_status_ = false;
			setParkingState(ParkingState::GOAL_REACHED);
		} else if (isCommittedSegmentFinished()) {
			ROS_DEBUG("[open_space] reached cusp and stopped; waiting for next planning cycle");
			has_execution_status_ = false;
			setParkingState(ParkingState::STOP_AT_CUSP);
		} else if (has_execution_status_ &&
			latest_execution_status_.state ==
				planning_msgs::OpenSpaceExecutionStatus::SEGMENT_END_HOLD) {
			ROS_WARN_STREAM("[open_space] segment exhausted and stopped without reaching "
				"cusp tolerance: remaining="
				<< latest_execution_status_.remaining_distance << " m, speed="
				<< latest_execution_status_.actual_speed
				<< " mps; replan without direction constraint");
			// 当前反馈已被本次状态迁移消费，立即清除，避免后续周期重复使用。
			has_execution_status_ = false;
			setParkingState(ParkingState::REPLAN_AFTER_SEGMENT_END_MISS);
		}
		return false;
	}

	// 安全停车轨迹只发布一次。确认实际车速为零后才允许基于最新地图重新规划。
	if (parking_state_ == ParkingState::EMERGENCY_BRAKING) {
		if (std::fabs(current_vehicle_state_ptr_->v) <= openspace_config.stop_speed_tolerance) {
			setParkingState(ParkingState::PLANNING);
		} else {
			return false;
		}
	}

	// 控制器已在轨迹末端稳定停车，但实际位姿未满足换挡点阈值。
	// 原候选下一档位不再具有可靠的几何意义，因此从实际位姿无档位约束重规划。
	if (parking_state_ == ParkingState::REPLAN_AFTER_SEGMENT_END_MISS) {
		has_committed_segment_ = false;
		committed_segment_.points.clear();
		has_expected_next_direction_ = false;
		constrain_replan_start_direction_ = false;
		has_execution_status_ = false;
		setParkingState(ParkingState::PLANNING);
		return false;
	}

	// 换挡点的最后一个发布段要求零速度；确认停稳后的下一规划周期才生成新段。
	if (parking_state_ == ParkingState::STOP_AT_CUSP) {
		constrain_replan_start_direction_ = has_expected_next_direction_;
		if (constrain_replan_start_direction_) {
			ROS_INFO_STREAM("[open_space] next replan must start "
				<< (expected_next_forward_ ? "forward" : "reverse"));
		}
		ROS_DEBUG_STREAM("[open_space] cusp handling complete; replan from current pose ("
						<< current_vehicle_state_ptr_->x << ", "
						<< current_vehicle_state_ptr_->y << ", "
						<< current_vehicle_state_ptr_->heading << ")");
		setParkingState(ParkingState::PLANNING);
		return false;
	}

	if (parking_state_ == ParkingState::GOAL_REACHED) {
		return false;
	}

	if (isGoalReached()) {
		has_committed_segment_ = false;
		has_expected_next_direction_ = false;
		constrain_replan_start_direction_ = false;
		setParkingState(ParkingState::GOAL_REACHED);
		return false;
	}

	setParkingState(ParkingState::PLANNING);

	if (!initializeCurrentMap()) {
		ROS_WARN_THROTTLE(
			2.0, "[open_space] replan waits for a valid current map");
		return false;
	}

    double start_yaw = current_vehicle_state_ptr_->heading;

	double goal_yaw = goal_vehicle_state_ptr_->heading;

    // 全局起点仅用于将局部搜索结果转换回控制器使用的 map 坐标系。
    HybridAStarType::Vec3d start_state = HybridAStarType::Vec3d(
		current_vehicle_state_ptr_->x,
		current_vehicle_state_ptr_->y,
		current_vehicle_state_ptr_->heading);
    HybridAStarType::Vec3d goal_state = HybridAStarType::Vec3d(
		goal_vehicle_state_ptr_->x,
		goal_vehicle_state_ptr_->y,
		goal_vehicle_state_ptr_->heading);
	// 局部栅格以自车为原点：搜索起点始终是 (0, 0, 0)。
	HybridAStarType::Vec3d start_state_map = HybridAStarType::Vec3d(
		0.0, 0.0, 0.0);

	const double goal_dx = goal_state.x() - start_state.x();
	const double goal_dy = goal_state.y() - start_state.y();
	const double goal_local_x = std::cos(start_yaw) * goal_dx + std::sin(start_yaw) * goal_dy;
	const double goal_local_y = -std::sin(start_yaw) * goal_dx + std::cos(start_yaw) * goal_dy;
	HybridAStarType::Vec3d goal_state_map = HybridAStarType::Vec3d(
		goal_local_x,
		goal_local_y,
		Mod2Pi(goal_yaw - start_yaw));
	const StateNode::DIRECTION required_start_direction =
		constrain_replan_start_direction_
			? (expected_next_forward_ ? StateNode::FORWARD : StateNode::BACKWARD)
			: StateNode::NO;
	if (kinodynamic_astar_searcher_ptr_->Search(
			start_state_map, goal_state_map, required_start_direction))
		{
			auto path = kinodynamic_astar_searcher_ptr_->GetPath();
		publishSearchDebugMarkers(kinodynamic_astar_searcher_ptr_->GetSearchedTree(),
		                          kinodynamic_astar_searcher_ptr_->GetRsConnectPath(),
		                          start_state);
		// 将path分段
		//在栅格地图里的路径
		std::vector<HybridAStarType::VectorVec4d> path_split_result_os = path_split(path);
		if (path_split_result_os.empty() || path_split_result_os.front().size() < 2) {
			kinodynamic_astar_searcher_ptr_->Reset();
			ROS_WARN_THROTTLE(
				2.0, "[open_space] search result has no executable segment");
			return false;
		}
		//转到UTM
		planning_msgs::TrajectoryPointArray path_os = GetTraject(path_split_result_os[0], start_state);
		// 得到具有位置、曲率、曲率变化的路径
		caculateKappaos(1, path_os);
		caculateAccumulated_s_os(path_os);
		caculateDkappaos(path_os);
		// 在路径上附速度
		// planning_msgs::TrajectoryPointArray path_os_out;
		final_trajectory_ = Velocity_Profile_output_os(
			openspace_config, std::move(path_os));
		// 每个提交段都必须在其末端停车；后续同档位段由状态机重新规划。
		if (!final_trajectory_.points.empty()) {
			final_trajectory_.points.back().v = 0.0;
			final_trajectory_.points.back().a = 0.0;
		}
        ros::Time time_stamp(cur_time);
		final_trajectory_.header.stamp = time_stamp;
		committed_segment_ = final_trajectory_;
		committed_goal_state_ = *goal_vehicle_state_ptr_;
		has_committed_goal_ = true;
		has_committed_segment_ = true;
		has_execution_status_ = false;
		constrain_replan_start_direction_ = false;
		if (path_split_result_os.size() > 1 &&
			!path_split_result_os[1].empty()) {
			expected_next_forward_ = path_split_result_os[1].front().w() > 0.5;
			has_expected_next_direction_ = true;
			ROS_DEBUG_STREAM("[open_space] remember next cusp direction: "
				<< (expected_next_forward_ ? "forward" : "reverse"));
		} else {
			has_expected_next_direction_ = false;
		}
			ROS_INFO_STREAM("[open_space] commit "
				<< (final_trajectory_.is_forward_shift ? "forward" : "reverse")
				<< " segment, points=" << final_trajectory_.points.size()
				<< ", remaining candidate segments=" << path_split_result_os.size() - 1);
		setParkingState(ParkingState::TRACKING_COMMITTED_SEGMENT);
		// 开放空间规划同样需要走显示回调，供可视化话题显示规划轨迹。
		if (callBack != nullptr && final_trajectory_.points.size() > 1) {
			callBack(&final_trajectory_, nullptr, nullptr, nullptr, nullptr);
		}
		}
	else
	{
		ROS_WARN_STREAM_THROTTLE(
			2.0, "[open_space] Hybrid A* search failed from ("
			<< start_state.x() << ", " << start_state.y() << ", " << start_state.z()
			<< ") to (" << goal_state.x() << ", " << goal_state.y() << ", "
			<< goal_state.z() << ")");
		return false;
	}
	kinodynamic_astar_searcher_ptr_->Reset();
	return true;
}

void OpenSpacePlanner::publishSearchDebugMarkers(
    const HybridAStarType::VectorVec4d &tree,
    const HybridAStarType::VectorVec3d &rs_path,
    const HybridAStarType::Vec3d &vehicle_pose) const
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const double cos_yaw = std::cos(vehicle_pose.z());
    const double sin_yaw = std::sin(vehicle_pose.z());
    const auto to_map_point = [&vehicle_pose, cos_yaw, sin_yaw](double x, double y) {
        geometry_msgs::Point point;
        point.x = vehicle_pose.x() + cos_yaw * x - sin_yaw * y;
        point.y = vehicle_pose.y() + sin_yaw * x + cos_yaw * y;
        point.z = 0.05;
        return point;
    };

    visualization_msgs::Marker expansion;
    expansion.header.frame_id = "map";
    expansion.header.stamp = ros::Time::now();
    expansion.ns = "hybrid_a_star_expansion";
    expansion.id = 0;
    expansion.type = visualization_msgs::Marker::LINE_LIST;
    expansion.action = visualization_msgs::Marker::ADD;
    expansion.scale.x = 0.015;
    expansion.color.g = 0.75;
    expansion.color.b = 1.0;
    expansion.color.a = 0.35;
    expansion.points.reserve(tree.size() * 2);
    for (const auto &edge : tree) {
        expansion.points.push_back(to_map_point(edge.x(), edge.y()));
        expansion.points.push_back(to_map_point(edge.z(), edge.w()));
    }
    markers.markers.push_back(expansion);

    visualization_msgs::Marker rs_connection;
    rs_connection.header.frame_id = "map";
    rs_connection.header.stamp = expansion.header.stamp;
    rs_connection.ns = "reeds_shepp_connection";
    rs_connection.id = 0;
    rs_connection.type = visualization_msgs::Marker::LINE_STRIP;
    rs_connection.action = visualization_msgs::Marker::ADD;
    rs_connection.scale.x = 0.06;
    rs_connection.color.r = 1.0;
    rs_connection.color.g = 0.1;
    rs_connection.color.b = 0.8;
    rs_connection.color.a = 1.0;
    rs_connection.points.reserve(rs_path.size());
    for (const auto &pose : rs_path) {
        rs_connection.points.push_back(to_map_point(pose.x(), pose.y()));
    }
    markers.markers.push_back(rs_connection);

    debug_marker_pub_.publish(markers);
}

void  OpenSpacePlanner::getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_)
{
	trajectory_ = final_trajectory_;
	return;
}

planning_msgs::TrajectoryPointArray OpenSpacePlanner::GetTraject(const HybridAStarType::VectorVec4d &path1, const HybridAStarType::Vec3d &start_state)
{
    planning_msgs::TrajectoryPointArray traject_in;

    std::vector<planning_msgs::TrajectoryPoint> tra_resample;
    for (const auto &path_point : path1)
    {
        planning_msgs::TrajectoryPoint point;
		point.x = start_state.x() + std::cos(start_state.z()) * path_point.x() -
			std::sin(start_state.z()) * path_point.y();
		point.y = start_state.y() + std::sin(start_state.z()) * path_point.x() +
			std::cos(start_state.z()) * path_point.y();
		point.theta = Mod2Pi(path_point.z() + start_state.z());
        tra_resample.push_back(point);
    }

    resamplePoints(0.5, tra_resample);

    // 控制器的横向 MPC 至少需要 3 个参考点。搜索末端附近的小段经重采样后
    // 可能仅剩起终两个点；在两点之间补一个几何中点，使其仍能作为独立换挡段执行。
    if (tra_resample.size() == 2)
    {
        planning_msgs::TrajectoryPoint middle_point;
        middle_point.x = 0.5 * (tra_resample.front().x + tra_resample.back().x);
        middle_point.y = 0.5 * (tra_resample.front().y + tra_resample.back().y);
        middle_point.theta = Mod2Pi(tra_resample.front().theta + 0.5 *
            Mod2Pi(tra_resample.back().theta - tra_resample.front().theta));
        tra_resample.insert(tra_resample.begin() + 1, middle_point);
    }

    for (unsigned int i = 0; i < tra_resample.size(); i++)
    {
        planning_msgs::TrajectoryPoint point;
        point.x = tra_resample[i].x;
        point.y = tra_resample[i].y;
        point.theta = tra_resample[i].theta;
        traject_in.points.push_back(point);
    }

    traject_in.is_forward_shift = path1.front().w() > 0.5;
    return traject_in;
}

// 附加速度
planning_msgs::TrajectoryPointArray OpenSpacePlanner::Velocity_Profile_output_os(
    const OpenSpace_config &vel_config,
    planning_msgs::TrajectoryPointArray trajectory)
{
    if (trajectory.points.empty()) {
        return trajectory;
    }

    const double direction = trajectory.is_forward_shift ? 1.0 : -1.0;
    const double cruise_speed = direction * std::fabs(vel_config.reverse_speed);
    const size_t last_index = trajectory.points.size() - 1;

    // 控制器会在每个新段前停车并准备前轮角，因此每段都从零速起步、在末端零速停车。
    for (auto &point : trajectory.points) {
        point.v = 0.0;
        point.a = 0.0;
    }

    size_t stop_index = last_index;
    double distance_to_end = 0.0;
    for (size_t i = last_index; i > 1; --i) {
        distance_to_end += distance2D(
            Point2D(trajectory.points[i].x, trajectory.points[i].y),
            Point2D(trajectory.points[i - 1].x, trajectory.points[i - 1].y));
        if (distance_to_end >= std::max(0.0, vel_config.safe_reverse_dis)) {
            stop_index = i;
            break;
        }
    }

    // 长轨迹后半程降低参考速度；短轨迹至少保留一个非零中间点，避免退化成停车指令。
    for (size_t i = 1; i < stop_index; ++i) {
        const bool use_cruise_speed = stop_index < 4 || i < stop_index / 2;
        trajectory.points[i].v = use_cruise_speed ? cruise_speed : 0.5 * cruise_speed;
    }

    double relative_time = 0.0;
    trajectory.points.front().relative_time = 0.0;
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        const double distance = distance2D(
            Point2D(trajectory.points[i].x, trajectory.points[i].y),
            Point2D(trajectory.points[i - 1].x, trajectory.points[i - 1].y));
        const double reference_speed = std::max(
            {std::fabs(trajectory.points[i - 1].v),
             std::fabs(trajectory.points[i].v), 0.02});
        relative_time += distance / reference_speed;
        trajectory.points[i].relative_time = relative_time;
    }
    return trajectory;
}

