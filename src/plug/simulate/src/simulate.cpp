#include "simulate.h"
#include <unistd.h>

// Keep the angle within [-pi, pi]
double normalized_angle(double angle)
{
	if (angle > 180.0)
	{
		angle -= 2 * 180.0;
	}
	else if (angle < -180.0)
	{
		angle += 2 * 180.0;
	}
	return angle;
}
#define M_PI 3.14159265358979323846

Simulate::Simulate(ros::NodeHandle &nh) : nh_(nh), private_nh_("~")
{
	private_nh_.param<bool>("is_forward_shift", param.is_forward, true);
	private_nh_.param<bool>("reverse_path", param.reverse_path, true);
	private_nh_.param<bool>("open_simulate_platoon", param.open_simulate_platoon, false);
	private_nh_.param<bool>("open_space_execution_mode", open_space_execution_mode_, false);

	geometry_msgs::Point origin;
	private_nh_.param<double>("latitude", origin.x, 0.0);
	private_nh_.param<double>("longitude", origin.y, 0.0);
	private_nh_.param<double>("altitude", origin.z, 0.0);
	ROS_DEBUG_STREAM("[simulate] map origin: latitude=" << std::setprecision(10)
		<< origin.x << ", longitude=" << origin.y << ", altitude=" << origin.z);
	map_origin_ = origin;
	projector_ = projection::UtmProjector(origin);

	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);
	multi_point_sub_ = nh_.subscribe("/multi_point_planning", 1, &Simulate::callBackMultiPointPlanning, this);
	loncmd_sub_ = nh_.subscribe("auto_chassis_drive_cmd", 10, &Simulate::onLonControlCommand, this);
	latcmd_sub_ = nh_.subscribe("auto_chassis_steeringwheel_cmd", 10, &Simulate::onLatControlCommand, this);
	// trajectory_sub_  = nh_.subscribe("trajectory", 10, &Simulate::trajectoryCallBack, this);
	pub_odometry_ = nh_.advertise<localization_msgs::Localization>("odomData", 10);
	pub_velcoty_ = nh_.advertise<std_msgs::Float32>("vehicle/status/velocity", 10);
	pub_chassis_ = nh_.advertise<driver_msgs::ChassisReport>("chassis", 10);
	if (open_space_execution_mode_)
	{
		private_nh_.param<int>("mapParams_length", map_length_, 40);
		private_nh_.param<int>("mapParams_width", map_width_, 40);
		private_nh_.param<int>("mapParams_pointNum", map_point_num_, 360);
		private_nh_.param<double>("mapParams_resolution", map_resolution_, 0.2);
		private_nh_.param<double>(
			"parking_obstacle_length", parking_obstacle_length_, 5.0);
		private_nh_.param<double>(
			"parking_obstacle_width", parking_obstacle_width_, 2.5);
		private_nh_.param<double>("open_space_front_tire_steering_rate_limit_radps",
			open_space_front_tire_steering_rate_limit_radps_, 0.25);
		open_space_front_tire_steering_rate_limit_radps_ = std::max(
			0.0, open_space_front_tire_steering_rate_limit_radps_);
		free_space_map_pub_ =
			nh_.advertise<nav_msgs::OccupancyGrid>("free_space_map", 1, true);
		parking_visualization_pub_ =
			nh_.advertise<visualization_msgs::MarkerArray>(
				"parking_visualization", 1, true);
		open_space_task_reset_sub_ = nh_.subscribe(
			"/open_space_task_reset", 1,
			&Simulate::openSpaceTaskResetCallback, this);
	}
	inited = false;
	lon_timer_inited = false;
	lat_timer_inited = false;
	gpsPub_ = nh_.advertise<sensor_driver_msgs::GpswithHeading>("gpsdata", 10);

	bit_report_pub_ = nh_.advertise<ray_msgs::Report>("ecudatareport", 10);

	// timer  = nh_.createTimer(ros::Duration(0.1),&Simulate::callbackTimer,this);
	timer2 = nh_.createTimer(ros::Duration(0.01), &Simulate::callbackTimer2, this);
	clickPoint_sub_ = nh_.subscribe("/initialpose", 1, &Simulate::initPoseCallBack, this);
	if (open_space_execution_mode_) {
		parking_grid_timer_ = nh_.createTimer(
			ros::Duration(0.1), &Simulate::parkingGridTimer, this);
		goal_sub_ = nh_.subscribe(
			"/clicked_point", 1, &Simulate::callBackgoal, this);
	} else {
		// 道路仿真保留原有“第一次2D Nav Goal设置车辆位置”的入口。
		initialPose_sub_ = nh_.subscribe(
			"/move_base_simple/goal", 1, &Simulate::callBackinitialPose, this);
	}

	std::string vehicle_platform_file;
	private_nh_.param<std::string>("vehicle_platform_file", vehicle_platform_file, "vehicle_platform.yaml");

	common::getPlatformParam(vehicle_platform_file, platformParam);

	platoonMember_sub_ = nh.subscribe("/PlatoonMember", 10, &Simulate::callbackPlatoonMember, this);
	platoonMission_sub_ = nh.subscribe("/PlatoonMission", 10, &Simulate::callbackPlatoonMission, this);
	platoonConfig_sub_ = nh.subscribe("/PlatoonConfig", 10, &Simulate::callbackPlatoonConfig, this);

	platoonMember_self_sub_ = nh.subscribe("PlatoonMember_self", 10, &Simulate::callbackPlatoonMember, this);
	platoonConfig_self_sub_ = nh.subscribe("PlatoonMission_self", 10, &Simulate::callbackPlatoonMission, this);
	platoonMission_self_sub_ = nh.subscribe("PlatoonConfig_self", 10, &Simulate::callbackPlatoonConfig, this);

	motion_start_sub_ = nh.subscribe("/chassis_motion_start_cmd", 1, &Simulate::motionStartCallback, this);
	init_point_sub_ = nh.subscribe("/init_point", 1, &Simulate::callBackInitPoint, this);
	cloud_map_sub_ = nh_.subscribe("/mapSign", 1, &Simulate::callbackCloudmap, this);

	sleep(5);
}

void Simulate::motionStartCallback(const driver_msgs::MotionStartCmd::ConstPtr &msg)
{
	if (motion_start != msg->motion_start)
	{
		ROS_INFO_STREAM("[open_space_sim] motion_start "
			<< static_cast<int>(motion_start) << " -> "
			<< static_cast<int>(msg->motion_start));
	}
	motion_start = msg->motion_start;
	if (0 == motion_start)
	{
		lon_timer_inited = false;
		lat_timer_inited = false;
	}
}

void Simulate::openSpaceTaskResetCallback(const std_msgs::Empty::ConstPtr &msg)
{
	(void)msg;
	// 新任务必须重新按下运动开始按钮。先冻结车辆并清除积分时间，避免
	// 上一任务最后一条速度或转角命令在新目标设置后继续生效。
	motion_start = 0;
	v = 0.0;
	acc = 0.0;
	lon_timer_inited = false;
	lat_timer_inited = false;
	publishChassis();
	ROS_INFO("[open_space_sim] previous parking motion cleared; waiting for motion start");
}

void Simulate::callbackPlatoonMember(const platoon_msgs::PlatoonMember::ConstPtr &msg)
{
}

//
void Simulate::callbackPlatoonMission(const platoon_msgs::PlatoonMission::ConstPtr &msg)
{

	if (PlatoonType::BUILD != msg->command_type)
	{
		vehicle_num_list = msg->vehicle_list;
		built = true;
	}
	if (PlatoonType::DISSOLVE != msg->command_type)
	{
		built = false;
	}
	return;
}

//
void Simulate::callbackPlatoonConfig(const platoon_msgs::PlatoonConfig::ConstPtr &msg)
{

	return;
}

void Simulate::callbackTimer2(const ros::TimerEvent &event)
{
	if (!inited)
		return;
	publishGpsdata();
	publishChassis();
}

void Simulate::parkingGridTimer(const ros::TimerEvent &event)
{
	if (!inited || map_resolution_ <= 0.0)
		return;

	const int grid_width = std::max(1, static_cast<int>(std::round(2.0 * map_length_ / map_resolution_)));
	const int grid_height = std::max(1, static_cast<int>(std::round(2.0 * map_width_ / map_resolution_)));
	nav_msgs::OccupancyGrid grid;
	grid.header.stamp = event.current_real;
	grid.header.frame_id = "base_link";
	grid.info.resolution = map_resolution_;
	grid.info.width = grid_width;
	grid.info.height = grid_height;
	grid.info.origin.position.x = -map_length_;
	grid.info.origin.position.y = -map_width_;
	grid.info.origin.orientation.w = 1.0;
	// 感知模型：射线扫过的空闲格为25，命中障碍物为100，障碍物后方保持未知。
	grid.data.assign(grid_width * grid_height, -1);
	std::vector<unsigned char> obstacle_cells(grid_width * grid_height, 0);

	const double cos_theta = std::cos(theta);
	const double sin_theta = std::sin(theta);
	for (const auto &obstacle : parking_obstacles_)
	{
		const double dx = obstacle.x - pos.x;
		const double dy = obstacle.y - pos.y;
		const double center_x = cos_theta * dx + sin_theta * dy;
		const double center_y = -sin_theta * dx + cos_theta * dy;
		const double obstacle_yaw = obstacle.yaw - theta;
		const double half_length = parking_obstacle_length_ * 0.5;
		const double half_width = parking_obstacle_width_ * 0.5;
		const double extent_x = std::fabs(std::cos(obstacle_yaw)) * half_length +
			std::fabs(std::sin(obstacle_yaw)) * half_width;
		const double extent_y = std::fabs(std::sin(obstacle_yaw)) * half_length +
			std::fabs(std::cos(obstacle_yaw)) * half_width;
		const int min_x = std::max(0, static_cast<int>(std::floor(
			(center_x - extent_x - grid.info.origin.position.x) / map_resolution_)));
		const int max_x = std::min(grid_width - 1, static_cast<int>(std::floor(
			(center_x + extent_x - grid.info.origin.position.x) / map_resolution_)));
		const int min_y = std::max(0, static_cast<int>(std::floor(
			(center_y - extent_y - grid.info.origin.position.y) / map_resolution_)));
		const int max_y = std::min(grid_height - 1, static_cast<int>(std::floor(
			(center_y + extent_y - grid.info.origin.position.y) / map_resolution_)));

		for (int y = min_y; y <= max_y; ++y)
		{
			for (int x = min_x; x <= max_x; ++x)
			{
				const double local_x = grid.info.origin.position.x + (x + 0.5) * map_resolution_;
				const double local_y = grid.info.origin.position.y + (y + 0.5) * map_resolution_;
				const double rel_x = local_x - center_x;
				const double rel_y = local_y - center_y;
				const double obstacle_x = std::cos(obstacle_yaw) * rel_x + std::sin(obstacle_yaw) * rel_y;
				const double obstacle_y = -std::sin(obstacle_yaw) * rel_x + std::cos(obstacle_yaw) * rel_y;
				if (std::abs(obstacle_x) <= half_length && std::abs(obstacle_y) <= half_width)
					obstacle_cells[y * grid_width + x] = 1;
			}
		}
	}

	// 用密集的二维激光射线扫描真值障碍物。每束射线在首次命中障碍物时终止，
	// 因此遮挡物后方不会被误标成可通行区域。
	const int ray_count = std::max(1440, map_point_num_ * 4);
	const double ray_step = map_resolution_ * 0.5;
	const double max_range = std::hypot(static_cast<double>(map_length_), static_cast<double>(map_width_));
	for (int ray = 0; ray < ray_count; ++ray)
	{
		const double ray_yaw = -M_PI + 2.0 * M_PI * ray / ray_count;
		const double cos_ray = std::cos(ray_yaw);
		const double sin_ray = std::sin(ray_yaw);
		for (double range = 0.0; range < max_range; range += ray_step)
		{
			const double local_x = range * cos_ray;
			const double local_y = range * sin_ray;
			const int x = static_cast<int>(std::floor((local_x - grid.info.origin.position.x) / map_resolution_));
			const int y = static_cast<int>(std::floor((local_y - grid.info.origin.position.y) / map_resolution_));
			if (x < 0 || x >= grid_width || y < 0 || y >= grid_height)
				break;

			const int index = y * grid_width + x;
			if (obstacle_cells[index])
			{
				grid.data[index] = 100;
				break;
			}
			grid.data[index] = 25;
		}
	}

	free_space_map_pub_.publish(grid);
	publishParkingVisualization(event.current_real);
}

void Simulate::publishParkingVisualization(const ros::Time &stamp)
{
	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker clear_marker;
	clear_marker.action = visualization_msgs::Marker::DELETEALL;
	markers.markers.push_back(clear_marker);

	visualization_msgs::Marker vehicle;
	vehicle.header.frame_id = "map";
	vehicle.header.stamp = stamp;
	vehicle.ns = "parking_vehicle";
	vehicle.id = 0;
	vehicle.type = visualization_msgs::Marker::CUBE;
	vehicle.action = visualization_msgs::Marker::ADD;
	vehicle.pose.position = pos;
	vehicle.pose.position.z = 0.5;
	vehicle.pose.orientation = amathutils::getQuaternionFromYaw(theta);
	vehicle.scale.x = vehcileInfo->vehicle_length_m;
	vehicle.scale.y = vehcileInfo->vehicle_width_m;
	vehicle.scale.z = 1.0;
	vehicle.color.r = 0.1;
	vehicle.color.g = 0.4;
	vehicle.color.b = 1.0;
	vehicle.color.a = 0.9;
	markers.markers.push_back(vehicle);

	// 仿真中的 steering 保存的是方向盘角度（度）；底盘反馈使用同一数值，
	// 因此这里换算为实际前轮转角后绘制，便于直接观察控制器的转向准备过程。
	const double front_tire_angle = static_cast<double>(steering) * M_PI / 180.0 /
		vehcileInfo->w2s_primary_coeff;
	// 车辆几何中心作为车体 Marker 原点，前轴位置由前悬和整车长度确定。
	const double front_axle_x = vehcileInfo->vehicle_length_m * 0.5 -
		vehcileInfo->front_overhang_m;
	// 为了在俯视 RViz 中不被车体 CUBE 遮挡，轮心稍微位于车身侧缘外，
	// 并将 Marker 绘制在车体上方；这只影响可视化，不改变车辆运动学。
	const double front_wheel_lateral_offset = vehcileInfo->vehicle_width_m * 0.56;
	const double wheel_length = 0.80;
	const double wheel_width = 0.28;
	for (int side : {-1, 1}) {
		const double local_y = side * front_wheel_lateral_offset;
		visualization_msgs::Marker front_wheel;
		front_wheel.header.frame_id = "map";
		front_wheel.header.stamp = stamp;
		front_wheel.ns = "parking_front_wheel";
		front_wheel.id = side > 0 ? 0 : 1;
		front_wheel.type = visualization_msgs::Marker::CUBE;
		front_wheel.action = visualization_msgs::Marker::ADD;
		front_wheel.pose.position.x = pos.x + std::cos(theta) * front_axle_x -
			std::sin(theta) * local_y;
		front_wheel.pose.position.y = pos.y + std::sin(theta) * front_axle_x +
			std::cos(theta) * local_y;
		front_wheel.pose.position.z = 1.08;
		front_wheel.pose.orientation = amathutils::getQuaternionFromYaw(
			theta + front_tire_angle);
		front_wheel.scale.x = wheel_length;
		front_wheel.scale.y = wheel_width;
		front_wheel.scale.z = 0.14;
		front_wheel.color.r = 0.15;
		front_wheel.color.g = 1.0;
		front_wheel.color.b = 0.10;
		front_wheel.color.a = 1.0;
		markers.markers.push_back(front_wheel);
	}

	for (std::size_t i = 0; i < parking_obstacles_.size(); ++i)
	{
		const auto &obstacle = parking_obstacles_[i];
		visualization_msgs::Marker obstacle_marker;
		obstacle_marker.header.frame_id = "map";
		obstacle_marker.header.stamp = stamp;
		obstacle_marker.ns = "parking_obstacle";
		obstacle_marker.id = static_cast<int>(i);
		obstacle_marker.type = visualization_msgs::Marker::CUBE;
		obstacle_marker.action = visualization_msgs::Marker::ADD;
		obstacle_marker.pose.position.x = obstacle.x;
		obstacle_marker.pose.position.y = obstacle.y;
		obstacle_marker.pose.position.z = 0.75;
		obstacle_marker.pose.orientation = amathutils::getQuaternionFromYaw(obstacle.yaw);
		obstacle_marker.scale.x = parking_obstacle_length_;
		obstacle_marker.scale.y = parking_obstacle_width_;
		obstacle_marker.scale.z = 1.5;
		obstacle_marker.color.r = 0.9;
		obstacle_marker.color.g = 0.1;
		obstacle_marker.color.b = 0.1;
		obstacle_marker.color.a = 0.85;
		markers.markers.push_back(obstacle_marker);
	}

	parking_visualization_pub_.publish(markers);
}

void Simulate::callbackTimer(const ros::TimerEvent &event)
{
	if (!inited)
		return;

	if (!motion_start)
		return;

	if (trajectory_map.empty())
	{
		v = 0.0;
		return;
	}

	ros::Time current_system_time = event.current_real;
	double current_relative_time = (current_system_time - header_time).toSec();
	auto it_upper = trajectory_map.lower_bound(current_relative_time);
	auto it_lower = it_upper;
	if (it_lower == trajectory_map.begin())
	{
		TrajectoryPointData &first_point = trajectory_map.begin()->second;
		pos.x = first_point.x;
		pos.y = first_point.y;
		theta = first_point.theta;
		v = first_point.v;
		return;
	}
	--it_lower;
	if (it_upper == trajectory_map.end())
	{
		TrajectoryPointData &last_point = trajectory_map.rbegin()->second;
		pos.x = last_point.x;
		pos.y = last_point.y;
		theta = last_point.theta;
		v = last_point.v;
		return;
	}

	TrajectoryPointData &point_before = it_lower->second;
	TrajectoryPointData &point_after = it_upper->second;
	/*
	std::cout << "Point Before (t < current):" << std::endl;
	std::cout << "  relative_time: " << point_before.relative_time << " s" << std::endl;
	std::cout << "  x: " << point_before.x
		 << ", y: " << point_before.y
		 << ", z: " << point_before.z << std::endl;
	std::cout << "  theta: " << point_before.theta << " rad ("
		 << (point_before.theta * 180 / M_PI) << "°)" << std::endl;

	std::cout << "Point After (t >= current):" << std::endl;
	std::cout << "  relative_time: " << point_after.relative_time << " s" << std::endl;
	std::cout << "  x: " <<  point_after.x
		 << ", y: " <<  point_after.y
		 << ", z: " <<  point_after.z << std::endl;
	std::cout << "  theta: " << point_after.theta << " rad ("
		<< (point_after.theta * 180 / M_PI) << "°)" << std::endl;
	*/

	double t1 = point_before.relative_time;
	double t2 = point_after.relative_time;
	double t_current = current_relative_time;
	double alpha = (std::fabs(t2 - t1) < 1e-6) ? 0.0 : (t_current - t1) / (t2 - t1);
	double x_interp = point_before.x + alpha * (point_after.x - point_before.x);
	double y_interp = point_before.y + alpha * (point_after.y - point_before.y);
	double z_interp = point_before.z + alpha * (point_after.z - point_before.z);
	double v_interp = point_before.v + alpha * (point_after.v - point_before.v);
	double theta_interp = interpolateTheta(point_before.theta, point_after.theta, alpha);
	pos.x = x_interp;
	pos.y = y_interp;
	theta = theta_interp;
	v = v_interp;
	// std::cout <<"delat t :: "<<t_current<<std::endl;
	// std::cout <<"vt :: "<<v<<std::endl;

	/**
	std::cout << "=== Interpolation Result ===" << std::endl;
	std::cout << "  x_interp: " <<  x_interp << std::endl;
	std::cout << "  y_interp: " <<  y_interp << std::endl;
	std::cout << "  z_interp: " <<  z_interp << std::endl;
	std::cout << "Interpolated theta: " << theta_interp << " rad ("
			  << (theta_interp * 180 / M_PI) << "°)" << std::endl;
	std::cout << "========================================" << std::endl << std::endl;
	**/
}

double Simulate::interpolateTheta(double theta1, double theta2, double alpha)
{
	double delta_theta = theta2 - theta1;
	if (delta_theta > M_PI)
	{
		delta_theta -= 2 * M_PI;
	}
	else if (delta_theta < -M_PI)
	{
		delta_theta += 2 * M_PI;
	}
	double theta_interp = theta1 + alpha * delta_theta;
	theta_interp = fmod(theta_interp + M_PI, 2 * M_PI) - M_PI;
	return theta_interp;
}

void Simulate::publishLocalizationMsg(double x, double y, double z, double theta)
{
	localization_msgs::Localization gpsposemsgs;
	geometry_msgs::Point pos;
	pos.x = x;
	pos.y = y;
	pos.z = z;
	gpsposemsgs.location.pose.pose.position = pos;
	geometry_msgs::Point latd_lon = projector_.reverse(pos);
	gpsposemsgs.original_ins.latitude = latd_lon.x;
	gpsposemsgs.original_ins.longitude = latd_lon.y;
	gpsposemsgs.original_ins.nav_uncertainty = 100;
	gpsposemsgs.location.pose.pose.orientation = amathutils::getQuaternionFromYaw(theta);
	pub_odometry_.publish(gpsposemsgs);
}

void Simulate::trajectoryCallBack(const planning_msgs::TrajectoryPointArray::ConstPtr &msg)
{
	header_time = msg->header.stamp;
	bool is_forward_shift = msg->is_forward_shift;
	if (is_forward_shift != last_is_forward_shift)
	{
		v = 0.0;
		pos.x = msg->points[0].x;
		pos.y = msg->points[0].y;
		// theta = amathutils::normalizeRadian(msg->points[0].theta + M_PI);
		last_is_forward_shift = is_forward_shift;
	}
	trajectory_map.clear();
	for (const auto &point : msg->points)
	{
		TrajectoryPointData point_data;
		point_data.relative_time = point.relative_time;
		point_data.x = point.x;
		point_data.y = point.y;
		point_data.z = point.z;
		point_data.theta = point.theta;
		point_data.v = point.v;
		if (!is_forward_shift)
		{
			if (point_data.v > 1e-6)
				point_data.v = -point_data.v;
		}
		if (msg->points.size() < 4)
			point_data.v = 0.0;
		trajectory_map[point.relative_time] = point_data;
	}
}

void Simulate::publishInitPose()
{
	if (inited)
		return;

	if (param.open_simulate_platoon)
		return;

	std::string fileName;
	private_nh_.param<std::string>("file_name", fileName, "gpsData.txt");
	std::string recode_data_dir = ros::package::getPath("launch_node");
	std::string pathFile = recode_data_dir + std::string("/data/") + fileName;

	std::vector<geometry_msgs::Point> points;
	readGpsTxts(pathFile, points);
	// readCordTxts(pathFile,points);
	geometry_msgs::Point p1, p2;

	if (param.is_forward)
	{
		p1 = points[0];
		p2 = points[1];
	}
	else
	{
		p1 = points[points.size() - 2];
		p2 = points[points.size() - 1];
	}

	ROS_DEBUG_STREAM("[simulate] reference points before projection: p1=("
		<< std::setprecision(10) << p1.x << ", " << p1.y << "), p2=("
		<< p2.x << ", " << p2.y << ")");
	p1 = projector_.forward(p1);

	p2 = projector_.forward(p2);
	ROS_DEBUG_STREAM("[simulate] reference points after projection: p1=("
		<< std::setprecision(10) << p1.x << ", " << p1.y << "), p2=("
		<< p2.x << ", " << p2.y << ")");

	pos = p1;
	double angle = std::atan2(p2.y - p1.y, p2.x - p1.x);
	theta = angle;

	acc = 0;
	v = 0.0;
	steering = 0;

	publishGpsdata();

	publishChassis();
	inited = true;
}

double Simulate::normalizeRadian(const double _angle)
{
	double n_angle = std::fmod(_angle, 2 * M_PI);
	n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle
																	: n_angle;

	// another way
	// Math.atan2(Math.sin(_angle), Math.cos(_angle));
	return n_angle;
}

void Simulate::publishGpsdata()
{
	// if (!init_point_initialized_)
	// return;
	localization_msgs::Localization gpsposemsgs;
	gpsposemsgs.location.pose.pose.position.x = pos.x;
	gpsposemsgs.location.pose.pose.position.y = pos.y;
	double theta_out = theta - M_PI / 2; //
	if (theta_out > M_PI)
	{
		theta_out -= 2 * M_PI;
	}
	else if (theta_out < -M_PI)
	{
		theta_out += 2 * M_PI;
	}

	geometry_msgs::Point latd_lon = projector_.reverse(pos);
	gpsposemsgs.original_ins.latitude = latd_lon.x;
	gpsposemsgs.original_ins.longitude = latd_lon.y;

	gpsposemsgs.original_ins.nav_uncertainty = 100;
	gpsposemsgs.location.pose.pose.orientation = amathutils::getQuaternionFromYaw(theta_out);
	pub_odometry_.publish(gpsposemsgs);

	tf::Transform map_to_base;
	map_to_base.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
	tf::Quaternion map_to_base_q;
	map_to_base_q.setRPY(0.0, 0.0, theta);
	map_to_base.setRotation(map_to_base_q);
	tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_base, ros::Time::now(), "map", "base_link"));
}

void Simulate::publishChassis()
{
	// if (!init_point_initialized_)
	// return;

	driver_msgs::ChassisReport chassis;
	chassis.steering_wheel_angle = steering;
	chassis.current_velocity = v;
	chassis.driving_mode = 1;

	pub_chassis_.publish(chassis);
	ray_msgs::Report bit_report_;
	bit_report_.motion.vehicle_speed = v; // m/s
	bit_report_pub_.publish(bit_report_);
}

void Simulate::onLonControlCommand(const driver_msgs::DriveCmd::ConstPtr msg)
{
	if (!inited)
		return;

	if (0 == motion_start)
	{
		return;
	}

	if (0 == lon_timer_inited)
	{
		lon_timer_inited = true;
		lon_last_time = std::chrono::system_clock::now();
	}

	auto time_now = std::chrono::system_clock::now();
	std::chrono::duration<double> diff = time_now - lon_last_time;
	double dt = diff.count();

	if (open_space_execution_mode_)
	{
		// 泊车仿真将速度保留为状态量；控制器提供受限加速度指令，
		// 不再把速度直接跳变为目标速度。
		const double previous_velocity = v;
		const double target_velocity = msg->velocity_target;
		acc = msg->acc_target;
		double next_velocity = previous_velocity + acc * dt;
		// 防止越过目标速度，或在停车过程中跨越零速度。
		if ((target_velocity - previous_velocity) * (target_velocity - next_velocity) <= 0.0 ||
			(std::fabs(target_velocity) <= 1.0e-3 && previous_velocity * next_velocity < 0.0))
		{
			next_velocity = target_velocity;
		}
		const double integration_velocity = 0.5 * (previous_velocity + next_velocity);
		pos.x = pos.x + integration_velocity * std::cos(theta) * dt;
		pos.y = pos.y + integration_velocity * std::sin(theta) * dt;
		v = next_velocity;
	}
	else
	{
		// 原有参考线仿真行为。
		v = msg->velocity_target;
		acc = msg->acc_target;
		pos.x = pos.x + v * std::cos(theta) * dt;
		pos.y = pos.y + v * std::sin(theta) * dt;
		v = v + acc * dt;
	}
	lon_last_time = time_now;
}

void Simulate::callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg)
{
	pos.x = msg->pose.position.x;
	pos.y = msg->pose.position.y;
	theta = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->pose));

	v = 0.0;
	publishGpsdata();
	publishChassis();
	inited = true;
	ROS_INFO_STREAM("[simulate] vehicle pose initialized from init_point: ("
		<< pos.x << ", " << pos.y << ", " << theta << ")");
}

void Simulate::callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	plan_point_count++;
	if (plan_point_count == 1)
	{
		theta = amathutils::getPoseYawAngle(msg->pose);
		pos.x = msg->pose.position.x;
		pos.y = msg->pose.position.y;
		v = 0.0;
		publishGpsdata();
		publishChassis();
		inited = true;
		ROS_DEBUG_STREAM("[simulate] vehicle pose initialized from pose: ("
			<< pos.x << ", " << pos.y << ", " << theta << ")");
	}
}

void Simulate::callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg)
{
	return;
}

void Simulate::callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg)
{
	if (!inited)
	{
		ROS_WARN("[open_space_sim] ignore obstacle before vehicle pose initialization");
		return;
	}
	ParkingObstacle obstacle;
	obstacle.x = msg->point.x;
	obstacle.y = msg->point.y;
	obstacle.yaw = theta;
	parking_obstacles_.push_back(obstacle);
	ROS_INFO_STREAM("[open_space_sim] obstacle added at ("
		<< obstacle.x << ", " << obstacle.y << ")");
}

void Simulate::initPoseCallBack(
	const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
	geometry_msgs::Pose pose = msg->pose.pose;
	if (!open_space_execution_mode_)
	{
		if (!param.open_simulate_platoon) {
			return;
		}
		static int click_count = 1;
		if (click_count++ != platformParam.num) {
			return;
		}
		theta = amathutils::getPoseYawAngle(pose);
		pos.x = pose.position.x;
		pos.y = pose.position.y;
		publishGpsdata();
		publishChassis();
		inited = true;
		return;
	}

	// 开放空间仿真将 /initialpose 视为一次完整的车辆重新初始化。
	theta = amathutils::getPoseYawAngle(pose);
	pos.x = pose.position.x;
	pos.y = pose.position.y;
	v = 0.0;
	acc = 0.0;
	steering = 0;
	motion_start = 0;
	lon_timer_inited = false;
	lat_timer_inited = false;
	parking_obstacles_.clear();
	publishGpsdata();
	publishChassis();
	inited = true;
	ROS_INFO_STREAM("[open_space_sim] vehicle initialized at ("
		<< pos.x << ", " << pos.y << ", " << theta << ")");
}

void Simulate::onLatControlCommand(const driver_msgs::SteeringWheelCmd::ConstPtr msg)
{

	if (!inited)
		return;

	if (!motion_start)
		return;

	if (!lat_timer_inited)
	{
		lat_timer_inited = true;
		lat_last_time = std::chrono::system_clock::now();
	}
	auto time_now = std::chrono::system_clock::now();

	std::chrono::duration<double> diff = time_now - lat_last_time;
	double dt = diff.count();
	const double target_tire_angle = msg->steering_wheel_angle * M_PI / 180.0 /
		vehcileInfo->w2s_primary_coeff;
	const double previous_tire_angle = steering * M_PI / 180.0 /
		vehcileInfo->w2s_primary_coeff;
	double next_tire_angle = target_tire_angle;
	if (open_space_execution_mode_)
	{
		// 泊车仿真中将前轮转角作为具有变化速度上限的实际状态，模拟真车
		// 转向执行器；控制器下发的是目标角，底盘反馈发布的是限速后的实际角。
		const double max_angle_change =
			open_space_front_tire_steering_rate_limit_radps_ * std::max(0.0, dt);
		const double angle_error = target_tire_angle - previous_tire_angle;
		const double limited_angle_change = std::max(-max_angle_change,
			std::min(angle_error, max_angle_change));
		next_tire_angle = previous_tire_angle + limited_angle_change;
		ROS_DEBUG_THROTTLE(0.5,
			"[open_space_sim] steering actuator: target=%.3f rad, actual=%.3f rad, "
			"rate_limit=%.3f rad/s",
			target_tire_angle, next_tire_angle,
			open_space_front_tire_steering_rate_limit_radps_);
	}

	// 限速期间用本周期平均转角积分航向，避免把周期末转角作用于整个周期。
	const double integration_tire_angle = open_space_execution_mode_
		? 0.5 * (previous_tire_angle + next_tire_angle) : next_tire_angle;
	theta = theta + v / vehcileInfo->wheel_base_m *
		std::tan(integration_tire_angle) * dt;
	steering = next_tire_angle * 180.0 / M_PI * vehcileInfo->w2s_primary_coeff;
	// publishGpsdata();
	// publishChassis();
	lat_last_time = time_now;
}

bool Simulate::readCordTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points)
{

	std::ifstream filename(fileName);
	if (!filename)
	{
		ROS_ERROR_STREAM("[simulate] failed to open coordinate file: " << fileName);
		return false;
	}
	std::string oneLine;
	getline(filename, oneLine);
	int first = 0;
	while (getline(filename, oneLine))
	{
		geometry_msgs::Point p;
		std::istringstream streamOneLine(oneLine);
		streamOneLine >> p.x;
		streamOneLine >> p.y;
		points.push_back(p);
	}

	if (param.reverse_path)
		std::reverse(points.begin(), points.end());

	filename.close();
	return true;
}

bool Simulate::readGpsTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points)
{
	std::ifstream filename(fileName);
	if (!filename)
	{
		ROS_ERROR_STREAM("[simulate] failed to open GPS file: " << fileName);
		return false;
	}
	std::string oneLine;
	getline(filename, oneLine);
	int first = 0;
	while (getline(filename, oneLine))
	{

		std::string ignore;
		geometry_msgs::Point trajectPoint;
		std::istringstream streamOneLine(oneLine);

		streamOneLine >> ignore;
		streamOneLine >> ignore;
		streamOneLine >> ignore;

		streamOneLine >> trajectPoint.x;
		streamOneLine >> trajectPoint.y;
		points.push_back(trajectPoint);
	}
	if (param.reverse_path)
		std::reverse(points.begin(), points.end());
	filename.close();
	return true;
}

void Simulate::callbackCloudmap(const std_msgs::UInt8::ConstPtr &msg)
{
	if (msg->data == 1)
		ROS_DEBUG("[simulate] cloud map origin updated");
	if (msg->data == 0)
	{
		ROS_DEBUG("[simulate] cloud map origin is unchanged");
		return;
	}
	std::string pkg_dir = ros::package::getPath("launch_node");
	std::string config_file = pkg_dir + std::string("/param/global/global_config.yaml");
	YAML::Node doc = YAML::LoadFile(config_file);
	map_origin_.x = doc["latitude"].as<double>();
	map_origin_.y = doc["longitude"].as<double>();
	map_origin_.z = doc["altitude"].as<double>();
	projector_ = projection::UtmProjector(map_origin_);
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "simulate");
	ros::NodeHandle nh;

	Simulate simulate(nh);
	// simulate.publishInitPose();
	// simulate.publishInitPoseExt();

	ros::spin();

	return 0;
}
