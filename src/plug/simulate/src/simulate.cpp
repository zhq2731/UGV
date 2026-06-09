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

	geometry_msgs::Point origin;
	private_nh_.param<double>("latitude", origin.x, 0.0);
	private_nh_.param<double>("longitude", origin.y, 0.0);
	private_nh_.param<double>("altitude", origin.z, 0.0);
	std::cout << "origin ---------------------" << std::endl;
	std::cout << std::setprecision(10) << origin.x << std::endl;
	std::cout << std::setprecision(10) << origin.y << std::endl;
	std::cout << origin.z << std::endl;
	map_origin_ = origin;
	projector_ = projection::UtmProjector(origin);

	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);
	auto start = std::chrono::system_clock::now();
	multi_point_sub_ = nh_.subscribe("/multi_point_planning", 1, &Simulate::callBackMultiPointPlanning, this);
	loncmd_sub_ = nh_.subscribe("auto_chassis_drive_cmd", 10, &Simulate::onLonControlCommand, this);
	latcmd_sub_ = nh_.subscribe("auto_chassis_steeringwheel_cmd", 10, &Simulate::onLatControlCommand, this);
	// trajectory_sub_  = nh_.subscribe("trajectory", 10, &Simulate::trajectoryCallBack, this);
	pub_odometry_ = nh_.advertise<localization_msgs::Localization>("odomData", 10);
	pub_velcoty_ = nh_.advertise<std_msgs::Float32>("vehicle/status/velocity", 10);
	pub_chassis_ = nh_.advertise<driver_msgs::ChassisReport>("chassis", 10);
	inited = false;
	lon_timer_inited = false;
	lat_timer_inited = false;
	gpsPub_ = nh_.advertise<sensor_driver_msgs::GpswithHeading>("gpsdata", 10);

	bit_report_pub_ = nh_.advertise<ray_msgs::Report>("ecudatareport", 10);

	// timer  = nh_.createTimer(ros::Duration(0.1),&Simulate::callbackTimer,this);
	timer2 = nh_.createTimer(ros::Duration(0.01), &Simulate::callbackTimer2, this);
	clickPoint_sub_ = nh_.subscribe("/initialpose", 1, &Simulate::initPoseCallBack, this);
	initialPose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &Simulate::callBackinitialPose, this);
	goal_sub_ = nh_.subscribe("/clicked_point", 1, &Simulate::callBackgoal, this);

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
	std::cout << "simulate : motionStartCallback  " << msg->motion_start << std::endl;
	motion_start = msg->motion_start;
	if (0 == motion_start)
	{
		lon_timer_inited = false;
		lat_timer_inited = false;
	}
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

	std::cout << "p1 " << std::setprecision(10) << p1.x << " " << p1.y << std::endl;
	std::cout << "p2 " << std::setprecision(10) << p2.x << " " << p2.y << std::endl;
	p1 = projector_.forward(p1);

	p2 = projector_.forward(p2);
	std::cout << "after ---" << std::endl;
	std::cout << "p1 " << std::setprecision(10) << p1.x << " " << p1.y << std::endl;
	std::cout << "p2 " << std::setprecision(10) << p2.x << " " << p2.y << std::endl;

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

	v = msg->velocity_target;
	acc = msg->acc_target;

	pos.x = pos.x + v * std::cos(theta) * dt;
	pos.y = pos.y + v * std::sin(theta) * dt;
	v = v + acc * dt;
	lon_last_time = time_now;
}

void Simulate::callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg)
{
	std::cout << "[simulate] ====== 更新车辆初始位置 ======" << std::endl;
	pos.x = msg->pose.position.x;
	pos.y = msg->pose.position.y;
	theta = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->pose));

	v = 0.0;
	publishGpsdata();
	publishChassis();
	inited = true;
}

void Simulate::callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	plan_point_count++;
	if (plan_point_count == 1)
	{
		std::cout << "simulate: callBackinitialPose" << std::endl;
		theta = amathutils::getPoseYawAngle(msg->pose);
		pos.x = msg->pose.position.x;
		pos.y = msg->pose.position.y;
		v = 0.0;
		publishGpsdata();
		publishChassis();
		inited = true;
	}
}

void Simulate::callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg)
{
	return;
	std::cout << "[simulate] ====== 更新车辆初始位置 ======" << std::endl;
	pos.x = msg->poses[0].position.x;
	pos.y = msg->poses[0].position.y;
	theta = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->poses[0]));
	v = 0.0;
	publishGpsdata();
	publishChassis();
	inited = true;
}

void Simulate::callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg)
{
	std::cout << "simulate: clear plan_point_count to zero" << std::endl;
	plan_point_count = 0;
}

void Simulate::initPoseCallBack(
	const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
	geometry_msgs::Pose pose = msg->pose.pose;

	if (param.open_simulate_platoon)
	{
		static int click_count = 1;
		// int selfIndex = platoonGetNumIndex(vehicle_num_list,platformParam.num);

		// std::cout <<"click_count "<<click_count<<std::endl;
		// std::cout <<"platformParam.num "<<platformParam.num<<std::endl;
		if (click_count == platformParam.num)
		{
			theta = amathutils::getPoseYawAngle(pose);
			pos.x = pose.position.x;
			pos.y = pose.position.y;
			publishGpsdata();
			publishChassis();
			inited = true;
		}
		click_count++;
	}
	/*
	else{

		theta = amathutils::getPoseYawAngle(pose);
		pos.x = pose.position.x;
		pos.y = pose.position.y;
		//projector_ = projection::UtmProjector(origin);
		publishGpsdata();
		publishChassis();
		inited = true;

	}
	*/
	return;
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
	double steering_angle = msg->steering_wheel_angle;
	steering_angle = steering_angle * 3.1415926 / 180.0 / vehcileInfo->w2s_primary_coeff;
	theta = theta + v / vehcileInfo->wheel_base_m * std::tan(steering_angle) * dt;
	steering = msg->steering_wheel_angle;
	// publishGpsdata();
	// publishChassis();
	lat_last_time = time_now;
}

bool Simulate::readCordTxts(std::string &fileName, std::vector<geometry_msgs::Point> &points)
{

	std::ifstream filename(fileName);
	if (!filename)
	{
		std::cout << "ReferenceNode file open error: " << fileName << std::endl;
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
		std::cout << "Simulate file open error: " << fileName << std::endl;
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
		std::cout << "[callbackCloudmap] 地图更新标志：" << (int)msg->data << " → 地图已更新" << std::endl;
	if (msg->data == 0)
	{
		std::cout << "[callbackCloudmap] 地图更新标志：" << (int)msg->data << " → 地图未更新" << std::endl;
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

	ROS_INFO("referenceLine start.");

	Simulate simulate(nh);
	// simulate.publishInitPose();
	// simulate.publishInitPoseExt();

	ros::spin();

	ROS_INFO(" simulate  iteration end.");

	return 0;
}
