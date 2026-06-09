#include "hybrid_a_star/openSpacePlanner.h"
//using namespace ugv::planning;

double angle(double x1, double y1)
{
    double radianAngle = std::atan2(y1, x1); // 使用反余弦函数得到弧度值
    return radianAngle;
}

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

// 一整条路应该是VectorVec4d
// 分成段变成std::vector<VectorVec4d>
std::vector<HybridAStarType::VectorVec4d> path_split(HybridAStarType::VectorVec3d &path)
{
    std::vector<HybridAStarType::VectorVec4d> path_split_all;
    HybridAStarType::VectorVec4d path_split;
    double heading_angle = path[0].z();
    const HybridAStarType::Vec2d init_tracking_vector(path[1].x() - path[0].x(), path[1].y() - path[0].y());
    double tracking_angle = angle(init_tracking_vector.x(), init_tracking_vector.y());
    // true是前进，false是后退
    bool current_gear =
        std::fabs(tracking_angle - heading_angle) < M_PI_2;
    for (unsigned int i = 0; i < path.size(); i++)
    {
        HybridAStarType::Vec4d point;
        heading_angle = path[i].z();
        const HybridAStarType::Vec2d tracking_vector(path[i + 1].x() - path[i].x(), path[i + 1].y() - path[i].y());
        double current_tracking_angle = angle(tracking_vector.x(), tracking_vector.y());
        bool gear = std::fabs(current_tracking_angle - heading_angle) < M_PI_2;
        //   if(gear){std::cout<<"forward"<<std::endl;}else{std::cout<<"reverse"<<std::endl;}
        if (gear != current_gear)
        {
            point.x() = path[i].x();
            point.y() = path[i].y();
            point.z() = path[i].z();
            if (current_gear == true)
            {
                point.w() = 1;
            }
            else
            {
                point.w() = 0;
            }

            path_split.emplace_back(point);
            path_split_all.emplace_back(path_split);
            path_split.clear();
            current_gear = gear;
        }
        point.x() = path[i].x();
        point.y() = path[i].y();
        point.z() = path[i].z();
        if (current_gear == true)
        {
            point.w() = 1;
        }
        else
        {
            point.w() = 0;
        }

        path_split.emplace_back(point);
    }
    path_split_all.emplace_back(path_split);
    // std::cout << "路径分段大小：" << path_split_all.size() << std::endl;
    return path_split_all;
}

// only for test
std::vector<HybridAStarType::VectorVec3d> path_split_3(HybridAStarType::VectorVec3d &path)
{
    std::vector<HybridAStarType::VectorVec3d> path_split_all;
    HybridAStarType::VectorVec3d path_split;
    double heading_angle = path[0].z();
    const HybridAStarType::Vec2d init_tracking_vector(path[1].x() - path[0].x(), path[1].y() - path[0].y());
    double tracking_angle = angle(init_tracking_vector.x(), init_tracking_vector.y());
    // true是前进，false是后退
    bool current_gear =
        std::fabs(tracking_angle - heading_angle) < M_PI_2;
    for (unsigned int i = 0; i < path.size(); i++)
    {
        HybridAStarType::Vec3d point;
        heading_angle = path[i].z();
        const HybridAStarType::Vec2d tracking_vector(path[i + 1].x() - path[i].x(), path[i + 1].y() - path[i].y());
        double current_tracking_angle = angle(tracking_vector.x(), tracking_vector.y());
        bool gear = std::fabs(current_tracking_angle - heading_angle) < M_PI_2;
        //   if(gear){std::cout<<"forward"<<std::endl;}else{std::cout<<"reverse"<<std::endl;}
        if (gear != current_gear)
        {
            point.x() = path[i].x();
            point.y() = path[i].y();
            point.z() = path[i].z();
            path_split.emplace_back(point);
            path_split_all.emplace_back(path_split);
            path_split.clear();
            current_gear = gear;
        }
        point.x() = path[i].x();
        point.y() = path[i].y();
        point.z() = path[i].z();
        path_split.emplace_back(point);
    }
    path_split_all.emplace_back(path_split);
    // std::cout << path_split_all.size() << std::endl;
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
    const std::vector<float64_t> &accumulated_lengths, const float64_t target_length)
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
    float64_t acculate_s = 0;
    std::vector<float64_t> accumulated_lengths;
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
        const float64_t target_length = (static_cast<float64_t>(i) / num_segments) * acculate_s;
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
    std::vector<planning_msgs::TrajectoryPoint>().swap(pts);
    pts = resampled_points;
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

// 将格子坐标转为世界坐标 无旋转
HybridAStarType::Vec2d GridToUtm(double &x, double &y, double &x_utm, double &y_utm, double &x_utm_in_grid, double &y_utm_in_grid, double yaw_cha, double &map_resolution)
{
    // x,y是路径点在格子下的坐标，x_utm,y_utm是车辆当前位置在UTM下的坐标，x_utm_in_grid,y_utm_in_grid是车辆当前位置在格子系下的坐标
    HybridAStarType::Vec2d global_utm;
    global_utm.x() = x_utm + (x - x_utm_in_grid) * map_resolution;
    global_utm.y() = y_utm + (y - y_utm_in_grid) * map_resolution;
    return global_utm;
}

//有旋转
// HybridAStarType::Vec2d GridToUtm(double &x, double &y, double &x_utm, double &y_utm, double &x_utm_in_grid, double &y_utm_in_grid, double yaw_cha, double &map_resolution)
// {
//     // x,y是路径点在格子下的坐标，x_utm,y_utm是车辆当前位置在UTM下的坐标，x_utm_in_grid,y_utm_in_grid是车辆当前位置在格子系下的坐标
//     HybridAStarType::Vec2d global_utm;
//     if(yaw_cha>3.14)
//     {
//         yaw_cha = yaw_cha - 6.28;
//     }
//     else if (yaw_cha < -3.14)
//     {
//         yaw_cha = yaw_cha +6.28;
//     }
    
//     double x_xuanzhuan = (x-x_utm_in_grid)*cos(yaw_cha)-(y-y_utm_in_grid)*sin(yaw_cha);
//     double y_xuanzhuan = (y-y_utm_in_grid)*cos(yaw_cha)+(x-x_utm_in_grid)*sin(yaw_cha);
//     global_utm.x() = x_xuanzhuan*map_resolution + x_utm;
//     global_utm.y() = y_xuanzhuan*map_resolution + y_utm;
//     return global_utm;
// }

//将世界坐标转为格子坐标 无旋转
HybridAStarType::Vec2d UtmToGrid(double &x, double &y, double &x_utm, double &y_utm, double &x_utm_in_grid, double &y_utm_in_grid, double yaw_cha, double &map_resolution)
{
    // x,y是终点在UTM下的坐标，x_utm,y_utm是车辆当前位置在UTM下的坐标，x_utm_in_grid,y_utm_in_grid是车辆当前位置在格子系下的坐标
    HybridAStarType::Vec2d utm_in_grid;
    utm_in_grid.x() = (x - x_utm) / map_resolution + x_utm_in_grid;
    utm_in_grid.y() = (y - y_utm) / map_resolution + y_utm_in_grid;
    // std::cout<<"goal utm("<<x<<","<<y<<")"<<std::endl;
    // std::cout<<"current utm("<<x_utm<<","<<y_utm<<")"<<std::endl;
    // std::cout<<"current_grid("<<x_utm_in_grid<<","<<y_utm_in_grid<<")"<<std::endl;
    return utm_in_grid;
}

OpenSpacePlanner::OpenSpacePlanner(displayCallback callBack_):BasePlanner(callBack_){
	std::string param_node_dir;
    param_node_dir = ros::package::getPath("open_space_planner");

    std::string openspace_config_yaml_file = param_node_dir + std::string("/param/") + std::string("openspace.yaml");
    YAML::Node config_;
    
    prev_direction_ = true;
    prev_profile_.clear();

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

    double steering_angle = openspace_config.steering_angle;
    int steering_angle_discrete_num = openspace_config.steering_angle_discrete_num;
    double wheel_base = openspace_config.wheel_base;
    double segment_length = openspace_config.segment_length;
    int segment_length_discrete_num = openspace_config.segment_length_discrete_num;
    double steering_penalty = openspace_config.steering_penalty;
    double steering_change_penalty = openspace_config.steering_change_penalty;
    ;
    double reversing_penalty = openspace_config.reversing_penalty;
    double shot_distance = openspace_config.shot_distance;

    // std::cout<<"车长:"<<openspace_config.car_length<<std::endl;
    // std::cout<<"车宽:"<<openspace_config.car_width<<std::endl;
    // std::cout<<"轴距:"<<openspace_config.wheel_base<<std::endl;

    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
        steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base,
        steering_penalty, reversing_penalty, steering_change_penalty, shot_distance);

    //
	// 初始化地图为false
    // has_map_ = false;
	current_vehicle_state_ptr_ = nullptr;
	goal_vehicle_state_ptr_ = nullptr;
	current_costmap_ptr_ = nullptr;
    /*
    //初始化glog
	FLAGS_max_log_size = 10;     //最大日志文件大小 10M 
	//定义日志文件路径
    std::string log_info_path =  param_node_dir + std::string("/log/");
    // FLAGS_log_dir = log_info_path;
    //自定义文件名前缀Test  
    std::string log_info = log_info_path +"osplanning_INFOlog_";
    std::string log_warn = log_info_path +"osplanning_WARNlog_";
    google::SetLogDestination(google::INFO, log_info.c_str());
    google::SetLogDestination(google::WARNING, log_warn.c_str());
    */
}
void  OpenSpacePlanner::setInputData(InputData const & input_data) {
	// current_init_pose_ptr_->pose.pose.position.x = input_data.vehicleState.x;
	current_vehicle_state_ptr_ = std::make_shared<VehicleState>(input_data.vehicleState);
	goal_vehicle_state_ptr_ = std::make_shared<VehicleState>(input_data.goalState);//待开发
	// current_goal_pose_ptr_ = ;
	current_costmap_ptr_ = boost::make_shared<nav_msgs::OccupancyGrid>(input_data.occGrid);
    car_vel = current_vehicle_state_ptr_->v;
}
bool  OpenSpacePlanner::implement(double cur_time,const DiscretizedTrajectory prev_trajectory) {

	if (current_costmap_ptr_== nullptr)
	{
		//LOG(ERROR)<<"no map"<<std::endl;
		std::cout <<"no map"<<std::endl;
		return false;
	}
	// 地图分辨率
	map_resolution = 0.2;
	double state_resolution = 1;
	// 初始化
	kinodynamic_astar_searcher_ptr_->Init(
		0,
		1.0 * current_costmap_ptr_->info.width,
		0,
		1.0 * current_costmap_ptr_->info.height,
		state_resolution,map_resolution,
		openspace_config.car_length,
		openspace_config.car_width,
		openspace_config.wheel_base);
	// 分成每一个小格  宽、高
	unsigned int map_w = std::floor(current_costmap_ptr_->info.width);
	unsigned int map_h = std::floor(current_costmap_ptr_->info.height); //
	for (unsigned int w = 0; w < map_w; ++w)
	{
		for (unsigned int h = 0; h < map_h; ++h)
		{
			// 满足该条件、设置障碍物 data等于 1 25或者100
			if (
				(current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w] == 100) ||
				(current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w] == 25) ||
				(current_costmap_ptr_->data[h * current_costmap_ptr_->info.width + w] == 1))
			{
				// double temp = current_costmap_ptr_->data[w * current_costmap_ptr_->info.width + h];
				kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
			}
		}
	}

	if(current_vehicle_state_ptr_ == nullptr)
    {
        //LOG(ERROR)<<"no starting point"<<std::endl;
		return false;
    }

    if(goal_vehicle_state_ptr_ == nullptr)
    {
        //LOG(ERROR)<<"no goal point"<<std::endl;
		return false;
    }
    
	// double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);

	// double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);

    double start_yaw = current_vehicle_state_ptr_->heading;

	double goal_yaw = goal_vehicle_state_ptr_->heading;

	// 初始与终止点的位置与航向(全局下的)
	// HybridAStarType::Vec3d start_state = HybridAStarType::Vec3d(
	// 	current_init_pose_ptr_->pose.pose.position.x,
	// 	current_init_pose_ptr_->pose.pose.position.y,
	// 	start_yaw);

    HybridAStarType::Vec3d start_state = HybridAStarType::Vec3d(
		current_vehicle_state_ptr_->x,
		current_vehicle_state_ptr_->y,
		current_vehicle_state_ptr_->heading);
	// std::cout<<"ins start point x:"<<start_state.x()<<",y:"<<start_state.y()<<",yaw:"<<start_yaw<<std::endl;
	// HybridAStarType::Vec3d goal_state = HybridAStarType::Vec3d(
	// 	current_goal_pose_ptr_->pose.position.x,
	// 	current_goal_pose_ptr_->pose.position.y,
	// 	goal_yaw);

    HybridAStarType::Vec3d goal_state = HybridAStarType::Vec3d(
		goal_vehicle_state_ptr_->x,
		goal_vehicle_state_ptr_->y,
		goal_vehicle_state_ptr_->heading);
	// 地图下的起始点与终止点
	HybridAStarType::Vec3d start_state_map = HybridAStarType::Vec3d(
		(0 - current_costmap_ptr_->info.origin.position.x) / map_resolution,
		(0 - current_costmap_ptr_->info.origin.position.y) / map_resolution,
		// (0 - current_costmap_ptr_->info.origin.position.x),
		// (0 - current_costmap_ptr_->info.origin.position.y) ,
		(Mod2Pi(start_yaw + 0.5 * M_PI)));

	// double test_resolution = 1;
	// std::cout<<"地图里起点x,y,yaw:("<<start_state_map.x()<<","<<start_state_map.y()<<","<<start_state_map.z()<<")"<<std::endl;
	
	double yaw_cha_gs = goal_yaw - start_yaw;
	HybridAStarType::Vec2d goal_grid = UtmToGrid(goal_state.x(), goal_state.y(), start_state.x(), start_state.y(), start_state_map.x(), start_state_map.y(), yaw_cha_gs, map_resolution);
	HybridAStarType::Vec3d goal_state_map = HybridAStarType::Vec3d(
		goal_grid.x(),
		goal_grid.y(),
		(Mod2Pi(goal_yaw + 0.5 * M_PI)));
	// std::cout<<"ins goal point x:"<<goal_state.x()<<",y:"<<goal_state.y()<<",yaw:"<<goal_yaw<<std::endl;
	// std::cout<<"地图里终点x,y,yaw:("<<goal_state_map.x()<<","<<goal_state_map.y()<<","<<goal_state_map.z()<<")"<<std::endl;

	if (kinodynamic_astar_searcher_ptr_->Search(start_state_map, goal_state_map))
	{
		
		// std::cout<<"搜索成功"<<std::endl;
		auto path = kinodynamic_astar_searcher_ptr_->GetPath();
		// 将path分段
		std::vector<HybridAStarType::VectorVec3d> path_split_result = path_split_3(path);
		//在栅格地图里的路径
		std::vector<HybridAStarType::VectorVec4d> path_split_result_os = path_split(path);
		//转到UTM
		planning_msgs::TrajectoryPointArray path_os = GetTraject(path_split_result_os[0], start_state, start_state_map);
		// 得到具有位置、曲率、曲率变化的路径
		caculateKappaos(1, path_os);
		caculateAccumulated_s_os(path_os);
		caculateDkappaos(path_os);
		// 在路径上附速度
		// planning_msgs::TrajectoryPointArray path_os_out;
		final_trajectory_ = Velocity_Profile_output_os(openspace_config, car_vel, path_os);
        ros::Time time_stamp(cur_time);
        final_trajectory_.header.stamp = time_stamp;
		// final_trajectory_.header.stamp = cur_time;
		// PublishTraject(path_os_out);
		// auto path_show = path_split_result[0];
		// // std::cout<<"分段成功"<<std::endl;
		// // 坐标变换到世界系
		// PublishPath(path, start_state, start_state_map);
		// PublishVehiclePath(start_state, start_state_map, path, openspace_config.car_length, openspace_config.car_width, 5u);
		//LOG(INFO)<<"vehicle_pos:("<<start_state.x()<<","<<start_state.y()<<","<<start_yaw<<")"<<std::endl
									//<<"goal_pos:("<<goal_state.x()<<","<<goal_state.y()<<","<<goal_yaw<<std::endl
									//<<"spilt num:"<<path_split_result_os.size();
		// std::cout<<"显示路径"<<std::endl;
		// PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
		// debug
		//        std::cout << "visited nodes: " << kinodynamic_astar_searcher_ptr_->GetVisitedNodesNumber() << std::endl;
	}
	else
	{
		//LOG(ERROR)<<"Planning fail"<<std::endl<<"vehicle_pos:("<<start_state.x()<<","<<start_state.y()<<","<<start_yaw<<")"<<std::endl
									//<<"goal_pos:("<<goal_state.x()<<","<<goal_state.y()<<","<<goal_yaw<<std::endl;
		return false;
	}
	kinodynamic_astar_searcher_ptr_->Reset();
    current_costmap_ptr_.reset();
    current_vehicle_state_ptr_.reset();
    goal_vehicle_state_ptr_.reset();
	return true;

}
void  OpenSpacePlanner::getTrajectory(planning_msgs::TrajectoryPointArray &trajectory_)
{
	trajectory_ = final_trajectory_;
	return;
}

planning_msgs::TrajectoryPointArray OpenSpacePlanner::GetTraject(const HybridAStarType::VectorVec4d &path1, HybridAStarType::Vec3d &start_state, HybridAStarType::Vec3d &start_state_map)
{
    planning_msgs::TrajectoryPointArray traject_in;
    HybridAStarType::VectorVec4d path;
    path = path1;

    std::vector<planning_msgs::TrajectoryPoint> tra_resample;
    for (unsigned int i = 0; i < path.size(); i++)
    {
        //终点在之前处理角度，这里不再处理
        planning_msgs::TrajectoryPoint point;
        HybridAStarType::Vec2d global_utm_;
        double x_grid = path[i].x();
        double y_grid = path[i].y();
        double x_utm = start_state.x();
        double y_utm = start_state.y();
        double start_yaw = start_state.z();
        double x_utm_in_grid = start_state_map.x();
        double y_utm_in_grid = start_state_map.y();
        double yaw_cha_utm = start_yaw;
        global_utm_ = GridToUtm(x_grid, y_grid, x_utm, y_utm, x_utm_in_grid, y_utm_in_grid, yaw_cha_utm, map_resolution);
        point.x = global_utm_.x();
        point.y = global_utm_.y();
        // point.theta = path[i].z() + start_yaw;
        point.theta = path[i].z();
        tra_resample.push_back(point);
    }

    resamplePoints(0.5, tra_resample);

    for (unsigned int i = 0; i < tra_resample.size(); i++)
    {
        planning_msgs::TrajectoryPoint point;
        point.x = tra_resample[i].x;
        point.y = tra_resample[i].y;
        point.theta = tra_resample[i].theta;
        traject_in.points.push_back(point);
    }

    if (path[0].w() == 1)
    {
        traject_in.is_forward_shift = true;
    }
    else
    {
        traject_in.is_forward_shift = false;
    }
    double path_length = 0;
    for (unsigned int i = 0; i < traject_in.points.size() - 1; i++)
    {
        path_length += distance2D(Point2D(traject_in.points[i].x, traject_in.points[i].y),
                                  Point2D(traject_in.points[i + 1].x, traject_in.points[i + 1].y));
    }
    // std::cout << "path_size:" << traject_in.points.size()<<"  path_length:" << path_length << std::endl;;
    return traject_in;
}

// 附加速度
planning_msgs::TrajectoryPointArray OpenSpacePlanner::Velocity_Profile_output_os(OpenSpace_config &vel_config, double car_vel,
                                                                                planning_msgs::TrajectoryPointArray &traject_os)
{

    // 倒车速度、最大加减速度、自车长、宽
    double reverse_speed = vel_config.reverse_speed;
    //   std::cout<<"速度:"<<reverse_speed<<std::endl;
    double reverse_brake = vel_config.reverse_brake;
    // double car_length = vel_config.car_length;
    // 安全距离
    double safe_reverse = vel_config.safe_reverse_dis;
    // 前进后退标志位
    bool is_foward_shift = traject_os.is_forward_shift;
    int path_size = traject_os.points.size();
    std::vector<Waypoint2D> vel_profile;
    vel_profile.resize(path_size);
    for (unsigned int i = 0; i < traject_os.points.size(); i++)
    {
        vel_profile[i].position.x = traject_os.points[i].x;
        vel_profile[i].position.y = traject_os.points[i].y;
    }

    ////前进后退切换
    if ((!is_foward_shift == prev_direction_) && !(car_vel == 0)) // 两次规划结果不同 并且速度不为0 则进行刹停
    {
        // std::cout<<"---changing-direction---"<<std::endl;
        vel_profile.resize(prev_profile_.size());
        vel_profile = prev_profile_;
        for (unsigned int i = 0; i < vel_profile.size(); i++)
        {
            vel_profile[i].acc = -2;
            if (i == 0)
            { // 考虑刹车进来
                vel_profile[i].vel = fabs(car_vel);
            }
            else
            {
                double temp = -2 * 2 * distance2D(vel_profile[i].position, vel_profile[i - 1].position) + vel_profile[i - 1].vel * vel_profile[i - 1].vel;
                if (temp >= 0)
                {
                    vel_profile[i].vel = std::sqrt(temp);
                    if (!is_foward_shift)
                    {
                        vel_profile[i].vel = -vel_profile[i].vel;
                    }
                }
                else
                {
                    vel_profile[i].vel = 0;
                    vel_profile[i].acc = 0;
                }
            }
        }
        if (!is_foward_shift)
        {
            vel_profile[0].vel = -vel_profile[0].vel;
        }
    }

    /// 没有切换，正常规划
    // 正向
    if (is_foward_shift == true)
    {
        vel_profile[0].vel = car_vel;
        vel_profile[0].acc = 0;
        // 先给末尾刹停的速度
        // 按1的加速度干到刹车速度，之后不给油门
        int index_os_stop = path_size;
        double dis_os_stop = 0;
        for (unsigned int i = vel_profile.size() - 1; i > 1; i--)
        {
            dis_os_stop += distance2D(vel_profile[i].position, vel_profile[i - 1].position);
            if (dis_os_stop >= safe_reverse)
            {
                std::cout<<"距离路径末端距离:"<<dis_os_stop<<std::endl;
                index_os_stop = i;
                for (unsigned int j = index_os_stop; j < vel_profile.size(); j++)
                {
                    vel_profile[j].vel = 0;
                    vel_profile[j].acc = 0;
                }
                break;
            }
            else
            {
                vel_profile[i].vel = 0;
                vel_profile[i].acc = 0;
            }
        }

        if (index_os_stop >= 4)
        {
            for (int i = 1; i < (index_os_stop /2); i++)
            {

                vel_profile[i].vel =  reverse_speed;
                vel_profile[i].acc = 0;
            }
            for (int i = (index_os_stop /2); i < index_os_stop; i++)
            {
                vel_profile[i].vel = reverse_speed / 2;
                vel_profile[i].acc = 0;
            }
        }
    }
    // 倒车
    // 第一个点加速度均0
    if (is_foward_shift == false)
    {
        vel_profile[0].vel = car_vel;
        vel_profile[0].acc = 0;
        // 先给末尾刹停的速度
        // 按1的加速度干到刹车速度，之后不给油门
        int index_reverse_stop = path_size;
        double dis_reverse_stop = 0;
        for (unsigned int i = (vel_profile.size() - 1); i > 1; i--)
        {
            dis_reverse_stop += distance2D(vel_profile[i].position, vel_profile[i - 1].position);
            if (dis_reverse_stop >= safe_reverse)
            {
                // std::cout<<"距离路径末端距离:"<<dis_reverse_stop<<std::endl;
                index_reverse_stop = i;
                for (unsigned int i = index_reverse_stop; i < vel_profile.size(); i++)
                {
                    vel_profile[i].vel = 0;
                    vel_profile[i].acc = 0;
                }
                break;
            }
            else
            {
                vel_profile[i].vel = 0;
                vel_profile[i].acc = 0;
            }
        }
        // 路比较长时
        if (index_reverse_stop >= 4)
        {
            for (int i = 1; i < (index_reverse_stop /2); i++)
            {
                    vel_profile[i].vel = -1 *reverse_speed;
                    vel_profile[i].acc = 0;
            }

            for (int i = (index_reverse_stop /2); i < index_reverse_stop; i++)
            {
                vel_profile[i].vel = -1*reverse_speed / 2;
                vel_profile[i].acc = 0;
            }

        }
        // 路很短时
    }

    std::vector<double> relat_time;
    // 计算时间
    if(is_foward_shift == true)
    {
    relat_time.clear();
    relat_time.push_back(0.0); // 先push进去0,作起始时间
    double t_re = 0;
    double t = 0;
    for (int i = 0; i < path_size - 1; i++)
    {
        double v0 = vel_profile[i].vel;
        double v1 = vel_profile[i + 1].vel;
        double a0 = vel_profile[i].acc;
        double a1 = vel_profile[i + 1].acc;
        if (v0 <= 0.02) // 速度低，原地没动
        {
            t = 0;
        }
        else if (!(v1 == 0))
        {
            double s = distance2D(vel_profile[i + 1].position, vel_profile[i].position);
            t = s / fabs(v1);
        }
        t_re += t;
        //std::cout<<"前进时间："<<t_re<<std::endl;
        //LOG(INFO)<<"当前规划速度:"<<v0<<","<<" 下一点规划速度:"<<v1<<std::endl;
        relat_time.push_back(t_re);
    }
    relat_time[relat_time.size() - 1] = t_re;
    }

    if (is_foward_shift == false)
    {
        relat_time.clear();
        relat_time.push_back(0.0); // 先push进去0,作起始时间
        double t_re = 0;
        double t = 0;
        for (int i = 0; i < path_size - 1; i++)
        {
            double v0 = vel_profile[i].vel;
            double v1 = vel_profile[i + 1].vel;
            double a0 = vel_profile[i].acc;
            double a1 = vel_profile[i + 1].acc;
            if (fabs(v0) <= 0.02) // 速度低，原地没动
            {
                t = 0;
            }
            else if (!(fabs(v1) == 0))
            {
                double s = distance2D(vel_profile[i + 1].position, vel_profile[i].position);
                t = s / fabs(v1);
            }
            t_re += t;
            relat_time.push_back(t_re);
        //std::cout<<"前进时间："<<t_re<<std::endl;
        // std::cout<<"当前速度，加速度:"<<v0<<","<<a0<<std::endl;
        // std::cout<<" 下一点速度，加速度:"<<v1<<","<<a1<<std::endl;
        }
        relat_time.push_back(t_re);

    }

    // 将输出转为TrajectoryPointArray返回
    for (int i = 0; i < path_size; i++)
    {
        traject_os.points[i].v = vel_profile[i].vel;
        traject_os.points[i].a = vel_profile[i].acc;
        traject_os.points[i].relative_time = relat_time[i];
    }

    prev_direction_ = is_foward_shift;
    prev_profile_ = vel_profile;
    return traject_os;
}


