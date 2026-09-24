#include "route/route.h"
#include <chrono>

namespace {

struct RgbColor {
	double r;
	double g;
	double b;
};

bool isVehicleTwoNamespace(const ros::NodeHandle& nh)
{
	const std::string ns = nh.getNamespace();
	return ns.find("vehicle_2") != std::string::npos;
}

RgbColor currentRouteColor(const ros::NodeHandle& nh)
{
	return isVehicleTwoNamespace(nh) ? RgbColor{1.0, 0.18, 0.12} : RgbColor{0.05, 0.35, 1.0};
}

RgbColor pendingRouteColor(const ros::NodeHandle& nh)
{
	return isVehicleTwoNamespace(nh) ? RgbColor{1.0, 0.62, 0.45} : RgbColor{0.42, 0.72, 1.0};
}

RgbColor completedRouteColor()
{
	return RgbColor{0.55, 0.55, 0.55};
}

}  // namespace


void Route::displayOsm()
{
	visualization_msgs::MarkerArray markerArray;
	int id = 1;
	DisplayConfig cfg;
	for(const auto &it : file->ways){
		auto nodes = it.second.nodes;
		std::vector<UtmPoint> points;
		for(const auto &node : nodes){
			points.push_back(node->point);
		}
		cfg.id = id;
		cfg.ns = std::string("/Osm/way");
		cfg.r = 50/255.0;cfg.g = 255/255.0;cfg.b = 250/255.0;cfg.a = 0.5;
		cfg.scale_x = 3.0;cfg.scale_y = 0.0;cfg.scale_z = 0.0;
		
		markerArray.markers.push_back(DisPlay::lineMarker(points,cfg));
		id++;
		cfg.id = id;
		cfg.ns = std::string("/Osm/pointsNormal");
		cfg.r = 255/255.0;cfg.g = 100/255.0;cfg.b = 190/255.0;cfg.a = 1.0;
		cfg.scale_x = 3.0;cfg.scale_y = 3.0;cfg.scale_z = 3.0;
		markerArray.markers.push_back(DisPlay::sphereListMarker(points,cfg));
		id++;
	}
    
	std::vector<UtmPoint> points;
	std::vector<UtmPoint> points_cross;
	for(const auto &it : topoGraph_.topoGraphNodes){
		UtmPoint point; 
		point.x = it.second.theNode->node_x;
		point.y = it.second.theNode->node_y;
		if (it.second.theNode->node_crossing){
			points_cross.push_back(point);
			continue;
		}
		points.push_back(point);
	}
	//std::cout<<"points:"<<points.size()<<std::endl;
	//std::cout<<"points_cross:"<<points_cross.size()<<std::endl;

	for (const auto &p: points){
		 cfg.id = id;
	     cfg.ns = std::string("/Osm/points");
	     cfg.r = 110/255.0;cfg.g = 195/255.0;cfg.b = 50/255.0;cfg.a = 1.0;
		 cfg.scale_x = 4.0;cfg.scale_y = 4.0;cfg.scale_z = 5.0;
	     markerArray.markers.push_back(DisPlay::cylinderMarker(p,cfg));
	     id++;
	}

	cfg.id = id;
	cfg.ns = std::string("/Osm/points_cross");
	cfg.r = 255/255.0;cfg.g = 215/255.0;cfg.b = 45/255.0;cfg.a = 1.0;
	cfg.scale_x = 5.0;cfg.scale_y = 5.0;cfg.scale_z = 5.0;

	markerArray.markers.push_back(DisPlay::cubesListMarker(points_cross,cfg));
    
	display_pub_.publish(markerArray);
}

void Route::publishRouteSegmentMarkers()
{
	visualization_msgs::MarkerArray markers;

	for (int i = 1; i <= lastRoutesNum; ++i) {
		markers.markers.push_back(DisPlay::deleteMarker("/task_route/current", i));
		markers.markers.push_back(DisPlay::deleteMarker("/task_route/pending", i));
		markers.markers.push_back(DisPlay::deleteMarker("/task_route/completed", i));
	}

	const int route_count = static_cast<int>(pubUtmResults.size());
	if (route_count == 0) {
		display_pub_.publish(markers);
		return;
	}

	int current_index = pubIndex;
	if (current_index < 0) {
		current_index = 0;
	}
	if (current_index > route_count) {
		current_index = route_count;
	}

	for (int i = 0; i < route_count; ++i) {
		DisplayConfig cfg;
		cfg.id = i + 1;
		cfg.scale_x = (i == current_index) ? 7.0 : 4.0;
		cfg.scale_y = 0.0;
		cfg.scale_z = 0.0;
		cfg.a = (i == current_index) ? 1.0 : 0.45;

		RgbColor color = task_route_color_override_
			? RgbColor{task_route_color_r_, task_route_color_g_, task_route_color_b_}
			: pendingRouteColor(nh_);
		cfg.ns = std::string("/task_route/pending");
		if (i < current_index) {
			color = task_route_color_override_
				? RgbColor{task_route_color_r_, task_route_color_g_, task_route_color_b_}
				: completedRouteColor();
			cfg.ns = std::string("/task_route/completed");
			cfg.a = 0.35;
		} else if (i == current_index) {
			color = task_route_color_override_
				? RgbColor{task_route_color_r_, task_route_color_g_, task_route_color_b_}
				: currentRouteColor(nh_);
			cfg.ns = std::string("/task_route/current");
		}

		cfg.r = color.r;
		cfg.g = color.g;
		cfg.b = color.b;
		markers.markers.push_back(DisPlay::lineMarker(pubUtmResults[i], cfg));
	}

	display_pub_.publish(markers);
}

bool Route::hasActiveRouteSegment() const
{
	return pubIndex >= 0 && pubIndex < static_cast<int>(pubUtmResults.size());
}

double Route::distanceToCurrentRouteEnd() const
{
	if (!hasActiveRouteSegment() || pubUtmResults[pubIndex].empty()) {
		return std::numeric_limits<double>::infinity();
	}
	return amathutils::distance2D(vehicleInfo.utmPoint, pubUtmResults[pubIndex].back());
}

bool Route::isRouteAdvanceAutoReady() const
{
	return distanceToCurrentRouteEnd() <= routeAdvanceDistanceThreshold;
}

void Route::publishCurrentRouteSegment()
{
	if (!hasActiveRouteSegment()) {
		return;
	}

	planning_msgs::TrajectoryPointArray trajectory;
	for (const auto &p : pubUtmResults[pubIndex]) {
		planning_msgs::TrajectoryPoint point;
		point.x = p.x;
		point.y = p.y;
		trajectory.points.push_back(point);
	}
	route_pub_.publish(trajectory);
	publishRouteSegmentMarkers();
	lastRouteStatusIndex = pubIndex;
}

void Route::tryAdvanceRouteSegment()
{
	if (!hasActiveRouteSegment()) {
		manualRouteAdvanceConfirmed = false;
		return;
	}

	const double distance_to_end = distanceToCurrentRouteEnd();
	if (distance_to_end > routeAdvanceDistanceThreshold || !manualRouteAdvanceConfirmed) {
		return;
	}

	ROS_INFO_STREAM("[RouteAdvance] double confirmation accepted. current_segment="
	                << pubIndex + 1 << ", distance_to_end=" << distance_to_end << " m");
	manualRouteAdvanceConfirmed = false;
	pubIndex++;

	if (pubIndex >= static_cast<int>(pubUtmResults.size())) {
		publishRouteSegmentMarkers();
		pubIndex = -1;
		std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
		lastRouteStatusIndex = -100;
		ROS_INFO("[RouteAdvance] all route segments completed.");
		return;
	}

	publishCurrentRouteSegment();
}


ErrCode Route::FindPath(TopoGraph &topoGraph,std::vector<UtmPoint> &singleUtmResult)
{
	topoGraph.InitGraph();
	auto s_id = topoGraph.sId;
	auto e_id = topoGraph.eId;
	
	//auto sAdjacentNodes_rmv = topoGraph.topoGraphNodes[topoGraph.sId].adjacentNodes;
	//for(const auto &iter : sAdjacentNodes_rmv)
		//std::cout << "[FindPath]  移除后的近邻点ID: " << iter->node_id<< std::endl;
	
	auto errCode = topoGraph.FindPath(s_id,e_id);
	//std::cout<<"单段规划结果：" << errCode <<std::endl;
	topoGraph.PrintGraph(singleUtmResult,e_id);
	/**
    std::cout << "[FindPath] 当前singleUtmResult的nodeId链路：" << std::endl;
    std::string id_chain;
    for (size_t point_idx = 0; point_idx < singleUtmResult.size(); ++point_idx) {
        const auto& utm_point = singleUtmResult[point_idx];
        auto Id = file->findNearestNode(utm_point.x, utm_point.y);
        auto nodeId = Id.second;
        id_chain += std::to_string(nodeId) + "->";
    }
    std::cout << id_chain << std::endl;
    std::cout << "=====================================" << std::endl;
	**/
	return errCode;
}

void Route::addObs(TopoGraph &tmpGraph){
    for (const auto &obsInfo : obsInfoVec){
        auto Id = tmpGraph.file_->findNearestNode(obsInfo.x, obsInfo.y);
        auto obsNodeId = Id.second;
        auto obsNode = tmpGraph.file_->nodes[obsNodeId].point;
        for(const auto &it : tmpGraph.osmSE2Topo){
            for(const auto &iter : it.first){
                double d = amathutils::distance2D(iter->point.x, iter->point.y, obsNode.x, obsNode.y);
                if(d < 1e-4){
                    auto s_id = it.second.sId;
                    auto e_id = it.second.eId;
                    auto enode = tmpGraph.GetNode(e_id);
                    tmpGraph.topoGraphNodes[s_id].RmvAdjacentNode(enode);
                    auto snode = tmpGraph.GetNode(s_id);
                    tmpGraph.topoGraphNodes[e_id].RmvAdjacentNode(snode);
                }
            }
        }
    }
}

 
void Route::callBackCurrentPose(const localization_msgs::Localization::ConstPtr &msg)
{
  	double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);
  	vehicleInfo.orient_x = cos(yaw);
  	vehicleInfo.orient_y = sin(yaw);
	vehicleInfo.utmPoint.x = msg->location.pose.pose.position.x;
	vehicleInfo.utmPoint.y = msg->location.pose.pose.position.y;

	if (pubIndex < 0 && 0 == pubUtmResults.size())
		return;
	
	if (pubIndex < 0 && pubUtmResults.size()) {
	    pubIndex++;
		manualRouteAdvanceConfirmed = false;
		publishCurrentRouteSegment();
	    return;
	}
	
	/**   
	if (amathutils::distance2D(vehicleInfo.utmPoint,pubUtmResults[pubIndex].back())< 2.0
		&& fabs(vehicleInfo.speed ) <= 1e-1){ 
	    pubIndex++;
		planning_msgs::TrajectoryPointArray trajectory;
		for (const auto &p:pubUtmResults[pubIndex]){
			planning_msgs::TrajectoryPoint point;
			point.x = p.x;
		    point.y = p.y;
			trajectory.points.push_back(point);
		}
		route_pub_.publish(trajectory);
	}
	**/
	tryAdvanceRouteSegment();
	
}

void Route::callbackChassis(const driver_msgs::ChassisReport::ConstPtr &msg)
{
	vehicleInfo.speed = (7 == msg->gear_location)?(-msg->current_velocity):msg->current_velocity;

	//vehicleInfo.speed = msg->current_velocity;
	
}

void Route::callbackNextRouteSegment(const std_msgs::Empty::ConstPtr &msg)
{
	(void)msg;
	if (!hasActiveRouteSegment()) {
		manualRouteAdvanceConfirmed = false;
		ROS_WARN("[RouteAdvance] manual confirmation ignored: no active route segment.");
		return;
	}

	const double distance_to_end = distanceToCurrentRouteEnd();
	if (distance_to_end > routeAdvanceDistanceThreshold) {
		manualRouteAdvanceConfirmed = false;
		ROS_WARN_STREAM("[RouteAdvance] manual confirmation ignored: distance_to_end="
		                << distance_to_end << " m, threshold="
		                << routeAdvanceDistanceThreshold << " m");
		return;
	}

	manualRouteAdvanceConfirmed = true;
	ROS_INFO_STREAM("[RouteAdvance] manual confirmation accepted. distance_to_end="
	                << distance_to_end << " m");
	tryAdvanceRouteSegment();
}

void Route::callBackinitialPose(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	
	UtmPoint point;
	point.x = msg->pose.position.x;
	point.y = msg->pose.position.y;
	if(s_flag){
		double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->pose));
		vehicleInfo.initial_orient_x = cos(yaw);
  		vehicleInfo.initial_orient_y = sin(yaw);
	}
	multiPoints.push_back(point);
	if (multiPoints.size() == 1){
		const auto& q = msg->pose.orientation;
		std::cout << "[callBackinitialPose] 第一个规划点：" << std::endl;
		std::cout << "  四元数：x = " << std::fixed << std::setprecision(9) << q.x 
			<< ", y = " << std::fixed << std::setprecision(9) << q.y 
			<< ", z = " << std::fixed << std::setprecision(9) << q.z 
			<< ", w = " << std::fixed << std::setprecision(9) << q.w << std::endl;
	}
	visualization_msgs::MarkerArray markerArrayDel;
	if (1 == multiPoints.size()){
		for (int i = 1; i<=  lastPointsNum; i++)
		    markerArrayDel.markers.push_back(DisPlay::deleteMarker("/routePoints",i));
		for (int i = 1; i<=  lastRoutesNum; i++)
		    markerArrayDel.markers.push_back(DisPlay::deleteMarker("/GlobalPlanner",i));
		for (int i = 1; i<=  lastRoutesNum; i++)
		    markerArrayDel.markers.push_back(DisPlay::deleteMarker("/clicked_point",i));
	}
	display_pub_.publish(markerArrayDel);

	visualization_msgs::MarkerArray markerArray;
    DisplayConfig cfg;
	cfg.scale_x  = 5.0;
	cfg.scale_y  = 1.0;
	cfg.scale_z  = 1.0;
	cfg.quaternion = msg->pose.orientation;
	cfg.position = msg->pose.position;
	cfg.id = multiPoints.size();
	cfg.ns = std::string("/routePoints");
	markerArray.markers.push_back(DisPlay::arrowMarkerMethod2(cfg));
	display_pub_.publish(markerArray);
	
	s_flag = false;
}

void Route::callBackgoal(const geometry_msgs::PointStamped::ConstPtr msg)
{
	s_flag = true;
    
	pubIndex = -1;
	manualRouteAdvanceConfirmed = false;
	std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
	std::vector<ResultPoint>().swap(reservePlannedResults);
	
	UtmPoint point;
	point.x = msg->point.x;
	point.y = msg->point.y;

	multiPoints.push_back(point);
    for (size_t i = 0; i < multiPoints.size(); ++i){
        std::cout << "[callBackgoal] multiPoints[" << i << "] : "
                  << "x = " << multiPoints[i].x 
                  << ", y = " << multiPoints[i].y << std::endl;
    }
	lastPointsNum = multiPoints.size();
	visualization_msgs::MarkerArray markerArray;
	DisplayConfig cfg;
	cfg.scale_x  = 5.0;
	cfg.scale_y  = 1.0;
	cfg.scale_z  = 1.0;
	cfg.quaternion.w = 1.0;
	cfg.position = msg->point;
	cfg.id = multiPoints.size();
	cfg.ns= std::string("/routePoints");
	markerArray.markers.push_back(DisPlay::arrowMarkerMethod2(cfg));
	display_pub_.publish(markerArray);
	
	if(checkMultiPointsOutOfBoundary()){
		std::cout << "[callBackgoal] ====== 警告：超出地图边界，向云控请求地图 ======" << std::endl;
		std_msgs::UInt8 map_request_msg;
	    map_request_msg.data = 1;
	    map_request_pub_.publish(map_request_msg);
		return;
	}
	std::vector<std::vector<UtmPoint>> multiUtmResults;
	double plannedLength;
	int turnCount = 0;
	auto result = multiPointsPlan(multiPoints,multiUtmResults,plannedLength,MULTI_PLAN,turnCount);
	if(!result)
		std::cout << "[callBackgoal] ====== 警告：rviz仿真多目标规划失败 ======" << std::endl;
	pubUtmResults = multiUtmResults;
	pubUtmResults_ = pubUtmResults;
	std::cout << "[callBackgoal] ====== rviz仿真多目标规划结果 ======" << std::endl;
	printResultsNodeId();
	lastRoutesNum = multiUtmResults.size();
	visualization_msgs::MarkerArray markerArray_;
	std::vector<std::vector<float>> colors;
	colors.push_back(std::vector<float>{255/255.0, 100/255.0, 190/255.0});
	colors.push_back(std::vector<float>{252/255.0, 223/255.0, 53/255.0});
	colors.push_back(std::vector<float>{204/255.0, 108/255.0, 255/255.0});
	colors.push_back(std::vector<float>{135/255.0, 204/255.0, 98/255.0});
	int c_index = 0;
	int id_ = 1;
	for(int i = 0;i < multiUtmResults.size();i++){
		DisplayConfig cfg;
		cfg.id = id_;
		cfg.ns = std::string("/clicked_point");
		int index =  c_index % colors.size();
		cfg.r = colors[index][0];cfg.g = colors[index][1];cfg.b = colors[index][2];cfg.a = 1.0;
		cfg.scale_x = 4.0;cfg.scale_y = 0.0;cfg.scale_z = 0.0;
		markerArray_.markers.push_back(DisPlay::lineMarker(multiUtmResults[i],cfg));
		c_index++;
		id_++;
	}
	std::vector<UtmPoint>().swap(lastMultiPoints);
	lastMultiPoints = multiPoints;
	std::vector<UtmPoint>().swap(multiPoints);
	display_pub_.publish(markerArray_);
}

bool Route::checkMultiPointsOutOfBoundary() {
    for (const auto& point : multiPoints) {
        bool outOfX = (point.x < map_boundary.min_x - 1e-6) || (point.x > map_boundary.max_x + 1e-6);
        bool outOfY = (point.y < map_boundary.min_y - 1e-6) || (point.y > map_boundary.max_y + 1e-6);
        if (outOfX || outOfY)
            return true;
    }
    return false;
}

void Route::callBackInitPoint(const route_msgs::InitPoint::ConstPtr msg)
{
	std::cout << "[callBackInitPoint] ====== 收到规划起点 ======" << std::endl;
	init_point.point.x = msg->pose.position.x;
    init_point.point.y = msg->pose.position.y;
    init_point.orientation = msg->pose.orientation;
}

void Route::callBackMultiPointPlanning(const route_msgs::MultiPoint::ConstPtr msg)
{
	std::cout << "[MultiPointPlanning] ====== 进入cloud多目标点规划回调函数 ======" << std::endl;
	pubIndex = -1;
	manualRouteAdvanceConfirmed = false;
	std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
	std::vector<UtmPoint>().swap(multiPoints);
	std::vector<UtmPoint>().swap(lastMultiPoints);
	if (msg->poses.empty()){
		std::cout << "[MultiPointPlanning] ====== 警告：收到的规划点集为空，跳过规划 ======" << std::endl;
		return;
	}
	
    visualization_msgs::MarkerArray markerArrayDel;
    for (int i = 1; i <= lastRoutesNum; i++)
        markerArrayDel.markers.push_back(DisPlay::deleteMarker("/clicked_point", i));
	for (int i = 1; i<=  lastRoutesNum; i++)
		    markerArrayDel.markers.push_back(DisPlay::deleteMarker("/GlobalPlanner",i));
	for (int i = 1; i <= lastPointsNum; i++)
        markerArrayDel.markers.push_back(DisPlay::deleteMarker("/routePoints", i));
    display_pub_.publish(markerArrayDel);
	
	std::vector<geometry_msgs::Quaternion> pointQuaternions;
	multiPoints.push_back(init_point.point);
	pointQuaternions.push_back(init_point.orientation);
	
	for (size_t i = 0; i < msg->poses.size(); ++i){
		std::cout << "[MultiPointPlanning] 规划点" << i + 1 << " 坐标：x = " << msg->poses[i].position.x << ", y = " << msg->poses[i].position.y << std::endl;
        UtmPoint point;
        point.x = msg->poses[i].position.x;
        point.y = msg->poses[i].position.y;
        multiPoints.push_back(point);
		pointQuaternions.push_back(msg->poses[i].orientation);
	}
	if(checkMultiPointsOutOfBoundary()){
		std::cout << "[MultiPointPlanning] ====== 警告：超出地图边界，向云控请求地图 ======" << std::endl;
		std_msgs::UInt8 map_request_msg;
        map_request_msg.data = 1;
        map_request_pub_.publish(map_request_msg);
		return;
	}
    const auto& start_pose = msg->poses[0];
   // double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(start_pose));
    geometry_msgs::Pose pose_tmp;
	pose_tmp.orientation = init_point.orientation;
    double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(pose_tmp));
    vehicleInfo.initial_orient_x = cos(yaw);
    vehicleInfo.initial_orient_y = sin(yaw);

    std::vector<std::vector<UtmPoint>> multiUtmResults;
    double plannedLength = 0.0;
    int turnCount = 0;
    auto result = multiPointsPlan(multiPoints, multiUtmResults, plannedLength, MULTI_PLAN, turnCount);
	if(!result) {
		std::cout << "[MultiPointPlanning] ====== 警告：cloud多目标规划失败 ======" << std::endl;
		return;
	}
	pubUtmResults = multiUtmResults;
	pubUtmResults_ = pubUtmResults;
	std::cout << "[MultiPointPlanning] ====== cloud多目标规划结果 ======" << std::endl;
	printResultsNodeId();
    lastMultiPoints = multiPoints;
    lastPointsNum = multiPoints.size(); 
    lastRoutesNum = multiUtmResults.size();
	planning_msgs::TrajectoryPointArray trajectory;
	for (const auto& current_path : pubUtmResults){
		for (const auto& utm_point : current_path){
            planning_msgs::TrajectoryPoint point;
            point.x = utm_point.x;
            point.y = utm_point.y;
            trajectory.points.push_back(point);
        }
    }
    cloud_display_pub_.publish(trajectory);
	std::cout << "[MultiPointPlanning] ====== 轨迹发布完成，发布轨迹点总数：" << trajectory.points.size() << " 个 ======" << std::endl;

	visualization_msgs::MarkerArray markerArrayPoints;
    for (size_t i = 0; i < multiPoints.size(); ++i){
        DisplayConfig cfg;
        cfg.scale_x  = 5.0;
        cfg.scale_y  = 1.0;
        cfg.scale_z  = 1.0;
        cfg.quaternion = pointQuaternions[i]; 
        cfg.position.x = multiPoints[i].x;
        cfg.position.y = multiPoints[i].y;
        cfg.position.z = 0.0;
        cfg.id = i + 1;
        cfg.ns = std::string("/routePoints");
        markerArrayPoints.markers.push_back(DisPlay::arrowMarkerMethod2(cfg));
    }
    display_pub_.publish(markerArrayPoints);
	
	visualization_msgs::MarkerArray markerArrayRoutes;
    std::vector<std::vector<float>> colors = {
        {255/255.0, 100/255.0, 190/255.0},
        {252/255.0, 223/255.0, 53/255.0},
        {204/255.0, 108/255.0, 255/255.0},
        {135/255.0, 204/255.0, 98/255.0}
    };
    int c_index = 0;
    int id_ = 1;
    for (const auto& path : multiUtmResults)
    {
        DisplayConfig cfg;
        cfg.id = id_++;
        cfg.ns = std::string("/clicked_point");
        int color_idx = c_index % colors.size();
        cfg.r = colors[color_idx][0];
        cfg.g = colors[color_idx][1];
        cfg.b = colors[color_idx][2];
        cfg.a = 1.0;
        cfg.scale_x = 4.0;
        cfg.scale_y = 0.0;
        cfg.scale_z = 0.0;
        markerArrayRoutes.markers.push_back(DisPlay::lineMarker(path, cfg));
        c_index++;
    }
    display_pub_.publish(markerArrayRoutes);
	publishRouteSegmentMarkers();
	lastRouteStatusIndex = pubIndex;
	std::vector<UtmPoint>().swap(multiPoints);
	std_msgs::Float64 completion_msg;
	completion_msg.data = ros::WallTime::now().toSec();
	global_route_planning_done_pub_.publish(completion_msg);
}

void Route::callbackReplan(const route_msgs::Replan::ConstPtr &msg)
{
    std::cout << "[Replan] ====== 进入重规划回调函数 ======" << std::endl;
    if (msg->points.empty()){
        std::cout << "[Replan] ====== 警告：无障碍物坐标 ======" << std::endl;
        return;
    }
    for (size_t i = 0; i < msg->points.size(); ++i)
    {
        const auto &curr_point = msg->points[i];
        std::cout << "[Replan] 障碍物点 " << i+1 << " 坐标：x = " << curr_point.x 
                  << ", y = " << curr_point.y << ", z = " << curr_point.z << std::endl;
    }
	visualization_msgs::MarkerArray markerArrayDel;
	for (int i = 1; i <= lastRoutesNum; i++)
        markerArrayDel.markers.push_back(DisPlay::deleteMarker("/clicked_point", i));
	for (int i = 1; i<=  lastPointsNum; i++)
		markerArrayDel.markers.push_back(DisPlay::deleteMarker("/routePoints",i));
	for (int i = 1; i<=  lastRoutesNum; i++)
		markerArrayDel.markers.push_back(DisPlay::deleteMarker("/GlobalPlanner",i));
	display_pub_.publish(markerArrayDel);

	std::vector<UtmPoint>().swap(obsInfoVec);
    replan_topoGraph_ = topoGraph_;
    for (size_t i = 0; i < msg->points.size(); ++i){
        const auto &curr_point = msg->points[i];
        UtmPoint obs;
        obs.x = curr_point.x;
        obs.y = curr_point.y;
        obs.z = curr_point.z;
        obsInfoVec.push_back(obs);
        auto obsId = replan_topoGraph_.file_->findNearestNode(obs.x, obs.y);
		auto obsPointId = obsId.second;
		auto obsWayId = obsId.first;
		auto obsPointFind = replan_topoGraph_.topoGraphNodes.find(obsPointId);
		//std::cout << "[Replan] obsPointId: " << obsPointId << std::endl;
		//std::cout << "[Replan] obsWayId: " << obsWayId << std::endl;
		if(obsPointFind == replan_topoGraph_.topoGraphNodes.end()){
			//std::cout << "[Replan] obsPointFind: 未在Topo找到节点，打断一条Way" << std::endl;
	        auto s_id = replan_topoGraph_.osm2Topo[obsWayId].sId;
	        auto e_id = replan_topoGraph_.osm2Topo[obsWayId].eId;
			//std::cout << "[Replan] obsWaySId: " << s_id << std::endl;
			//std::cout << "[Replan] obsWayEId: " << e_id << std::endl;
	        auto enode = replan_topoGraph_.GetNode(e_id);
	        replan_topoGraph_.topoGraphNodes[s_id].RmvAdjacentNode(enode);
	        auto snode = replan_topoGraph_.GetNode(s_id);
	        replan_topoGraph_.topoGraphNodes[e_id].RmvAdjacentNode(snode);
		}
		else{
			//std::cout << "[Replan] obsPointFind: 在Topo找到节点，打断多条way (节点ID: " << obsPointFind->first << ")" << std::endl;
			auto obsNode = replan_topoGraph_.topoGraphNodes[obsPointId].theNode;
			auto adjacentNodes = replan_topoGraph_.topoGraphNodes[obsPointId].adjacentNodes;
			for (auto& adjNode : adjacentNodes) {
			    replan_topoGraph_.topoGraphNodes[obsPointId].RmvAdjacentNode(adjNode);
			    int adjNodeId = adjNode->node_id;
				//std::cout << "[Replan] 移除邻接节点 (节点ID: " << adjNodeId << ")" << std::endl;
			    replan_topoGraph_.topoGraphNodes[adjNodeId].RmvAdjacentNode(obsNode);
			}
		}
    }
	/**
	obsInfo.x = msg->x;
	obsInfo.y = msg->y;
	obsInfo.z = msg->z;
	replan_topoGraph_ = topoGraph_;
	auto obsId = replan_topoGraph_.file_->findNearestNode(msg->x, msg->y);
	auto obsWayId = obsId.first;
	auto s_id = replan_topoGraph_.osm2Topo[obsWayId].sId;
	auto e_id = replan_topoGraph_.osm2Topo[obsWayId].eId;
	auto enode = replan_topoGraph_.GetNode(e_id);
	replan_topoGraph_.topoGraphNodes[s_id].RmvAdjacentNode(enode);
	auto snode = replan_topoGraph_.GetNode(s_id);
	replan_topoGraph_.topoGraphNodes[e_id].RmvAdjacentNode(snode);
	**/
	
	if (pubIndex == -1){
        reMultiPoints.push_back(lastMultiPoints.back());
		//std::cout << "[Replan] pubIndex = -1 " << std::endl;
	}

	else
		reMultiPoints.insert(reMultiPoints.end(),lastMultiPoints.begin()+pubIndex+1,lastMultiPoints.end());

	for (size_t i = 0; i < reMultiPoints.size(); ++i) {
		std::cout << "[Replan] 重规划点" << i + 1 << " 坐标：x = " 
				  << reMultiPoints[i].x << ", y = " << reMultiPoints[i].y << std::endl;
	}

	UtmPoint target_point;
	auto target_point_ = reMultiPoints[0];
	auto targetId = file->findNearestNode(target_point_.x, target_point_.y);
	auto targetNodeId = targetId.second;
	target_point.x = file->nodes[targetNodeId].point.x;
	target_point.y = file->nodes[targetNodeId].point.y;


	
	//std::cout<<"出现阻断路时剩余待规划集合长度："<<reMultiPoints.size()<<std::endl;
	
	auto vehicleId = replan_topoGraph_.file_->findNearestNode(vehicleInfo.utmPoint.x, vehicleInfo.utmPoint.y);
	auto vehicleNodeId = vehicleId.second;
	//std::cout << "[Replan] 车辆当前位置vehicleNodeId: " << vehicleNodeId << std::endl;
	auto vehicleNode = replan_topoGraph_.file_->nodes[vehicleNodeId].point;
	//std::cout << "[Replan] 车辆当前位置: x = " << vehicleNode.x 
          //<< ", y = " << vehicleNode.y 
          //<< ", z = " << vehicleNode.z << std::endl;
	

	/**
	vehicleInfo.initial_orient_x = vehicleInfo.orient_x;
	vehicleInfo.initial_orient_y = vehicleInfo.orient_y;
    std::vector<std::vector<UtmPoint>> pt_multiUtmResults;
    double pt_plannedLength = 0.0;
    int pt_turnCount = 0;
	auto pt_reMultiPoints = reMultiPoints;
	pt_reMultiPoints.insert(pt_reMultiPoints.begin(),vehicleNode);
    auto result = multiPointsPlan(pt_reMultiPoints, pt_multiUtmResults, pt_plannedLength, MULTI_REPLAN, pt_turnCount);
	if(result){
		std::cout << "[Replan] ====== 无需倒车，正向规划 ======" << std::endl;
		std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
   		pubUtmResults = pt_multiUtmResults;
		std::vector<UtmPoint>().swap(lastMultiPoints);
		lastMultiPoints = reMultiPoints;
		std::vector<UtmPoint>().swap(reMultiPoints);
		lastRoutesNum = pubUtmResults.size();
		visualization_msgs::MarkerArray markerArray_;
		std::vector<std::vector<float>> colors;
		colors.push_back(std::vector<float>{255/255.0, 100/255.0, 190/255.0});
		colors.push_back(std::vector<float>{252/255.0, 223/255.0, 53/255.0});
		colors.push_back(std::vector<float>{204/255.0, 108/255.0, 255/255.0});
		colors.push_back(std::vector<float>{135/255.0, 204/255.0, 98/255.0});
		
		int c_index = 0;
		int id_ = 1;
		for(int i = 0;i < pubUtmResults.size();i++){
			DisplayConfig cfg;
			cfg.id = id_;
			cfg.ns = std::string("/GlobalPlanner");
			int index =  c_index % colors.size();
			cfg.r = colors[index][0];cfg.g = colors[index][1];cfg.b = colors[index][2];cfg.a = 1.0;
			cfg.scale_x = 4.0;cfg.scale_y = 0.0;cfg.scale_z = 0.0;
			markerArray_.markers.push_back(DisPlay::lineMarker(pubUtmResults[i],cfg));
			c_index++;
			id_++;
		}
		display_pub_.publish(markerArray_);
		return;
	}
	**/
	
	std::map<int,UtmPoint> graph_cross;
	for(const auto &it : replan_topoGraph_.topoGraphNodes){
		if (it.second.theNode->node_crossing){
			UtmPoint point; 
			point.x = it.second.theNode->node_x;
			point.y = it.second.theNode->node_y;	
			graph_cross.emplace(it.second.theNode->node_id,point);
		}
	}
	std::map<double,std::pair<std::vector<UtmPoint>,int>> cross_choice;
	std::map<double,std::pair<std::vector<UtmPoint>,int>> cross_choice_inthreshold;
	std::map<double,std::pair<std::vector<UtmPoint>,int>> cross_choice_outthreshold;
	for(const auto &it : graph_cross){
		std::vector<UtmPoint> cur_cross;
		cur_cross.push_back(vehicleNode);
		cur_cross.push_back(it.second);
		double cur_cross_length = 0.0;
		int turnCount = 0;
		std::vector<std::vector<UtmPoint>> cur_cross_utmResult;
		vehicleInfo.initial_orient_x = - vehicleInfo.orient_x;
		vehicleInfo.initial_orient_y = - vehicleInfo.orient_y;
		if(multiPointsPlan(cur_cross,cur_cross_utmResult,cur_cross_length,MULTI_REPLAN,turnCount) && turnCount == 0){
			//std::cout << "[callbakReplan] 倒车路口长度:" << cur_cross_length<<std::endl;
			cross_choice.emplace(cur_cross_length,std::make_pair(cur_cross_utmResult[0],it.first));	
		}
	}
	
	//std::cout<<"可选择交叉路口的数目："<<cross_choice.size()<<std::endl;
	if(cross_choice.begin()->first + 500.0 > threshold)
		threshold = cross_choice.begin()->first + 500.0;
	
	for(const auto &it : cross_choice){
		//std::cout<<"交叉路口ID："<<it.second.second<<std::endl;
		//std::cout<<"当前位置到交叉路口距离："<<it.first<<std::endl;
	    if(it.first <= threshold)
			cross_choice_inthreshold.emplace(it);
		else
			cross_choice_outthreshold.emplace(it);
	}
	//std::cout<<"阈值："<<threshold<<std::endl;
	//std::cout<<"阈值范围内交叉路口的数目："<<cross_choice_inthreshold.size()<<std::endl;
	//std::cout<<"超出阈值的交叉路口的数目："<<cross_choice_outthreshold.size()<<std::endl;
	
	std::map<double,std::pair<std::vector<std::vector<UtmPoint>>,int>> inThresholdLengths;
	
	for(const auto &it : cross_choice_inthreshold){
		//std::cout<<"-----------------------------阈值范围内倒车交叉路口的ID："<<it.second.second<<std::endl;
		std::vector<std::pair<UtmPoint,std::vector<UtmPoint>>> backEnds;
		std::vector<UtmPoint> backReplannedResult;
		double backReplannedLength = it.first;
		auto crossingNodeId = it.second.second;
		auto crossingAdjacentNodes = replan_topoGraph_.topoGraphNodes[crossingNodeId].adjacentNodes;
		backReplannedResult = it.second.first;
		/**
		double x;
		double y;
		if(backReplannedResult.size() == 1){
			auto cross = backReplannedResult.back();
			bool found = false;
			for (const auto& seg : pubUtmResults_) {
			    for (size_t i = 0; i < seg.size()- 1; ++i) {
			        const auto& c = seg[i];
			        if (std::hypot(c.x - cross.x, c.y - cross.y) < 1e-6) {
			            x = seg[i+1].x;
			            y = seg[i+1].y;
			            found = true;
			            break;
			        }
			    }
			    if (found) break;
			}
		}
		else{
			x = backReplannedResult[backReplannedResult.size()-2].x;
			y = backReplannedResult[backReplannedResult.size()-2].y;
		}
		auto removeId = replan_topoGraph_.file_->findNearestNode(x,y);
		auto removeWayId = removeId.first;
		auto remove_s_id = replan_topoGraph_.osm2Topo[removeWayId].sId;
		auto remove_e_id = replan_topoGraph_.osm2Topo[removeWayId].eId;
		**/
		int remove_s_id = -1;
		int remove_e_id = -1;
		//if(backReplannedResult.size() == 1)
			//std::cout<<"[callbakReplan]当前车辆位置为交叉路口"<<std::endl;
		if(backReplannedResult.size() >= 2){
			//std::cout<<"[callbakReplan]当前车辆位置不是交叉路口"<<std::endl;
			double x = backReplannedResult[backReplannedResult.size()-2].x;
			double y = backReplannedResult[backReplannedResult.size()-2].y;
			auto removeId = replan_topoGraph_.file_->findNearestNode(x,y);
			auto removeWayId = removeId.first;
			//std::cout << "[Replan] removeWayId: " << removeWayId << std::endl;
			//std::cout << "[Replan] removeNodeId: " << removeId.second << std::endl;
			remove_s_id = replan_topoGraph_.osm2Topo[removeWayId].sId;
			remove_e_id = replan_topoGraph_.osm2Topo[removeWayId].eId;
		}
		for(const auto &iter : crossingAdjacentNodes){
			if(iter->node_id == remove_s_id || iter->node_id == remove_e_id)
				continue;
			WaySEId waySEId_;
			waySEId_.sId = crossingNodeId;
			waySEId_.eId = iter->node_id;
			std::vector<UtmPoint> backTenM;
			double backLength = 0.0;
			for(auto it : replan_topoGraph_.osm2Topo){
				auto pointEqual = replan_topoGraph_.ArePoint2DEqual(waySEId_, it.second);
				auto trajectory = replan_topoGraph_.file_->ways[it.first].nodes;
				std::vector<UtmPoint> backTenM_;
				for(int j = 0;j < trajectory.size(); j++){
					backTenM_.push_back(trajectory[j]->point);
				}
				if(POINT2D_POSITIVE_EQUAL == pointEqual){
					backTenM = backTenM_;
					break;
				}
				if(POINT2D_NEGATIVE_EQUAL == pointEqual){
					std::reverse(backTenM_.begin(), backTenM_.end());
					backTenM = backTenM_;
					break;
				}
			}
			for (auto k = 1 ;k < backTenM.size();k++){
				backLength += amathutils::distance2D(backTenM[k],backTenM[k-1]); 
				std::vector<UtmPoint> tempResult;
				if(backLength > 10.0){
				    tempResult.insert(tempResult.end(),backTenM.begin(),backTenM.begin()+k+1);
				    backEnds.push_back(std::make_pair(backTenM[k],tempResult));
				    break;
				}
				if(k == backTenM.size()-1){
				    tempResult.insert(tempResult.end(),backTenM.begin(),backTenM.end());
				    backEnds.push_back(std::make_pair(backTenM[k],tempResult));
				}
			}
		}
		//std::cout<<"当前选择的交叉路口可倒车终点的数目："<< backEnds.size()<<std::endl;
		for(int m = 0;m < backEnds.size();m++){
			//std::cout<<"倒车终点的x："<<backEnds[m].first.x<<std::endl;
			//std::cout<<"倒车终点的y："<<backEnds[m].first.y<<std::endl;
		}
		
		for(int m = 0;m < backEnds.size();m++){
			std::vector<std::vector<UtmPoint>> wholeUtmResults;
			double backEndsLength = 0.0;
			for (auto j = 1 ;j < backEnds[m].second.size();j++){
				backEndsLength += amathutils::distance2D(backEnds[m].second[j],backEnds[m].second[j-1]); 
			}
			backReplannedLength = backReplannedLength + backEndsLength;
			reMultiPoints.insert(reMultiPoints.begin(),backEnds[m].first);
			vehicleInfo.initial_orient_x = backReplannedResult.back().x - backEnds[m].first.x;
			vehicleInfo.initial_orient_y = backReplannedResult.back().y - backEnds[m].first.y;
			double plannedLength = 0.0;
			std::vector<std::vector<UtmPoint>> reMultiUtmResults;
			int turnCount = 0;
			//std::cout<<"---------------重要****------------------"<<std::endl;
			if(multiPointsPlan(reMultiPoints,reMultiUtmResults,plannedLength,MULTI_REPLAN,turnCount)){
				//if(turnCount == 0)
					//std::cout<<"以当前倒车终点重规划无掉头行为"<<std::endl;
				//else
					//std::cout<<"当前倒车终点重规划有掉头行为"<<std::endl;
				wholeUtmResults = reMultiUtmResults;
				std::vector<UtmPoint> backReplannedResult_ = backReplannedResult ;
				/**
			    std::cout << "[callbakReplan] 当前到交叉路口：" << std::endl;
			    std::string id_chain;
			    for (size_t point_idx = 0; point_idx < backReplannedResult_.size(); ++point_idx) {
			        const auto& utm_point = backReplannedResult_[point_idx];
			        auto Id = file->findNearestNode(utm_point.x, utm_point.y);
			        auto nodeId = Id.second;
			        id_chain += std::to_string(nodeId) + "->";
			    }
			    std::cout << id_chain << std::endl;
			    std::cout << "=====================================" << std::endl;
			    **/
				backReplannedResult_.insert(backReplannedResult_.end(),backEnds[m].second.begin(),backEnds[m].second.end());
				/**
				std::cout << "[callbakReplan] 交叉路口到十米点：" << std::endl;
			    std::string id_chain_;
			    for (size_t point_idx = 0; point_idx < backEnds[m].second.size(); ++point_idx) {
			        const auto& utm_point = backEnds[m].second[point_idx];
			        auto Id = file->findNearestNode(utm_point.x, utm_point.y);
			        auto nodeId = Id.second;
			        id_chain_ += std::to_string(nodeId) + "->";
			    }
			    std::cout << id_chain_ << std::endl;
			    std::cout << "=====================================" << std::endl;
				if (wholeUtmResults.empty())
			        return;
				projection::UtmProjector projector;
			    geometry_msgs::Point map_origin;
			    map_origin.x = mapInfo.origin_lat;
			    map_origin.y = mapInfo.origin_lon;
			    map_origin.z = mapInfo.origin_alt;
			    projector = projection::UtmProjector(map_origin);
			    for (size_t segment_idx = 0; segment_idx < wholeUtmResults.size(); ++segment_idx) {
					const auto& singleUtmResult = wholeUtmResults[segment_idx];
			        std::cout << "[printResultsNodeId]第" << segment_idx + 1 << "段规划结果：" << std::endl;
			        std::string id_chain_1;
			        std::cout << id_chain_1 << std::endl;
					for (size_t point_idx = 0; point_idx < singleUtmResult.size(); ++point_idx){
					    const auto& utm_point = singleUtmResult[point_idx];
					    geometry_msgs::Point temp_utm;
					    temp_utm.x = utm_point.x;
					    temp_utm.y = utm_point.y;
					    temp_utm.z = utm_point.z;
					    geometry_msgs::Point gps_point = projector.reverse(temp_utm);
					    auto Id = file->findNearestNode(utm_point.x, utm_point.y);
					    auto nodeId = Id.second;
						id_chain_1 += std::to_string(nodeId) + "->";
					}
					std::cout << id_chain_1 << std::endl;
			    }
			    std::cout << "=====================================" << std::endl;
			    **/
				wholeUtmResults.insert(wholeUtmResults.begin(),backReplannedResult_);
				inThresholdLengths.emplace(plannedLength/5.0+backReplannedLength/1.4,std::make_pair(wholeUtmResults,turnCount));
				//std::cout << "[callbakReplan] 规划长度：" << plannedLength/5.0+backReplannedLength/1.4<<std::endl;
			}
			reMultiPoints.erase(reMultiPoints.begin());
		}
	}

	if(!inThresholdLengths.empty()){
		pubIndex = -1;
		manualRouteAdvanceConfirmed = false;
		std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
		for(auto const &it : inThresholdLengths){
			if(it.second.second == 0){
				//std::cout<<"[Replan] ====== 本次重规划倒车路口在阈值范围内且重规划无掉头行为 ======"<<std::endl;
				pubUtmResults = it.second.first;
				//std::cout<<"规划的最终长度"<<it.first<<std::endl;
				break;
			}
		}
		if(pubUtmResults.size() == 0){
			//std::cout<<"[Replan] ====== 本次重规划倒车路口在阈值范围内但重规划有掉头行为 ======"<<std::endl;
			int maxReverseNum = 0;
			for(auto const &it : inThresholdLengths){
		    	if (it.second.second > maxReverseNum)
			    	maxReverseNum = it.second.second;	
			}
		 	for (int reverseNum = 1 ; reverseNum <= maxReverseNum ; reverseNum++ ){
			    std::map<double,std::pair<std::vector<std::vector<UtmPoint>>,int>> tmpLengths;
				for(auto const &it : inThresholdLengths){
					if(it.second.second != reverseNum)
						continue;
					tmpLengths.emplace(it);
				}
				if (!tmpLengths.empty())
					pubUtmResults = tmpLengths.begin()->second.first;
					break;
		 	}
		}
	
		double DISTANCE_THRESHOLD = 1e-6;
	    std::vector<UtmPoint>& first_segment = pubUtmResults[0];
	    int match_index = -100;
	    for (size_t i = 0; i < first_segment.size(); ++i) {
	        UtmPoint& curr_point = first_segment[i];
	        double dx = curr_point.x - target_point.x;
	        double dy = curr_point.y - target_point.y;
	        double distance = std::hypot(dx, dy);
	        if (distance < DISTANCE_THRESHOLD) {
	            match_index = i;
	            break;
	        }
	    }
	    if (match_index != -100) {
	        first_segment.erase(first_segment.begin() + match_index + 1, first_segment.end());
	        if (pubUtmResults.size() >= 2)
	            pubUtmResults.erase(pubUtmResults.begin() + 1);
	    } 
	
		reMultiPoints.insert(reMultiPoints.begin(),pubUtmResults[1].front());
		reMultiPoints.insert(reMultiPoints.begin(),vehicleNode);
		
		std::vector<UtmPoint>().swap(lastMultiPoints);
		lastMultiPoints = reMultiPoints;
		std::vector<UtmPoint>().swap(reMultiPoints);
		
		lastRoutesNum = pubUtmResults.size();
		visualization_msgs::MarkerArray markerArray_;
		std::vector<std::vector<float>> colors;
		colors.push_back(std::vector<float>{255/255.0, 100/255.0, 190/255.0});
		colors.push_back(std::vector<float>{252/255.0, 223/255.0, 53/255.0});
		colors.push_back(std::vector<float>{204/255.0, 108/255.0, 255/255.0});
		colors.push_back(std::vector<float>{135/255.0, 204/255.0, 98/255.0});
		
		int c_index = 0;
		int id_ = 1;
		for(int i = 0;i < pubUtmResults.size();i++){
			DisplayConfig cfg;
			cfg.id = id_;
			cfg.ns = std::string("/GlobalPlanner");
			int index =  c_index % colors.size();
			cfg.r = colors[index][0];cfg.g = colors[index][1];cfg.b = colors[index][2];cfg.a = 1.0;
			cfg.scale_x = 4.0;cfg.scale_y = 0.0;cfg.scale_z = 0.0;
			markerArray_.markers.push_back(DisPlay::lineMarker(pubUtmResults[i],cfg));
			c_index++;
			id_++;
		}
		display_pub_.publish(markerArray_);
	}
	else{
		bool replan_success = false;
		for(const auto &it : cross_choice_outthreshold){
			//std::cout<<"--------------------超出阈值倒车交叉路口的ID："<<it.second.second<<std::endl;
			std::map<double,std::pair<std::vector<std::vector<UtmPoint>>,int>> outThresholdLengths;
			std::vector<std::pair<UtmPoint,std::vector<UtmPoint>>> backEnds;
			std::vector<UtmPoint> backReplannedResult;
			double backReplannedLength = it.first;
			auto crossingNodeId = it.second.second;
			auto crossingAdjacentNodes = replan_topoGraph_.topoGraphNodes[crossingNodeId].adjacentNodes;
			backReplannedResult = it.second.first;
			double x = backReplannedResult[backReplannedResult.size()-2].x;
			double y = backReplannedResult[backReplannedResult.size()-2].y;
			auto removeId = replan_topoGraph_.file_->findNearestNode(x,y);
			auto removeWayId = removeId.first;
			auto remove_s_id = replan_topoGraph_.osm2Topo[removeWayId].sId;
			auto remove_e_id = replan_topoGraph_.osm2Topo[removeWayId].eId;
			
			for(const auto &iter : crossingAdjacentNodes){
				if(iter->node_id == remove_s_id || iter->node_id == remove_e_id)
					continue;
				WaySEId waySEId_;
				waySEId_.sId = crossingNodeId;
				waySEId_.eId = iter->node_id;
				std::vector<UtmPoint> backTenM;
				double backLength = 0.0;
				for(auto it : replan_topoGraph_.osm2Topo){
					auto pointEqual = replan_topoGraph_.ArePoint2DEqual(waySEId_, it.second);
					auto trajectory = replan_topoGraph_.file_->ways[it.first].nodes;
					std::vector<UtmPoint> backTenM_;
					for(int j = 0;j < trajectory.size(); j++){
						backTenM_.push_back(trajectory[j]->point);
					}
					if(POINT2D_POSITIVE_EQUAL == pointEqual){
						backTenM = backTenM_;
						break;
					}
					if(POINT2D_NEGATIVE_EQUAL == pointEqual){
						std::reverse(backTenM_.begin(), backTenM_.end());
						backTenM = backTenM_;
						break;
					}
				}
				for (auto k = 1 ;k < backTenM.size();k++){
					backLength += amathutils::distance2D(backTenM[k],backTenM[k-1]); 
					std::vector<UtmPoint> tempResult;
					if(backLength > 10.0){
						tempResult.insert(tempResult.end(),backTenM.begin(),backTenM.begin()+k+1);
						backEnds.push_back(std::make_pair(backTenM[k],tempResult));
						break;
					}
					if(k == backTenM.size()-1){
						tempResult.insert(tempResult.end(),backTenM.begin(),backTenM.end());
						backEnds.push_back(std::make_pair(backTenM[k],tempResult));
					}
				}
			}

			//std::cout<<"当前选择的交叉路口可倒车终点的数目："<< backEnds.size()<<std::endl;
			for(int m = 0;m < backEnds.size();m++){
				//std::cout<<"倒车终点的x："<<backEnds[m].first.x<<std::endl;
				//std::cout<<"倒车终点的y："<<backEnds[m].first.y<<std::endl;
			}
			
			for(int m = 0;m < backEnds.size();m++){
				std::vector<std::vector<UtmPoint>> wholeUtmResults;
				double backEndsLength = 0.0;
				for (auto j = 1 ;j < backEnds[m].second.size();j++){
					backEndsLength += amathutils::distance2D(backEnds[m].second[j],backEnds[m].second[j-1]); 
				}
				backReplannedLength = backReplannedLength + backEndsLength;
				reMultiPoints.insert(reMultiPoints.begin(),backEnds[m].first);
				vehicleInfo.initial_orient_x = backReplannedResult.back().x - backEnds[m].first.x;
				vehicleInfo.initial_orient_y = backReplannedResult.back().y - backEnds[m].first.y;
				double plannedLength = 0.0;
				std::vector<std::vector<UtmPoint>> reMultiUtmResults;
				int turnCount = 0;
				//std::cout<<"重规划点的数目："<< reMultiPoints.size()<<std::endl;
				if(multiPointsPlan(reMultiPoints,reMultiUtmResults,plannedLength,MULTI_REPLAN,turnCount)){
					wholeUtmResults = reMultiUtmResults;
					backReplannedResult.insert(backReplannedResult.end(),backEnds[m].second.begin(),backEnds[m].second.end());
					wholeUtmResults.insert(wholeUtmResults.begin(),backReplannedResult);
					outThresholdLengths.emplace(plannedLength/5.0+backReplannedLength/1.4,std::make_pair(wholeUtmResults,turnCount));
				}
				reMultiPoints.erase(reMultiPoints.begin());
			}
		
			if(!outThresholdLengths.empty()){
				pubIndex = -1;
				manualRouteAdvanceConfirmed = false;
				std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
				for(const auto  &it : outThresholdLengths){
					if(it.second.second == 0){
						//std::cout<<"本次重规划在阈值范围外没有掉头行为"<<std::endl;
						pubUtmResults = it.second.first;
						break;
					}
				}
				if(pubUtmResults.size() == 0){
					//std::cout<<"本次重规划在阈值范围外且有掉头行为"<<std::endl;
					int maxReverseNum = 0;
					for(auto const &it : outThresholdLengths){
				    	if (it.second.second > maxReverseNum)
					    	maxReverseNum = it.second.second;	
					}
				 	for (int reverseNum = 1 ; reverseNum <= maxReverseNum ; reverseNum++ ){
					    std::map<double,std::pair<std::vector<std::vector<UtmPoint>>,int>> tmpLengths;
						for(auto const &it : inThresholdLengths){
							if(it.second.second != reverseNum)
								continue;
							tmpLengths.emplace(it);
						}
						if (!tmpLengths.empty()){
							pubUtmResults = tmpLengths.begin()->second.first;
							break;
						}
				 	}
				}

				reMultiPoints.insert(reMultiPoints.begin(),pubUtmResults[1].front());
				reMultiPoints.insert(reMultiPoints.begin(),vehicleNode);
				
				std::vector<UtmPoint>().swap(lastMultiPoints);
				lastMultiPoints = reMultiPoints;
				std::vector<UtmPoint>().swap(reMultiPoints);
				
				lastRoutesNum = pubUtmResults.size();
				visualization_msgs::MarkerArray markerArray_;
				std::vector<std::vector<float>> colors;
				colors.push_back(std::vector<float>{255/255.0, 100/255.0, 190/255.0});
				colors.push_back(std::vector<float>{252/255.0, 223/255.0, 53/255.0});
				colors.push_back(std::vector<float>{204/255.0, 108/255.0, 255/255.0});
				colors.push_back(std::vector<float>{135/255.0, 204/255.0, 98/255.0});
				
				int c_index = 0;
				int id_ = 1;
				for(int i = 0;i < pubUtmResults.size();i++){
					DisplayConfig cfg;
					cfg.id = id_;
					cfg.ns = std::string("/GlobalPlanner");
					int index =  c_index % colors.size();
					cfg.r = colors[index][0];cfg.g = colors[index][1];cfg.b = colors[index][2];cfg.a = 1.0;
					cfg.scale_x = 4.0;cfg.scale_y = 0.0;cfg.scale_z = 0.0;
					markerArray_.markers.push_back(DisPlay::lineMarker(pubUtmResults[i],cfg));
					c_index++;
					id_++;
				}
				display_pub_.publish(markerArray_);
				replan_success = true;
				break;
			}
		}
		if(!replan_success){
			std::cout << "[Replan] ====== 警告：重规划失败 ======" << std::endl;
			std::vector<UtmPoint>().swap(reMultiPoints);
			std::vector<std::vector<UtmPoint>>().swap(pubUtmResults);
			pubIndex = -1;
			manualRouteAdvanceConfirmed = false;
		}
	}

	pubUtmResults_ = pubUtmResults;
    planning_msgs::TrajectoryPointArray trajectory;
    for (const auto& current_path : pubUtmResults){
        for (const auto& utm_point : current_path){
			planning_msgs::TrajectoryPoint point;
            point.x = utm_point.x;
            point.y = utm_point.y;
            trajectory.points.push_back(point);
        }
    }
	cloud_display_pub_.publish(trajectory);
	std::cout << "[Replan] ====== 重规划结果 ======" << std::endl;
	printResultsNodeId();
	std::cout << "[Replan] ====== 重规划轨迹发布完成，发布轨迹点总数：" << trajectory.points.size() << " 个 ======" << std::endl;
}

void Route::callbackCloudmap(const std_msgs::UInt8::ConstPtr &msg){
    if (msg->data == 1)
        std::cout << "[callbackCloudmap] 地图更新标志：" << (int)msg->data << " → 地图已更新" << std::endl;
    if (msg->data == 0){
		std::cout << "[callbackCloudmap] 地图更新标志：" << (int)msg->data << " → 地图未更新" << std::endl;
	    return;
    }
	
	visualization_msgs::MarkerArray clear_marker;
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::DELETEALL;
	clear_marker.markers.push_back(marker);
	display_pub_.publish(clear_marker);
	
	std::string pkg_dir = ros::package::getPath("launch_node");
    std::string config_file = pkg_dir + std::string("/param/global/global_config.yaml");
	YAML::Node doc = YAML::LoadFile(config_file);
	mapInfo.origin_lat	 = doc ["latitude"].as<double>();
	mapInfo.origin_lon	 = doc ["longitude"].as<double>();
	mapInfo.origin_alt	 = doc ["altitude"].as<double>();
	std::string config_file_map = pkg_dir + std::string("/param/route/route.yaml");
	YAML::Node docRoute = YAML::LoadFile(config_file_map);
	mapInfo.map_file = pkg_dir + std::string("/data/") + docRoute ["cloud_map_filename"].as<std::string>();
	private_nh_.param<double>("route_advance_distance_threshold", routeAdvanceDistanceThreshold, 6.0);
	file = mapLoader.loadOsmMap(mapInfo);
	topoGraph_.ClearGraph();
	topoGraph_.BulidTopoGraph(file);
}

std::map<double,int> Route::eCalculate(TopoGraph &tmpGraph,double ePointX,double ePointY,std::vector<UtmPoint> &SingleUtmResult)
{
	auto sAdjacentNodes = tmpGraph.topoGraphNodes[tmpGraph.eId].adjacentNodes;
	std::map<double,int> thetas;
	for(const auto &iter : sAdjacentNodes){
		double adjacent_orient_x = ePointX - iter->node_x ;
		double adjacent_orient_y = ePointY - iter->node_y ;
		double planned_orient_x = SingleUtmResult[1].x-SingleUtmResult[0].x;
		double planned_orient_y = SingleUtmResult[1].y-SingleUtmResult[0].y;
		double dot = planned_orient_x * adjacent_orient_x + planned_orient_y * adjacent_orient_y;
		double vector_norm_1 = sqrt(planned_orient_x * planned_orient_x + planned_orient_y * planned_orient_y);
		double vector_norm_2 = sqrt(adjacent_orient_x * adjacent_orient_x + adjacent_orient_y * adjacent_orient_y);
		double cos_theta = dot/(vector_norm_1 * vector_norm_2);
		cos_theta = std::max(-1.0,std::min(1.0,cos_theta));
		double theta = acos(cos_theta);
		thetas.insert(std::make_pair(theta,iter->node_id));
	}

	return thetas;
}

std::map<double,int> Route::sCalculate(TopoGraph &tmpGraph,double sPointX,double sPointY)
{
	auto sAdjacentNodes = tmpGraph.topoGraphNodes[tmpGraph.sId].adjacentNodes;
	std::map<double,int> thetas;
	for(const auto &iter : sAdjacentNodes){
		double adjacent_orient_x = iter->node_x - sPointX;
		double adjacent_orient_y = iter->node_y - sPointY;
		double dot = vehicleInfo.initial_orient_x * adjacent_orient_x + vehicleInfo.initial_orient_y * adjacent_orient_y;
		double vector_norm_1 = sqrt(vehicleInfo.initial_orient_x * vehicleInfo.initial_orient_x + vehicleInfo.initial_orient_y * vehicleInfo.initial_orient_y);
		double vector_norm_2 = sqrt(adjacent_orient_x * adjacent_orient_x + adjacent_orient_y * adjacent_orient_y);
		double cos_theta = dot/(vector_norm_1 * vector_norm_2);
		cos_theta = std::max(-1.0,std::min(1.0,cos_theta));
		double theta = acos(cos_theta);
		thetas.insert(std::make_pair(theta,iter->node_id));
		//std::cout << "[sCalculate]  theta: " << theta << " → 对应ID值: " << iter->node_id << std::endl;
	}
	return thetas;
}



void Route::sThetasRmv(std::map<double,int> &thetas,TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv)
{
	for(const auto &it: thetas){
		if(it.first > 2*M_PI/3){
			auto remove_node = tmpGraph.GetNode(it.second);
			tmpGraph.topoGraphNodes[tmpGraph.sId].RmvAdjacentNode(remove_node);
			nodesRmv.push_back(remove_node);
		}
	}
}

void Route::sThetasAdd(TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv)
{
	for(auto &iter: nodesRmv){
		tmpGraph.topoGraphNodes[tmpGraph.sId].AddAdjacentNode(iter);
	}
}

void Route::eThetasRmv(std::map<double,int> &thetas,TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv)
{
	auto e_node = tmpGraph.GetNode(tmpGraph.eId);
	for(const auto &it: thetas){
		//std::cout<<"终点移除"<<it.first<<std::endl;
		if(it.first > 2*M_PI/3){
			tmpGraph.topoGraphNodes[it.second].RmvAdjacentNode(e_node);
			auto remove_node = tmpGraph.GetNode(it.second);
			nodesRmv.push_back(remove_node);
		}
	}
}

void Route::eThetasAdd(TopoGraph &tmpGraph,std::vector<std::shared_ptr<ANode>> &nodesRmv)
{
	auto e_node = tmpGraph.GetNode(tmpGraph.eId);
	for(const auto &iter: nodesRmv){
		tmpGraph.topoGraphNodes[iter->node_id].AddAdjacentNode(e_node);
	}
}



ErrCode Route::positivePlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &ptUtmResults,PlanChoice planChoice,int &ptTurnCount)
{
	//std::cout<<"[positivePlan] ====== 进入正序规划 ======"<<std::endl;
	std::vector<UtmPoint> ptSingleUtmResult;
	for(auto i = 0 ;i < multiPoints.size()-1;i++){
		auto sPointX = multiPoints[i].x;
		auto sPointY = multiPoints[i].y;
		auto ePointX = multiPoints[i+1].x;
		auto ePointY = multiPoints[i+1].y;	
		TopoGraph tmpGraph;
		if(MULTI_REPLAN == planChoice){
			tmpGraph = replan_topoGraph_;
			tmpGraph.BulidTmpTopoGraph(sPointX, sPointY, ePointX, ePointY);
			addObs(tmpGraph);
		}
		else{	
			tmpGraph = topoGraph_;
			tmpGraph.BulidTmpTopoGraph(sPointX, sPointY, ePointX, ePointY);
		}
		auto sAdjacentNodes = tmpGraph.topoGraphNodes[tmpGraph.sId].adjacentNodes;
		//std::cout<<"[positivePlan] ====== lianbin ======"<<tmpGraph.sId<<std::endl;
		std::map<double,int> thetas;
		for(const auto &iter : sAdjacentNodes){
			double adjacent_orient_x = iter->node_x - sPointX;
			double adjacent_orient_y = iter->node_y - sPointY;
			double dot;
			double vector_norm_1;
			if(0 == i){
				dot = vehicleInfo.initial_orient_x * adjacent_orient_x + vehicleInfo.initial_orient_y * adjacent_orient_y;
				vector_norm_1 = sqrt(vehicleInfo.initial_orient_x * vehicleInfo.initial_orient_x + vehicleInfo.initial_orient_y * vehicleInfo.initial_orient_y);
			}
			else{
				double planned_orient_x = ptSingleUtmResult[ptSingleUtmResult.size()-1].x-ptSingleUtmResult[ptSingleUtmResult.size()-2].x;
				double planned_orient_y = ptSingleUtmResult[ptSingleUtmResult.size()-1].y-ptSingleUtmResult[ptSingleUtmResult.size()-2].y;
				dot = planned_orient_x * adjacent_orient_x + planned_orient_y * adjacent_orient_y;
				vector_norm_1 = sqrt(planned_orient_x * planned_orient_x + planned_orient_y * planned_orient_y);
			}
			double vector_norm_2 = sqrt(adjacent_orient_x * adjacent_orient_x + adjacent_orient_y * adjacent_orient_y);
			//std::cout << "[INFO]  vector_norm_1: " << vector_norm_1  << std::endl;
			//std::cout << "[INFO]  vector_norm_2: " << vector_norm_2  << std::endl;
			//std::cout << "[INFO]  dot/(vector_norm_1 * vector_norm_2): " << dot/(vector_norm_1 * vector_norm_2)<< std::endl;
			//std::cout << "[INFO]  dot: " << dot<< std::endl;
			double cos_theta = dot/(vector_norm_1 * vector_norm_2);
			cos_theta = std::max(-1.0,std::min(1.0,cos_theta));
			double theta = acos(cos_theta);
			thetas.insert(std::make_pair(theta,iter->node_id));
			//std::cout << "[INFO]  theta: " << theta << " → 对应ID值: " << iter->node_id << std::endl;
		}
        		
		std::vector<std::shared_ptr<ANode>> nodesRmv;
		sThetasRmv(thetas,tmpGraph,nodesRmv);
		
		//auto sAdjacentNodes_rmv = tmpGraph.topoGraphNodes[tmpGraph.sId].adjacentNodes;
		//for(const auto &iter : sAdjacentNodes_rmv)
			//std::cout << "[INFO]  移除后的近邻点ID: " << iter->node_id<< std::endl;

	
		std::vector<UtmPoint>().swap(ptSingleUtmResult);
		auto err_regular = FindPath(tmpGraph,ptSingleUtmResult);
		if(err_regular == ERRCODE_SUCCESS)
			ptUtmResults.push_back(ptSingleUtmResult);
		else{
			//std::cout<<"[INFO]  ===========需要掉头的行为=========="<<std::endl;
			ptTurnCount++;
			sThetasAdd(tmpGraph,nodesRmv);
			std::vector<UtmPoint>().swap(ptSingleUtmResult);
			auto err_turn = FindPath(tmpGraph,ptSingleUtmResult);
			if(err_turn == ERRCODE_SUCCESS)
				ptUtmResults.push_back(ptSingleUtmResult);
			else{
				std::vector<std::vector<UtmPoint>>().swap(ptUtmResults);
				return err_turn;
			}
		}
	}
	return ERRCODE_SUCCESS;
}


ErrCode Route::reservePlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &reUtmResults,PlanChoice planChoice,int &reTurnCount)
{
	//std::cout<<"[reservePlan] ====== 进入逆序规划 ======"<<std::endl;
	std::vector<UtmPoint> reSingleUtmResult;
	if(multiPoints.size()>2){
		for(auto i = multiPoints.size()-1;i > 0;i--){
			auto sPointX = multiPoints[i-1].x;
			auto sPointY = multiPoints[i-1].y;
			auto ePointX = multiPoints[i].x;
			auto ePointY = multiPoints[i].y;
			TopoGraph tmpGraph;
			if(MULTI_REPLAN == planChoice){
				tmpGraph = replan_topoGraph_;
				tmpGraph.BulidTmpTopoGraph(sPointX, sPointY, ePointX, ePointY);
				addObs(tmpGraph);
			}
			else{	
				tmpGraph = topoGraph_;
				tmpGraph.BulidTmpTopoGraph(sPointX, sPointY, ePointX, ePointY);
			}
			if(multiPoints.size()-1 == i){
				auto err = FindPath(tmpGraph,reSingleUtmResult);
				if(err == ERRCODE_SUCCESS)
					reUtmResults.push_back(reSingleUtmResult);
				else{
					std::vector<std::vector<UtmPoint>>().swap(reUtmResults);
					return err;
				}
			}
			else if(1 == i){
				//std::cout<<"进入逆序规划的第一段"<<std::endl;
				auto e_thetas = eCalculate(tmpGraph,ePointX,ePointY,reSingleUtmResult);
				auto s_thetas = sCalculate(tmpGraph,sPointX,sPointY);
				std::vector<std::shared_ptr<ANode>> eNodesRmv;
				std::vector<std::shared_ptr<ANode>> sNodesRmv;
				eThetasRmv(e_thetas,tmpGraph,eNodesRmv);
				sThetasRmv(s_thetas,tmpGraph,sNodesRmv);
				auto s_adjacent = tmpGraph.topoGraphNodes[tmpGraph.sId].adjacentNodes;
				auto e_adjacent = tmpGraph.topoGraphNodes[tmpGraph.eId].adjacentNodes;
				/**
				std::cout<<"逆序规划第一段起始点的近邻点"<<std::endl;
				for(const auto &iter : s_adjacent){
					std::cout<<iter->node_id<<std::endl;
				}
				std::cout<<"逆序规划第一段终点的近邻点"<<std::endl;
				for(const auto &iter : e_adjacent){
					auto e_adjacent_adjacent = tmpGraph.topoGraphNodes[iter->node_id].adjacentNodes;
					std::cout<<iter->node_id<<"近邻点"<<std::endl;
					for(const auto &p : e_adjacent_adjacent)
						std::cout<<p->node_id<<std::endl;
				}
				**/
				std::vector<UtmPoint>().swap(reSingleUtmResult);
				auto err_regular = FindPath(tmpGraph,reSingleUtmResult);
				if(err_regular == ERRCODE_SUCCESS)
					reUtmResults.push_back(reSingleUtmResult);
				else{
					reTurnCount++;
					eThetasAdd(tmpGraph,eNodesRmv);
					std::vector<UtmPoint>().swap(reSingleUtmResult);
					auto err_turn = FindPath(tmpGraph,reSingleUtmResult);
					if(err_turn == ERRCODE_SUCCESS)
						reUtmResults.push_back(reSingleUtmResult);
					else{
						std::vector<std::shared_ptr<ANode>>().swap(eNodesRmv);
						eThetasRmv(e_thetas,tmpGraph,eNodesRmv);
						sThetasAdd(tmpGraph,sNodesRmv);
						std::vector<UtmPoint>().swap(reSingleUtmResult);
						auto err_turn_1 = FindPath(tmpGraph,reSingleUtmResult);
						if(err_turn_1 == ERRCODE_SUCCESS)
							reUtmResults.push_back(reSingleUtmResult);
						else{
							reTurnCount++;
							eThetasAdd(tmpGraph,eNodesRmv);
							std::vector<UtmPoint>().swap(reSingleUtmResult);
							auto err_turn_2 = FindPath(tmpGraph,reSingleUtmResult);
							if(err_turn_2 == ERRCODE_SUCCESS)
								reUtmResults.push_back(reSingleUtmResult);
							else{
								std::vector<std::vector<UtmPoint>>().swap(reUtmResults);
								return err_turn_2;
							}
						}
					}	
				}
			}
			else{
				auto thetas = eCalculate(tmpGraph,ePointX,ePointY,reSingleUtmResult);
				std::vector<std::shared_ptr<ANode>> nodesRmv;
				eThetasRmv(thetas,tmpGraph,nodesRmv);
				std::vector<UtmPoint>().swap(reSingleUtmResult);
				auto err_regular = FindPath(tmpGraph,reSingleUtmResult);
				if(err_regular == ERRCODE_SUCCESS)
					reUtmResults.push_back(reSingleUtmResult);
				else{
					reTurnCount++;
					eThetasAdd(tmpGraph,nodesRmv);
					std::vector<UtmPoint>().swap(reSingleUtmResult);
					auto err_turn = FindPath(tmpGraph,reSingleUtmResult);
					if(err_turn == ERRCODE_SUCCESS)
						reUtmResults.push_back(reSingleUtmResult);
					else{
						std::vector<std::vector<UtmPoint>>().swap(reUtmResults);
						return err_turn;
					}
				}
			}
		}
		return ERRCODE_SUCCESS;
	}
	else{
		//std::cout<<"[reservePlan]逆序规划一段路"<<std::endl;
		TopoGraph tmpGraph;
		if(MULTI_REPLAN == planChoice){
			tmpGraph = replan_topoGraph_;
			tmpGraph.BulidTmpTopoGraph(multiPoints[0].x, multiPoints[0].y, multiPoints[1].x, multiPoints[1].y);
			addObs(tmpGraph);
		}
		else{	
			tmpGraph = topoGraph_;
			tmpGraph.BulidTmpTopoGraph(multiPoints[0].x, multiPoints[0].y, multiPoints[1].x, multiPoints[1].y);
		}
		//std::cout<<"====================排查重点====================="<<std::endl;
		auto thetas = sCalculate(tmpGraph,multiPoints[0].x,multiPoints[0].y);
		
		//for(const auto &iter : thetas)
			//std::cout << "[reservePlan]  theta: " << iter.first << " → 对应ID值: " << iter.second << std::endl;
		
		std::vector<std::shared_ptr<ANode>> nodesRmv;
		sThetasRmv(thetas,tmpGraph,nodesRmv);

		//auto sAdjacentNodes_rmv = tmpGraph.topoGraphNodes[tmpGraph.sId].adjacentNodes;
		//for(const auto &iter : sAdjacentNodes_rmv)
			//std::cout << "[reservePlan]  移除后的近邻点ID: " << iter->node_id<< std::endl;

		
		auto err_regular = FindPath(tmpGraph,reSingleUtmResult);
		if(err_regular == ERRCODE_SUCCESS)
			reUtmResults.push_back(reSingleUtmResult);
		else{
			reTurnCount++;
			sThetasAdd(tmpGraph,nodesRmv);
			std::vector<UtmPoint>().swap(reSingleUtmResult);
			auto err_turn = FindPath(tmpGraph,reSingleUtmResult);
			if(err_turn == ERRCODE_SUCCESS)
				reUtmResults.push_back(reSingleUtmResult);
			else{
				std::vector<std::vector<UtmPoint>>().swap(reUtmResults);
				return err_turn;
			}
		}	
		return ERRCODE_SUCCESS;
	}

}

void Route::printResultsNodeId(){
    if (pubUtmResults.empty())
        return;
	projection::UtmProjector projector;
    geometry_msgs::Point map_origin;
    map_origin.x = mapInfo.origin_lat;
    map_origin.y = mapInfo.origin_lon;
    map_origin.z = mapInfo.origin_alt;
    projector = projection::UtmProjector(map_origin);
    for (size_t segment_idx = 0; segment_idx < pubUtmResults.size(); ++segment_idx) {
		const auto& singleUtmResult = pubUtmResults[segment_idx];
        std::cout << "[printResultsNodeId]第" << segment_idx + 1 << "段规划结果：" << std::endl;
        std::string id_chain;
        std::cout << id_chain << std::endl;
		for (size_t point_idx = 0; point_idx < singleUtmResult.size(); ++point_idx){
		    const auto& utm_point = singleUtmResult[point_idx];
		    geometry_msgs::Point temp_utm;
		    temp_utm.x = utm_point.x;
		    temp_utm.y = utm_point.y;
		    temp_utm.z = utm_point.z;
		    geometry_msgs::Point gps_point = projector.reverse(temp_utm);
		    auto Id = file->findNearestNode(utm_point.x, utm_point.y);
		    auto nodeId = Id.second;
			id_chain += std::to_string(nodeId) + "->";
		}
		std::cout << id_chain << std::endl;
    }
    std::cout << "=====================================" << std::endl;
}



bool Route::multiPointsPlan(std::vector<UtmPoint> multiPoints,std::vector<std::vector<UtmPoint>> &multiUtmResults,double &plannedLength,PlanChoice planChoice,int &turnCount)
{
	std::vector<std::vector<UtmPoint>> reUtmResults;
	std::vector<std::vector<UtmPoint>> ptUtmResults;
	std::vector<std::vector<UtmPoint>> utmResults;
	int reTurnCount = 0;
	int ptTurnCount = 0;
	auto start = std::chrono::high_resolution_clock::now();	
	auto reErr = reservePlan(multiPoints,reUtmResults,planChoice,reTurnCount);
	//std::cout<<"逆序规划掉头次数："<<reTurnCount<<std::endl;
	std::reverse(reUtmResults.begin(), reUtmResults.end());
	auto ptErr = positivePlan(multiPoints,ptUtmResults,planChoice,ptTurnCount);
	//std::cout<<"正序规划掉头次数："<<ptTurnCount<<std::endl;
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	//std::cout << "本次全局路径规划求解时间: " << elapsed.count() <<std::endl;
	
	if(ERRCODE_SUCCESS == reErr && ERRCODE_SUCCESS == ptErr){
		double rePlannnedLength_ = 0.0;
		double ptPlannnedLength_ = 0.0;
		for(auto const &it : reUtmResults){
			for (auto i = 1 ;i < it.size();i++){
				rePlannnedLength_ += amathutils::distance2D(it[i],it[i-1]); 
			}
		}
		for(auto const &it : ptUtmResults){
			for (auto i = 1 ;i < it.size();i++){
				ptPlannnedLength_ += amathutils::distance2D(it[i],it[i-1]); 
			}
		}
		//std::cout<<"本次全局路径规划逆序长度"<<rePlannnedLength_<<std::endl;
		//std::cout<<"本次全局路径规划正序长度"<<ptPlannnedLength_<<std::endl;
		/***
		if(reIsForward && ptIsForward){
			if(rePlannnedLength_ <= ptPlannnedLength_)
				utmResults = reUtmResults;
			else
				utmResults = ptUtmResults;
		}
		else if(reIsForward && !ptIsForward)
			utmResults = reUtmResults;
		else if(!reIsForward && ptIsForward)
			utmResults = ptUtmResults;
		else{
			isForward = false;
			if(rePlannnedLength_ <= ptPlannnedLength_)
				utmResults = reUtmResults;
			else
				utmResults = ptUtmResults;
		}	
		else if(ERRCODE_SUCCESS == reErr && ERRCODE_SUCCESS != ptErr){
			utmResults = reUtmResults;
			if(!reIsForward)
				isForward = false;
		}
		else if(ERRCODE_SUCCESS != reErr && ERRCODE_SUCCESS == ptErr){
			utmResults = ptUtmResults;
			if(!ptIsForward)
				isForward = false;
		}
		***/
		if(0 == reTurnCount && 0 == ptTurnCount){
			if(rePlannnedLength_ <= ptPlannnedLength_)
				utmResults = reUtmResults;
			else
				utmResults = ptUtmResults;
		}
		else if(0 == reTurnCount && 0 != ptTurnCount)
			utmResults = reUtmResults;
		else if(0 != reTurnCount && 0 == ptTurnCount)
			utmResults = ptUtmResults;
		else{
			if(reTurnCount < ptTurnCount){
				utmResults = reUtmResults;
				turnCount = reTurnCount;
			}
			else if(reTurnCount > ptTurnCount){
				utmResults = ptUtmResults;
				turnCount = ptTurnCount;
			}
			else{
				turnCount = ptTurnCount;
				if(rePlannnedLength_ <= ptPlannnedLength_)
					utmResults = reUtmResults;
				else
					utmResults = ptUtmResults;
			}
		}	
	}
	else if(ERRCODE_SUCCESS == reErr && ERRCODE_SUCCESS != ptErr){
		utmResults = reUtmResults;
		if(reTurnCount != 0)
			turnCount = reTurnCount;
	}
	else if(ERRCODE_SUCCESS != reErr && ERRCODE_SUCCESS == ptErr){
		utmResults = ptUtmResults;
		if(ptTurnCount != 0)
			turnCount = ptTurnCount;
	}
	else{
		std::cout << "[multiPointsPlan] ====== 本次多目标全局路径规划失败 ======" << std::endl;
		//std::cout << "[multiPointsPlan] ====== 逆序规划失败代码 ======" <<reErr<<std::endl;
		//std::cout << "[multiPointsPlan] ====== 正序规划失败代码 ======" <<ptErr<<std::endl;
		return false;
	}
	std::vector<UtmPoint> utmResults_;
	for(int i = 0;i < utmResults.size();i++){
		utmResults_.insert(utmResults_.end(),utmResults[i].begin(),utmResults[i].end());
	}
	if(utmResults_.size() == 1)
		plannedLength = 0;
	for (auto i = 1 ;i < utmResults_.size();i++){
		plannedLength += amathutils::distance2D(utmResults_[i],utmResults_[i-1]); 
	}
	multiUtmResults = utmResults;
	return true;
}




/**
void Route::addObs(TopoGraph &tmpGraph)
{
	auto Id = tmpGraph.file_->findNearestNode(obsInfo.x, obsInfo.y);
	auto obsNodeId = Id.second;
	auto obsNode = tmpGraph.file_->nodes[obsNodeId].point;
	for(const auto &it : tmpGraph.osmSE2Topo){
		for(const auto &iter : it.first){

			double d = amathutils::distance2D(iter->point.x,iter->point.y,obsNode.x,obsNode.y);
			if(d < 1e-4){
				auto s_id = it.second.sId;
				auto e_id = it.second.eId;
				auto enode = tmpGraph.GetNode(e_id);
				tmpGraph.topoGraphNodes[s_id].RmvAdjacentNode(enode);
				auto snode = tmpGraph.GetNode(s_id);
				tmpGraph.topoGraphNodes[e_id].RmvAdjacentNode(snode);
			}
		}
	}
}
**/




void Route::displayLoop()
{
   while(1)
    {
        sleep(5);
	    displayOsm();
	    publishRouteSegmentMarkers();
    }
}


void Route::calculateMapBoundary() {
    map_boundary.min_x = std::numeric_limits<double>::max();
    map_boundary.max_x = std::numeric_limits<double>::lowest();
    map_boundary.min_y = std::numeric_limits<double>::max();
    map_boundary.max_y = std::numeric_limits<double>::lowest();
    for (const auto& node_pair : file->nodes) {
        const auto& node = node_pair.second;
        const auto& point = node.point;
        if (point.x < map_boundary.min_x) map_boundary.min_x = point.x;
        if (point.x > map_boundary.max_x) map_boundary.max_x = point.x;
        if (point.y < map_boundary.min_y) map_boundary.min_y = point.y;
        if (point.y > map_boundary.max_y) map_boundary.max_y = point.y;
    }
}


Route::Route(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{
	task_route_color_override_ =
		private_nh_.getParam("task_route_color_r", task_route_color_r_) &&
		private_nh_.getParam("task_route_color_g", task_route_color_g_) &&
		private_nh_.getParam("task_route_color_b", task_route_color_b_);
	if (task_route_color_override_) {
		ROS_INFO_STREAM("[route] task path color: rgb=("
			<< task_route_color_r_ << ", " << task_route_color_g_ << ", "
			<< task_route_color_b_ << ")");
	}

	std::string pkg_dir = ros::package::getPath("launch_node");
    std::string config_file = pkg_dir + std::string("/param/global/global_config.yaml");
	YAML::Node doc = YAML::LoadFile(config_file);
	mapInfo.origin_lat	 = doc ["latitude"].as<double>();
	mapInfo.origin_lon	 = doc ["longitude"].as<double>();
	mapInfo.origin_alt	 = doc ["altitude"].as<double>();

	std::string config_file_map = pkg_dir + std::string("/param/route/route.yaml");
	YAML::Node docRoute = YAML::LoadFile(config_file_map);
	mapInfo.map_file = pkg_dir + std::string("/data/") + docRoute ["cloud_map_filename"].as<std::string>();
	file = mapLoader.loadOsmMap(mapInfo);
	calculateMapBoundary();
	topoGraph_.BulidTopoGraph(file);

	display_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/route_display", 1);
	route_pub_ = nh_.advertise<planning_msgs::TrajectoryPointArray>("/route_result", 1);
	initialPose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &Route::callBackinitialPose, this);
	goal_sub_ = nh_.subscribe("/clicked_point", 1, &Route::callBackgoal, this);
	chassis_sub_ = nh_.subscribe("/chassis", 1, &Route::callbackChassis, this); 
	current_pose_sub_ = nh_.subscribe("odomData", 10, &Route::callBackCurrentPose, this);
	//cloud
	replan_sub_ = nh_.subscribe("/replan", 1, &Route::callbackReplan, this); 
	multi_point_sub_ = nh_.subscribe("/multi_point_planning", 1, &Route::callBackMultiPointPlanning, this);
	init_point_sub_ = nh_.subscribe("/init_point", 1, &Route::callBackInitPoint, this);
	next_route_segment_sub_ = nh_.subscribe("next_route_segment", 1, &Route::callbackNextRouteSegment, this);
	cloud_map_sub_ = nh_.subscribe("/mapSign", 1, &Route::callbackCloudmap, this);
	cloud_display_pub_ = nh_.advertise<planning_msgs::TrajectoryPointArray>("/cloud_route_display", 1);
	map_request_pub_ = nh_.advertise<std_msgs::UInt8>("/request_new_map", 1);
	global_route_planning_done_pub_ =
		nh_.advertise<std_msgs::Float64>("global_route_planning_done", 1, true);

	display_thread_ = std::thread (&Route::displayLoop,this);
	display_thread_.detach();
}


