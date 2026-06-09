
#include "RecordDataNode.h"
#include <algorithm>
#include <memory>
#include <utility>
#include <fstream>
#include <ros/package.h>


RecordDataNode::RecordDataNode(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{

    vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);
	private_nh_.param<double>("resolution", resolution, 0.5);
	private_nh_.param<bool>("smooth_whole_line", smooth_whole_line, false);

	private_nh_.param<bool>("init_set", origin_set, true);
    std::cout <<"resolution "<<resolution<<std::endl;
    std::cout <<"smooth_whole_line "<<smooth_whole_line<<std::endl;
    std::cout <<"init_set  "<<origin_set<<std::endl;
    std::cout <<"--------------------------------------------"<<std::endl;

    prePoint.x = 1e6;
	prePoint.y = 1e6;
    if (origin_set)
    {
		geometry_msgs::Point origin;
		private_nh_.param<double>("latitude", origin.x , 0.0);
		private_nh_.param<double>("longitude", origin.y , 0.0);
		private_nh_.param<double>("altitude", origin.z , 0.0);

		std::cout <<"origin latitude " <<origin.x<<std::endl;
		std::cout <<"origin longitude "<<origin.y<<std::endl;
		std::cout <<"origin altitude  "<<origin.z<<std::endl;

	    projector =  projection::UtmProjector(origin);
	    prePoint.z = origin.z;

    }
	
	sub_gps_info_ = nh_.subscribe("/odomData", 1, &RecordDataNode::gpsInfoDataCallback, this);
	record_sub_ = nh_.subscribe("/record_flag", 10, &RecordDataNode::recordFlagCallback, this);
}


void RecordDataNode::recordFlagCallback(const std_msgs::Int32::ConstPtr &msg)
{
    std::cout <<"recordFlagCallback: "<<msg->data<<std::endl;
	if ((record_flag) && (0 == msg->data))
		stopRecord();
    record_flag = msg->data;
}


void RecordDataNode::gpsInfoDataCallback(const localization_msgs::Localization::ConstPtr &msg)
{
	if (!record_flag)
		return;
		
	geometry_msgs::Point origin;
	origin.x = msg->original_ins.latitude;
	origin.y = msg->original_ins.longitude;
	origin.z = msg->original_ins.altitude;

	if (!origin_set)
    {
		origin_set = true;
        projector =  projection::UtmProjector(origin);
	    prePoint.z = origin.z;

	    std::string param_node_dir  = ros::package::getPath("launch_node");
	    std::string global_config_yaml_file = param_node_dir + std::string("/param/global/")+std::string("global_config.yaml");			

		YAML::Node configFile = YAML::LoadFile(global_config_yaml_file);
		std::ofstream fout(global_config_yaml_file);
		configFile["latitude"] = origin.x;
		configFile["longitude"] = origin.y;
		configFile["altitude"] = origin.z;
		configFile["init_set"] = true;
		fout << configFile;
		fout.close();

		std::cout <<"----------- set record data origin data---------"<<std::endl;
		std::cout <<"latitude: "<< configFile["latitude"]<<std::endl;
		std::cout <<"longitude "<<configFile["longitude"]<<std::endl;
		std::cout <<"altitude "<<configFile["altitude"]<<std::endl;
		std::cout <<"init_set "<<configFile["init_set"]<<std::endl;
		std::cout <<"--------------------------------------------"<<std::endl;

    }
	
	geometry_msgs::Point utm;

	utm = projector.forward(origin);
    
	auto motion_distance = std::hypot(utm.x - prePoint.x, utm.y - prePoint.y);

	if(motion_distance > resolution)
	{ 

		nav_msgs::Odometry odom ;
	    double yaw = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);
	    odom = amathutils::getOdometryFromPosAndYaw(utm,yaw);

		
	    std::vector<double> savePoint;
		savePoint.push_back(utm.x);
		savePoint.push_back(utm.y);
		savePoint.push_back(utm.z);
		savePoint.push_back(msg->original_ins.latitude);
		savePoint.push_back(msg->original_ins.longitude);
		savePoint.push_back(msg->original_ins.altitude);

		routePoints.push_back(savePoint);
		prePoint = utm;


		std::vector<geometry_msgs::Point> trajectory;
		for (auto &point :routePoints) {
		    geometry_msgs::Point point_vis;
		    point_vis.x = point[0];
		    point_vis.y = point[1];
		    point_vis.z = point[2];
		    trajectory.push_back(point_vis);
		}
        /*
		visualization_msgs::MarkerArray markerArray;
		markerArray.markers.push_back(DisPlay::generateLineMarker(1,trajectory,0.5,0.0,0.5,0.2,"/record/record_data",false));	
		visualization_msgs::MarkerArray v_markArray= DisPlay::generateVehicleMarkerArray(2,odom,vehcileInfo,"/record/record_data");
		markerArray.markers.insert(markerArray.markers.end(),v_markArray.markers.begin(),v_markArray.markers.end());
		realRoute_pub.publish(markerArray);
       */
	}
	
	return;
}



void RecordDataNode::stopRecord()
{   
	std::string recode_data_dir  = ros::package::getPath("launch_node");
	std::string recode_data_store_file = recode_data_dir + std::string("/data/gpsData.txt");

	time_t now = time(0);
	tm *now_time = localtime(&now);
	std::string strTime = std::to_string(now_time->tm_year + 1900) + "-" + std::to_string(now_time->tm_mon + 1) + "-" + std::to_string(now_time->tm_mday)
		+ "-" + std::to_string(now_time->tm_hour) + "-" + std::to_string(now_time->tm_min);
	
	std::string recode_data_store_file_add = recode_data_dir + std::string("/data/gpsData")+strTime+".txt";

	std::ofstream f_gps_data_out;
	std::ofstream f_gps_data_out_add;

	f_gps_data_out = std::ofstream(recode_data_store_file);
	f_gps_data_out<< std::fixed;
	f_gps_data_out.precision(8); //设置输出精度

	f_gps_data_out_add = std::ofstream(recode_data_store_file_add);
	f_gps_data_out_add<< std::fixed;
	f_gps_data_out_add.precision(8); //设置输出精度


	f_gps_data_out <<"x        "<<"y        "<<"z        "<<"latitude        "<<"longitude         "<<"altitude         "<<std::endl;
	f_gps_data_out_add <<"x        "<<"y        "<<"z        "<<"latitude        "<<"longitude         "<<"altitude         "<<std::endl;
    for (auto &point :routePoints)
    {
		for (auto &ele :point){
	        f_gps_data_out <<ele<<"  ";
	        f_gps_data_out_add <<ele<<"  ";
			
		}
	    f_gps_data_out_add <<std::endl;
	    f_gps_data_out <<std::endl;
    }
	
	f_gps_data_out.flush();
	f_gps_data_out.close();

	f_gps_data_out_add.flush();
	f_gps_data_out_add.close();	

	
    std::cout <<"-------------stopRecord-----------------"<<std::endl;
	return;
}


int main(int argc ,char *argv[])
{        
	ros::init(argc, argv, "recordDataNode");
	ros::NodeHandle nh;
	ROS_INFO("recordDataNode start.");
	auto node_ptr = std::make_shared<RecordDataNode>(nh);
	ros::spin();
	ROS_INFO(" recordDataNode  iteration end.");

	return 0;
}


