#include "velocity_planner/velocity_planner.h"
#include "velocity_planner/optimizer.h"
#include <iostream>
#include "velocity_planner/velocity_planner_flow.h"

VelocityPlannerFlow::VelocityPlannerFlow(ros::NodeHandle &nh)
{
    std::string param_node_dir;
	param_node_dir = ros::package::getPath("launch_node");

	if (param_node_dir.empty())
	    param_node_dir = ros::package::getPath("velocity_planner");
	
	std::string vel_config_yaml_file = param_node_dir + std::string("/param/planning/")+std::string("velocity_planner.param.yaml");	

	vel_result_file = param_node_dir + std::string("/data/velocity_result/")+std::string("velocity_planner_result");
    // std::cout <<"VelocityPlannerFlow construct "<<std::endl;

    YAML::Node config_;
    // 加载配置参数
    //车长、车宽、轴距用getinstance取
    auto vehicle_param = vehicle_info_util::VehicleInfoUtil::get_instance();
    vel_config.car_length = vehicle_param->vehicle_length_m;
    vel_config.car_width =  vehicle_param->vehicle_width_m ;
    std::cout<<"car length:"<<vel_config.car_length<<std::endl;
    std::cout<<"car width:"<<vel_config.car_width<<std::endl;
    //其他的还在yaml里
    config_ = YAML::LoadFile(vel_config_yaml_file);
    vel_config.task_type = config_["task_type"].as<int>();
    vel_config.traject_length_default = config_["traject_length_default"].as<double>();
    vel_config.curv_limit = config_["curv_limit"].as<double>();
    vel_config.curv_speed_weight = config_["curv_speed_weight"].as<double>();
    vel_config.curv_upper = config_["curv_upper"].as<double>();
    vel_config.curv_upper_speed = config_["curv_upper_speed"].as<double>();
    vel_config.curv_upper_boudary =config_["curv_upper_boudary"].as<double>();
    vel_config.vision_weight = config_["vision_weight"].as<double>();
    vel_config.cruise_speed = config_["cruise_speed"].as<double>();
    vel_config.meet_speed = config_["meet_speed"].as<double>();
    vel_config.reverse_speed = config_["reverse_speed"].as<double>();
    vel_config.acc_max = config_["acc_max"].as<double>();
    vel_config.acc_min = config_["acc_min"].as<double>();
    vel_config.reverse_brake = config_["reverse_brake"].as<double>();
    vel_config.safe_brake_dis = config_["safe_brake_dis"].as<double>();
    vel_config.safe_follow_dis = config_["safe_follow_dis"].as<double>();
    vel_config.safe_meet_dis = config_["safe_meet_dis"].as<double>();
    vel_config.safe_reverse_dis = config_["safe_reverse_dis"].as<double>();
    vel_config.weight_cruise = config_["weight_cruise"].as<double>();
    vel_config.weight_brake = config_["weight_brake"].as<double>();
    vel_config.weight_follow = config_["weight_follow"].as<double>();
    vel_config.weight_meet = config_["weight_meet"].as<double>();
    vel_config.weight_car_speed = config_["weight_car_speed"].as<double>();
    vel_config.weight_obs_speed = config_["weight_obs_speed"].as<double>();
    vel_config.LengthChange = config_["LengthChange"].as<double>();
    vel_config.debug_self = config_["debug_self"].as<int>();

	velocity_planner_ptr_ = std::make_shared<VelocityPlanner>();

	if( vel_config.debug_self){
		traject_pub_ = nh.advertise<planning_msgs::TrajectoryPointArray>("trajectory_final", 1);
        driver_msgs_sub_ = nh.subscribe<driver_msgs::ChassisReport>("/chassis", 1, &VelocityPlannerFlow::driver_msgs_callback, this);
        obs_msgs_sub_ = nh.subscribe("/obs_msgs", 1, &VelocityPlannerFlow::obs_msgs_callback, this);
        path_msgs_sub_ = nh.subscribe("/trajectory", 1, &VelocityPlannerFlow::path_msgs_callback, this);
	}
}


void VelocityPlannerFlow::Run()
{
    if(!tra_bool ){
        return;
    }
    tra_bool = false;
    index++;
    // std::cout<<"******************速度规划参数加载成功**************"
    unsigned char unceratain = 100;
     bool have_negative_obs = false;
    // 得到速度规划结果
    planning_msgs::TrajectoryPointArray traject = velocity_planner_ptr_->Velocity_Profile_output(vel_config, car_info, obs_info, path_info,unceratain,have_negative_obs);

    // 将结果保存为txt
    std::string fila_path_i = vel_result_file + std::to_string(index) +".txt";
    bool saving = velocity_planner_ptr_->Save(fila_path_i, traject);
    if (saving)
    {
        std::cout<<"----------速度规划输入写入---------"<<index<<"次规划结果"<<std::endl;
    }

    // 发布出去
    PublishTrajectoryProfile(traject);
}


void VelocityPlannerFlow::process(driver_msgs::ChassisReport &car_state,
  perception_msgs::PredictionObstacles &obs,planning_msgs::TrajectoryPointArray &trajectInput,
  planning_msgs::TrajectoryPointArray &trajectOutput, unsigned char  navUncertainty,bool have_negative)
{
    trajectOutput = velocity_planner_ptr_->Velocity_Profile_output(vel_config, car_state, obs, trajectInput,navUncertainty,have_negative);
}



void VelocityPlannerFlow::driver_msgs_callback(const driver_msgs::ChassisReport::ConstPtr &msg)
{
    car_info = *msg;
}

void VelocityPlannerFlow::obs_msgs_callback(const perception_msgs::PredictionObstacles::ConstPtr &msg)
{
    obs_info = *msg;
}

void VelocityPlannerFlow::path_msgs_callback(const planning_msgs::TrajectoryPointArray::ConstPtr &msg)
{
    tra_bool = true;
    path_info = *msg;
}

void VelocityPlannerFlow::PublishTrajectoryProfile(planning_msgs::TrajectoryPointArray &traject)
{
    traject_pub_.publish(traject);
}
