#include  "velocity_planner/velocity_planner.h"
#include "ros/ros.h"
#include "fstream"
#include <ros/package.h>

#include "vehicle_info_util/vehicle_info_util.hpp"
using namespace planning;

class VelocityPlannerFlow{
    
    public:
    VelocityPlannerFlow() = default;

    explicit VelocityPlannerFlow(ros::NodeHandle &nh);
	
    void Run();
	void setSpeed(double desireSpeed){ vel_config.cruise_speed	  = desireSpeed;}

    void process(driver_msgs::ChassisReport &car_state,
      perception_msgs::PredictionObstacles &obs,planning_msgs::TrajectoryPointArray &trajectInput,
      planning_msgs::TrajectoryPointArray &trajectOutput, unsigned char  navUncertainty,bool have_negative);


    bool tra_bool = false;
    std::string vel_result_file;
    private:
    Vel_config vel_config;
    int index = 0;
    //订阅者，订阅发布的底盘、障碍物与路径信息
    ros::Subscriber driver_msgs_sub_;
    ros::Subscriber obs_msgs_sub_;
    ros::Subscriber path_msgs_sub_;
    //订阅的数据
    driver_msgs::ChassisReport car_info;
    perception_msgs::PredictionObstacles obs_info;
    planning_msgs::TrajectoryPointArray path_info;
    //订阅回调函数
    void driver_msgs_callback(const driver_msgs::ChassisReport::ConstPtr &msg);
    void obs_msgs_callback(const perception_msgs::PredictionObstacles::ConstPtr &msg);
    void path_msgs_callback(const planning_msgs::TrajectoryPointArray::ConstPtr &msg);
    //发布者,发布包含速度信息的轨迹
    void PublishTrajectoryProfile(planning_msgs::TrajectoryPointArray &traject);
    std::shared_ptr<VelocityPlanner> velocity_planner_ptr_;
    ros::Publisher traject_pub_;

};
