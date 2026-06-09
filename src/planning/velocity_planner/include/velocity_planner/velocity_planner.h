#ifndef VELOCITY_PLANNER_H
#define VELOCITY_PLANNER_H

#include <cfloat>
#include <cmath>
#include <vector>

#include "amathutils_lib/geometry.hpp"
#include "amathutils_lib/geometry_point.hpp"

#include "velocity_planner/state.h"
#include <fstream>
#include "perception_msgs/PredictionObstacles.h"
#include "perception_msgs/TrajectoryPoint.h"
#include "driver_msgs/ChassisReport.h"
#include "planning_msgs/TrajectoryPointArray.h"
#include "yaml-cpp/yaml.h"
#include "velocity_planner/optimizer.h"
#include <string>
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "amathutils_lib/amathutils.hpp"
#include "common/common.h"
#include "platoon_msgs/PlatoonConfig.h"
#include "platoon_msgs/PlatoonMember.h"
#include "platoon_msgs/PlatoonMission.h"
#include "platoon_msgs/PlatoonLog.h"
#include "platoon_common/platoon_common.h"

using namespace std;
using namespace geometry_point;
namespace planning {

struct Vel_config
{
  //默认路径长度
  double traject_length_default;
  int task_type;
  //曲率下限(小于这个数认为直线)、曲率因子(大于0小于1、越小过弯速度越小)、曲率上限(大于这个数认为弯太急)、曲率上限速度(在大弯的速度)
  double curv_limit;
  double curv_speed_weight;
  double curv_upper;
  double curv_upper_speed;
  double curv_upper_boudary;
  //车辆信息 车长、车宽、视界系数(0到0.1之间，越大考虑越宽范围的障碍物)
  double car_length;
  double car_width;
  double vision_weight;
  //速度信息 巡航速度、会车速度、最大加速度、减速度 
  double cruise_speed;
  double meet_speed;
  double reverse_speed;
  double acc_max;
  double acc_min;
  double reverse_brake;
  //安全距离 刹车、跟车、会车安全距离
  double safe_brake_dis;
  double safe_follow_dis;
  double safe_meet_dis;
  double safe_reverse_dis;
  //权重系数 巡航、刹车、跟车、会车 (权重系数不要低于50)  自车车速权重、障碍物车速权重(大于0小于1，越大越提前刹车)
  double weight_cruise;
  double weight_brake;
  double weight_follow;
  double weight_meet;
  double weight_car_speed;
  double weight_obs_speed;
  //路径突变长度
  double  LengthChange;
  int debug_self;
};

/*
struct Waypoint2D {
	Point2D position;
	double vel;
	double kappa;
	double theta;
  double acc;
  double jerk;

	Waypoint2D() {
	this->position = Point2D(0, 0);
	this->vel = 0;
  this->acc = 0;
  this->jerk = 0;
	}

	Waypoint2D(Point2D position, double vel) {
	this->position = position;
	this->vel = vel;
	}

	Waypoint2D(double x, double y, double vel) {
	this->position.x = x;
	this->position.y = y;
	this->vel = vel;
	}

	Waypoint2D(double x, double y,double theta,double kappa,double vel) {
	  this->position.x = x;
	  this->position.y = y;
	  this->vel = vel;
	  this->theta = theta;
	  this->kappa = kappa;
	}

  Waypoint2D(double x, double y,double theta,double kappa,double vel,double acc,double jerk) {
	  this->position.x = x;
	  this->position.y = y;
	  this->vel = vel;
	  this->theta = theta;
	  this->kappa = kappa;
    this->acc = acc;
    this->jerk = jerk;
	}

};
  */
///////////////////////////////////////////////////////////////////////////////
// class VelocityPlanner
///////////////////////////////////////////////////////////////////////////////

class VelocityPlanner {
public:
  VelocityPlanner();


  planning_msgs::TrajectoryPointArray Velocity_Profile_output(Vel_config &vel_config,driver_msgs::ChassisReport &car_state,
  perception_msgs::PredictionObstacles &obs,planning_msgs::TrajectoryPointArray &traject,unsigned char  uncertain,bool have_negative);


  //第一步
  std::vector<Waypoint2D> getMaximumSpeed(double cur_upper_boundary,double desired_speed, double a_max, std::vector<Pose2D> &path,
double &curv_limit,double &curv_speed_weight,double &curv_upper,double &curv_upper_speed);
  //第二步
  std::vector<std::vector<Waypoint2D>> modifyMaximumSpeed
  (double& car_obs_dis,double &obs_vel,std::vector<Waypoint2D> &profile,double car_length,
  double &safe_bra,double &safe_foll_,double &safe_meet,double &safe_reverse,double &meet_vel,double &reverse_speed,double &cur_vel,double reverse_brake);
  //第三步
  std::vector<Waypoint2D>select_Scenario
  (double traject_length_default,std::vector<std::vector<Waypoint2D>> &scenario_profile,
  double &car_obs_dis,double &obs_vel,
  double &weight_cru,double &weight_bra,double  &weight_foll,double &weight_meet,double path_length,
  double current_vel,double &smooth_of_weight,double &car_speed_weight,double &obs_speed_weight,bool is_foward_shift,double cur_vel);

  //第四步
  bool optimizeVelocity(std::vector<Waypoint2D> &profile,double a_max,double a_min,double current_vel,double current_acc,double &smmoth_of_weight,double &traject_length,
  int &index_to_stop,double &traject_length_default,double &curv_upper_speed,int &curv_upper_index,double curv_upper_boudary,double desired_speed);



  bool ComputeVelocityFinal(double cur_upper_boundary,double traject_length_default,double desired_speed,std::vector<Pose2D> &path,State &current_state_,
  std::vector<Waypoint2D> &vel_profile,double &car_obs_dis,double &obs_vel,double a_max,double a_min,double reverse_brake,double car_length,
  double path_length,double &curv_limit,double &curv_speed_weight,
  double &weight_cru,double &weight_bra,double  &weight_foll,double &weight_meet,
  double &safe_bra,double &safe_foll_,double &safe_meet,double &safe_reverse,double &meet_vel,double &reverse_speed,
  double &cur_upper,double &cur_upper_speed,double &car_speed_weight,double &obs_speed_weight,bool is_foward_shift,double LengthChange);

  bool BackUp(bool qp_result,double a_min,std::vector<Waypoint2D> &profile,std::vector<Waypoint2D> &pre_profile, double LengthChange, double vel);

  bool Save(std::string output_file,  planning_msgs::TrajectoryPointArray traject);
  int scenario_num;
  int task_type;
  string task_area;
  bool traj_end;
  bool neg_obs = false;
  // vehicle_info_util::VehicleInfoUtil  *vehcileInfo;

private:
  // double time_gap_;
  // double acc_max_;
  // double slow_speed_;
  int index_to_stop;
  int index_cur_upper;
  std::vector<Waypoint2D> prev_profile_;
  bool prev_direction;

};

} // namespace planning

#endif /* VELOCITY_PLANNER_H */
