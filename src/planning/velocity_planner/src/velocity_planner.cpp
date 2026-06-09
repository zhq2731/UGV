#include "velocity_planner/velocity_planner.h"

#include <iostream>

using namespace osqp;

namespace planning {

///////////////////////////////////////////////////////////////////////////////
// class VelocityPlanner
///////////////////////////////////////////////////////////////////////////////


VelocityPlanner::VelocityPlanner() {

  prev_profile_ = {Waypoint2D()};
  //初始默认向前
  prev_direction = true; 

  std::cout<<"速度规划初始化"<<std::endl;
}


double getVelLength(std::vector<Waypoint2D> &profile)
{
  double length;
    for(int i=0;i<profile.size()-1;i++)
  {
   length += distance2D(profile[i].position,profile[i+1].position);
  }
  return length;
}

//总的调用，输入输出在此 cout出优化成功与失败  输出返回 包含v，a的trajectoryponitarray.msg
  planning_msgs::TrajectoryPointArray VelocityPlanner::Velocity_Profile_output(Vel_config &vel_config,driver_msgs::ChassisReport &car_state,
  perception_msgs::PredictionObstacles &obs,planning_msgs::TrajectoryPointArray &traject,unsigned char  uncertain,bool have_negative)
{
  //全局坐标系下，认为规划路径的起始点位置航向为自车位置航向， 车辆坐标系下，自车位置0，0 航向0
  //现在给的障碍物信息是全局东北天下的 感知定义的车辆系是右前上
  //当前车速与加速度在底盘信息里
  State current_state;
  current_state.vel = car_state.current_velocity;
  current_state.acc = car_state.current_acceleration;
  double car_theta = traject.points[0].theta;
  double car_x = traject.points[0].x;
  double car_y = traject.points[0].y;
  //曲率下限
  double cur_lim = vel_config.curv_limit;
  double cur_spe_weight = vel_config.curv_speed_weight;
  double cur_upper = vel_config.curv_upper;
  double cur_upper_speed = vel_config.curv_upper_speed;
  double cur_upper_boundary =vel_config.curv_upper_boudary;
  //车辆参考速度、会车速度、倒车速度、最大加减速度、自车长、宽、视界系数
  double desired_speed = vel_config.cruise_speed;
  double meet_speed = vel_config.meet_speed;
  double reverse_speed = vel_config.reverse_speed;
  double a_max = vel_config.acc_max;
  double a_min = vel_config.acc_min;
  double reverse_brake = vel_config.reverse_brake;
  double car_length = vel_config.car_length;
  double car_width = vel_config.car_width;
  double vision_weight = vel_config.vision_weight;
  double car_speed_weight = vel_config.weight_car_speed;
  double obs_speed_weight = vel_config.weight_obs_speed;
  //安全距离
  double safe_brake =vel_config.safe_brake_dis ;
  double safe_follow =vel_config.safe_follow_dis;
  double safe_meet = vel_config.safe_meet_dis;
  double safe_reverse = vel_config.safe_reverse_dis;
  //权重系数
  double weight_cru=vel_config.weight_cruise;
  double weight_bra = vel_config.weight_brake;
  double weight_foll = vel_config.weight_follow;
  double weight_meet = vel_config.weight_meet;
  //路径默认长度
  double traject_length_default = vel_config.traject_length_default;
  task_type = vel_config.task_type;
  task_area = traject.task_area;
  task_type = 0;
  //路径突变长度
  double LengthChange = vel_config.LengthChange;

  //终点停车标识位
  // traj_end = traject.close_to_end;

  //凹坑标志位
  neg_obs = have_negative;

  if(task_area == "over_taking_area")
   {
     //std::cout<<"overtakingarea,velocity"<<std::endl; 
    task_type = 1; 
	  desired_speed = desired_speed + 3;	}

	
	
  if( task_area == "car_meeting_area")
  {
	  // std::cout<<"car_meeting_area,velocity"<<std::endl;
	  task_type = 3;
    cur_spe_weight = cur_spe_weight*1.5;
	  desired_speed =  6.0; 
  }
    

// if (uncertain < 100){
//        desired_speed = 5.0;
// }
 

    if(task_area == "road_obstacle_area" || task_area == "smoke_road_area" || task_area ==  "collect_info_area" )
   { 
      //std::cout<<"road_obstacle_area,smoke_road_area,collect_info_area,velocity"<<std::endl;
      task_type = 2;
	    desired_speed = 3.0;	
   }

  if(neg_obs)
	{
	     desired_speed = 2.5;
	}

    
  //障碍物信息
  double obs_vel_local;//局部速度
  double obs_vel;//障碍物全局速度
  int obs_size = obs.prediction_obstacles.size();
  //轨迹是否倒车
  bool is_foward_shift = traject.is_forward_shift;
  //
  std::vector<std::vector<Point2D>> obses_local;
  //障碍物全局坐标转换到车辆坐标系下 我们采用的前左上
  for(int i = 0; i<obs_size; i++)
  {
    std::vector<Point2D> obs_local;
    for(unsigned int j = 0; j<obs.prediction_obstacles[i].perception_obstacle.polygon.points.size(); j++)
    {
      double x = obs.prediction_obstacles[i].perception_obstacle.polygon.points[j].x;
      double y = obs.prediction_obstacles[i].perception_obstacle.polygon.points[j].y;
      Point2D point_local = globalToLocal( Point2D(car_x,car_y), car_theta, Point2D(x,y) );
      obs_local.push_back(point_local);
    }
    obses_local.push_back(obs_local);
  }
  //所有障碍物点转换到车辆坐标系后，进行筛选

  //最近障碍物对应的索引与车辆坐标系下x
  int index = -1;
  double min_x=50;
  for(int i = 0; i<obs_size; i++)
  {
    for(unsigned int j = 0; j<obses_local[i].size(); j++)
    {//第i个障碍物的第j个点 
      double x =  obses_local[i][j].x;
      double y =  obses_local[i][j].y;
      //动态的，横向2倍车宽，纵向在车前的障碍物
      if( !obs.prediction_obstacles[i].is_static &&  fabs(y)< 1.5*(1+vision_weight*current_state.vel)*car_width &&  x>car_length )
      {
        if(x<min_x)
        {
          min_x = x;
          index = i;
        }
      }
    }
  }

  //将traject转换为需要的path,并且获得轨迹长度
  std::vector<Pose2D> path;
  int path_size = traject.points.size();
  double trajec_length = 0;
  for(int i=0;i<path_size;i++)
  {
    double x = traject.points[i].x;
    double y = traject.points[i].y;
    double kappa = traject.points[i].kappa;
    double theta = traject.points[i].theta;
    path.push_back(Pose2D(x,y,theta,kappa));
  }
  for(int i=0;i<path_size-1;i++)
  {
    trajec_length += distance2D(path[i].position,path[i+1].position);
  }


  if(trajec_length == 0)
  {
    // std::cout<<"------------没有路径输入----------------"<<std::endl;
    return traject  ;
   }
  


  std::vector<double> obs_local_x;
//按index取得最近动态障碍物信息，转换到全局下
  if(index == -1) //index = -1,没有障碍物时,认为障碍物速度是0
{
  // std::cout<<"****************************感知无障碍物*******************************"<<std::endl;
  obs_vel = 0;
}
else
{//车辆坐标系下障碍物在自车航向即x方向速度 感知给的是绝对速度、需要把速度转换到车辆坐标系，向车头方向投影

  double obs_vel_global_x = obs.prediction_obstacles[index].perception_obstacle.velocity.x;
  double obs_vel_global_y = obs.prediction_obstacles[index].perception_obstacle.velocity.y;
  Point2D car_vel_point;
  car_vel_point.x = current_state.vel*cos(car_theta);
  car_vel_point.y = current_state.vel*sin(car_theta);
  Point2D obs_vel_local_point = globalToLocal(car_vel_point,car_theta,Point2D(obs_vel_global_x,obs_vel_global_y));
  obs_vel_local = obs_vel_local_point.x;
  for(unsigned int i = 0; i<obses_local[index].size(); i++)
  {
    //该障碍物在车辆坐标系下的包络点x坐标
    double x = obses_local[index][i].x;
    // double y = obs.prediction_obstacles[index].perception_obstacle.polygon.points[i].y;
    obs_local_x.push_back(x);
    // double x_global = x*cos(cur_theta) - y*sin(cur_theta) + cur_x;
    // double y_global = x*sin(cur_theta) + y*cos(cur_theta) + cur_y;
    // obs_point.push_back(Point2D(x_global,y_global));
  }
 
  obs_vel = obs_vel_local + current_state.vel;

  
}
  

 //获得最近的距离、作为车到障碍物的距离，若没有障碍物，则令车到障碍物距离为路径长度
 
 double car_to_obs=trajec_length;
  if(!(index == -1))
  {
    auto minPos = std::min_element(obs_local_x.begin(),obs_local_x.end());
    car_to_obs = *minPos;
  }
  
  //空的输出，用于传递引用
  std::vector<Waypoint2D> vel_profile;
  vel_profile.resize(path_size);
  

  bool final_result =ComputeVelocityFinal(cur_upper_boundary,traject_length_default,desired_speed,path,current_state,vel_profile,car_to_obs,obs_vel,a_max,a_min,reverse_brake,
  car_length,trajec_length,cur_lim,cur_spe_weight,weight_cru,weight_bra,weight_foll,weight_meet,safe_brake,safe_follow,safe_meet,safe_reverse,
  meet_speed,reverse_speed,cur_upper,cur_upper_speed,car_speed_weight,obs_speed_weight,is_foward_shift,LengthChange);
    if(final_result)
    { 
      //sdt::cout<<"success"<<std::endl
    }
  


  //V1=V0+at,给出相对时间  
  std::vector<double> relat_time;
  relat_time.push_back(0.0); //先push进去0,作起始时间
  double t_re = 0;
  double t = 0;
  for(int i =0; i<path_size-1; i++)
  {
    double v0 = vel_profile[i].vel;
    double v1 = vel_profile[i+1].vel;
    double a0 = vel_profile[i].acc;
    double a1 = vel_profile[i+1].acc;
    if(v0 <= 0.41)//速度低，原地没动
    {
     t = 0;
    }
    else if(v0 >0.41 && fabs(a1-a0) <= 0.15)//速度起来，a很小时，认为匀速,此时用s1-s0 / V得到时间
    {
      double s = distance2D(vel_profile[i+1].position ,vel_profile[i].position);
      t = s/v0;
    }
    else//a起来了，用v1-v0 /a得到时间
    {
      if(a0 == 0)
      {
      double s = distance2D(vel_profile[i+1].position ,vel_profile[i].position);
      t = s/v0;
      }
      else
      {t = (v1-v0)/a0;}
    }
    t_re +=t;
    relat_time.push_back(t_re);
  }




//倒车时间
  if( is_foward_shift == false)
  {
  relat_time.clear();
  relat_time.resize(path_size);
  relat_time[0] = 0.0; //先push进去0,作起始时间
  double t_re = 0;
  double t = 0;
    for(int i =0; i<path_size-1; i++)
  {
    double v0 = vel_profile[i].vel;
    double v1 = vel_profile[i+1].vel;
    double a0 = vel_profile[i].acc;
    double a1 = vel_profile[i+1].acc;
    if(fabs(v0) <= 0.41)//速度低，原地没动
    {
     t = 0;
    }
    else if(fabs(v0) >0.41 && fabs(a1-a0) <= 0.1)//速度起来，a很小时，认为匀速,此时用s1-s0 / V得到时间
    {
      double s = distance2D(vel_profile[i+1].position ,vel_profile[i].position);
      t = s/fabs(v0);
    }
    else//a起来了，用v1-v0 /a得到时间
    {
     if(a0==0)
    {
     double s = distance2D(vel_profile[i+1].position ,vel_profile[i].position);
      t = s/fabs(v0);
    }
    else
      {t = -(v1-v0)/a0;}
    }
    t_re +=t;
    relat_time[i]=t_re;
  }
  //处理最后一个点时间是0 
  if(relat_time[path_size-1] == 0)
  {relat_time[path_size-1] = relat_time[path_size-2];}

}


  //将输出转为TrajectoryPointArray返回 
  for(int i = 0; i<path_size; i++)
  {
    traject.points[i].v = vel_profile[i].vel;
    traject.points[i].a = vel_profile[i].acc;
    traject.points[i].relative_time = relat_time[i];
  }

  prev_direction = is_foward_shift;
  prev_profile_ = vel_profile;

  return traject;
  
  

}

//ros2时的用法
bool VelocityPlanner::ComputeVelocityFinal(double cur_upper_boundary,double traject_length_default,double desired_speed,std::vector<Pose2D> &path,State &current_state_,
  std::vector<Waypoint2D> &vel_profile,double &car_obs_dis,double &obs_vel,double a_max,double a_min,double reverse_brake,double car_length,
  double path_length,double &curv_limit,double &curv_speed_weight,
  double &weight_cru,double &weight_bra,double  &weight_foll,double &weight_meet,
  double &safe_bra,double &safe_foll_,double &safe_meet,double &safe_reverse,double &meet_vel,double &reverse_speed,
  double &cur_upper,double &cur_upper_speed,double &car_speed_weight,double &obs_speed_weight,bool is_foward_shift, double LengthChange)
{
  ////前进后退切换
    if( (!is_foward_shift  ==  prev_direction) && !(current_state_.vel ==0))//两次规划结果不同 并且速度不为0 则进行刹停
    {
    // std::cout<<"---changing-direction---"<<std::endl;
    vel_profile.resize(prev_profile_.size());
    vel_profile  = prev_profile_;
     for(unsigned int i=0; i <vel_profile.size(); i++)
    {
       vel_profile[i].acc = -2;
      if(i==0)
      { //考虑倒车进来
        vel_profile[i].vel = fabs(current_state_.vel);
      }
      else
      {
        double temp  =-2*2*distance2D(vel_profile[i].position,vel_profile[i-1].position)+vel_profile[i-1].vel*vel_profile[i-1].vel;
       if(temp >=0)
        {
          vel_profile[i].vel = std::sqrt(temp);
          if(!is_foward_shift){ vel_profile[i].vel = - vel_profile[i].vel;}

        }
       else
        {
          vel_profile[i].vel = 0;
          vel_profile[i].acc = 0;
        }
      }
    }

    if(!is_foward_shift) { vel_profile[0].vel = -vel_profile[0].vel; }

    return true;
    }
    ////////
    //相同，则按照正常规划


    std::vector<Waypoint2D> max_velocity_profile = getMaximumSpeed(cur_upper_boundary,desired_speed,a_max,path,curv_limit,curv_speed_weight,cur_upper,cur_upper_speed);

      //第二步 将不同场景的障碍物时间窗约束转化为最大速度约束
      //注意这里的障碍物信息需要提前经过筛选，障碍物是只在车辆行驶前方的障碍物，速度为沿着车辆行驶方向的速度
      std::vector<std::vector<Waypoint2D>> modify_profile = modifyMaximumSpeed
      (car_obs_dis,obs_vel,max_velocity_profile,car_length,safe_bra,safe_foll_,safe_meet,safe_reverse,meet_vel,reverse_speed,current_state_.vel,reverse_brake);



    //  std::cout<<"----------------------修改速度约束成功-----------------"<<std::endl;
     double smooth_of_weight = 50;
      //第三步 根据感知信息,确定当前场景
      vel_profile= select_Scenario
      (traject_length_default,modify_profile,car_obs_dis,obs_vel,weight_cru,weight_bra,weight_foll,weight_meet,path_length,
      current_state_.vel,smooth_of_weight,car_speed_weight,obs_speed_weight,is_foward_shift,current_state_.vel);
      //权重初始值
        // std::cout<<"---------------------场景选择成功----------------------"<<std::endl;
      //倒车部分不需要优化 直接return
      if(is_foward_shift == false)
      {
        // prev_profile_ =vel_profile;
        return true;
      }

      //第四步 生成代价函数为速度尽量快与jerk尽量小的速度，加速度曲线
      bool qp_result = 
      optimizeVelocity(vel_profile,a_max,a_min,current_state_.vel, current_state_.acc,smooth_of_weight,path_length,
      index_to_stop,traject_length_default,cur_upper_speed,index_cur_upper,cur_upper_boundary,desired_speed);

      bool final_result = BackUp(qp_result,a_min,vel_profile,prev_profile_,LengthChange,current_state_.vel);

      return final_result;
}



//----------------------------------------下面是各个分步骤------------------------------------------------------


// 第一步，计算路径上的最大速度，取期望速度与曲率约束速度的最小值
std::vector<Waypoint2D> VelocityPlanner::getMaximumSpeed(double cur_upper_doundary,double desired_speed, double a_max, std::vector<Pose2D> &path,
double &curv_limit,double &curv_speed_weight,double &curv_upper,double &curv_upper_speed)
{
  //profiel为路径点信息[x,y,theta,k,v]
  std::vector<Waypoint2D> profile;
  int index_curv=-1;
  //期望最大速度
  double on_road_maxspeed = desired_speed;
  //曲率约束速度
  for(unsigned int i=0; i<path.size(); i++){
    //这里需要做一个判断，若曲率近似0，近似为直线，则最大速度就为期望最大速度
    if ( fabs(path[i].kappa) <= curv_limit )
    {
      profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y,path[i].yaw,path[i].kappa,on_road_maxspeed));
    }
    else if(fabs(path[i].kappa)>=curv_upper)
    {
      profile.push_back(Waypoint2D(path[i].position.x,path[i].position.y,path[i].yaw,path[i].kappa,curv_upper_speed));
      index_curv = i;
    }
    else //平缓的弯道处，则取曲率约束速度与期望速度中的最小值
    { //为防止提前加速冲出弯道，这里取0.8的曲率约束速度
      double curvature_speed = curv_speed_weight*sqrt(a_max/fabs(path[i].kappa)) ;
      double temp_speed = std::min(desired_speed,curvature_speed);
      profile.push_back(Waypoint2D(path[i].position.x, path[i].position.y,path[i].yaw,path[i].kappa,temp_speed));
    }
  }

  
  if(index_curv == -1){index_cur_upper=-1;}
  else
  {
    index_cur_upper  = index_curv;
    for(unsigned int j =(index_curv/cur_upper_doundary); j<profile.size();j++)
    {
      profile[j].vel = curv_upper_speed;
    }
  }



  return profile;
}

//第二步，将不同场景的障碍物时间窗约束转化为最大速度约束
//输入:最大参考速度,障碍物信息
std::vector<std::vector<Waypoint2D>> VelocityPlanner::modifyMaximumSpeed
(double& car_obs_dis,double &obs_vel,std::vector<Waypoint2D> &profile,double car_length1,
  double &safe_bra,double &safe_foll_,double &safe_meet,double &safe_reverse,double &meet_vel,double &reverse_speed,double &cur_vel,double reverse_brake)
{
  std::vector<std::vector<Waypoint2D>> scenario_profile;
  int scenario_num = 5;
  scenario_profile.resize(scenario_num);
  //0对应无障碍物的正常行驶,1对应刹车,2对于跟车，3对应会车，4对应倒车
  //无障碍物,正常行驶
  for (unsigned int i = 0; i <profile.size(); i++)
  {
    scenario_profile[0].push_back(profile[i]);
  }
  
  //刹车(刹停)场景
    double car_length = car_length1;
    if(traj_end)
    {
      car_length = 0;
    }

  index_to_stop = 0;
  double distance_to_obs =  car_obs_dis;
  for (unsigned int i = 0; i < profile.size(); i++)
  {
   scenario_profile[1].push_back(profile[i]);
  }
  
        if ((car_length+safe_bra)<=distance_to_obs  && distance_to_obs<=  (car_length +45) )
        { 
          // std::cout<<"视线内存在无法避开障碍物，需要停车"<<std::endl;
          double dis_to_stop = 0;
          for(unsigned int i = profile.size()-1;i>1;i--)
          {
            // std::cout<<"计算需要刹停的位置"<<std::endl;
            dis_to_stop += distance2D(profile[i].position,profile[i-1].position); 
            if (dis_to_stop>= (car_length+safe_bra) ) //在自车前方留出米空间，确保安全
            {
              index_to_stop = i;  
              //std::cout<<"停止点索引："<< index_to_stop<<std::endl;
              //std::cout<<"距终点距离："<< dis_to_stop<<std::endl;
              for(unsigned int j = index_to_stop;j<profile.size();j++)
              {
                 scenario_profile[1][j].vel = 0.5;
              } 
              break;
            }
          }
        }
        else if (distance_to_obs<(car_length+safe_bra))//自车到障碍物的距离不足m，直接修改所有路径点速度为刹车速度
        {
            for(unsigned int i =0;i<profile.size();i++)
          {
            scenario_profile[1][i].vel =0.5;
          }
        }




  //跟车
  //将距离前车米后的速度设为与前车相同,在我前方的我才跟
  //前方若没有，按原速度
  int index_to_follow= profile.size()-1;
  double distance_to_leading_vehicle = car_obs_dis;
  for (unsigned int i = 0; i < profile.size(); i++)
  {
    scenario_profile[2].push_back(profile[i]);
  }
  
      if( (car_length+safe_foll_) <=distance_to_leading_vehicle && distance_to_leading_vehicle <=45)
      {//自车5m前，规划路径内的前车
      double dis_to_follow=0;
      for(unsigned int i = profile.size()-1;i>1;i--)
      {
          // std::cout<<"计算需要跟随的位置"<<std::endl;
          dis_to_follow += distance2D(profile[i].position,profile[i-1].position); 
          if ((car_length+safe_foll_*(1+0.05*obs_vel))<=dis_to_follow)
          {//距离前车5m处保持相同速度
            index_to_follow =i;
            for(unsigned int i = index_to_follow;i<profile.size();i++)
           {
             scenario_profile[2][i].vel = obs_vel;
            }
            break;
          }
      }
      }
      else if ( car_length <=distance_to_leading_vehicle  && distance_to_leading_vehicle <(car_length + safe_foll_))
      {//在自车0到5m间有前车驶入，减速到前车车速的60%
        for (unsigned int i = 0; i < profile.size(); i++)
        {
          // std::cout<<"距离过近，降低速度拉开距离"<<std::endl;
          scenario_profile[2][i].vel = 0.6*obs_vel;
        }
      }


  //会车部分 对向来车时，距离对向车5米处降低速度到会车车速 
  int index_to_meet = profile.size()-1;
  double distance_to_meeting_vehicle = car_obs_dis;
  for(unsigned int i = 0; i<profile.size();i++)
  {
    scenario_profile[3].push_back(profile[i]);
  }
  
    if(car_length+safe_meet<=distance_to_meeting_vehicle && distance_to_meeting_vehicle<=car_length+45)
    {
      double distance_to_meet = 0 ;
      for(unsigned int i =profile.size()-1; i>1; i--)
      {
        distance_to_meet += distance2D(profile[i].position,profile[i-1].position);
        if(distance_to_meet> (car_length+safe_meet))
        {
            index_to_meet = i;
            for(unsigned int i =index_to_meet; i<profile.size(); i++)
          {
            scenario_profile[3][i].vel = meet_vel;
          }
          break;
        }
      }
    }
    else if(car_length<=distance_to_meeting_vehicle && distance_to_meeting_vehicle<(car_length+safe_meet))
    {
        for(unsigned int i=0; i<profile.size();i++)
      {
        scenario_profile[3][i].vel = 0.6*meet_vel;
      }
    }
  
  //倒车部分
  for(unsigned int i = 0; i<profile.size();i++)
  {
    scenario_profile[4].push_back(profile[i]);
  }
  //第一个点加速度均0
  scenario_profile[4][0].vel = fabs(cur_vel);
  scenario_profile[4][0].acc =0;
  //先给末尾刹停的速度
  //按1的加速度干到刹车速度，之后不给油门
  int index_reverse_stop = profile.size();
  double dis_reverse_stop =0;
  for (unsigned int i = profile.size()-1 ; i >1 ; i--)
  {
    dis_reverse_stop += distance2D(profile[i].position,profile[i-1].position);
    if (dis_reverse_stop >= safe_reverse )
    {
      index_reverse_stop = i;
      for (unsigned int i = index_reverse_stop; i < profile.size(); i++)
      {
        scenario_profile[4][i].vel = 0;
        scenario_profile[4][i].acc = 0;
      }
      break;
    }
    else if ( 0 <dis_reverse_stop && dis_reverse_stop <safe_reverse)
    {
      scenario_profile[4][i].vel = 0;
      scenario_profile[4][i].acc = 0;
    }
  }
  //路比较长时

      for(int i =1 ; i <index_reverse_stop ;i++)
  {
    double temp  =2*distance2D(scenario_profile[4][i].position,scenario_profile[4][i-1].position)+scenario_profile[4][i-1].vel*scenario_profile[4][i-1].vel;
    if(temp <  reverse_speed*reverse_speed) { scenario_profile[4][i].vel =  - 1*sqrt(temp) ;  scenario_profile[4][i].acc = 1;}
    else{ scenario_profile[4][i].vel = -reverse_speed  ;  scenario_profile[4][i].acc = 0;}
  }

    


  // std::cout<<"最大速度修正"<<std::endl;
  return scenario_profile;
}


//第三步,根据感知信息,确定当前场景
std::vector<Waypoint2D> VelocityPlanner::select_Scenario
(double traject_length_default,std::vector<std::vector<Waypoint2D>> &scenario_profile,
  double &car_obs_dis,double &obs_vel,
  double &weight_cru,double &weight_bra,double  &weight_foll,double &weight_meet,double path_length,
  double current_vel,double &smooth_of_weight,double &car_speed_weight,double &obs_speed_weight,bool is_foward_shift,double cur_vel )
{
  std::vector<Waypoint2D>final_profile;
  //判断,规划轨迹前方没有障碍物,正常行驶
  if((traject_length_default-5)<=path_length && is_foward_shift == true)
  {
    //std::cout<<"---面前无障碍物,加速行驶---"<<std::endl;
    int num_norm = scenario_profile[0].size();
    final_profile.resize(num_norm);
    final_profile = scenario_profile[0];
    smooth_of_weight = weight_cru;
    scenario_num = 0;
  }
  //当路径长度小于50，且障碍物速度小于0.5时，给出刹车信号
  else if ( path_length< (traject_length_default - 5) && fabs(obs_vel) <=0.5 && is_foward_shift == true) 
  {
    // std::cout<<"---停车---"<<std::endl;
    int num_stop = scenario_profile[1].size();
    final_profile.resize(num_stop);
    final_profile = scenario_profile[1];
    smooth_of_weight = weight_bra*(1+0.1*current_vel);
    scenario_num = 1;

  }
  //前方有车辆移动时,与前车距离保持m,即在据障碍物m时速度与前车速度相同
  else if ( path_length<(traject_length_default-5) && obs_vel > 0.5 && is_foward_shift == true )
  {
    // std::cout<<"---跟随前车行驶---"<<std::endl;    
    int num_follow = scenario_profile[2].size();
    final_profile.resize(num_follow);
    final_profile = scenario_profile[2];
    smooth_of_weight = weight_foll*(1+car_speed_weight*current_vel+obs_speed_weight*obs_vel);
    scenario_num = 2;
  }
  //对向车辆驶来，减速会车
  else if( path_length< (traject_length_default-5) && obs_vel < -0.5 && is_foward_shift == true)
  {
    // std::cout<<"---减速会车---"<<std::endl;
    int num_meet = scenario_profile[3].size();
    final_profile.resize(num_meet);
    final_profile = scenario_profile[3];
    smooth_of_weight = weight_meet*(1+car_speed_weight*current_vel+obs_speed_weight*fabs(obs_vel));
    scenario_num = 3;
  }
  //倒车
  else if(is_foward_shift == false )
  {
    // std::cout<<"---倒车---"<<std::endl;
    final_profile.resize(scenario_profile[4].size());
    final_profile = scenario_profile[4];
    scenario_num = 4;
  }

  if(task_type == 1){  smooth_of_weight = smooth_of_weight/5; }
  if(task_type == 3){smooth_of_weight = smooth_of_weight *1.2;}
  return final_profile;
}



//第四步，生成代价函数为速度尽量快与jerk尽量小的速度，加速度曲线
bool VelocityPlanner::optimizeVelocity
(std::vector<Waypoint2D> &profile,double a_max,double a_min,double current_vel,double current_acc,double &smooth_of_weight,double &traject_length,
int &index_to_stop,double &traject_length_default,double &curv_upper_speed,int &curv_upper_index,double curv_upper_boudary,double desired_speed)
{
  //创建求解器
  osqp::OSQPInterface qp_solver_;
  //求解器设置
  qp_solver_.updateMaxIter(100000);
  qp_solver_.updateRhoInterval(0);  // 0 means automatic
  qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
  qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-8
  qp_solver_.updateVerbose(false);
  //参数设置 最大加速度、速度点个数、平滑权重、初始速度、初始加速度
  long N = static_cast<long>(profile.size());
  // std::cout<<"路径点个数："<<N<<"刹车点:"<<index_to_stop<<std::endl;
  // std::cout << "path_length: " << traject_length << std::endl;
  const long var_size = 2*N;
  const long constraint_size = 3*N + 1;
  const double amax = a_max;
  const double amin = a_min;
  double smooth_weight = smooth_of_weight;//平滑权重，越大越平滑，越小加速越快
  //处理初速度
  if(current_vel>(desired_speed+5))  
  { return false;}
  if((current_vel>=desired_speed) && (current_vel<= (5+desired_speed)))
  {current_vel= desired_speed; }
  double initial_vel = current_vel;
  double initial_acc = current_acc;
// Hessian
  Eigen::MatrixXd hessian(var_size,var_size);
  hessian = Eigen::MatrixXd::Zero(var_size, var_size);
  // hessian = Zero(var_size, var_size);
  for(int i=0; i<N-1; ++i)
  {   //j_presudo = a(i+1)-a(i) / s(i+1)-s(i) 这里的ds就算s(i+1)-s(i)即离散的弧长步长
      double ds = distance2D(profile[i].position,profile[i+1].position);
      hessian(i+N, i+N) += smooth_weight/ds;
      hessian(i+1+N, i+1+N) += smooth_weight/ds;
      hessian(i+N, i+1+N) += -smooth_weight/ds;
      hessian(i+1+N, i+N) += -smooth_weight/ds;
  }

  // Gradient
  std::vector<double> gradient(var_size, 0.0);
  for(long i=0; i<N; ++i)
  {
      const double v_max = std::max(profile.at(i).vel, 0.1);
      //这里除v方是为了将速度尽量快的权重限制在1附近
      gradient[i] = -1.0/(v_max*v_max);
  }

  //上方的Hesiian与gradient矩阵确定了目标函数
  // Constraint Matrix
  Eigen::MatrixXd constraint_matrix(constraint_size,var_size);
  constraint_matrix = Eigen::MatrixXd::Zero(constraint_size, var_size);
  std::vector<double>lowerBound(constraint_size, 0.0);
  std::vector<double>upperBound(constraint_size, 0.0);
  long constraint_num = 0;


  // 1. Velocity Constraint
  for(long i=0; i<N; ++i, ++constraint_num)
  {
      constraint_matrix(constraint_num, i) = 1.0; //b[i]
      lowerBound[constraint_num] = 0.0;
      upperBound[constraint_num] = profile[i].vel * profile[i].vel;
      //冗余
      if ((current_vel > desired_speed) && ( current_vel < (desired_speed + 1))  && ( i <20 ))
      {
         upperBound[constraint_num] = (profile[i].vel + 0.5 ) * (profile[i].vel + 0.5 ) ;
      }

    //如果是停车场景，结尾速度冗余放小
      if (( scenario_num == 1) &&  (i>= index_to_stop ))
      {
        upperBound[constraint_num] = (profile[i].vel + 0.01) * (profile[i].vel + 0.01 ) ;/* code */
        if( current_vel <0.45)
        {upperBound[constraint_num] = 0.41*0.41;}
      }
  }

  // 2. Acceleration Constraint
  for(long i=0; i<N; ++i, ++constraint_num)
  {
      constraint_matrix(constraint_num, i+N) = 1.0; //a[i]
      lowerBound[constraint_num] = amin;
      upperBound[constraint_num] = amax;
  }
  // std::cout<<"---------------pass_acc----------------"<<std::endl;

  //3. Dynamic Constraint
  for(long i=0; i<N-1; ++i, ++constraint_num)
  {
      //v方-v0方等于2a*ds
      double ds = distance2D(profile[i].position,profile[i+1].position);
      constraint_matrix(constraint_num, i) = -1.0; // -b[i]
      constraint_matrix(constraint_num, i+1) = 1.0; // b[i+1]
      constraint_matrix(constraint_num, i+N) = -2.0*ds; // a[i] * -2.0 * ds
      lowerBound[constraint_num] = 0.0;
      upperBound[constraint_num] = 0.0;
  }

  // std::cout<<"---------------pass_dyn------------"<<std::endl;
  //5. Initial Condition
  // std::cout<<"初始速度"<<initial_vel<<std::endl;

  constraint_matrix(constraint_num, 0) = 1.0;//速度初始约束   
  lowerBound[constraint_num] = initial_vel*initial_vel;
  upperBound[constraint_num] = initial_vel*initial_vel;
  ++constraint_num;
  constraint_matrix(constraint_num, N) = 1.0; //加速度初始约束
  lowerBound[constraint_num] = initial_acc;
  upperBound[constraint_num] = initial_acc;
  ++constraint_num;
  assert(constraint_num == constraint_size);
  // std::cout<<"---------------pass_init----------------"<<std::endl;

  const auto result = qp_solver_.optimize(hessian, constraint_matrix, gradient, lowerBound, upperBound);

  const std::vector<double> optval = std::get<0>(result);

  profile.resize(N);
  for(unsigned int i=0; i<N; ++i)
  {
    profile[i].vel = std::sqrt(std::max(optval[i], 0.1));
    
    //防止加速度在0附近抖动、以及预防过界加速度出现
    if (fabs(optval[i+N])<1e-2)
    {
      profile[i].acc = 0.0;
    }
    else if (optval[i+N]>amax)
    {
      profile[i].acc = amax;
    }
    else if (optval[i+N]<amin)
    {
      profile[i].acc = amin;
    }
    else
    {
      profile[i].acc = optval[i+N];
    }

    //速度很小时将速度设置为0,防止速度抖动
      if (profile[i].vel<0.42)
    {
      profile[i].vel =0;
    }
    else if(profile[i].vel > 50)
    {
      return false;
    }
    // std::cout<<"规划速度:"<<final_profile[i].vel<<"---规划加速度:"<<final_profile[i].acc<<std::endl;
  }


  //大弯道的速度给1.5非停车场景
  if(!(scenario_num==1)){
  for(unsigned int j=0; j<N; j++)
  { 
      if(!(curv_upper_index==-1))
      {
        if(  j >  curv_upper_index/curv_upper_boudary)
      {
          profile[j].vel = curv_upper_speed;
          profile[j].acc =0;
      }
      }
  }
  }
 
  //终点停车场景
  if(scenario_num == 1)
  {
      for(unsigned int i=index_to_stop; i<N; i++)
  { 
          profile[i].vel = 0;
          profile[i].acc =0;
  }
  }
  

  for(unsigned int i=0; i<N-1; ++i)
  {
    //反回来求jerk
    double ds = distance2D(profile[i].position,profile[i+1].position);
    double a_current = optval[i+N];
    double a_next    = optval[i+N+1];
    profile[i].jerk = (a_next - a_current) * profile[i].vel / ds;
    profile[i].position = profile[i].position;
  }
  profile[N-1].jerk = profile[N-2].jerk;

  prev_profile_ = profile;

  return true;

}
//*****************************************************************//


bool VelocityPlanner::BackUp(bool qp_result,double a_min,std::vector<Waypoint2D> &profile,std::vector<Waypoint2D> &pre_profile,double LengthChange,double vel)
{
  bool is_success = false;
  double now_length	=getVelLength(profile);
  double pre_length	= getVelLength(pre_profile);
  double lengthchange = pre_length - now_length ;
  // std::cout<<"now_length:"<<now_length<<std::endl;
  // std::cout<<"pre_length:"<<pre_length<<std::endl;
  // std::cout<<"lengthchange:"<<lengthchange<<std::endl;
  is_success = qp_result;
  int size = profile.size();
  int pre_size = pre_profile.size();
  if(!is_success)
  {
    std::cout<<"优化失败，采用之前速度"<<std::endl;
    // std::cout<<"lengthchange:"<<lengthchange<<std::endl;

    if(lengthchange >= LengthChange){
      std::cout<<"路径长度突变，紧急制动"<<std::endl;
      for(int i=0; i <size; i++)
      {   
      profile[i].acc = a_min;          
      if(i==0)
      {
        profile[i].vel = vel;
      }
      else
      {
        double temp  =2*a_min*distance2D(profile[i].position,profile[i-1].position)+profile[i-1].vel*profile[i-1].vel;
        if(temp >=0)
        {
          profile[i].vel = std::sqrt(temp);
        }
        else
        {
          profile[i].vel = 0;
          profile[i].acc = 0;
        }
      }
    } 
    }
    else{
    if(pre_size>= size){
    for(int i=0; i <size; i++){
      profile[i].acc = pre_profile[i].acc;
      profile[i].vel = pre_profile[i].vel;
    }
    }
    else{
    for(int i =0; i<pre_size;i++){
      profile[i].acc = pre_profile[i].acc;
      profile[i].vel = pre_profile[i].vel;
    }
    for(int i = pre_size; i < size ; i++){
      profile[i].acc = pre_profile[pre_size-1].acc;
      profile[i].vel = pre_profile[pre_size-1].vel;
    }

    }
    }
    return is_success;   
   }
  else
  {
    return true;
  }
}

bool VelocityPlanner::Save(std::string output_file,   planning_msgs::TrajectoryPointArray traject)
    {
        std::ofstream outFile;
        //打开文件
        outFile.open(output_file.c_str());

        for (const auto &p : traject.points)
        {
            outFile <<"x"<<p.x << " y" << p.y << " v" << p.v<< " a" << p.a << " s" << p.kappa<<" t"<<p.relative_time << std::endl;
        }
        //关闭文件
        outFile.close();
        return true;
    }


//////////////////////////////////////////////////////////////////////////////////////////
} // namespace planning
