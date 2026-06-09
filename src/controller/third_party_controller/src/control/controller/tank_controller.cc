#include "control/controller/tank_controller.h"
#include "common/util/log.h"
#include "common/math/math_utils.h"
#include "common/time/time.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <utility>
#include <ros/package.h>
#include <ros/ros.h>
#include <sys/stat.h>
//some temporary value for global
bool is_simulation_global;
double v_max;
int jjjj;
double calibration_speed_to_stop_global;
bool is_calibration_global;
double v_start_stop;
bool direct_to_cmd;
double standstill_brake_global;
double calibration_throttle_global;
std::vector<std::vector<double>> data_record;
bool flag_first_time_in_datarecord = true;
double x_00,y_00,dist;
const double kEpsilon = 0.05;//最小速度检测值
bool is_stop_final = 0;
int nearst_index = 1;//for get remain path

namespace car
{ 
  namespace control
  {
    using ::car::common::Status;
    using ::car::common::time::Clock;
    using ::car::common::vehicle_state::VehicleState;
    namespace control_msgs = ::car::control::msgs;
    namespace common_msgs = ::car::common::msgs;
    namespace common_time = ::car::common::time;
    namespace math = ::car::common::math;
    // namespace fs = std::filesystem;

    TankController::TankController() : name_("tank_controller") {}

    void TankController::Stop()
    { 
      // CloseLogFile(); 
    }

    TankController::~TankController()  
    {
      if(is_calibration_global)
      {
        SaveLonTable(calibration_throttle_global);
      }
      SaveData();
    }

    void TankController::SaveLonTable(double throttle)
    {
        std::string recode_data_dir  = ros::package::getPath("launch_node");  /*标定表存储路径*/

        std::string wenjianjia_path = recode_data_dir + std::string("/param/control/carla/lon_table");
        if(mkdir(wenjianjia_path.c_str(),0777)==0)
        {
          std::cout << "lon_table directory mkdir successfully" <<std::endl;
        }
        else
        {
          // std::cout << " /launch_node/param/control/carla/lon_table directory has been create" <<std::endl;
        }

        std::string strTime = std::to_string((int)throttle);
        std::string recode_data_store_file = recode_data_dir 
                              + std::string("/param/control/carla/lon_table/lon_table")+strTime+"_.txt";
        std::ofstream f_lon_data_out;
        
        // f_lon_data_out = std::ofstream(recode_data_store_file,std::ios::app); /*文件写入设置为追加模式*/
        f_lon_data_out = std::ofstream(recode_data_store_file);
        f_lon_data_out<< std::fixed;              /*设置输出精度*/
        f_lon_data_out.precision(4);              /*设置输出精度*/

        for(int i=0;i<lon_table.size();i++)
        {
          for(int j=0;j<3;j++)
          {
            f_lon_data_out <<lon_table[i][j]<<"  ";
          }
          f_lon_data_out << std::endl;
        }

        f_lon_data_out.flush();
	      f_lon_data_out.close();
        std::cout <<"---------ZZ stopRecord Calibration throttle:"<<strTime<<" ------"<<std::endl;
    }

    void TankController::SaveData()
    {
        std::string recode_data_dir  = ros::package::getPath("launch_node");  /*标定表存储路径*/
        std::string wenjianjia_path = recode_data_dir + std::string("/log/zz_data");

        if(mkdir(wenjianjia_path.c_str(),0777)==0)
        {
          std::cout << " zz_data  directory mkdir successfully" <<std::endl;
        }
        else
        {
          // std::cout << " /launch_node/log/zz_data directory has been create" <<std::endl;
        }

        time_t now = time(0);
        tm *now_time = localtime(&now);
        std::string strTime = std::to_string(now_time->tm_year + 1900) 
        + "-" + std::to_string(now_time->tm_mon + 1) 
        + "-" + std::to_string(now_time->tm_mday)
		    + "-" + std::to_string(now_time->tm_hour) 
        + "-" + std::to_string(now_time->tm_min);
        
        std::string recode_data_store_file = recode_data_dir 
                              + std::string("/log/zz_data/")+strTime+"_.txt";
        std::ofstream f_lon_data_out;
        
        // f_lon_data_out = std::ofstream(recode_data_store_file,std::ios::app); /*文件写入设置为追加模式*/
        f_lon_data_out = std::ofstream(recode_data_store_file);
        f_lon_data_out<< std::fixed;              /*设置输出精度*/
        f_lon_data_out.precision(4);              /*设置输出精度*/

        for(int i=0;i<data_record.size();i++)
        {
          for(int j=0;j<6;j++)
          {
            f_lon_data_out <<data_record[i][j]<<"  ";
          }
          f_lon_data_out << std::endl;
        }

        f_lon_data_out.flush();
	      f_lon_data_out.close();
        std::cout <<"---------ZZ stopRecord data.size:"<< data_record.size() <<" ----------------"<<std::endl;
    }

    void TankController::DataRecord(control_msgs::SimpleLongitudinalDebug *debug)
    {
      double x,y;
      dist = 0;
      if(flag_first_time_in_datarecord == true)
      {
        x_00 = vehicle_state_.x();
        y_00 = vehicle_state_.y();
        flag_first_time_in_datarecord = false;
      }
      x = vehicle_state_.x();
      y = vehicle_state_.y();

      double dx = x-x_00;
      double dy = y-y_00;
      dist = std::sqrt(dx * dx + dy * dy);

      if(dist >= 0.2)
      {
        x_00=x;y_00=y;
        data_record.push_back({debug->path_remain,
        debug->preview_speed_reference,
        debug->preview_acceleration_reference,
        debug->preview_speed_error,
        debug->throttle_cmd,
        debug->brake_cmd});
      }
    }

    /*初始化*/
    Status TankController::Init(const ControlConfig *control_conf)
    {
      // 初始化history_speed_
      while (!history_speed_.empty())
      {
        history_speed_.pop_front();
      }
      control_conf_ = control_conf;

      if (control_conf_ == nullptr)
      {
        controller_initialized_ = false;
        AWARN("get longitudinal param nullptr.");
        return Status(common_msgs::CONTROL_INIT_ERROR,"Failed to load TankController conf");
      }
      const LongitudinalControllerConfig &lon_controller_conf = control_conf_->lon_controller_conf();

      speed_pid_controller_.Init(lon_controller_conf.low_speed_pid_conf());
      acc_pid_controller_.Init(lon_controller_conf.acc_pid_conf());

      SetDigitalFilterPitchAngle(lon_controller_conf);

      LoadControlCalibrationTable(lon_controller_conf);

      controller_initialized_ = true;
      return Status::OK();
    }
    
    /*设置俯仰角滤波器*/
    void TankController::SetDigitalFilterPitchAngle(const LongitudinalControllerConfig &lon_controller_conf)
    {
      double cutoff_freq = lon_controller_conf.pitch_angle_filter_conf().cutoff_freq();
      double ts = lon_controller_conf.ts();
      SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
    }

    /*加载标定表*/
    void TankController::LoadControlCalibrationTable(const LongitudinalControllerConfig &lon_controller_conf)
    {
      const auto &calibration_table = lon_controller_conf.calibration_table();
      int control_table_size = calibration_table.size();
      Interpolation1D::DataType xy;

      for (const auto &calibration : calibration_table)
      {
        xy.push_back(std::make_pair(calibration.speed(), calibration.command()));
      }

      control_interpolation_.reset(new Interpolation1D);

      if (!control_interpolation_->Init(xy))
      {
        AERROR("Fail to load control calibration vt table.");
      }
    }

    /*脱困模式*/
    double TankController::EscapeCompensate(double current_speed,double target_speed)
    {
      const auto escape_determine_times = control_conf_->lon_controller_conf().escape_determine_times();
      const auto escape_mean_speed_threhold = control_conf_->lon_controller_conf().escape_mean_speed_threhold();
      const auto escape_stdev_threhold = control_conf_->lon_controller_conf().escape_stdev_threhold();
      const auto escape_compensate = control_conf_->lon_controller_conf().escape_compensate();
      static auto k = 0;

      history_speed_.push_back(current_speed);

      if (history_speed_.size() > escape_determine_times)
      {
        double sum = 0.0;// 求和
        for (auto &q : history_speed_)
        {
          sum += q;
        }

        double mean = sum / history_speed_.size();// 均值
        double accum = 0.0;// 方差

        for (auto &q : history_speed_)
        {
          accum += (q - mean) * (q - mean);
        }
        double stdev = sqrt(accum / (history_speed_.size()));        // 标准差

        history_speed_.pop_front();
        if (mean < escape_mean_speed_threhold && stdev < escape_stdev_threhold && current_speed + 0.2 < target_speed)
        {
          return escape_compensate * (++k);
        }
      }
      k = 0;
      return 0.0;
    }

    /*计算环节，输出油门开度，刹车开度，期望加速度*/
    Status TankController::ComputeControlCommand(
        const control_msgs::LocalizationEstimate *localization,
        const control_msgs::Chassis *chassis,
        const control_msgs::Trajectory *planning_published_trajectory,
                    control_msgs::ControlCommand *cmd)
    {
      const LongitudinalControllerConfig &lon_controller_conf = control_conf_->lon_controller_conf();

      const auto standstill_brake = lon_controller_conf.standstill_brake();
      const auto overspeed_light_brake = lon_controller_conf.overspeed_light_brake();
      const auto off_throttle_acc = lon_controller_conf.off_throttle_acc();
      const auto point_size_to_end = lon_controller_conf.point_size_to_end(); //停车距离
      const auto path_to_stop = lon_controller_conf.path_to_stop();
      const auto is_simulation = lon_controller_conf.is_simulation(); //是否是仿真
      const auto is_calibration = lon_controller_conf.is_calibration();//是否是标定模式
      double calibration_throttle = lon_controller_conf.calibration_throttle();//标定时的固定油门
      double calibration_speed_to_stop = lon_controller_conf.calibration_speed_to_stop();

      //some global value
      is_simulation_global = is_simulation;
      is_calibration_global = is_calibration;
      calibration_speed_to_stop_global = calibration_speed_to_stop;
      standstill_brake_global = standstill_brake;
      calibration_throttle_global = calibration_throttle;

      localization_ = localization;
      chassis_ = chassis;
      trajectory_message_ = planning_published_trajectory;
      vehicle_state_ = std::move(VehicleState(localization, chassis));
      
      cmd->acceleration = 0.0;
      

      if (trajectory_analyzer_ == nullptr || trajectory_analyzer_->seq_num() != trajectory_message_->header.sequence_num)
      {
        trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
      }
      
      /*车辆未进入无人模式不进行控制*/
      if (common_msgs::COMPLETE_AUTO_DRIVE != chassis_->driving_mode && !is_simulation )
      {
          //AINFO("car is in manual+++++++++++++++++++++++");
          cmd->gear_location = chassis_->gear_location;
          cmd->throttle = 0.0;
          cmd->acceleration = 0.0;
          return Status::OK();
      }

      /*接收运动停止指令，下发刹车命令*/
      if(!is_simulation)
      {
      if (common_msgs::MOTION_STOP == chassis_->motion_start_cmd) 
      {
        //AINFO("car receive stop cmd ++++++++++++++++++++++++++++++");
        is_stop_final = 0;
        if (common_msgs::GEAR_PARKING != chassis_->gear_location) 
        {
          if (std::fabs(chassis_->speed_mps) < kEpsilon)
          {
            cmd->gear_location = common_msgs::GEAR_PARKING;
            cmd->acceleration = standstill_brake;/*驻车前速度不为零，给一定的刹车*/
          } 
          else 
          {
            cmd->gear_location = chassis_->gear_location;    
            cmd->acceleration = standstill_brake;/*停车, 该值可以根据实际情况选取*/
          }
          cmd->throttle = 0.0;
          return Status::OK();
        } 
        else 
        {
          // 档位为停车P档
          cmd->gear_location = chassis_->gear_location;
          cmd->throttle = 0.0;
          cmd->acceleration = 0.0;
          return Status::OK();
        }
      } 
      else if (common_msgs::MOTION_START != chassis_->motion_start_cmd) 
      {
        //AINFO("car not start ++++++++++++++++++++++++++++++ speed is %f",chassis_->speed_mps);
          cmd->acceleration = 0.0;
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
          return Status::OK();
      }
      }
      
      /*接收运动开始指令，计算油门和刹车*/
      /*规划没轨迹输出,进行停车处理*/
      if (trajectory_message_->trajectory_point.size() < point_size_to_end)          
      {
        if(std::fabs(chassis_->speed_mps) < kEpsilon)
        {
            cmd->acceleration = standstill_brake;
            cmd->throttle = 0.0;
            cmd->gear_location = common_msgs::GEAR_PARKING;
        } 
        else
        {
          AINFO("dont have trajectory, stop car");
          cmd->acceleration = standstill_brake;
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
        }

        if(chassis_->gear_location == common_msgs::GEAR_NEUTRAL)
        {
          cmd->acceleration = 0.0;
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
        }

        return Status::OK();
      }

      if(is_calibration)
      {
        cmd->throttle = calibration_throttle;
        cmd->acceleration = 0.0; 
        MakePolationThrottle(calibration_throttle); 
        return Status::OK();
      }

      cmd->gear_location = chassis_->gear_location;
      //AINFO("first fuzhi ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ gear is %d",cmd->gear_location);
      if(!is_simulation)
      {
        //AINFO("now is stop final is %d, and stop poing 1 and 2 is %f , ---%f",is_stop_final,trajectory_message_->trajectory_point[1].v,trajectory_message_->trajectory_point[2].v);
      //底盘档位和规划档位不一致,如果速度为0,则直接刹车换档,否则先刹车
      if (chassis_->gear_location != trajectory_message_->gear && (is_stop_final == 0||trajectory_message_->trajectory_point[nearst_index].v>0.5))
      {
        is_stop_final = 0;
        //AINFO("gear is not same  ~~~ gear is %d,but trajectory gear is %d",cmd->gear_location,trajectory_message_->gear);
        if (std::fabs(chassis_->speed_mps) < kEpsilon)
        {
          cmd->gear_location = common_msgs::GEAR_NEUTRAL;
          cmd->acceleration = standstill_brake;
          if(chassis_->gear_location == common_msgs::GEAR_NEUTRAL)
          {
            cmd->gear_location = trajectory_message_->gear;
          }
        }
        else
        {
          cmd->gear_location = chassis_->gear_location;
          cmd->acceleration = standstill_brake;
        }
        cmd->throttle = 0.0;
       // AINFO("before return in trajectory ~~~ gear is %d,and trajectory gear is %d",cmd->gear_location,trajectory_message_->gear);
        return Status::OK();
      }
      }

      if (!control_interpolation_)
      {
        AERROR("Fail to initialize calibration table.");
        cmd->throttle = 0.0;
        cmd->acceleration = standstill_brake;
        return Status(common_msgs::CONTROL_COMPUTE_ERROR,"Fail to initialize calibration table.");
      }

      // if (trajectory_analyzer_ == nullptr || trajectory_analyzer_->seq_num() != trajectory_message_->header.sequence_num)
      // {
      //   trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
      // }

      auto debug = &cmd->debug.simple_lon_debug;
      debug->Clear();

      double brake_cmd = 0.0;
      double brake_dcc = 0.0;
      double throttle_cmd = 0.0;
      double ts = lon_controller_conf.ts();
      double preview_time = lon_controller_conf.preview_window() * ts;
      double speed_controller_input_limited = 0.0;
      double escape_compensate = 0.0;
      double throttle_cmd_closeloop = 0.0;
      double brake_acc = 0.0;
      double brake_acc_all = 0.0;
      double slope_offset_compenstaion = 0.0;

      if(!is_simulation)
      {
          ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, debug,is_simulation);
      }

      GetPathRemain(trajectory_analyzer_.get(),debug);
      //std::cout << debug->path_remain <<std::endl;

      if(is_simulation)
      {
        debug->preview_speed_reference = 5;
        debug->preview_speed_error = debug->preview_speed_reference-std::fabs(vehicle_state_.linear_velocity());
        debug->preview_acceleration_reference = 0.0;

        if(debug->path_remain < path_to_stop) 
        {
          debug->preview_speed_reference = 0.0;
          debug->preview_speed_error = debug->preview_speed_reference-std::fabs(vehicle_state_.linear_velocity());
          debug->preview_acceleration_reference = 0.0;
        }
      }
        /*坡道补偿，惯导驱动输出俯仰角为rad*/
        slope_offset_compenstaion = 9.80 * std::sin(vehicle_state_.pitch());
        if (control_conf_->enable_slope_offset())
        {
          debug->slope_offset_compensation = control_conf_->slope_offset_coefficient() * slope_offset_compenstaion;
        }
        else
        {
          debug->slope_offset_compensation = 0.0;
        }
        //AINFO("debug->slope_offset_compensation-------%f",debug->slope_offset_compensation);
        
        //仿真暂时关闭坡道补偿
        if(is_simulation)
        {
          slope_offset_compenstaion = 0;
        }
      //is ready to stop the vehicle
      if(debug->path_remain < path_to_stop) //1
      {
        direct_to_cmd = false;
        if(v_start_stop==0.0)
        {
          v_start_stop = std::fabs(vehicle_state_.linear_velocity());
        }

        StopVehicle(debug,path_to_stop,cmd);

        if(direct_to_cmd)
        {
          debug->brake_cmd = cmd->acceleration;
          debug->throttle_cmd = cmd->throttle;
          DataRecord(debug);
        AINFO("debug : debug->path_remain < path_to_stop  ~~~ acc is %f",cmd->acceleration);
          return Status::OK();
        }
      }
      
      /*廉老师停车操作  zwy:debug->path_remain剩余距离为路径距离*/
      // static bool lastbrake = false;

      // if (debug->path_remain < 20.0)
      // {
      //   AWARN("Ready to Stop.");
      //   std::cout <<"dist "<<debug->path_remain<<std::endl;
      //   cmd->throttle = 5;
      //   cmd->acceleration = 0.0;     

		  //   if((debug->path_remain < 0.5) || lastbrake)
      //   {
      //     cmd->acceleration = -2.0;
      //     cmd->throttle = 0.0;
      //     cmd->gear_location = common_msgs::GEAR_PARKING;
      //     AWARN("Already Stop.");
      //     std::cout <<"last brake "<<lastbrake<<std::endl;
      //     lastbrake = true;
      //     return Status::OK();  
      //   }

      //   if (debug->path_remain < 5.0 && chassis_->gear_location != common_msgs::GEAR_REVERSE)
      //   {
      //     std::cout <<"last 5m use for forward move"<<std::endl;
      //     cmd->acceleration = 0.0;
      //     cmd->throttle = 0.0; 
      //     return Status::OK(); 
      //   }
      //   return Status::OK();      
      // }
      // else
      // {
      //   lastbrake = false;
      // }

      double max_speed_error_for_brake = lon_controller_conf.max_speed_error_for_brake();

      if (common_msgs::GEAR_REVERSE == chassis_->gear_location) 
      {
        max_speed_error_for_brake = lon_controller_conf.max_reverse_speed_error_for_brake();
      }

      bool is_throttle = false;
      
      if (max_speed_error_for_brake < debug->preview_speed_error)
      {
        is_throttle = true;

        /*限制速度误差上限 ，避免发散*/
        double speed_controller_input_limit = lon_controller_conf.speed_controller_input_limit();
        speed_controller_input_limited 
        = math::Clamp(debug->preview_speed_error,-speed_controller_input_limit,speed_controller_input_limit);

        /*三组PID参数：前进低速，前进高速，倒档*/
        if (std::fabs(vehicle_state_.linear_velocity()) <= lon_controller_conf.switch_speed() && chassis_->gear_location != common_msgs::GEAR_REVERSE )
        {
            speed_pid_controller_.SetPID(lon_controller_conf.low_speed_pid_conf());
            throttle_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited, ts);
        }
        else if(std::fabs(vehicle_state_.linear_velocity()) > lon_controller_conf.switch_speed() && chassis_->gear_location != common_msgs::GEAR_REVERSE)
        {
          speed_pid_controller_.SetPID(lon_controller_conf.high_speed_pid_conf());
          throttle_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited, ts);
        }
        //else if(debug->preview_speed_reference < 0.0)        
        else if(chassis_->gear_location == common_msgs::GEAR_REVERSE)
        {
          speed_pid_controller_.SetPID(lon_controller_conf.back_pid_conf());
          throttle_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited, ts);
        }
        /*脱困模式*/
        if (lon_controller_conf.enable_escape_mode())
        {
          escape_compensate = EscapeCompensate(vehicle_state_.linear_velocity(),debug->preview_speed_reference);
        }

        /*基础油门，插表确定*/
        debug->calibration_value = control_interpolation_->Interpolate(debug->preview_speed_reference);
        
        /*油门汇总*/
        // std::cout <<"当前车辆档位："<< chassis_->gear_location << std::endl;
        if(chassis_->gear_location == common_msgs::GEAR_REVERSE)//1.前进下坡，前进上坡；2.倒车上坡，倒车下坡。
        {
          throttle_cmd = throttle_cmd_closeloop + debug->calibration_value - debug->slope_offset_compensation + escape_compensate;
        }
        else
        {
          // std::cout <<"计算油门："<< throttle_cmd << std::endl;
          throttle_cmd = throttle_cmd_closeloop + debug->calibration_value + debug->slope_offset_compensation + escape_compensate;
          //throttle_cmd = throttle_cmd<5? 5:throttle_cmd;
        }
        acc_pid_controller_.Reset();
      }
      else //超过期望速度一定冗余时
      {

        while (!history_speed_.empty())
        {
          history_speed_.pop_front();
        }
        acc_pid_controller_.SetPID(lon_controller_conf.acc_pid_conf());

        brake_acc = acc_pid_controller_.Control(debug->preview_speed_error, ts);
        //AINFO("-------------going to break,break is %f------------",brake_acc);
        /*期望刹车减速度总和：包含坡道加速度，期望加速度，长时间超速通过PID调节的加速度*/
        if(chassis_->gear_location == common_msgs::GEAR_REVERSE)//（1）前进下坡，前进上坡；（2）倒车上坡，倒车下坡
        {
          brake_acc_all = -slope_offset_compenstaion + debug->preview_acceleration_reference + brake_acc;
        }
        else
        {
          brake_acc_all = slope_offset_compenstaion + debug->preview_acceleration_reference + brake_acc;
        }
        /*超速时.刹车降速*/
        if (0.0 <= brake_acc_all)
        {
          // brake_cmd = 0;
          brake_dcc = 0.0;
          is_throttle = false;//zwy 构想再增加一组pid参数，用于超速时使用。
        }
        else
        {
          /*高速时才利用发动机制动，当到达怠速车速以下时，发动机制动不起效果，必须刹车介入*/
          if(std::fabs(vehicle_state_.linear_velocity()) > 0)
          {
            if (brake_acc_all > off_throttle_acc)
            {
              // brake_cmd = 0;
              brake_dcc = 0.0;
              throttle_cmd = throttle_cmd<5? 5:throttle_cmd-5;
              is_throttle = false;
            }
            else   
            {
              brake_dcc = brake_acc_all;
              is_throttle = false;
            }
          }
          else
          {
              brake_dcc = brake_acc_all;
              is_throttle = false;
          }
        }
        speed_pid_controller_.Reset();
      }

      /*设置油门与刹车发送区间，避免给底盘发送错误数据*/
      double throttle_deadzone = lon_controller_conf.throttle_deadzone();
      double brake_deadzone = lon_controller_conf.brake_deadzone();
      
      if (is_throttle)
      {
        brake_dcc = 0.0;
      }
      else
      {
        throttle_cmd = 0.0;
      }

      debug->station_error_limited = debug->path_remain;
      debug->speed_controller_input_limited = speed_controller_input_limited;
      debug->throttle_cmd = throttle_cmd;
      debug->brake_cmd = brake_cmd;
      debug->acceleration_cmd = brake_dcc;
      double timestamp = common_time::ToSecond(Clock::Now());
      //AINFO("-------------val break all is %f,break dcc is %f------------",brake_acc_all,brake_dcc);

      cmd->throttle = throttle_cmd;
      cmd->acceleration = brake_dcc;
       // AINFO("at last ~~~~~~~~~~~~~ gear is %d",cmd->gear_location);
      //std::cout << "--------------------------------------------------"  << std::endl;
      DataRecord(debug);

      return Status::OK();
    }
    void TankController::GetPathRemain(const TrajectoryAnalyzer *trajectory_analyzer,control_msgs::SimpleLongitudinalDebug *debug)
    {
      //从最后一个点开始寻找
      int stop_index = trajectory_message_->trajectory_point.size() - 1;
      auto nearest_point = trajectory_analyzer->QueryNearestPointByPosition(vehicle_state_.x(), vehicle_state_.y());
      for(int i=0;i<trajectory_message_->trajectory_point.size();i++)
      {
        if(nearest_point.path_point.x==trajectory_message_->trajectory_point[i].path_point.x &&
        nearest_point.path_point.y==trajectory_message_->trajectory_point[i].path_point.y)
        {
          nearst_index = i;
          break;
        }
      }
      /*while (stop_index > 0 && fabs(trajectory_message_->trajectory_point[stop_index].v) < 0.5 &&
            trajectory_message_->trajectory_point[stop_index].a <= 0.03 && !is_simulation_global)
      {
        --stop_index;
      }
      stop_index = stop_index == trajectory_message_->trajectory_point.size() - 1? stop_index:stop_index+1;*/
      //AINFO("stop_index _________________ %d",stop_index);
      debug->path_remain = trajectory_message_->trajectory_point[stop_index].path_point.s-trajectory_message_->trajectory_point[nearst_index].path_point.s;
	  if(debug->path_remain < 0) debug->path_remain = - debug->path_remain;
	  //AINFO("path_remain _________________ %f",debug->path_remain);
      //AINFO("first point v_________ %f,a_____________%f",trajectory_message_->trajectory_point[stop_index-1].v,trajectory_message_->trajectory_point[stop_index-1].a);
    }
    // void TankController::GetPathRemain(const TrajectoryAnalyzer *trajectory_analyzer,control_msgs::SimpleLongitudinalDebug *debug)
    // {
    //   //从DI一个点开始寻找
    //   int stop_index = 1;
    //   auto nearest_point = trajectory_analyzer->QueryNearestPointByPosition(vehicle_state_.x(), vehicle_state_.y());
    //   for(int i=0;i<trajectory_message_->trajectory_point.size();i++)
    //   {
    //     if(nearest_point.path_point.x==trajectory_message_->trajectory_point[i].path_point.x &&
    //     nearest_point.path_point.y==trajectory_message_->trajectory_point[i].path_point.y)
    //     {
    //       stop_index = i+1;
    //       nearst_index = i;
    //       break;
    //     }
    //   }
    //   AINFO("nearest_point index _________________ %d",stop_index);
    //   while (stop_index < trajectory_message_->trajectory_point.size()-1 && fabs(trajectory_message_->trajectory_point[stop_index].v) > 0.5 &&
    //         trajectory_message_->trajectory_point[stop_index].a >= 0 && !is_simulation_global)
    //   {
    //     ++stop_index;        debug->preview_speed_reference = std::max(1.5,v_start_stop * debug->path_remain/l);
    //   }
    //   AINFO("stop_index _________________ %d",stop_index);
    //   debug->path_remain = trajectory_message_->trajectory_point[stop_index].path_point.s-trajectory_message_->trajectory_point[nearst_index].path_point.s;
    //   AINFO("path_remain _________________ %f",debug->path_remain);
    //   //AINFO("first point v_________ %f,a_____________%f",trajectory_message_->trajectory_point[stop_index-1].v,trajectory_message_->trajectory_point[stop_index-1].a);
    // }
    Status TankController::Reset()
    {
      speed_pid_controller_.Reset();
      return Status::OK();
    }

    std::string TankController::Name() const { return name_; }

    void TankController::ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,control_msgs::SimpleLongitudinalDebug *debug,const bool sign)
    {
      auto nearest_point = trajectory_analyzer->QueryNearestPointByPosition(vehicle_state_.x(), vehicle_state_.y());

      double current_control_time = common_time::ToSecond(Clock::Now());
      double preview_control_time = current_control_time + preview_time;
      auto preview_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(preview_control_time);
      //AINFO("preview_control_time-------------------v  %f",preview_control_time);

      debug->speed_error = nearest_point.v - vehicle_state_.linear_velocity();
      debug->speed_reference = nearest_point.v;
      debug->acceleration_reference = nearest_point.a;

      // debug->preview_speed_error = preview_point.v - vehicle_state_.linear_velocity();
      debug->preview_speed_reference = preview_point.v;
      debug->preview_speed_error = std::fabs(preview_point.v) - std::fabs(vehicle_state_.linear_velocity());
      debug->preview_acceleration_reference = preview_point.a;
     // AINFO("preview_point-------------------v  %f",preview_point.v);
    }

    void TankController::SetDigitalFilter(double ts, double cutoff_freq,DigitalFilter *digital_filter)
    {
      std::vector<double> denominators;
      std::vector<double> numerators;
      LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
      digital_filter->set_coefficients(denominators, numerators);
    }


    void TankController::StopVehicle(control_msgs::SimpleLongitudinalDebug *debug,double l,control_msgs::ControlCommand *cmd)
    {
      if(debug->preview_speed_reference < 1.5)
      {
        if(debug->path_remain > 5.0){
          debug->preview_speed_reference = std::max(2.0,v_start_stop * (debug->path_remain-5)/(l-5));
        }
        else{
          debug->preview_speed_reference = std::max(0.8,v_start_stop * (debug->path_remain-1.5)/(l-1.5));
        }
        debug->preview_speed_error = std::fabs(debug->preview_speed_reference)-std::fabs(vehicle_state_.linear_velocity());
      }
      AINFO("going to stop----------debug->preview_speed_reference %f",debug->preview_speed_reference);
      double vel_cur = std::fabs(vehicle_state_.linear_velocity()) < kEpsilon? 0:vehicle_state_.linear_velocity();
      // if(debug->path_remain <= vel_cur*vel_cur/10+0.5)//与v相关的系数
       if(debug->path_remain <= 1)
       {
        direct_to_cmd = true;
        bool is_up = debug->slope_offset_compensation>0?1:0;//-为下坡
        if(std::fabs(vehicle_state_.linear_velocity()) < kEpsilon)
        {
          cmd->acceleration = -standstill_brake_global;//-0.9
          cmd->throttle = 0.0;
          cmd->gear_location = common_msgs::GEAR_PARKING;
          is_stop_final = 1;
          
        }
        else if(is_up)
        {
          cmd->acceleration = -0.2*(vel_cur+2)-0.05*debug->slope_offset_compensation;//经验得出
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
        }
        else
        {
          cmd->acceleration = -0.2*(vel_cur+2)+0.05*debug->slope_offset_compensation;//经验得出
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
        }

        if(chassis_->gear_location == common_msgs::GEAR_PARKING)
        {
          cmd->acceleration = 0.0;
          cmd->throttle = 0.0;
          cmd->gear_location = chassis_->gear_location;
          is_stop_final = 1;
        }
      }
      //AINFO("stop vehicle acc is %f -----------------",cmd->acceleration);
    }

    /*-----------------------------------------纵向动力学标定表初始化---------------------------------------------*/
    void TankController::SetpolationThrottle(const TrajectoryAnalyzer *trajectory_analyzer, const double preview_time,control_msgs::SimpleLongitudinalDebug *debug)
    {  
      for(auto &throttle_now:throttle_list)     /*循环标定表文件名*/
      {
        Interpolation1D::DataType va;
        lon_interpolation_.reset(new Interpolation1D);

        //假设传进来的是个pid计算出来的a,还有当前车速v
        std::string recode_data_dir  = ros::package::getPath("launch_node");
        std::string strTime = std::to_string((int)throttle_now);
        std::string recode_data_store_file = recode_data_dir + std::string("/param/control/carla/lon_table/lon_table_")+strTime+"_.txt";
        
        if(ReadLonTableTxt(recode_data_store_file))
        {
          for (int i = 0;i<lon_txt.size();i++)
          {
            va.push_back(std::make_pair(lon_txt[i][2], lon_txt[i][1]));
          }
          lon_txt.clear();
        }

        if (!lon_interpolation_->Init(va))
        {
          AERROR("Fail to load lon table.");
        }

        lon_interpolation_list.push_back(lon_interpolation_);
      }
    }

    /*---------------------------------------插值计算---------------------------------------------*/
    //输入:当前车速，当前期望加速度   
    //输出:油门开度
    double TankController::InterpolationThrottle(double v,double a)
    {
      //throttle_duiyingde为lon_interpolation_list[0]对应的油门开度
      for(auto &t:throttle_list)
      {
        throttle_acc.push_back({t,lon_interpolation_list[0]->Interpolate(v)});
      }

      Interpolation1D::DataType ta;
      throttle_interpolation_.reset(new Interpolation1D);

      for (int i = 0;i<throttle_acc.size();i++)
      {
        ta.push_back(std::make_pair(throttle_acc[i][1], throttle_acc[i][0]));
      }
      throttle_interpolation_->Init(ta);
      return throttle_interpolation_->Interpolate(a);
    }

    /*-------------------------------读取不同油门开度的纵向标定表txt--------------------------------*/
    bool TankController::ReadLonTableTxt(std::string fileName)
    {
      std::ifstream filename(fileName);      
      if (!filename)
      {
        std::cout <<"LonTable file open error: "<<fileName <<std::endl;
        return false;
      }
      std::string oneLine;
	    getline(filename,oneLine);
      while(getline(filename,oneLine))
      { 
        std::istringstream streamOneLine(oneLine);
        std::string ignore;	
        double throttle,a,v;	
        streamOneLine >>throttle;
        streamOneLine >>a;
        streamOneLine >>v;
        lon_txt.push_back({throttle,a,v});
      }
      filename.close();
      std::cout << lon_txt.size()<<std::endl;
      return true;
    }

    /*---------------------------------------制作标定表START-----------------------------------------*/
    void TankController::MakePolationThrottle(double throttle)
    {
      ros::Time end_time; 
      ros::Duration time_inter;
      double v_2;
      double a_time = 0.21;

      if(vehicle_state_.linear_velocity()>=0.2 && cal_sign ) //记录车辆刚动起来时的时间和速度   
      {
        start_time = ros::Time::now();          /*记录起步所对应的时间*/
        v_1 = vehicle_state_.linear_velocity(); /*记录起步速度*/
        cal_sign = false;
      }

      if(vehicle_state_.linear_velocity()>=0.39)//确保车辆已经动起来
      {
        while(vehicle_state_.linear_velocity() - v_1 >= 0.2)  /*0.2 用于控制标定精度*/
        {
          end_time = ros::Time::now();  
          v_2 = vehicle_state_.linear_velocity();
          time_inter = end_time - start_time;
          lon_table.push_back({throttle,(v_2-v_1)/(time_inter.toSec()),(v_2+v_1)/2.0});
          a_time = (v_2-v_1)/time_inter.toSec();
          start_time = end_time;
          v_1 = v_2 ;
        }
      }

      // if(vehicle_state_.linear_velocity() <= v_max && vehicle_state_.linear_velocity() >= 1.0)
      // {
      //   jjjj++;
      // }
      // else
      // {
      //   jjjj=0;
      // }

      v_max = std::max(v_max,vehicle_state_.linear_velocity());

      // std::cout << jjjj <<"    "<< calibration_speed_to_stop_global << "   "<< a_time<<std::endl;

      //when forget close the paragram,auto save lon_tabale
      if(vehicle_state_.linear_velocity()>= calibration_speed_to_stop_global && save_sign ||( a_time <= 0.2 && save_sign)) 
      {
          SaveLonTable(throttle);
      }

      return;
    } 
    /*--------------------------------制作标定表END------------------------------------*/

  } // namespace control
} // namespace car
