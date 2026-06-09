#include "steering_offset_estimator/SteeringOffsetEstimator.h"

SteeringOffsetEstimator::SteeringOffsetEstimator(ros::NodeHandle &nh):nh_(nh),private_nh_("~")
{
	private_nh_.param<int>("average_num", estimatorInfo.average_num,1000);
	private_nh_.param<double>("vel_thres", estimatorInfo.vel_thres,8.0);
	private_nh_.param<double>("steer_thres", estimatorInfo.steer_thres,0.05);
	private_nh_.param<double>("offset_limit", estimatorInfo.offset_limit,0.02);

	vehcileInfo = vehicle_info_util::VehicleInfoUtil::get_instance();
	vehcileInfo->loadVehicleingParam(private_nh_);
	estimatorInfo.wheelbase = vehcileInfo->wheel_base_m;

	localization_sub_ = nh_.subscribe("odomData", 1, &SteeringOffsetEstimator::callbackLocalization, this);
	chassis_sub_ = nh_.subscribe("/chassis", 1, &SteeringOffsetEstimator::callbackChassis, this); 

	steering_offset_estimator_pub_=nh_.advertise<std_msgs::Float32>("/steering_offset_estimator",1);
	
	/*方法1：由航向角推算*/
	timer_localization_ = nh_.createTimer(ros::Duration(0.1),&SteeringOffsetEstimator::callbackTimerLocalizationFeedback,this);

	new_localization_flag = false;
	new_chassis_flag = false;
	first_flag = false;

	udp_thread_ = std::thread (&SteeringOffsetEstimator::udpThreadFunc,this);
	
	udp_thread_.detach();
}


void SteeringOffsetEstimator::udpThreadFunc()
{
	while (true)
	{
		if(new_localization_flag && new_chassis_flag)
		{
			new_localization_flag = false;
			new_chassis_flag = false;
			updateOffset();
			limited_front_wheel_angle_deviation = getOffset();
			//std::cout <<"限制后的解算值:"<<limited_front_wheel_angle_deviation<<std::endl;
			steering_offset_estimator = vehcileInfo->wheelToSteer(limited_front_wheel_angle_deviation)*180/M_PI;
			if (front_wheel_angle_deviation_storage.size() > 0) 
			{
				std_msgs::Float32 steering_offset_estimator_;
				steering_offset_estimator_.data = steering_offset_estimator;
				steering_offset_estimator_pub_.publish(steering_offset_estimator_);
			}
			//std::cout <<"------------------方向盘中位估计结果-----------:"<<steering_offset_estimator<<std::endl;
		}
	}
}

void SteeringOffsetEstimator::callbackTimerLocalizationFeedback(const ros::TimerEvent &event)
{
	if(first_flag)
	{
		twistInfo.angular_z = (heading-heading_)/0.5;
	}
	//std::cout <<"解算的角速度:"<<twistInfo.angular_z<<std::endl;
	heading_ = heading;
	first_flag = true;
	new_localization_flag = true;
}


void SteeringOffsetEstimator::callbackLocalization(const localization_msgs::Localization::ConstPtr &msg)
{
	heading = amathutils::normalizeRadian(amathutils::getPoseYawAngle(msg->location.pose.pose)+ M_PI/2);
	//std::cout <<"获取的航向角:"<<heading<<std::endl;
	/*方法2：直接获取数据*/
	//twistInfo.angular_z = msg->original_ins.angular_z;
	
}

void SteeringOffsetEstimator::callbackChassis(const driver_msgs::ChassisReport::ConstPtr & msg)
{
	twistInfo.linear_x = msg->current_velocity;
	twistInfo.front_wheel_angle =  vehcileInfo->steerToWheel(msg->steering_wheel_angle / 180.0 * M_PI);;
	//std::cout <<"获取的速度:"<<twistInfo.linear_x<<std::endl;
	//std::cout <<"获取的前轮转角:"<<twistInfo.front_wheel_angle<<std::endl;
    new_chassis_flag = true;
}

void SteeringOffsetEstimator::updateOffset()
{
	const bool update_offset =
			(std::abs(twistInfo.linear_x) > estimatorInfo.vel_thres &&
			 std::abs(twistInfo.front_wheel_angle) <estimatorInfo.steer_thres);
	if (!update_offset) return;
	
	const auto calculated_front_wheel_angle = std::atan2(twistInfo.angular_z * estimatorInfo.wheelbase, twistInfo.linear_x);
	const auto deviation = twistInfo.front_wheel_angle - calculated_front_wheel_angle;
	front_wheel_angle_deviation_storage.push_back(deviation);
	
	if (front_wheel_angle_deviation_storage.size() > estimatorInfo.average_num) 
	{
		front_wheel_angle_deviation_storage.pop_front();
	}
	//std::cout <<"队列存储数目:"<<front_wheel_angle_deviation_storage.size()<<std::endl;
	front_wheel_angle_deviation =
		std::accumulate(std::begin(front_wheel_angle_deviation_storage), std::end(front_wheel_angle_deviation_storage), 0.0) /
		std::size(front_wheel_angle_deviation_storage);
	//std::cout <<"未加限制的解算值："<<front_wheel_angle_deviation<<std::endl;
}

float64 SteeringOffsetEstimator::getOffset() const
{
  return std::clamp(front_wheel_angle_deviation, -estimatorInfo.offset_limit, estimatorInfo.offset_limit);
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"steering_offset_estimator_node");
	ros::NodeHandle nh;
	SteeringOffsetEstimator SteeringOffsetEstimator_(nh);
	ros::spin();
	return 0;
}






