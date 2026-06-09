
	单独跑速度规划：
	launch_node里的debug改为1
	单独起velocity_planner_node
	数据在launch_node/data/velocioty_result中


	
	参数说明：
		  //曲率下限(小于这个数认为直线)、曲率因子(大于0小于1、越小过弯速度越小)、曲率上限(大于这个数认为弯太急)、曲率上限速度(在大弯的速度)
		  double curv_limit;
		  double curv_speed_weight;
		  double curv_upper;
		  double curv_upper_speed;
		  //车辆信息 车长、车宽、视界系数(0到0.1之间，越大考虑越宽范围的障碍物)
		  double car_length;
		  double car_width;
		  double vision_weight;
		  //速度信息 巡航速度、会车速度、最大加速度、减速度 
		  double cruise_speed;
		  double meet_speed;
		  double acc_max;
		  double acc_min;
		  //安全距离 刹车、跟车、会车安全距离
		  double safe_brake_dis;
		  double safe_follow_dis;
		  double safe_meet_dis;
		  //权重系数 巡航、刹车、跟车、会车 (权重系数不要低于50)  自车车速权重、障碍物车速权重(大于0小于1，越大越提前刹车)
		  double weight_cruise;
		  double weight_brake;
		  double weight_follow;
		  double weight_meet;
		  double weight_car_speed;
		  double weight_obs_speed;
		 
	调用速度规划节点：
	编译速度规划包，起速度规划节点
	发布的的是planning_msgs::TrajectoryPointArray，话题名是velocity_planner/trajectory_final
	
