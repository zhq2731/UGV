#pragma once

namespace common{
	
enum ErrorCode
{       
	// 0x60xx、0x70xx、0x80xx为硬件错误，其余为软件错误
    NO_ERROR                          = 0X0000,
	//参考线
	REF_OPEN_TRAJECTORY_ERR           = 0X0101,  //打开轨迹文件失败
	REF_SMOOTH_ERR                    = 0X0102,  //平滑失败
									    
	//局部路径规划                      
	LOCPLANNER_ADD_OBS_ERR            = 0X0201,  //添加障碍物到参考线失败
	LOCPLANNER_PATHBOUND_DECIDER_ERR  = 0X0202,  //开辟解空间失败
	LOCPLANNER_INIT_BOUND_ERR         = 0X0203,  //初始化解空间失败
	LOCPLANNER_ROUGH_BOUND_ERR        = 0X0204,  //以道路边界生成解空间失败
									    
	//开放空间规划                     
	OPENSPACE_NOGRID_ERR              = 0x0301,  //无栅格地图
	OPENSPACE_NOSTART_ERR             = 0x0302,  //无规划起始点
	OPENSPACE_NOEND_ERR               = 0x0303,  //无规划终点
	OPENSPACE_PLANNED_ERR             = 0x0304,  //混合a星规划失败
									    
	//全局路径规划                      
	ROUTE_NODE_NOT_EXSIT_ERR          = 0X0401,  //起点或者终点的节点不存在
	ROUTE_NO_PATH_ERR                 = 0X0402,  //规划失败
	ROUTE_NODE_BEYOND_GRAPH_ERR       = 0X0403,  //节点超出拓扑图
									    
	//速度规划                         
	VELOCITY_OPT_ERR                  = 0X0500,  //优化失
									    
	//地图                            
	MAP_READ_OSM_ERR                  = 0X0601,  //地图加载错误
    MAP_LOAD_CFG_ERR                  = 0X0602,  //地图参数文件打开错误
									    
	//云控交互                         
	ROMOTE_OPT_FILE_ERR               = 0x0701,  //找不到解析文件
	ROMOTE_OPT_NOMATCH_ERROR          = 0x0702,  //报文标识与协议不对应
									    
	//横向控制输入数据错误             
	LAT_INPUT_ERR                     = 0x0800,  //横向控制输入数据错误	
	LAT_COMPUTE_ERR                   = 0x0801,  //MPC计算失败	            
	LAT_BICYCLE_MODEL_ERR             = 0x0802,  //自行车模型初始化失败	
	LAT_INIT_OSQP_ERR                 = 0x0803,  //osqp初始化失败	        
	LAT_RECV_INS_ERR                  = 0x0804,  //车辆惯导数据接收错误	
	LAT_RECV_STEER_ERR                = 0x0805,  //车轮转角数据接收错误	
	LAT_TRAJECTORY_ERR                = 0x0806,  //期望路径数据错误	
									    
	//纵向控制:                        
	LON_NO_CFGFILE_ERR                = 0x0901,  //找不到配置文件
	LON_CALBRATE_TAB_ERR              = 0x0902,  //加载标定表失败
									    
	//底盘驱动:                                
	CHASSIS_NO_CFGFILE_ERROR          = 0x0B01,  //找不到配置文件
	CHASSIS_DECODE_DBC_ERROR          = 0x0B02,  //dbc解析失败
	CHASSIS_NODATA_ERR                = 0x7001,  //连续10个周期接收不到底盘数据
									    
	//组合导航                         
	INS_OPEN_DBC_ERR                  = 0x0C01,  //文件打开失败	
	INS_DECODE_DBC_ERR                = 0x0C02,  //DBC文件解析失败	    
	INS_OPEN_UART_ERR                 = 0x0C03,  //打开串口失败	    
	INS_SET_UART_ERR                  = 0x0C04,  //设置文件状态失败	
	INS_INIT_UART_ERR                 = 0x0C05,  //串口初始化失败	    
									    
	INS_RECV_CAN_ERR                  = 0x8001,  //CAN消息接收失败	
	INS_LOC_STATE_ERR                 = 0x8002,  //INS定位状态错误
    //安全模块	
	SAFE_INIT_ERR                     = 0X0E01,   //初始化失败									    
	SAFE_MODEL_SHIFT_ERR              = 0X0E02,   //模式切换失败	
	SAFE_CLOUD_ERR                    = 0X0E03,   //订阅云控心跳失败
	SAFE_CHASSIS_ERR                  = 0X0E04,    //订阅底盘心跳失败
	SAFE_LIMIT_ERR                    = 0X0E05,	  //订阅智能限速失败    
	
	//感知相关                  
	PERCEPTION_NOGROUND_DATA_ERR      = 0X2000,  //地面点云分割结果为空
	PERCEPTION_NOOBS_DATA_ERR         = 0X2001,  //障碍物检测结果为空
									    
	PERCEPTION_NOCAM_DATA_ERR         = 0X6000,  //无视频流数据
	PERCEPTION_NOLIDAR_DATA_ERR       = 0X6001,  //无激光雷达点云数据包
	PERCEPTION_NORADAR_DATA_ERR       = 0X6002,  //无毫米波点云数据包
									    
	//SLAM故障码                       
	SLAM_LOAD_MAP_ERR                 = 0X4000,  //地图加载失败
	SLAM_INIT_POS_ERR                 = 0X4002,  //初始位置查找失败
	SLAM_RELOCATION_ERR               = 0X4004,  //重定位位姿估计失败
	SLAM_LIDAR_ODOM_NOPOINTS_ERR      = 0X4006,  //激光雷达里程计定位无点云失败
	SLAM_VISION_ODOM_NOIMG_ERR        = 0X4008,  //视觉里程计无图像失败
	SLAM_INVALID_ERR                  = 0X400A,  //组合导航失效   
	SLAM_BACKEND_COUPLED_ERR          = 0X400C   //后端耦合失效  
};                                 
} 