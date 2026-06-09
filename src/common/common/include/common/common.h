#pragma once

#include <string>
#include <stdlib.h>
#include <sys/types.h>
#include <pwd.h>
#include "yaml-cpp/yaml.h"
#include <unistd.h> 
#include <map>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>


namespace common{
	struct VehicleIpParam
	{
		std::string ip;
		std::string port;
	};

	struct PlatformParam
	{
		std::string vehicle_type;// "zhitong"  # 车辆类型  geometry_c   sanzhou tank500 zhito zw zhitong
		std::string ins_type;// "cgi610" # cgi610 cgi1010 ht33	570d shengda
		std::string id; // "vehicle_1"	# vehicle_1 vehicle_2...
		unsigned char  num; // # 1 2 3 4...
	};

    struct IpParam
	{
		std::string local_ip; // 本机IP
		int         local_port;
		std::string cloud_single_ip;
		int         cloud_single_port;
		std::string cloud_multi_ip;
		int         cloud_multi_port ;
		std::string info_multi_ip;
		int         info_multi_port;
		std::string rtk_ip;
		int         rtk_port;
	};
	
	enum  IP_Type {
	   CLOUD_SINGLE  = 0,  //
	   CLOUD_MULTI = 1,	 //
	   INFO_MULTI = 2,  //
	};
	bool  getHomeDir(std::string &homeDir);
	bool  getPlatformParam(std::string fileName,PlatformParam &param);
	bool  getPlatformIp(PlatformParam param, VehicleIpParam &Ipparam);//查询单车Ip，传入PlatformParam对象，将本车ip/port赋值在Ipparam
	bool  loadVehicleIp(std::map<std::string, VehicleIpParam> &getVehiclesIp);//查询所有车辆ip，赋值在map，getVehiclesIp中
	std::vector<double> getArrayParam(ros::NodeHandle		 &private_nh,std::string paramName,size_t size);
	std::vector<std::string> getStrArrayParam(ros::NodeHandle		 &private_nh ,std::string paramName,size_t size);
	IpParam  getCloudIp(std::string &cloudIpFile,IP_Type ipType);
	IpParam  getVehicleIp(std::string &vehicle_ip_file,PlatformParam &param);

    
}  // namespace 
