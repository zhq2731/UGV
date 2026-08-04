
#include "common.h"

namespace common{


bool  getHomeDir(std::string &homeDir)
{
	const char *homedir;
	
	if ((homedir = getenv("HOME")) == NULL) {
		homedir = getpwuid(getuid())->pw_dir;
	}
	homeDir = std::string(homedir);
	return true;
}


bool  getPlatformParam(std::string fileName,PlatformParam &param)
{
	std::string homedir ;
	getHomeDir(homedir);
	// 历史配置传入相对HOME的路径；roslaunch的$(find ...)会生成绝对路径。
	// 同时支持两者，车型档案即可保存在工作空间内而无需复制到HOME固定目录。
	std::string filePath = fileName;
	if (fileName.empty() || fileName.front() != '/') {
		filePath = homedir + std::string("/") + fileName;
	}
	YAML::Node platform_config;
	platform_config = YAML::LoadFile(filePath);
	
	param.vehicle_type	 = platform_config["vehicle_type"].as<std::string>();
	param.ins_type	= platform_config["ins_type"].as<std::string>();
	param.id  = platform_config["id"].as<std::string>();
	param.num  = static_cast<unsigned char>(platform_config["num"].as<int>());	
    return true;
}

IpParam  getCloudIp(std::string &cloudIpFile,IP_Type ipType)
{
	YAML::Node doc = YAML::LoadFile(cloudIpFile);
	std::string ipName;
    if (CLOUD_SINGLE == ipType)
		ipName = std::string("cloud_single");
    if (CLOUD_MULTI == ipType)
		ipName = std::string("cloud_multi");
    if (INFO_MULTI == ipType)
		ipName = std::string("info_multi");
	
	IpParam ipParam;
	const YAML::Node &info_multi = doc[ipName];
	ipParam.local_ip	 = info_multi["local_ip"].as<std::string>();
	ipParam.local_port	= info_multi["local_port"].as<int>();
    return ipParam;
}


IpParam  getVehicleIp(std::string &vehicle_ip_file,PlatformParam &param)
{
	YAML::Node doc = YAML::LoadFile(vehicle_ip_file);
	IpParam ipParam;
	const YAML::Node &ipNode = doc[param.id];
	
	if (ipNode["local_ip"])
	    ipParam.local_ip	 = ipNode["local_ip"].as<std::string>();
	if (ipNode["local_port"])
	    ipParam.local_port	 = ipNode["local_port"].as<int>();
	
	if (ipNode["cloud_single_ip"])
	    ipParam.cloud_single_ip	 = ipNode["cloud_single_ip"].as<std::string>();
	if (ipNode["cloud_single_port"])
	    ipParam.cloud_single_port	 = ipNode["cloud_single_port"].as<int>();

	if (ipNode["cloud_multi_ip"])
	    ipParam.cloud_multi_ip	 = ipNode["cloud_multi_ip"].as<std::string>();
	if (ipNode["cloud_multi_port"])
	    ipParam.cloud_multi_port	 = ipNode["cloud_multi_port"].as<int>();

	if (ipNode["info_multi_ip"])
	    ipParam.info_multi_ip	 = ipNode["info_multi_ip"].as<std::string>();
	if (ipNode["info_multi_port"])
	    ipParam.info_multi_port	 = ipNode["info_multi_port"].as<int>();

	if (ipNode["rtk_ip"])
	    ipParam.rtk_ip	 = ipNode["rtk_ip"].as<std::string>();
	if (ipNode["rtk_port"])
	    ipParam.rtk_port	 = ipNode["rtk_port"].as<int>();

	
    return ipParam;
}


std::vector<double> getArrayParam(ros::NodeHandle        &private_nh,std::string paramName,size_t size)
{  
    std::vector<double> v_values;
	XmlRpc::XmlRpcValue xmlParam;
	private_nh.getParam(paramName, xmlParam);
	XmlRpc::XmlRpcValue xmlValue = xmlParam[0];
	for (size_t i = 0;i < size; i++)
		v_values.push_back(xmlValue[i]);
	return v_values;
}


std::vector<std::string> getStrArrayParam(ros::NodeHandle        &private_nh ,std::string paramName,size_t size)
{  
    std::vector<std::string> v_values;
	XmlRpc::XmlRpcValue xmlParam;
	private_nh.getParam(paramName, xmlParam);
	XmlRpc::XmlRpcValue xmlValue = xmlParam[0];
	for (size_t i = 0;i < size; i++){
		v_values.push_back(xmlValue[i]);
	}
	return v_values;
}


bool  getPlatformIp(PlatformParam param, VehicleIpParam &Ipparam)
{
	std::string ip, port;
	std::string filepath = ros::package::getPath("launch_node");
    filepath = filepath + std::string("/param/global/vehicles_ip.yaml");
	try {
        // 打开 YAML 文件
        std::ifstream fin(filepath);
        if (!fin) {
            std::cerr << "无法打开文件: " << filepath << std::endl;
            return false;
        }

        // 解析 YAML 文件
        YAML::Node config = YAML::Load(fin);

        // 遍历 YAML 文件中的每个节点
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            std::string vehicle_type = it->first.as<std::string>();

            // 获取当前节点的子节点
            YAML::Node subNode = it->second;
            if (subNode["ip"]) {
                ip = subNode["ip"].as<std::string>();
            }
            if (subNode["port"]) {
                port = subNode["port"].as<std::string>();
            }
            if(vehicle_type.compare(param.vehicle_type) == 0){
				Ipparam.ip = ip;
				Ipparam.port = port;
				return true;
			}
        }
		return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML 解析错误: " << e.what() << std::endl;
		return false;
    }
}

bool  loadVehicleIp(std::map<std::string, VehicleIpParam> &getVehiclesIp)
{
	std::string ip, port;
	std::string filepath = ros::package::getPath("launch_node");
    filepath = filepath + std::string("/param/global/vehicles_ip.yaml");
	try {
        // 打开 YAML 文件
        std::ifstream fin(filepath);
        if (!fin) {
            std::cerr << "无法打开文件: " << filepath << std::endl;
            return false;
        }

        // 解析 YAML 文件
        YAML::Node config = YAML::Load(fin);

        // 遍历 YAML 文件中的每个节点
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            std::string vehicle_type = it->first.as<std::string>();

            // 获取当前节点的子节点
            YAML::Node subNode = it->second;
            if (subNode["ip"]) {
                ip = subNode["ip"].as<std::string>();
            }
            if (subNode["port"]) {
                port = subNode["port"].as<std::string>();
            }
            getVehiclesIp.insert(std::pair<std::string, VehicleIpParam>(vehicle_type, {ip, port}));
        }
		return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML 解析错误: " << e.what() << std::endl;
		return false;
    }
}




}  // namespace 

