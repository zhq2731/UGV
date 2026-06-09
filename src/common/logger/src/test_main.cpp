
#include "logger/logger.h"
#include <ros/ros.h>
#include <ros/package.h>

//在log.yaml中，本节点的日志参数如下所示：
/*
logger:
    fileLevel: 3
    consoleLevel: 1
    file: "logger.txt"
    loggerName: "logger_test" 
    max_size: 10.0
    max_files:    

*/

int main(int argc,char *argv[]){  

    
    std::string param_node_dir  = ros::package::getPath("launch_node");
	std::string log_param_file = param_node_dir + std::string("/param/log/")+std::string("log.yaml");

	//自身的pkg名称，对应log.yaml中的名称
	std::string pkg_name = std::string("logger"); 
	LogCfg logCfg = Logger::logCfg(log_param_file,pkg_name);
    Logger *logger = Logger::get_instance();
	logger->init(logCfg);

	//打印值。{}表示占位符。
	LOG_INFO("cfg->console_level {}",logCfg.consoleLevel);
	LOG_INFO("cfg->fileLevel {}",logCfg.fileLevel);
	LOG_INFO("cfg->file {}",logCfg.file);
	LOG_INFO("cfg->loggerName {}",logCfg.loggerName);
	LOG_INFO("cfg->max_size {}",logCfg.max_size);
	LOG_INFO("cfg->max_files {}",logCfg.max_files);

    //
	LOG_INFO("test info log"); //只会打印，因为fileLevel的等级为WARN所以，只打印，不会写文件
	LOG_INFO_PURE("test_pure info");//不带格式头的打印
	LOG_INFO_WRITE("test only write log");//这里不会写文件，因为在上面注释部分，写文件的等级为WARN


	LOG_WARN("test warn log"); //带格式头的打印 + 写文件。
	LOG_WARN_PURE("test_pure warn");//不带格式头的打印
	LOG_WARN_WRITE("test only write log"); //会写入文件

	
    //打印值。{}表示占位符。
    std::string a("I'm come from");
	int num = 11;
    LOG_WARN(" {} {} department",a,num); 

	//sleep(10);
}
