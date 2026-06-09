#include "logger/logger.h"

void Logger::init(LogCfg &cfg)
{
    if (inited)
		return;
	
	auto sink_console = std::make_shared<spdlog::sinks::stdout_sink_mt>();
	
	auto sink_console_pure = std::make_shared<spdlog::sinks::stdout_sink_mt>();

	auto max_size = 1024*1024*cfg.max_size;  //10
    auto sink_file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(cfg.file,max_size, cfg.max_files,false);
    
    sink_console->set_level((spdlog::level::level_enum)cfg.consoleLevel);
    sink_file->set_level((spdlog::level::level_enum)cfg.fileLevel);
	sink_console_pure->set_level((spdlog::level::level_enum)cfg.consoleLevel);

	
	sink_console->set_pattern("[%n][%Y-%m-%d %H:%M:%S.%e] [%l] [%s %#]:%v");
	sink_file->set_pattern("[%n][%Y-%m-%d %H:%M:%S.%e] [%l] [%s %#]:%v");
	sink_console_pure->set_pattern("%v");
	
	spdlog::sinks_init_list sinks = {sink_console,sink_file};
    
	std::string loggerName = cfg.loggerName;
	std::string loggerWriteName = loggerName+"_writer";
	std::string loggerPureName = cfg.loggerName+"pure";
	m_logger = std::make_shared<spdlog::logger>(loggerName, sinks.begin(),sinks.end());
	m_logger_write = std::make_shared<spdlog::logger>(loggerWriteName, sink_file);
	m_logger_pure = std::make_shared<spdlog::logger>(loggerPureName, sink_console_pure);

	spdlog::register_logger(m_logger);
    spdlog::register_logger(m_logger_write);
	spdlog::register_logger(m_logger_pure);
	//spdlog::flush_every(std::chrono::seconds(5));
	spdlog::flush_on((spdlog::level::level_enum)cfg.fileLevel);

	inited = true;

}


LogCfg Logger::logCfg(std::string &cfgFile,std::string &pkgName){
	std::string homedir ;
	common::getHomeDir(homedir);
	YAML::Node doc = YAML::LoadFile(cfgFile);
    LogCfg logCfg;
	const YAML::Node &pkgNode = doc[pkgName];
	logCfg.fileLevel	= pkgNode["fileLevel"].as<int>();
	logCfg.consoleLevel	= pkgNode["consoleLevel"].as<int>();
	logCfg.file = homedir + std::string("/log/") + pkgNode["file"].as<std::string>();
	logCfg.loggerName  = pkgNode["loggerName"].as<std::string>();
	logCfg.max_size = pkgNode["max_size"].as<double>();
	logCfg.max_files = pkgNode["max_files"].as<int>();
	return logCfg;
}

