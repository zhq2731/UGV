#pragma once

#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/sink.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "common/common.h"

enum LogLevel_enum : int {
    LEVEL_TRACE = 0x00,
    LEVEL_DEBUG = 0x01,
    LEVEL_INFO = 0x02,
    LEVEL_WARN = 0x03,
    LEVEL_ERROR = 0x04,
    LEVEL_CRITICAL = 0x05,
    LEVEL_OFF = 0x06
};

class LogCfg
{
public:
    int  fileLevel; 
	int  consoleLevel;
	std::string file;
	std::string loggerName;
	double max_size;
	double max_files; 
};


class Logger
{
public:
	Logger(const Logger&)=delete;
    Logger& operator=(const Logger&) = delete;
	
	std::shared_ptr<spdlog::logger>  m_logger;
	std::shared_ptr<spdlog::logger>  m_logger_pure;
	std::shared_ptr<spdlog::logger>  m_logger_write;
	static Logger* get_instance(){
		static Logger instance;
		return &instance;
	}
	void init(LogCfg &cfg);
	static LogCfg logCfg(std::string &cfgFile,std::string &pkgName);
	bool inited = false;
private:
	Logger(){};
};

//同时打印（带格式头，如时间+文件名+行数+日志）+写文件 但是打印和写可以设置不同的级别
//如fileLevel的级别为WARN，consoleLevel的级别为INFO
//则使用LOG_INFO的时候，则只会打印不会写文件,使用LOG_WARN打印，则同时打印和写文件。
#define LOG_DEBUG(...) \
	SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger, spdlog::level::debug, __VA_ARGS__)

#define LOG_INFO(...) \
		SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger, spdlog::level::info, __VA_ARGS__)

#define LOG_WARN(...) \
	SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger, spdlog::level::warn, __VA_ARGS__)

#define LOG_ERROR(...) \
		SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger, spdlog::level::err, __VA_ARGS__)

#define LOG_CRITICAL(...) \
		SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger, spdlog::level::critical, __VA_ARGS__)



		
//纯打印，并且不带格式头的。等同于std::cout。
//这个是为了方便某些场景，比如循环中，只想循环打印某些数值使用

#define LOG_DEBUG_PURE(...) \
			SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_pure, spdlog::level::debug, __VA_ARGS__)
		
#define LOG_INFO_PURE(...) \
				SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_pure, spdlog::level::info, __VA_ARGS__)
		
#define LOG_WARN_PURE(...) \
			SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_pure, spdlog::level::warn, __VA_ARGS__)
		
#define LOG_ERROR_PURE(...) \
				SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_pure, spdlog::level::err, __VA_ARGS__)
		
#define LOG_CRITICAL_PURE(...) \
				SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_pure, spdlog::level::critical, __VA_ARGS__)
		

//只写文件，带格式头的。
//这个是为了满足在某些场景中，只想写文件，而不想将内容打印出来的时候使用。

#define LOG_DEBUG_WRITE(...) \
					SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_write, spdlog::level::debug, __VA_ARGS__)
				
#define LOG_INFO_WRITE(...) \
						SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_write, spdlog::level::info, __VA_ARGS__)
				
#define LOG_WARN_WRITE(...) \
					SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_write, spdlog::level::warn, __VA_ARGS__)
				
#define LOG_ERROR_WRITE(...) \
						SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_write, spdlog::level::err, __VA_ARGS__)
				
#define LOG_CRITICAL_WRITE(...) \
						SPDLOG_LOGGER_CALL(Logger::get_instance()->m_logger_write, spdlog::level::critical, __VA_ARGS__)


