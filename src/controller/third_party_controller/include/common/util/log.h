#ifndef COMMON_UTIL_LOG_H_
#define COMMON_UTIL_LOG_H_

#ifdef __ROS__
#include <ros/ros.h>
#else
#include <sys/time.h>
#endif

#ifdef __ROS__
#define AINFO(...) ROS_INFO(__VA_ARGS__)
#define ADEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define AWARN(...) ROS_WARN(__VA_ARGS__)
#define AERROR(...) ROS_ERROR(__VA_ARGS__)
#define AFATAL(...) ROS_FATAL(__VA_ARGS__)
#else
#define AINFO(...)                                                             \
  do {                                                                         \
    timeval tp;                                                                \
    gettimeofday(&tp, NULL);                                                   \
    fprintf(stdout, "[ INFO] [%lld.%lld]: ", (long long int)tp.tv_sec,         \
            (long long int)tp.tv_usec);                                        \
    fprintf(stdout, __VA_ARGS__);                                              \
    fprintf(stdout, "\n");                                                     \
  } while (0)

#define ADEBUG(...)                                                            \
  do {                                                                         \
    timeval tp;                                                                \
    gettimeofday(&tp, NULL);                                                   \
    fprintf(stdout, "[DEBUG] [%lld.%lld]: ", (long long int)tp.tv_sec,         \
            (long long int)tp.tv_usec);                                        \
    fprintf(stdout, __VA_ARGS__);                                              \
    fprintf(stdout, "\n");                                                     \
  } while (0)

#define AWARN(...)                                                             \
  do {                                                                         \
    timeval tp;                                                                \
    gettimeofday(&tp, NULL);                                                   \
    fprintf(stdout, "[ WARN] [%lld.%lld]: ", (long long int)tp.tv_sec,         \
            (long long int)tp.tv_usec);                                        \
    fprintf(stdout, __VA_ARGS__);                                              \
    fprintf(stdout, "\n");                                                     \
  } while (0)

#define AERROR(...)                                                            \
  do {                                                                         \
    timeval tp;                                                                \
    gettimeofday(&tp, NULL);                                                   \
    fprintf(stdout, "[ERROR] [%lld.%lld]: ", (long long int)tp.tv_sec,         \
            (long long int)tp.tv_usec);                                        \
    fprintf(stdout, __VA_ARGS__);                                              \
    fprintf(stdout, "\n");                                                     \
  } while (0)

#define AFATAL(...)                                                            \
  do {                                                                         \
    timeval tp;                                                                \
    gettimeofday(&tp, NULL);                                                   \
    fprintf(stdout, "[FATAL] [%lld.%lld]: ", (long long int)tp.tv_sec,         \
            (long long int)tp.tv_usec);                                        \
    fprintf(stdout, __VA_ARGS__);                                              \
    fprintf(stdout, "\n");                                                     \
  } while (0)

#endif

#endif