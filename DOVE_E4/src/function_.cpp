#include <iostream>
#include <math.h>
bool bRunning_Flag = true;
#include <glog/logging.h>

void initGlog(char *argv)
{
    google::InitGoogleLogging(argv); // 初始化
    google::SetStderrLogging(google::GLOG_INFO);
    // 设置INFO WARINNING ERROR FATALD等 级别以上的信息log文件的路径和前缀名
    google::SetLogDestination(google::GLOG_INFO, "log/INFO_");
    google::SetLogDestination(google::GLOG_WARNING, "log/WARNING_");
    google::SetLogDestination(google::GLOG_ERROR, "log/ERROR_");
    google::SetLogDestination(google::GLOG_FATAL, "log/FATAL_");
    FLAGS_colorlogtostderr = true; // 开启终端颜色区分
}
