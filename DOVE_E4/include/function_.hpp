#ifndef FUNCTION_HPP
#define FUNCTION_HPP
#include "gpchc.h"
#include <glog/logging.h>

// 和main工程间的消息队列
typedef struct
{
    __syscall_slong_t mtype;
    double latitude;
    double longitude;
    double trackAngle;
} NavMSG2Main;

void initGlog(char *argv);
#endif /* CONTROL_HPP */