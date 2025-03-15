#ifndef NAV712_HPP
#define NAV712_HPP
#include "gpchc.h"

// 和main工程间的消息队列
// typedef struct
// {
//     __syscall_slong_t mtype;
//     nmea_gpchc_s navmsg2Human;
// } NavMSG2Main;

typedef struct
{
    __syscall_slong_t mtype;
    double latitude;
    double longitude;
    double trackAngle;
} NavMSG2Main;

class nav712
{
private:
    /* data */
public:
    nav712(/* args */);
    ~nav712();
    void IMU_calculate_run(int argc, char *argv[]);
};

#endif /* NAV712_HPP */