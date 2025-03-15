#include <iostream>
#include "doveE4.hpp"
#include <glog/logging.h>
#include "XMLParser.hpp"
#include <math.h>
#define _USE_MATH_DEFINES
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <chrono>
#include "global.hpp"
#include <sys/msg.h>

using namespace std;
#define MSGQUE_Nav2MAIN 500

nav712::nav712(/* args */)
{
}

nav712::~nav712()
{
}

void nav712::IMU_calculate_run(int argc, char *argv[])
{
    global global_nav;
    NavMSG2Main NavMSG2Main_;
    pthread_mutex_t Nav2GHB_mutex;

    // msg que to main
    char IMUBuf[1024];
    int msgQue_Nav2MAIN = -1;
    memset(&NavMSG2Main_, 0, sizeof(NavMSG2Main_));
    int Nav2MAIN_BufSize = sizeof(NavMSG2Main_);
    msgQue_Nav2MAIN = msgget((key_t)MSGQUE_Nav2MAIN, 0666 | IPC_CREAT);
    while (msgQue_Nav2MAIN < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               msgQue_Nav2MAIN);
        msgQue_Nav2MAIN = msgget((key_t)MSGQUE_Nav2MAIN, 0666 | IPC_CREAT);
        sleep(1);
    }
    LOG(INFO) << "IMU_calculate_run start..." << endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz
        int ret = msgrcv(msgQue_Nav2MAIN, (void *)IMUBuf, Nav2MAIN_BufSize, 0, 0);
        if (ret > 0)
        {
            memcpy(&NavMSG2Main_, IMUBuf, Nav2MAIN_BufSize);
            GPSPOS_flag_mtx.lock();
            // std::cout << "navmsg2Human.gpchc.latitude : " << NavMSG2Main_.latitude << std::endl;
            // std::cout << "navmsg2Human.gpchc.longitude : " << NavMSG2Main_.longitude << std::endl;
            // std::cout << "navmsg2Human.gpchc.heading : " << NavMSG2Main_.trackAngle << std::endl;
            global_nav.heading = NavMSG2Main_.trackAngle;
            global_nav.latitude = NavMSG2Main_.latitude;
            global_nav.longitude = NavMSG2Main_.longitude;
            GPSPOS_flag_mtx.unlock();
        }
        else
        {
            printf("[WARNING] msgrcv return :%d.\n", ret);
        }
    } // while (true)
} // end of main()
