#include <iostream>
#include <glog/logging.h>
#include "XMLParser.hpp"
#include "global.hpp"
#include "AvoidAbstacle.hpp"
#include "control.hpp"
#include "communication.hpp"
#include "Driver.hpp"
#include "doveE4.hpp"
#include "robottrace.hpp"
#include "coppeliaSimMsg.hpp"
#include <thread>
#include <signal.h>
int main(int argc, char *argv[])
{
    global global_Patrol;
    control RobCtrl_Patrol;
    Driver Driver_Patrol;
    communication communication_Patrol;
    nav712 nav712_Patrol;
    robottrace robottrace_Patrol;
    CoppeliaSim CoppeliaSim_;
    AvoidAbstacle AvoidAbstacle_;
    global_Patrol.initGlog(*argv);
    if (signal(SIGINT, signal_handler) == SIG_ERR)
        LOG(INFO) << "Failed to caught signal!" << endl;
    // 巡检机器人的数据收发
    std::thread ZSHControl(&control::ZSH_CtrlThread, &RobCtrl_Patrol);
    // 驱动参数的读取
    std::thread RobDriver(&Driver::RobDriver_run, &Driver_Patrol);
    // 界面交互相关参数
    std::thread RobReceiver(&communication::communication_run, &communication_Patrol);
    // 轨迹跟踪核心算法
    std::thread RobTrace(&robottrace::RobotTrace_RUN, &robottrace_Patrol);
    // 千寻惯导程序
    std::thread RobNAV(&nav712::IMU_calculate_run, &nav712_Patrol, argc, argv);
    ZSHControl.join();
    RobDriver.join();
    RobReceiver.join();
    RobTrace.join();
    RobNAV.join();
    google::ShutdownGoogleLogging(); // 全局关闭glog
    return EXIT_SUCCESS;
}