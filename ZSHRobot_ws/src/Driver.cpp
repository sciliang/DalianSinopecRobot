#include <iostream>
#include "Driver.hpp"
#include <glog/logging.h>
#include "XMLParser.hpp"
#include "global.hpp"
#include <math.h>
#include <controlcan.h>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include <iomanip>
#include <chrono>
#include <thread>
#include <boost/thread/thread.hpp>
#include <Python.h>
using namespace boost;
ControllerMsg Driver::ControllerMsg_ = {0};
Driver::RobotParameter Driver::RobotParameter_ = {0};
Driver::GasParameter Driver::GasParameter_ = {0};

Driver::Driver(/* args */)
{
}

Driver::~Driver()
{
}

void Driver::ReceiveRobPDO()
{
    LOG(INFO) << "ReceiveRobPDO start..." << endl;
}

int Driver::SETrobPDO()
{
    LOG(INFO) << "SETrobPDO start..." << endl;
}

int Driver::SetCtrlMSG_BY_CAN(float _Vel_Velocity, float _Angle_Velocity)
{
    LOG(INFO) << "SetCtrlMSG_BY_CAN start..." << endl;
}

int Driver::SETrobVel(float _Vel_Velocity, float MAX_Vel, float _Angle_Velocity, float MAX_Angle)
{
    LOG(INFO) << "SETrobVel start..." << endl;
    Driver Patrol_run1;
    // 线速度过滤
    if (_Vel_Velocity <= -MAX_Vel)
    {
        _Vel_Velocity = -MAX_Vel;
    }
    else if (_Vel_Velocity > MAX_Vel)
    {
        _Vel_Velocity = MAX_Vel;
    }
    LOG(INFO) << "_Vel_Velocity =" << _Vel_Velocity << endl;
    // 角速度过滤
    if (_Angle_Velocity <= -MAX_Angle)
    {
        _Angle_Velocity = -MAX_Angle;
    }
    else if (_Angle_Velocity > MAX_Angle)
    {
        _Angle_Velocity = MAX_Angle;
    }
    LOG(INFO) << "_Angle_Velocity =" << _Angle_Velocity << endl;
    LOG(INFO) << "Prepare SET Velocity .." << endl;
    SetCtrlMSG_BY_CAN(_Vel_Velocity, _Angle_Velocity);
    return true;
}

double Driver::CacuTracezxAngle(double LinearVelocity, double AngleVelocity)
{
    double ThitMid, ThitZX, Wheel_distance = 0.44, Caraxis_distace = 0.65, difference; // 求取转角时,除法计算的 除数 被除数
    difference = fabs(LinearVelocity / AngleVelocity) - 0.5 * Wheel_distance;

    if (difference >= 0)
    {
        ThitMid = atan(0.5 * Caraxis_distace / fabs(LinearVelocity / AngleVelocity));
        // 多类情形
        if ((LinearVelocity > 0 && AngleVelocity > 0) || (LinearVelocity < 0 && AngleVelocity < 0))
        {
            ThitZX = ThitMid * (180 / M_PI);
        }
        else if ((LinearVelocity > 0 && AngleVelocity < 0) || (LinearVelocity < 0 && AngleVelocity > 0))
        {
            ThitZX = -ThitMid * (180 / M_PI);
        }
        else if (LinearVelocity == 0 || AngleVelocity == 0)
        {
            ThitZX = ThitMid = 0;
        }
        // 限制到正负20度
        if (ThitZX >= 20)
        {
            ThitZX = 20;
        }
        if (ThitZX <= -20)
        {
            ThitZX = -20;
        }
        printf("CacuTracezxAngleandSend's ZX angle is  ThitZX(genzong) = %lf, difference = %lf \n\n", ThitZX, difference);
    }
    else
    {
        printf("CacuTracezxAngleandSend's ZX angle is too big or too small,might >=90||<=90\n,difference=%lf \n\n", difference);
    }
    return ThitZX;
}

void Driver::RobDriver_run(void)
{
    XMLParser xmlParser;
    global global_driver;
    Driver Patrol_run;
    timer clock;
    //---车运动参数读取---//
    xmlParser.getValue("MotionPara", "MaxVelocity", Patrol_run.RobotParameter_.MaxVel);
    xmlParser.getValue("MotionPara", "MaxAngular", Patrol_run.RobotParameter_.MaxAnguar);
    xmlParser.getValue("MotionPara", "MaxVelAcc", Patrol_run.RobotParameter_.MaxVelAcc);
    xmlParser.getValue("MotionPara", "MaxAngularAcc", Patrol_run.RobotParameter_.MaxAnguarAcc);
    xmlParser.getValue("MotionPara", "VelAcc", Patrol_run.RobotParameter_.VelAcc);
    xmlParser.getValue("MotionPara", "AngularAcc", Patrol_run.RobotParameter_.AngularAcc);
    LOG(INFO) << "PatrolConfig_XML, Motion Parameter Read(1):" << endl
              << " MaxVelocity=" << setprecision(12) << Patrol_run.RobotParameter_.MaxVel << endl
              << " MaxAngular=" << Patrol_run.RobotParameter_.MaxAnguar << endl
              << " MaxVelAcc=" << Patrol_run.RobotParameter_.MaxVelAcc << endl
              << " MaxAngularAcc=" << Patrol_run.RobotParameter_.MaxAnguarAcc << endl
              << " VelAcc=" << Patrol_run.RobotParameter_.VelAcc << endl
              << " AngularAcc=" << Patrol_run.RobotParameter_.AngularAcc;
    xmlParser.getValue("Chassis", "AxisWidth", Patrol_run.RobotParameter_.AxisWidth);
    xmlParser.getValue("Chassis", "WheelWidth", Patrol_run.RobotParameter_.WheelWidth);
    xmlParser.getValue("Chassis", "MaxFrontWheelAngle", Patrol_run.RobotParameter_.MaxFrontWheelAngle);
    LOG(INFO) << "PatrolConfig_XML, Motion Parameter Read(2):" << endl
              << " AxisWidth=" << Patrol_run.RobotParameter_.AxisWidth << endl
              << " WheelWidth=" << Patrol_run.RobotParameter_.WheelWidth << endl
              << " MaxFrontWheelAngle=" << Patrol_run.RobotParameter_.MaxFrontWheelAngle;
    /**
    * @brief
    * 4*2维的数组，代表待巡察面积的四个经纬度的值；
    * (0): ([0][0]) 经度1, ([0][1]) 纬度1
    * (1): ([1][0]) 经度2, ([1][1]) 纬度2
    * (2): ([2][0]) 经度3, ([2][1]) 纬度3
    * (3): ([3][0]) 经度4, ([3][1]) 纬度4
    *      （0）            （1）
    *        ----------------
    *        |              |
    *        |              |
    *        |              |
    *        ----------------
    *      （2）            （3）
     // 以[0][0]点建立坐标原点
    */
    // std::lock_guard<std::mutex> lock(global_driver.TargetArea_flag_mtx);
    xmlParser.getValue("TargetArea", "TargetFieldLon0", global_driver.TargetArea[0][0]);
    xmlParser.getValue("TargetArea", "TargetFieldLat0", global_driver.TargetArea[0][1]);
    xmlParser.getValue("TargetArea", "TargetFieldLon1", global_driver.TargetArea[1][0]);
    xmlParser.getValue("TargetArea", "TargetFieldLat1", global_driver.TargetArea[1][1]);
    xmlParser.getValue("TargetArea", "TargetFieldLon2", global_driver.TargetArea[2][0]);
    xmlParser.getValue("TargetArea", "TargetFieldLat2", global_driver.TargetArea[2][1]);
    xmlParser.getValue("TargetArea", "TargetFieldLon3", global_driver.TargetArea[3][0]);
    xmlParser.getValue("TargetArea", "TargetFieldLat3", global_driver.TargetArea[3][1]);
    LOG(INFO) << " PatrolConfig_XML, TargetArea0 :" << endl
              << " TargetFieldLon0=" << setprecision(12) << global_driver.TargetArea[0][0] << ","
              << " TargetFieldLat0=" << setprecision(12) << global_driver.TargetArea[0][1] << endl
              << " TargetFieldLon1=" << setprecision(12) << global_driver.TargetArea[1][0] << ","
              << " TargetFieldLat1=" << setprecision(12) << global_driver.TargetArea[1][1] << endl
              << " TargetFieldLon2=" << setprecision(12) << global_driver.TargetArea[2][0] << ","
              << " TargetFieldLat2=" << setprecision(12) << global_driver.TargetArea[2][1] << endl
              << " TargetFieldLon3=" << setprecision(12) << global_driver.TargetArea[3][0] << ","
              << " TargetFieldLat3=" << setprecision(12) << global_driver.TargetArea[3][1] << endl;
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    double Driver_elapsed_time = clock.elapsed();
    // LOG(INFO) << "Driver_elapsed_time front:" << Driver_elapsed_time << endl;
    clock.restart();
    // UDP通信
    // UDP_busvel();
    // CAN_bus();
    while (true)
    {
        boost::this_thread::sleep(boost::posix_time::seconds(2));
    }
}