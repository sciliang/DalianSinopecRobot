#ifndef GLOBAL_HPP
#define GLOBAL_HPP
#pragma once
#include <iostream>
#include <glog/logging.h>
#include "communication.hpp"
#include <mutex>

// 全局变量
class global
{
private:
    /* data */
public:
    global(/* args */);
    ~global();

    enum
    {
        // 有人端至控制器
        RBKSSD_ID, /// 0 Robot开始上电
        KQYKMS_ID, /// 1 开启遥控模式
        GBYKMS_ID, /// 2 关闭遥控模式
        KQZZMS_ID, /// 3 开启自主模式
        GBZZMS_ID, /// 4 关闭自主模式
        KQDPYK_ID, /// 5 开启底盘遥控
        GBDPYK_ID, /// 6 关闭底盘遥控
        DPYDKZ_ID, /// 7 底盘运动控制
        KQSBYK_ID, /// 8 开启手臂遥控
        GBSBYK_ID, /// 9 关闭手臂遥控

        KQZZBX_ID, /// 10 开启轴坐标系
        GBZZBX_ID, /// 11 关闭轴坐标系
        KQSJZB_ID, /// 12 开启世界坐标系
        GBSJZB_ID, /// 13 关闭世界坐标系
        KQGJZB_ID, /// 14 开启工具坐标系
        GBGJZB_ID, /// 15 关闭工具坐标系
        SBYDKZ_ID, /// 16 手臂运动控制

        KQSZYK_ID, /// 17 开启手掌遥控
        GBSZYK_ID, /// 18 关闭手掌遥控
        SZYDKZ_ID, /// 19 手掌运动控制
        ZZRWGH_ID, /// 20 自主任务规划
        RBKSXL_ID, /// 21 Robot开始巡逻
        RBZSTZ_ID, /// 22 Robot暂时停止
        RBJXZX_ID, /// 23 Robot继续执行
        RBZZRW_ID, /// 24 Robot终止任务
        RBYDJT_ID, /// 25 Robot运动急停
        RBDJMS_ID, /// 26 Robot运动急停
        RBKSXD_ID, /// 27 Robot运动急停

        // 控制器至有人端
        RBDPXX_ID, /// 28 Robot底盘的实时位置、姿态、速度 信息
        RBMBXX_ID, /// 29 目标的位置、姿态信息，或者传感器等异常信息
        RBSBXX_ID, /// 30 机械臂的姿态信息（各个关节的状态）或者机械臂的异常信息
        RBSZXX_ID, /// 31 灵巧手的姿态信息或者灵巧手的异常信息
        RBDCXX_ID, /// 32 反馈车体电池电量信息或者异常信息
        RBRWXX_ID, /// 33 机器人送水、送药、送饭等任务的完成情况信息

        // 路径跟踪
        FSLD_ID, /// 34
        KSGZ_ID, /// 35

        // 开、关搜索伤员
        KQSSP_ID, // 36
        JSSSP_ID, // 37

        // 大工指令ID
        SLAMXX_ID, // 38
        KQYT_ID,   // 39

        GBYT_ID, // 40

        KQYZMS_ID, // 41由前期的越障模式更改为，高海拔机器人server模式，接收其他人的线速度和角速度
        DWBG_ID,   // 42

        YBJDSZ_ID, // 43

        DPGXMS_ID, // 44
        DLXWSZ_ID, // 45电流限位设置
        RBCMZT_ID, // 46开启舱门

        BUSCJLD_ID, // 47采集路点  点击界面采集路点
        YGGX1_ID,   // 48 摇杆构型1  摇杆A键
        YGGX2_ID,   // 49 摇杆构型2  摇杆Y键

        /// ZY
        BBFW_ID,   // 50 摆臂复位
        JGJDSZ_ID, // 51 激光角度设置
        JGBBFW_ID, // 52 激光摆臂复位
        KYC_ID,    // 53 靠右侧
        GJZZ_ID,   // 54 轨迹跟踪
        LJGH_ID,   // 55 路径规划
        LDGZ_ID,   // 56 路点跟踪

        // 摇杆的标志位
        YGGX3_ID, // 57 摇杆构型3  摇杆B键
        YGGX4_ID, // 58 摇杆构型4  摇杆X键
        YGYZ_ID,  // 59 摇杆越障
        YGYK_ID,  // 60 摇杆遥控
        SCBG_ID,  // 61 刹车变更
    };

    // 机器人控制模式
    enum
    {
        ROBWAIT,   // 待机
        ROBTElE,   // 遥控
        ROBAUTO,   // 自主
        ROBSERVER, // 自主调油门
    };

    // 机器人路径规划
    enum
    {
        RECTANGLE_XML_PLAN = 1, // 矩形XML轨迹规划
        TXT_PLAN,               // txt轨迹规划
        RECTANGLE_CAN_PLAN,     // 矩形来自CAN的轨迹规划
        EXCEL_PLAN,             // excetl路点
    } PathPlan;

    typedef struct
    {
        double longitude[300];
        double latitude[300];
    } LatANDLon;

    // 路点接收标志位
    volatile static short Waypoint_flag;
    static bool figureSaveFlag;
    volatile static uint8_t CANPathPoint_NUM;
    volatile static bool Trace_Jump;
    volatile static bool LJGH_Flag;
    volatile static bool Abstacle_Flag;
    volatile static bool DPYDKZteleFlag; // 底盘运动控制
    volatile static short RobotModel;    // 0待机、1遥控、2自主
    volatile static short RobStatusFlag; // 1暂停、2终止、3继续
    static LatANDLon LatANDLon_;

    static double longitude;
    static double latitude;
    static float heading;
    static int PointTXT_lineCount;
    static int PointCollectFlag;

    static double TargetArea[4][2];
    static double TargetCANArea[4][2];
    void initGlog(char *argv);
};

struct tm *getsystime(void);
void signal_handler(int signum);
extern std::mutex VELandANG_mtx;
extern std::mutex RobotStatus_mtx;
extern std::mutex RobotModel_mtx;
extern std::mutex Trace_Jump_mtx;
extern std::mutex GPSPOS_flag_mtx;
extern FILE *fp_RoadPointSave;
#endif