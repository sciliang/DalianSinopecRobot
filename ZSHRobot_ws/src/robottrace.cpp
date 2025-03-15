#define _USE_MATH_DEFINES
#include <stdlib.h> //实物的
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iomanip>
#include <iostream>
#include <assert.h>
#include <thread>
#include <chrono>
#include <stdbool.h>
#include <NEWprotocol.h>
#include <errno.h>
#include <define.h>
#include <glog/logging.h>
#include "global.hpp"
#include "Driver.hpp"
#include "XMLParser.hpp"
#include "Coordinate_System.hpp"
#include "robottrace.hpp"
#include "orientus_packets.h"
#include <cmath>
#include "matplotlibcpp.h"
#include <list>
#include "kCurves.h"
#include "XMLParser.hpp"
#include <vector>
namespace plt = matplotlibcpp;
using namespace std;
extern QT_msg2Coppeliasim msg_QT2coppSim;
#define RADIANS_TO_DEGREES 57.29578
#define R_earth 6378393.0 // 地球半径 6378393.0  6371
// kcurve之后的绘图
extern std::vector<double> KcurvePlot_X, KcurvePlot_Y;
// kcurve期望路点
extern std::vector<double> KcurvePlot_X_target, KcurvePlot_Y_target;
// 计算两点的距离长度跟方位角
Nav992MSG2HumanFrame message_Nav2GHB;

robottrace::robottrace(/* args */)
{
    /*-速度分割用-*/
    traceMax = 1;      // 跟踪最大速度
    traceBigger = 1.0; // 跟踪比较大速度
    traceMin = 0.05;   // 跟踪最小速度

    /*-误差-*/
    traceZX_gain = 0.7; // 航向误差的比例，求取角速度

    /*-速度滤波用-*/
    trace_MostZxAngle = 20.0; // 跟踪最大角速度
    trace_MostAcc = 1.0;      // 跟踪最大线速度
    trace_minAcc = 0.1;       // 跟踪最大线速度

    /*-转向角分割用极限值-*/
    CarAngleLimit = 180.0;  // 跟踪角度分割界限180度
    StraightAngle1 = 3.0;   // 跟踪直线行驶
    StraightAngle2 = 357.0; // 跟踪直线行驶
    MoreVel_Angle1 = 30.0;  // 跟踪较大速度转弯
    MoreVel_Angle2 = 330.0; // 跟踪较大速度转弯

    /*-车的半径-*/
    car_radious = 0.5;

    /*-终点误差-*/
    final_distanerror = 0.2;

    /*轨迹跟踪计算用*/
    lon = 0.0, lat = 0.0; // 车的经、纬度
    // point_x1为坐标原点0的X轴距离
    point_x1 = 0.0, point_x2 = 0.0;
    // point_y1为坐标原点0的Y轴距离
    point_y1 = 0.0, point_y2 = 0.0;
    error_x = 0.0, error_y = 0.0;
    // e1,e2,e3为方位角误差
    e1 = 0.0, e2 = 0.0, e3 = 0.0;
    Thit = 0.0, Thit_r = 0.0;
    Thit_r_ = 0.0, Thit_error = 0.0;

    /*-经纬度的传递，存储实际的经纬度-*/
    last_Longitude = 0.0, last_Latitude = 0.0, hangxiang = 0.0;
    Next_Longitude = 0.0, Next_Latitude = 0.0;
    Next_Longitude_before = 0.0, Next_Latitude_before = 0.0;
    save_Linear_deta_velocity = 0.0, save_Angular_deta_velocity = 0.0;

    /*-txt的存储路点-*/
    fp_RoadPointRead = NULL;

    /*-控制律内部-*/
    Point_line_velocity = 2.5, Point_angular_velocity = 0.4;
    Linear_deta_velocity = 0.0, Angular_deta_velocity = 0.0;
    Angular_velocity_robot = 0.0, Linear_velocity_robot = 0.0;

    /*-绘图用的-*/
    Tar_X_axis_vector.resize(0), Tar_Y_axis_vector.resize(0);
    PathPoint_lon.resize(0), PathPoint_lat.resize(0);
    PathPoint_lon_PlanTar.resize(0), PathPoint_lat_PlanTar.resize(0);
}

robottrace::~robottrace()
{
}

void robottrace::Plot_TarANDActual_POS(void)
{
    bool PlotInitialize = false;
    global global_plot;
    LOG(INFO) << "Plot_TarANDActual_POS plotview will start:" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    while (true)
    {
        // 绘图
        x_axis_vector.push_back(global_plot.longitude);
        y_axis_vector.push_back(global_plot.latitude);
        HX_vector.push_back(global_plot.heading);
        // cout << "PLOT:"
        //      << "global_plot.longitude =" << setprecision(12) << global_plot.longitude << ","
        //      << "global_plot.longitude =" << setprecision(12) << global_plot.latitude << ","
        //      << "global_plot.heading =" << setprecision(12) << global_plot.heading << endl;
        // Clear previous plot
        // LOG(INFO) << "Plot_TarANDActual_POS circle2" << endl;

        plt::clf();
        //-图1：经纬度-
        plt::figure(1);
        plt::plot(x_axis_vector, y_axis_vector, "bo-");
        // 期望路点
        plt::named_plot("tarpoint", Tar_X_axis_vector, Tar_Y_axis_vector, "kx-");
        // const char *filename1 = "./target points22.png";
        // plt::save(filename1);
        // plt::xlim(123.4409, 123.4410);
        // plt::ylim(41.7613, 41.7614);
        // Plot a line whose name will show up as "log(x)" in the legend.
        // plt::named_plot("lon-lat", x_axis, y_axis);
        // plt::subplot(2, 1, 2);

        //-图2：航向角-
        plt::figure(2);
        plt::plot(HX_vector, "rx-");
        // const char *filename2 = "./hangxiang22.png";
        // plt::save(filename2);
        // plt::title("position figure");
        // Clear previous plot

        //-图3：期望路点，全局的规划-
        plt::figure(3);
        plt::plot(PathPoint_lon, PathPoint_lat, "bo-");
        // 四个期望的路点
        plt::plot(PathPoint_lon_PlanTar, PathPoint_lat_PlanTar, "rx-");

        //-图4：绘制kcurve之后的路点-
        plt::figure(4);
        plt::plot(KcurvePlot_X_target, KcurvePlot_Y_target, "bo-");
        // 四个期望的路点
        plt::plot(KcurvePlot_X, KcurvePlot_Y, "rx-");
        // const char *filename3 = "./sangedian22.png";
        // plt::save(filename3);
        // Plot a line whose name will show up as "log(x)" in the legend.
        // plt::plot(HX_vector, "rx-");
        // plt::legend();
        plt::pause(0.01);
        // if (global_plot.RobStatusFlag == TraceStop)
        // {
        //     plt::save(filename1);
        //     plt::save(filename2);
        //     plt::save(filename3);
        //     LOG(INFO) << "Get signal, save,filename1、2、3" << endl;
        // }
        // plt::ticklabel_format(stime = 'plain');
        // plt::ticklabel_format(useOffset = False, style = 'plain');
        // const char *filename = "./basic.png";
        // std::cout << "Saving result to " << filename << std::endl;
        // plt::save(filename);
        // Display plot continuously
        // plt::title("Sample figure");*/
    }
}

void robottrace::PlanPathCurves(std::vector<Eigen::Vector2d> input)
{
    std::cout << "in PlanPathCurves!!!!" << std::endl;
    // std::vector<Eigen::Vector2d> input(6); // the path points
    input.assign(6, Eigen::Vector2d::Zero()); // 初始化
    // 输入
    // input = {Eigen::Vector2d(0, 0),
    //          Eigen::Vector2d(0, 1),
    //          Eigen::Vector2d(1.5, 3),
    //          Eigen::Vector2d(2, 2),
    //          Eigen::Vector2d(2, 1),
    //          Eigen::Vector2d(1, 0)};
    // input[0] << 0, 0;
    // input[1] << 0, 1;
    // input[2] << 1.5, 3;
    // input[3] << 2, 2;
    // input[4] << 2, 1;
    // input[5] << 1, 0;
    // 打印
    std::cout << "the size of input: " << input.size() << std::endl;
    std::size_t count_input = 0;
    double value1;
    for (size_t i = 0; i < input.size(); i++)
    {
        KcurvePlot_X_target.push_back(input[i][0]);
        KcurvePlot_Y_target.push_back(input[i][1]);
        std::cout << "KcurvePlot_X_target[" << i << "] = " << KcurvePlot_X_target[i] << " KcurvePlot_Y_target[" << i << "] = " << KcurvePlot_Y_target[i] << std::endl;
    }

    for (const auto &v : input)
    {
        std::cout << " Element_input : " << count_input << " = " << std::setw(4) << v.transpose() << " " << std::endl;
        value1 = input[2][0];
        count_input++;
    }
    std::cout << " value1 = " << value1 << std::endl;

    // 开始平滑轨迹
    // input the path points, and return the control points of Bezier curves
    // 这面返回的是4*3的矩阵,原因是里面的计算是3列的要求
    std::size_t count_bezierOpen = 0;
    std::vector<std::vector<Eigen::Vector2d>> bezierOpen = kcurve::kCurveOpen(input);
    // 打印
    std::cout << "the size of bezierOpen: " << bezierOpen.size() << std::endl;
    for (const auto &innerVec : bezierOpen)
    {
        for (const auto &v : innerVec)
        {
            std::cout << "  Element_bezierOpen:" << count_bezierOpen << " = " << std::setw(9) << v.transpose() << " ";
            count_bezierOpen++;
        }
        std::cout << std::endl;
    }
    std::size_t count_result = 0;

    // input the control points and number of interpolation points, and return the smoothed path
    std::vector<Eigen::Vector2d> result = bezier::bezier_curve(bezierOpen, 100);
    std::cout << "the size of result: " << result.size() << std::endl;
    for (size_t i = 0; i < result.size(); i++)
    {
        KcurvePlot_X.push_back(result[i][0]);
        KcurvePlot_Y.push_back(result[i][1]);
        std::cout << "KcurvePlot_X[" << i << "] = " << KcurvePlot_X[i] << " KcurvePlot_Y[" << i << "] = " << KcurvePlot_Y[i] << std::endl;
    }
    for (const auto &v : result)
    {
        std::cout << "Element_result : " << count_result << " = " << std::setw(9) << v.transpose() << "     " << std::endl;
        count_result++;
    }
}

/**
 * @brief
 * 4*2维的数组，代表待巡察面积的四个经纬度的值；
 * (0): ([0][0]) 经度1, ([0][1]) 纬度1
 * (1): ([1][0]) 经度2, ([1][1]) 纬度2
 * (2): ([2][0]) 经度3, ([2][1]) 纬度3
 * (3): ([3][0]) 经度4, ([3][1]) 纬度4
 *
 *  采集的路点顺序：
 * （2个点:123.49433990、41.67868330,点2） （3个点:123.49428360、41.67873270,点3）
 *      --------------------------------------------------------------
 * 为   |                                                            |
 * 纵   |                                                            |
 * 向   |                                                            |
 *      --------------------------------------------------------------
 * （1个点:123.49443100、41.67872510,点0） （4个点:123.49436060、41.67878180,点1）
 *                               为  横  向
 * 根据关系映射的轨迹规划的电子围栏点：
 *      <TargetFieldLon0>123.49443100</TargetFieldLon0>
 *      <TargetFieldLat0>41.67872510 </TargetFieldLat0>
 *      <TargetFieldLon1>123.49436060</TargetFieldLon1>
 *      <TargetFieldLat1>41.67878180 </TargetFieldLat1>
 *      <TargetFieldLon2>123.49433990</TargetFieldLon2>
 *      <TargetFieldLat2>41.67868330 </TargetFieldLat2>
 *      <TargetFieldLon3>123.49428360</TargetFieldLon3>
 *      <TargetFieldLat3>41.67873270 </TargetFieldLat3>
 *
 *      （0）            （1）
 *        ----------------
 *        |              |
 *        |              |
 *        |              |
 *        ----------------
 *      （2）            （3）
 *
    // 以[0][0]点建立坐标原点
 */
void robottrace::PatrolPathPlan(double lonAndlatFirst[4][2], int VerticalScale, int CrossScale)
{
    for (short i = 0; i < 4; i++)
    {
        for (short j = 0; j < 2; j++)
        {
            LOG(INFO) << "PatrolPathPlan::lonAndlatFirst[" << i << "]"
                      << "[" << j << "]"
                      << "=" << setprecision(12) << lonAndlatFirst[i][j] << endl;
        }
    }
    //  纵向划分间隔、横向划分间隔
    // VerticalScale = 4, CrossScale = 2;
    double NewLonCUT_X, NewLatCUT_X, NewLonCUT_Y, NewLatCUT_Y;
    // 距离计算
    double Line0_2 = Calculation_distance(lonAndlatFirst[0][0], lonAndlatFirst[0][1], lonAndlatFirst[2][0], lonAndlatFirst[2][1]);
    double Line0_1 = Calculation_distance(lonAndlatFirst[0][0], lonAndlatFirst[0][1], lonAndlatFirst[1][0], lonAndlatFirst[1][1]);
    LOG(INFO) << "Line0_2=" << Line0_2 << ","
              << "Line0_1=" << Line0_1 << endl;
    // 点与点之间的连接向量和正北方向的夹角计算
    double Azimuth0_2 = Azimuth_caculate(lonAndlatFirst[0][0], lonAndlatFirst[0][1], lonAndlatFirst[2][0], lonAndlatFirst[2][1]);
    double Azimuth0_1 = Azimuth_caculate(lonAndlatFirst[0][0], lonAndlatFirst[0][1], lonAndlatFirst[1][0], lonAndlatFirst[1][1]);
    LOG(INFO) << "du"
              << ":"
              << "Azimuth0_2=" << Azimuth0_2 << ","
              << "Azimuth0_1=" << Azimuth0_1 << endl;
    // 夹角由度转弧度
    Azimuth0_2 = Azimuth0_2 / RADIANS_TO_DEGREES;
    Azimuth0_1 = Azimuth0_1 / RADIANS_TO_DEGREES;
    LOG(INFO) << "rad"
              << ":"
              << "Azimuth0_2=" << Azimuth0_2 << ","
              << "Azimuth0_1=" << Azimuth0_1 << endl;
    // 刻度变量
    double Longitude_kedu_cut = (2.0 * M_PI * R_earth) / (360.0);
    double Latitude_kedu_cut = fabs((2.0 * M_PI * R_earth * cos(lonAndlatFirst[0][1] / RADIANS_TO_DEGREES)) / (360.0));
    LOG(INFO) << " Longitude_kedu_cut = " << setprecision(12) << Longitude_kedu_cut << ","
              << " Latitude_kedu_cut =" << setprecision(12) << Latitude_kedu_cut << endl;
    // 经度、纬度变化的微小变量
    //[0][1]横轴
    double LonData_X = (CrossScale * sin(Azimuth0_1)) / Latitude_kedu_cut;
    double LatData_X = (CrossScale * cos(Azimuth0_1)) / Longitude_kedu_cut;
    LOG(INFO) << "[0][1]hengzhou"
              << ":"
              << " LonData_X = " << setprecision(12) << LonData_X << ","
              << " LatData_X =" << setprecision(12) << LatData_X << endl;
    //[0][2]纵轴
    double LonData_Y = (VerticalScale * sin(Azimuth0_2)) / Latitude_kedu_cut;
    double LatData_Y = (VerticalScale * cos(Azimuth0_2)) / Longitude_kedu_cut;
    LOG(INFO) << "[0][2]hengzhou"
              << ":"
              << " LonData_Y = " << setprecision(12) << LonData_Y << ","
              << " LatData_Y =" << setprecision(12) << LatData_Y << endl;
    // 切分后的路点总数，向上取整
    int Point_quantity0_1 = floor(Line0_1 / CrossScale);
    int Point_quantity0_2 = floor(Line0_2 / VerticalScale);
    LOG(INFO) << "the cutting points num"
              << ":"
              << "Point_quantity0_1=" << Point_quantity0_1 << ","
              << "Point_quantity0_2=" << Point_quantity0_2 << endl;
    // 取余数
    double Point_remainder0_1 = fmod(Line0_1, CrossScale);
    double Point_remainder0_2 = fmod(Line0_2, VerticalScale);
    LOG(INFO) << "the cutting points num"
              << ":"
              << "Point_remainder0_1=" << Point_remainder0_1 << ","
              << "Point_remainder0_2=" << Point_remainder0_2 << endl;

    // 路点开始切分，+1是加上原点
    for (short i = 0; i <= Point_quantity0_1 + 1; i++)
    {
        NewLonCUT_X = lonAndlatFirst[0][0] + LonData_X * i;
        NewLatCUT_X = lonAndlatFirst[0][1] + LatData_X * i;
        if (i == Point_quantity0_1 + 1)
        {
            NewLonCUT_X = lonAndlatFirst[1][0];
            NewLatCUT_X = lonAndlatFirst[1][1];
        }
        // 全局的路点存储器 先存储横轴
        PathPoint_lon.push_back(NewLonCUT_X);
        PathPoint_lat.push_back(NewLatCUT_X);
        for (short j = 1; j <= Point_quantity0_2 + 1; j++) // j从1开始，越过0点坐标
        {
            NewLonCUT_Y = NewLonCUT_X + LonData_Y * j;
            NewLatCUT_Y = NewLatCUT_X + LatData_Y * j;
            if (j == Point_quantity0_2 + 1)
            {
                // 将最后一处的路点进行比例尺映射
                NewLonCUT_Y = NewLonCUT_X + LonData_Y * (Point_quantity0_2 + (Point_remainder0_2 / VerticalScale));
                NewLatCUT_Y = NewLatCUT_X + LatData_Y * (Point_quantity0_2 + (Point_remainder0_2 / VerticalScale));
            }
            // 全局的路点存储器，再依照横轴存储纵轴
            PathPoint_lon.push_back(NewLonCUT_Y); // 全局的路点存储器，在每次运行的时候，都进行一次初始化，清空内容
            PathPoint_lat.push_back(NewLatCUT_Y);
            /*至此，路点存储完毕*/
        }
    }

    // 拷贝进最后一个路点，至此，路点准备完毕
    // PathPoint_lon.push_back(NewLonCUT_Y);
    // PathPoint_lat.push_back(NewLatCUT_X);

    // 输出vector的容量大小:当前分配的存储容量，也就是当前情况下能够存储的元素个数
    LOG(INFO) << "PathPoint_lon.capacity = " << PathPoint_lon.capacity() << ","
              << "PathPoint_lat.capacity = " << PathPoint_lat.capacity() << endl;
    LOG(INFO) << "PathPoint_lon.size = " << PathPoint_lon.size() << ","
              << "PathPoint_lat.size = " << PathPoint_lat.size() << endl;

    for (short i = 0; i < (Point_quantity0_1 + 2); i++) // Tar_X_axis.end()
    {
        LOG(INFO) << " num =" << i << ","
                  << "PathPoint_lon Vector =" << setprecision(12) << PathPoint_lon[i] << ","
                  << "PathPoint_lat Vector =" << PathPoint_lat[i] << endl;
        // 将奇数顺序不对的路点掉转顺序
        if ((i % 2) != 0)
        {
            reverse(PathPoint_lon.begin() + i * (Point_quantity0_2 + 2), PathPoint_lon.begin() + i * (Point_quantity0_2 + 2) + (Point_quantity0_2 + 2));
            reverse(PathPoint_lat.begin() + i * (Point_quantity0_2 + 2), PathPoint_lat.begin() + i * (Point_quantity0_2 + 2) + (Point_quantity0_2 + 2));
        }
    }
    // 输出vector的容量大小:当前分配的存储容量，也就是当前情况下能够存储的元素个数
    LOG(INFO) << "PathPoint_lon.capacity = " << PathPoint_lon.capacity() << ","
              << "PathPoint_lat.capacity = " << PathPoint_lat.capacity() << endl;

    // for (auto i = 0; i != PathPoint_lon.size(); i++) // Tar_X_axis.end()
    // {
    //     LOG(INFO) << " num =" << i << ","
    //               << "diaozhuan hou ,PathPoint_lon Vector =" << setprecision(12) << PathPoint_lon[i] << ","
    //               << "PathPoint_lat Vector =" << PathPoint_lat[i] << endl;
    // }
    /*图3：期望路点，全局的规划*/
    // plt::figure(3);
    // plt::plot(PathPoint_lon, PathPoint_lat, "bo-");
    // // 四个期望的路点
    // plt::plot(PathPoint_lon_PlanTar, PathPoint_lat_PlanTar, "rx-");
    // const char *filename3 = "./sangedian .png";
    // plt::save(filename3);
    // // Plot a line whose name will show up as "log(x)" in the legend.
    // // plt::plot(HX_vector, "rx-");
    // // plt::legend();
    // plt::pause(0.01);
}

// 两点之间的距离
/*这部分做了修改，原来他的求解是没有绝对值的，正常应该有绝对值，但是考虑在北半球，纬度两者的余弦值都是正数！写的没错！*/
double robottrace::Calculation_distance(double Longitude, double Latitude, double Next_Longitude, double Next_Latitude)
{
    double rad1, rad2, Line; //
    rad1 = sin(Latitude * M_PI / 180) * sin(Next_Latitude * M_PI / 180) + cos(Latitude * M_PI / 180) * cos(Next_Latitude * M_PI / 180) * cos((Longitude - Next_Longitude) * M_PI / 180);
    //	printf("sin(Latitude*Pi/180)=%.12lf\n",sin(Latitude*Pi/180.0));
    //	printf("sin(Next_Latitude*Pi/180)=%.12lf\n",sin(Next_Latitude*Pi/180.0));
    //	printf("cos(Latitude*Pi/180)=%.12lf\n",cos(Latitude*Pi/180.0));
    //	printf("cos(Next_Latitude*Pi/180)=%.12lf\n",cos(Next_Latitude*Pi/180.0));
    //	printf("cos((Longitude-Next_Longitude)*Pi/180)=%.12lf\n",cos((Longitude-Next_Longitude)*Pi/180.0));
    //	printf("rad1=%.16lf\n",rad1);
    if (rad1 < 1)
    {
        rad2 = acos(rad1);
    }
    else
    {
        // printf("rad2 of Calculation_distance nan!\n");
        rad2 = 1e-12;
    }
    // printf("rad2=%.16lf\n",rad2);
    Line = R_earth * rad2; // 两点间的距离长度
    // distance=Line;
    // printf("Calculation_distance between two points: %lf\n",Line);
    return Line;
}

/**
 * @brief Azimuth_caculate
 * 1.经度A
 * 2.纬度A
 * 3.经度M
 * 4.纬度M
 * 求取起点到终点的方位角
 * 两点之间的方位角,两者间距离过大时本算法误差就会迅速增大,
 * 主要原因是因为double字长不够！！在经纬度1到2度范围内没有问题
 */
double robottrace::Azimuth_caculate(double Longitude_A, double Latitude_A, double Longitude_M, double Latitude_M)
{
    double Longitude_B, Latitude_B;
    double X_A, Y_A, Z_A, X_B, Y_B, Z_B, R_A;
    double X_C, Y_C, Z_C;
    double AB_x, AB_y, AB_z, AC_x, AC_y, AC_z, AB_xlj, AB_moj, fangwei_zjl, fangwei;

    Latitude_A = Latitude_A / RADIANS_TO_DEGREES;
    Longitude_A = Longitude_A / RADIANS_TO_DEGREES;
    Latitude_M = Latitude_M / RADIANS_TO_DEGREES;
    Longitude_M = Longitude_M / RADIANS_TO_DEGREES;

    Latitude_B = Latitude_M;
    Longitude_B = Longitude_M;

    // printf("Latitude_B,Longitude_B=%.8lf,%.8lf\n",Latitude_B,Longitude_B);
    R_A = R_earth;
    // printf("R_earth=%lf\n",R_earth);

    if ((Latitude_M != 0.0) &&
        (Latitude_M != (90.0 / RADIANS_TO_DEGREES)) &&
        (Longitude_M != 0.0) &&
        (Longitude_M != (90.0 / RADIANS_TO_DEGREES)) &&
        (Longitude_M != Longitude_A) &&
        (Latitude_M != Latitude_A))
    // if (((Latitude_M - 0.0) >= 1e-9) &&                          // 目标点纬度不等于0
    //     ((Latitude_M - (90.0 / RADIANS_TO_DEGREES)) >= 1e-9) &&  // 目标点纬度不等于90
    //     ((Longitude_M - 0.0) >= 1e-9) &&                         // 目标点经度不等于0
    //     ((Longitude_M - (90.0 / RADIANS_TO_DEGREES)) >= 1e-9) && // 目标点经度不等于0
    //     (fabs(Longitude_M - Longitude_A) >= 1e-9) &&             // 起始点与目标点经度不相等
    //     (fabs(Latitude_M - Latitude_A) >= 1e-9))                 // 起始点与目标点纬度不相等
    {
        X_A = R_A * cos(Latitude_A) * cos(Longitude_A);
        Y_A = R_A * cos(Latitude_A) * sin(Longitude_A);
        Z_A = R_A * sin(Latitude_A);
        // R_B=((R_A*R_A)/(cos(Latitude_B)*cos(Longitude_B)*X_A+cos(Latitude_B)*sin(Longitude_B)*Y_A+sin(Latitude_B)*Z_A));
        X_B = (R_A * R_A) / ((X_A + Y_A * tan(Longitude_B) + Z_A * tan(Latitude_B) / cos(Longitude_B)));
        Y_B = (R_A * R_A) / ((X_A * (1.0 / tan(Longitude_B)) + Y_A + Z_A * tan(Latitude_B) / sin(Longitude_B)));
        Z_B = (R_A * R_A) / ((X_A * cos(Longitude_B) * (1.0 / tan(Latitude_B)) + Y_A * sin(Longitude_B) * (1.0 / tan(Latitude_B)) + Z_A));

        X_C = 0.0;
        Y_C = 0.0;
        Z_C = R_A / sin(Latitude_A);

        AB_x = (X_B - X_A);
        AB_y = (Y_B - Y_A);
        AB_z = (Z_B - Z_A);
        // LOG(INFO) << " AB_x = " << AB_x << ","
        //           << " AB_y = " << AB_y << ","
        //           << " AB_z = " << AB_z << endl;

        AC_x = (X_C - X_A);
        AC_y = (Y_C - Y_A);
        AC_z = (Z_C - Z_A);
        // LOG(INFO) << " AC_x = " << AC_x << ","
        //           << " AC_y = " << AC_y << ","
        //           << " AC_z = " << AC_z << endl;

        AB_xlj = AB_x * AC_x + AB_y * AC_y + AB_z * AC_z;
        // LOG(INFO) << " AB_xlj = " << AB_xlj << endl;

        AB_moj = (sqrt(AB_x * AB_x + AB_y * AB_y + AB_z * AB_z)) * (sqrt(AC_x * AC_x + AC_y * AC_y + AC_z * AC_z));
        // LOG(INFO) << " AB_moj = " << AB_moj << endl;

        fangwei_zjl = acos(AB_xlj / AB_moj);

        if (Longitude_M > Longitude_A)
        {
            fangwei = fangwei_zjl * (180.0 / M_PI);
            LOG(INFO) << "fangwei right_dikaer = " << fangwei << endl;
        }
        else if (Longitude_M < Longitude_A)
        {
            fangwei = 360.0 - (fangwei_zjl * (180.0 / M_PI));
            LOG(INFO) << "fangwei left_dikaer = " << fangwei << endl;
        }
    }
    else if (fabs(Longitude_M - Longitude_A) < 1e-9)
    {
        if ((Latitude_M > Latitude_A))
        {
            fangwei = 0.0;
            LOG(INFO) << "fangwei_south = " << fangwei << endl;
        }
        else if (Latitude_M < Latitude_A)
        {
            fangwei = 180.0;
            LOG(INFO) << "fangwei_north = " << fangwei << endl;
        }
        else if (fabs(Latitude_M - Latitude_A) < 1e-9)
        {
            fangwei = 0.0;
            LOG(INFO) << "equal one point = " << fangwei << endl;
        }
    }
    else if (fabs(Latitude_M - Latitude_A) < 1e-9)
    {
        if ((Longitude_M > Longitude_A))
        {
            fangwei = 90.0;
            LOG(INFO) << "fangwei_south = " << fangwei << endl;
        }
        else if (Longitude_M < Longitude_A)
        {
            fangwei = 270.0;
            LOG(INFO) << "fangwei_north = " << fangwei << endl;
        }
        else if (fabs(Longitude_M - Longitude_A) < 1e-9)
        {
            fangwei = 0;
            LOG(INFO) << "equal one point = " << fangwei << endl;
        }
    }
    else
    {
        LOG(ERROR) << "Calculation_Azimuth ERROR!!!!" << endl;
    }
    return fangwei;
}

/*建立全局的直角坐标系,正北方向为X轴的正方向,正西方向为Y轴的正方向
    坐标原点为(123.442166,41.761146), 求小车控制律*/
void robottrace::Coordinate_System(void)
{
    double Global_coordinate_system_Longitude = 123.44200619;
    double Global_coordinate_system_Latitude = 41.76108001;
    double Azimuth_1, Azimuth_2, line_1, line_2, k1, k2, k3, k4, k5; // 车当前点与原点的方位角、目标点与原点的方位角、与坐标原点之间的距离
    // double V_differential,W_differential,V_differentValue,W_differentValue;
    double Angular_velocity_rad, Linear_velocity_vector; //,Linear_velocity,Angular_velocity;//移动机器人的线速度、角速度
    double Linear_velocity, Angular_velocity;
    // 全局坐标原点与车的距离
    line_1 = Calculation_distance(Global_coordinate_system_Longitude, Global_coordinate_system_Latitude, last_Longitude, last_Latitude);
    // 全局坐标原点与车的方位角
    Azimuth_1 = Azimuth_caculate(Global_coordinate_system_Longitude, Global_coordinate_system_Latitude, last_Longitude, last_Latitude);
    //	printf("Azimuth_1 = %lf\n",Azimuth_1);//write_log(pFile,"Azimuth_1 = %lf\n",Azimuth_1);

    // 全局坐标原点与近交点的距离
    line_2 = Calculation_distance(Global_coordinate_system_Longitude, Global_coordinate_system_Latitude, lon, lat);
    //	printf("line_2 = %lf\n",line_2);//write_log(pFile,"line_2 = %lf\n",line_2);
    // 全局坐标原点与近交点的方位角
    Azimuth_2 = Azimuth_caculate(Global_coordinate_system_Longitude, Global_coordinate_system_Latitude, lon, lat);
    //	printf("Azimuth_2 = %lf\n",Azimuth_2);//write_log(pFile,"Azimuth_2 = %lf\n",Azimuth_2);

    // 车的实时位置与目标点方位角
    Thit_r_ = Azimuth_caculate(last_Longitude, last_Latitude, lon, lat);
    //	printf("Thit_r_ = %lf\n",Thit_r_);//write_log(pFile,"Thit_r_ = %lf\n",Thit_r_);

    // 期望Thit_r
    if (Thit_r_ > 90)
    {
        Thit_r = 450 - Thit_r_;
    }
    else
    {
        Thit_r = 90 - Thit_r_;
    }

    // 实时Thit
    if (hangxiang > 90)
    {
        Thit = 450 - hangxiang;
    }
    else
    {
        Thit = 90 - hangxiang;
    }

    // 全局坐标系下车的横、纵坐标,近交点的横、纵坐标
    point_x1 = line_1 * sin((Azimuth_1)*M_PI / 180);
    point_y1 = line_1 * cos((Azimuth_1)*M_PI / 180);
    point_x2 = line_2 * sin((Azimuth_2)*M_PI / 180);
    point_y2 = line_2 * cos((Azimuth_2)*M_PI / 180);
    //	printf("point_x1:%lf \t point_y1:%lf \t point_x2:%lf \t point_y2:%lf \n",point_x1,point_y1,point_x2,point_y2);

    // 坐标误差Xr-X, Yr-Y, 角度误差
    error_x = point_x2 - point_x1;
    error_y = point_y2 - point_y1;
    Thit_error = Thit_r - Thit;

    e1 = (error_x)*cos(Thit * M_PI / 180) + (error_y)*sin(Thit * M_PI / 180);
    e2 = (error_y)*cos(Thit * M_PI / 180) - (error_x)*sin(Thit * M_PI / 180);
    e3 = Thit_error;
    //	printf("e1=%lf \t e2=%lf \t e3=%lf\n",e1,e2,e3);//write_log(pFile,"e1=%lf \t e2=%lf \t e3=%lf\n",e1,e2,e3);

    // 计算比例系数k1,k2,k3,k4,k5
    k1 = 0.03;
    k2 = 0.03;
    k3 = 0.03;
    k4 = 0.4;
    k5 = 0.5;
    //	printf("k1=%lf,k2=%lf,k3=%lf,k4=%lf,k5=%lf\t\n",k1,k2,k3,k4,k5);//write_log(pFile,"k1=%lf,k2=%lf,k3=%lf,k4=%lf,k5=%lf\t\n",k1,k2,k3,k4,k5);

    // 对全局变量Point_line_velocity限制、修改
    if (save_Linear_deta_velocity > Linear_deta_velocity)
    {
        Point_line_velocity = Point_line_velocity - fabs(save_Linear_deta_velocity - Linear_deta_velocity);
    }
    else
    {
        Point_line_velocity = Point_line_velocity + fabs(save_Linear_deta_velocity - Linear_deta_velocity);
    }
    // 对全局变量Point_angular_velocity限制、修改
    if (save_Angular_deta_velocity > Angular_deta_velocity)
    {
        Point_angular_velocity = Point_angular_velocity - fabs(save_Angular_deta_velocity - Angular_deta_velocity);
    }
    else
    {
        Point_angular_velocity = Point_angular_velocity + fabs(save_Angular_deta_velocity - Angular_deta_velocity);
    }
    //	printf("Point_line_velocity=%f \t Linear_deta_velocity=%lf\n",Point_line_velocity,Linear_deta_velocity);
    //	printf("Point_angular_velocity=%f \t Angular_deta_velocity=%lf\n",Point_angular_velocity,Angular_deta_velocity);

    // 移动机器人的控制律
    Linear_velocity_vector = Point_line_velocity * cos(e3 * M_PI / 180) + k1 * e1; // 定理求解
    Angular_velocity_rad = Point_angular_velocity + k2 * Point_line_velocity * e2 + k3 * Point_line_velocity * sin(e3 * M_PI / 180);
    //	printf("Linear_velocity_vector_first=%f \t Angular_velocity_rad_first=%lf\n",Linear_velocity_vector,Angular_velocity_rad);

    //	//求速度差值(按照有加速度的环节进行加速)
    //	V_differentValue=k4*(Linear_velocity_vector-save_linear_v);
    //	W_differentValue=k5*(Angular_velocity_rad-save_Angular_w_rad);

    //	//当前时刻速度和角速度的微分
    //	V_differential=(k1*e2+Point_line_velocity*sin(e3))*Angular_velocity_rad-k1*Linear_velocity+k1*Point_line_velocity*cos(e3)-Point_line_velocity*sin(e3)*Point_angular_velocity;
    //	W_differential=Point_line_velocity*Angular_velocity_rad*(k2*e1+k3*cos(e3))+Point_line_velocity*Point_angular_velocity*k3*cos(e3)-k2*Point_line_velocity*Point_line_velocity*sin(e3);
    //	printf("W_differential=%f \t V_differential=%lf\n",W_differential,V_differential);
    //	write_log(pFile,"W_differential=%f \t V_differential=%lf\n",W_differential,V_differential);

    // 速度终值,角速度转换单位
    Linear_velocity = fabs(Linear_velocity_vector);
    // Angular_velocity =fabs(Angular_velocity_rad*180/Pi);//度：履带车
    Angular_velocity = fabs(Angular_velocity_rad); // 弧度：室内巡逻车
    //	printf("Linear_velocity_jdz=%f \t Angular_velocity_jdz=%lf\n",Linear_velocity,Angular_velocity);

    //	Linear_velocity = fabs(V_differentValue+V_differential);
    //	Angular_velocity = fabs((W_differentValue+W_differential)*180/Pi);
    //	printf("Linear_velocity_jdz=%f \t Angular_velocity_jdz=%lf\n",Linear_velocity,Angular_velocity);
    //	write_log(pFile,"Linear_velocity_jdz=%f \t Angular_velocity_jdz=%lf\n",Linear_velocity,Angular_velocity);

    //	//限速,谨防速度过大或者过小
    //	if(fabs(Linear_velocity_robot)>=0.6)
    //	{
    //		Linear_velocity_robot = 0.6;
    //	}
    //	if(fabs(Linear_velocity_robot)<=0.01)
    //	{
    //		Linear_velocity_robot = 0.01;
    //	}
    //	if(fabs(Angular_velocity_robot)>=1)
    //	{
    //		Angular_velocity_robot =1;
    //	}
    //	if(fabs(Angular_velocity_robot)<=0.1)//5
    //	{
    //		Angular_velocity_robot = 0.1;
    //	}
    //	Linear_velocity_robot=0.8;
    Linear_velocity_robot = 1; //
    //	printf("Linear_velocity_robot_last= %f，Angular_velocity_robot_last= %lf\n",Linear_velocity_robot,Angular_velocity_robot);
}

void robottrace::RobotTrace_RUN(void)
{
    global global_trace;
    Driver Driver_trace;
    FILE *trace_data;
    struct tm *getTime;
    short dataSAVE = 0, dataPLOT = 0, GeneralStatusDog = 0;
    int roadpointNum_max = 1000, PointLinesCount;
    int mid_point = 0, cap_point = 0, RoadPointCopy = 0, roadpoint_num = 0, road_flag = 0;            // 拷贝的路点个数
    double pos_error = 0.0, save_Angular_w_rad = 0.0, FindminRoadLine = 0.0, FindMostLineAngle = 0.0; // 转向角增益
    double convert_next_y = 0.0, convert_next_x = 0.0, TraceZXAngle = 0.0;                            // 笛卡尔坐标系目标点(x,y),
    double convert_car_y = 0.0, convert_car_x = 0.0;                                                  // 笛卡尔坐标系下的车位置
    double convert_car_y_hope = 0.0, convert_car_y_hoplat = 0.0;                                      // 车在笛卡尔坐标系下的位置(x,y)
    double Latitude_kedu = 0.0, Longitude_kedu = 0.0, k = 0.0, jie_criteria = 0.0;                    // 经纬度和局部坐标系间的转换刻度
    double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, lonAndlatFirst_tar[4][2] = {0};                    // 圆与直线的两个交点
    double x_chui = 0.0, y_chui = 0.0, x_chui_lon = 0.0, y_chui_lat = 0.0;                            // 圆和直线的切点的坐标和经纬度
    double xy_chui_line = 0.0, xy_chui_line1 = 0.0, xy_chui_line2 = 0.0;                              // 车和垂点间距离的长度
    double x1_lon = 0.0, y1_lat = 0.0, x2_lon = 0.0, y2_lat = 0.0, x1y1_line = 0.0, x2y2_line = 0.0;  // 圆与直线交点的经纬度
    double new_track_line = 0.0, aimPoints_lenth = 0.0, TraceLine_V = 0.0;                            // 车距离下一个跟踪路点的长度和速度
    double roadpoint_latmiddle = 0.0, roadpoint_lonmiddle = 0.0;                                      // 计算的中间过程的路点
    double roadpoint_lat[roadpointNum_max] = {0}, roadpoint_lon[roadpointNum_max] = {0};              // 总的路点个数
    double k_inf_L1, k_inf_L2, aim_hangx, track_line, track_Azimuth;                                  // 求取的两条直线的比例系数
    double system_latitude, system_longitude, pointA_mid, pointB_mid;                                 // 笛卡尔坐标系坐标原点纬度
    float traceLineVelocity;                                                                          // 轨迹跟踪线速度、角速度
    bool TraceInitialize = false;

    // 过滤经纬度信息，保证车的安全！
    std::vector<double> lon_protect, lat_protect, Compare2GetStartRoad, CompareAllPointAngle;
    lon_protect.resize(0), lat_protect.resize(0), Compare2GetStartRoad.resize(0), CompareAllPointAngle.resize(0);

    // 用来存储两个期望路点之间方位角过大的位置i
    std::vector<int> AngleBiggerPos;
    AngleBiggerPos.resize(0);

    // 用来存储期望路点之间转角过大的路点值,以及该路点前后的值,总之是待需要优化的经纬度值
    std::vector<Eigen::Vector2d> TargetPointAngleBigsNear;
    TargetPointAngleBigsNear.resize(0);
    Eigen::Vector2d tarPointAngleBig_Pos[3];
    // 绘图线程
    // std::thread Plot_tarANDactual_POS(&robottrace::Plot_TarANDActual_POS, this);

TraceInitialize: // 自主开始
    LOG(INFO) << "Robot start running NOW ======>>>>>>>>" << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    /*程序刚进来的时候，对所有的值进行清空，不然值可能一直存在！*/
    /**
     * @brief PathPoint_lon_PlanTar
     *  用来存储XML电子围栏的数据点
     *  然后用来Python绘制期望路点图
     */
    PathPoint_lon_PlanTar.clear();
    PathPoint_lat_PlanTar.clear();
    PathPoint_lon_PlanTar.erase(PathPoint_lon_PlanTar.begin(), PathPoint_lon_PlanTar.end());
    PathPoint_lat_PlanTar.erase(PathPoint_lat_PlanTar.begin(), PathPoint_lat_PlanTar.end());

    /**
     * @brief PathPoint_lon
     *  存储路径规划部分生成的所有路点
     *  清空存储所有期望路点的内存
     *  PlanPath()函数的输出
     */
    PathPoint_lon.erase(PathPoint_lon.begin(), PathPoint_lon.end());
    PathPoint_lat.erase(PathPoint_lat.begin(), PathPoint_lat.end());
    PathPoint_lon.clear();
    PathPoint_lat.clear();

    /**
     * @brief Tar_X_axis_vector
     * 所有的离散的期望经纬度路点
     * 为了给Python绘图用
     */
    Tar_X_axis_vector.erase(Tar_X_axis_vector.begin(), Tar_X_axis_vector.end());
    Tar_Y_axis_vector.erase(Tar_Y_axis_vector.begin(), Tar_Y_axis_vector.end());
    Tar_X_axis_vector.clear();
    Tar_Y_axis_vector.clear();
    LOG(INFO) << "start:::::: Tar_X_axis_vector.capacity = " << Tar_X_axis_vector.capacity() << ","
              << "  Tar_Y_axis_vector.capacity = " << Tar_Y_axis_vector.capacity() << ","
              << "  Tar_X_axis_vector.size = " << Tar_X_axis_vector.size() << ","
              << "  Tar_Y_axis_vector.size = " << Tar_Y_axis_vector.size() << endl;

    /**
     * @brief x_axis_vector , y_axis_vector , HX_vector
     * 这三个数组是用来进行绘图用的 , 存储实时的
     * 经度、纬度和航向的信息
     */
    x_axis_vector.erase(x_axis_vector.begin(), x_axis_vector.end());
    y_axis_vector.erase(y_axis_vector.begin(), y_axis_vector.end());
    HX_vector.erase(HX_vector.begin(), HX_vector.end());
    x_axis_vector.clear();
    y_axis_vector.clear();
    HX_vector.clear();

    /**
     * @brief Compare2GetStartRoad
     * 为了对所有的路点进行距离比较
     * 找到距离车的当前位置最近的路点 , 并进行轨迹跟踪
     */
    Compare2GetStartRoad.erase(Compare2GetStartRoad.begin(), Compare2GetStartRoad.end());
    Compare2GetStartRoad.clear();

    /**
     * @brief lonAndlatFirst_tar
     * 这个结构体式存储从xml文件中读回来的
     * 电子围栏关键点的经纬度
     */
    memset(lonAndlatFirst_tar, 0, sizeof(lonAndlatFirst_tar));

    /**
     * @brief roadpoint_lon roadpoint_lat
     * 存储经过插值之后的所有离散的期望路点经纬度的数组
     */
    memset(roadpoint_lon, 0, sizeof(roadpoint_lon));
    memset(roadpoint_lat, 0, sizeof(roadpoint_lat));

    // 状态切换初始化
    global_trace.RobStatusFlag = TraceContinue;
    LOG(INFO) << "global_trace.Waypoint_flag= " << global_trace.Waypoint_flag << endl;

    /*1、(3点)矩形路径规划*/
    if ((global_trace.Waypoint_flag == global_trace.RECTANGLE_XML_PLAN) || (global_trace.Waypoint_flag == global_trace.RECTANGLE_CAN_PLAN))
    {
        // 从xml进来的路点
        if ((global_trace.Waypoint_flag == global_trace.RECTANGLE_XML_PLAN))
        {
            cout << "XML!! Copy Point Start!" << endl;
            // 拷贝期望路点
            for (int i = 0; i < 4; i++)
            {
                // std::lock_guard<std::mutex> lock(global_trace.TargetArea_flag_mtx);
                cout << "Point till ===> " << i;
                lonAndlatFirst_tar[i][0] = global_trace.TargetArea[i][0];  // 全局变量经度赋值
                lonAndlatFirst_tar[i][1] = global_trace.TargetArea[i][1];  // 全局变量纬度赋值
                PathPoint_lon_PlanTar.push_back(lonAndlatFirst_tar[i][0]); // 拷贝 经度
                PathPoint_lat_PlanTar.push_back(lonAndlatFirst_tar[i][1]); // 拷贝 纬度
            }
            cout << endl;
            cout << "CAN!! Copy Point Finish!" << endl;
        }
        // 从CAN进来的路点
        if ((global_trace.Waypoint_flag == global_trace.RECTANGLE_CAN_PLAN))
        {
            cout << "CAN!! Copy Point Start!" << endl;
            for (int i = 0; i < 4; i++)
            {
                // std::lock_guard<std::mutex> lock(global_trace.TargetCANArea_flag_mtx);
                cout << "Point till ===> " << i;
                lonAndlatFirst_tar[i][0] = global_trace.TargetCANArea[i][0]; // 全局变量经度赋值
                lonAndlatFirst_tar[i][1] = global_trace.TargetCANArea[i][1]; // 全局变量纬度赋值
                PathPoint_lon_PlanTar.push_back(lonAndlatFirst_tar[i][0]);   // 拷贝 经度
                PathPoint_lat_PlanTar.push_back(lonAndlatFirst_tar[i][1]);   // 拷贝 纬度
            }
            cout << endl;
            cout << "CAN!! Copy Point Finish!" << endl;
        }
        LOG(INFO) << "Copy TargetArea finish!" << endl;
        // 绘图用的，掉转两个点的顺序
        reverse(PathPoint_lon_PlanTar.begin() + 2, PathPoint_lon_PlanTar.end());
        reverse(PathPoint_lat_PlanTar.begin() + 2, PathPoint_lat_PlanTar.end());
        PathPoint_lon_PlanTar.push_back(lonAndlatFirst_tar[0][0]);
        PathPoint_lat_PlanTar.push_back(lonAndlatFirst_tar[0][1]);
        // 轨迹规划，折线
        PatrolPathPlan(lonAndlatFirst_tar, 1, 1);
        LOG(INFO) << "Robot PatrolPathPlan finish!" << endl;

        for (int i = 0; i != PathPoint_lon.size(); i++) // Tar_X_axis.end()
        {
            // 路点给到轨迹跟踪的容器
            roadpoint_lon[i] = PathPoint_lon[i]; // PathPlan函数输出的路点数组
            roadpoint_lat[i] = PathPoint_lat[i];
            // 顺便更新下车的位置信息
            last_Latitude = global_trace.latitude;
            last_Longitude = global_trace.longitude;
            if (((last_Latitude - 0.0) <= 1e-7) || ((last_Longitude - 0.0) <= 1e-7))
            {
                global_trace.RobotModel = -1;
                LOG(ERROR) << "last_Latitude and last_Longitude recv ERROR!" << endl;
                break;
            }
            LOG(INFO) << " Point_Num =" << i << ","
                      << "roadpoint_lon <-- Vector =" << setprecision(12) << roadpoint_lon[i] << ","
                      << "roadpoint_lat <-- Vector =" << roadpoint_lat[i] << endl;
            // 求两个期望路点之间的距离
            FindminRoadLine = Calculation_distance(last_Longitude, last_Latitude, roadpoint_lon[i], roadpoint_lat[i]);
            // 求两个路点之间的方位角
            FindMostLineAngle = Azimuth_caculate(roadpoint_lon[i], roadpoint_lat[i], roadpoint_lon[i + 1], roadpoint_lat[i + 1]);
            LOG(INFO) << "FindminRoadLine!!,i = " << i << "   FindminRoadLine = " << FindminRoadLine << endl;
            Compare2GetStartRoad.push_back(FindminRoadLine);   // 存储期望路点间的距离
            CompareAllPointAngle.push_back(FindMostLineAngle); // 存储期望路点间的航向
        }
        global_trace.PointCollectFlag = global_trace.RECTANGLE_XML_PLAN;
        // 找到最小的距离，并从离当前最近的下一个目标点开始跟踪
        vector<double>::iterator minestLine = std::min_element(std::begin(Compare2GetStartRoad), std::end(Compare2GetStartRoad));
        road_flag = (distance(begin(Compare2GetStartRoad), minestLine) + 1);
        LOG(INFO) << "Min element is " << *minestLine << " at position " << distance(begin(Compare2GetStartRoad), minestLine) << endl;

        // 找到所有期望路点间方位角度超过30度的路点
        // for (int i = 0; i < CompareAllPointAngle.size(); i++)
        // {
        //     if ((CompareAllPointAngle[i] >= 30) && (i >= 3) && (i <= CompareAllPointAngle.size() - 3))
        //     {
        //         // 存储期望路点方位角大于30度的向量位置
        //         AngleBiggerPos.push_back(i);
        //         // 将路点方位角大于30度的位置前后各1个路点,总计3个路点进行存储
        //         for (int j = 0; j < 3; j++)
        //         {
        //             tarPointAngleBig_Pos[j] = Eigen::Vector2d(roadpoint_lon[i + j - 2], roadpoint_lat[i + j - 2]);
        //             TargetPointAngleBigsNear.push_back(tarPointAngleBig_Pos[j]);
        //         }
        //         std::cout << " CompareAllPointAngle >30" << std::endl;
        //         std::cout << "CompareAllPointAngle[i] =" << CompareAllPointAngle[i] << " i =" << i << std::endl;
        //     }
        //     else
        //     {
        //         std::cout << " CompareAllPointAngle <30" << std::endl;
        //     }
        // }

        // LOG(INFO) << "Copy RECTANGxLE_PLAN Road Points finish!" << endl;
        // std::this_thread::sleep_for(std::chrono::seconds(4));
        // 轨迹规划正常后,将标志位置1,并准备告诉上位机
        // 接受的CAN数据正常后,将这个标志位置0
        // uint8_t gps_cur_staus1_[2] = {0}, gps_cur_staus2_[2] = {0};
        // 轨迹规划结束后,给上位机发消息,告诉轨迹规划结束
        // for (short i = 0; i < 5; i++)
        // {
        //     usleep(10000);
        //     V_W_CanSend(0x01, 0x01, global_trace.CANPathPoint_NUM, 0, gps_cur_staus1_, gps_cur_staus2_);
        //     cout << "Send PlanPath Succeed " << endl;
        // }

    } /*end of if (矩形规划)*/

    /*2、Excel表格路点规划*/
    if (global_trace.Waypoint_flag == global_trace.EXCEL_PLAN)
    {
        // 路点拷贝
        memcpy(roadpoint_lat, global_trace.LatANDLon_.latitude, sizeof(global_trace.LatANDLon_.latitude));
        memcpy(roadpoint_lon, global_trace.LatANDLon_.longitude, sizeof(global_trace.LatANDLon_.longitude));
        LOG(INFO) << "Copy EXCEL_PLAN Road Points finish!" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } /*end of if (Excel)*/

    /*3、自存储的txt文件的路点规划*/
    if (global_trace.Waypoint_flag == global_trace.TXT_PLAN)
    {
        LOG(INFO) << "global_trace.TXT_PLAN start! " << endl;
        global_trace.PointCollectFlag = global_trace.TXT_PLAN;
        cap_point = 0; // 每一次规划时,都是从0开始计数,来计算txt的路点数
        // txt路点读取
        if ((fp_RoadPointRead = fopen("RoadPointSave.txt", "rb")) == NULL)
        {
            fprintf(stderr, "RoadPointSave.txt,OPEN ERROR, at file: %s, function: %s, line: %d\n, error info: %s\n",
                    __FILE__, __FUNCTION__, __LINE__, strerror(errno));
            exit(0);
        }
        rewind(fp_RoadPointRead);
        while (!feof(fp_RoadPointRead))
        {
            mid_point = fgetc(fp_RoadPointRead);
            if (mid_point == '\n')
                cap_point++;
        }
        LOG(INFO) << "RoadPointSave.txt Lines num, cap_point = " << cap_point << endl;
        // 光标回城
        rewind(fp_RoadPointRead);
        // 路点拷贝
        for (RoadPointCopy = 0; RoadPointCopy < cap_point; RoadPointCopy++)
        {
            // 顺便更新下车的位置信息
            last_Latitude = global_trace.latitude;
            last_Longitude = global_trace.longitude;
            // 判断经纬度有没有问题
            if (((last_Latitude - 0.0) <= 1e-7) || ((last_Longitude - 0.0) <= 1e-7))
            {
                global_trace.RobotModel = -1;
                LOG(ERROR) << "last_Latitude and last_Longitude recv ERROR!" << endl;
                break;
            }
            // 存储经、纬度, 读也是经、纬度
            fscanf(fp_RoadPointRead, "%d %lf %lf\n", &PointLinesCount, &pointA_mid, &pointB_mid);
            roadpoint_lon[RoadPointCopy] = pointA_mid;
            roadpoint_lat[RoadPointCopy] = pointB_mid;
            LOG(INFO) << " Point_Num =" << RoadPointCopy << ","
                      << "roadpoint_lon <-- Vector =" << setprecision(12) << roadpoint_lon[RoadPointCopy] << ","
                      << "roadpoint_lat <-- Vector =" << setprecision(12) << roadpoint_lat[RoadPointCopy] << endl;
            FindminRoadLine = Calculation_distance(last_Longitude, last_Latitude, roadpoint_lon[RoadPointCopy], roadpoint_lat[RoadPointCopy]);
            LOG(INFO) << "FindminRoadLine!!,i = " << RoadPointCopy << "   FindminRoadLine = " << FindminRoadLine << endl;
            Compare2GetStartRoad.push_back(FindminRoadLine);
        }
        vector<double>::iterator minestLine = std::min_element(std::begin(Compare2GetStartRoad), std::end(Compare2GetStartRoad));
        // 从离当前最近的下一个目标点开始跟踪
        road_flag = (distance(begin(Compare2GetStartRoad), minestLine));
        LOG(INFO) << "Min element is " << *minestLine << " at position " << distance(begin(Compare2GetStartRoad), minestLine) << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        LOG(INFO) << "Copy TXT_PLAN Road Points Over!" << endl;
    } /*end of if (_Txtpoint_flag)*/

    //------------------------++交汇++-------------------------------//
    /*---计算路点数量---*/
    for (int roadpointnum_flag = 0; roadpointnum_flag <= roadpointNum_max; roadpointnum_flag++)
    {
        roadpoint_latmiddle = roadpoint_lat[roadpointnum_flag];
        roadpoint_lonmiddle = roadpoint_lon[roadpointnum_flag];
        if (roadpoint_latmiddle == 0 || roadpoint_lonmiddle == 0)
        {
            roadpoint_num = roadpointnum_flag;
            global_trace.PointTXT_lineCount = roadpoint_num;
            LOG(INFO) << "CHECK! After Get ALL Points, Calcu the roadpoint_num"
                      << " roadpoint_num =" << roadpoint_num << endl;
            break;
        }
    }

    /*---绘制期望路点轨迹---*/
    for (int i = 0; i < roadpoint_num; i++)
    {
        Tar_X_axis_vector.push_back(roadpoint_lon[i]);
        Tar_Y_axis_vector.push_back(roadpoint_lat[i]);
        LOG(INFO) << "--Plot Tar Points-- "
                  << " Tar_X_axis_vector[" << i << "]=" << setprecision(12) << Tar_X_axis_vector[i] << ","
                  << " Tar_Y_axis_vector[" << i << "]=" << setprecision(12) << Tar_Y_axis_vector[i]
                  << endl;
    }
    /*--输出vector的容量大小:当前分配的存储容量，也就是当前情况下能够存储的元素个数--*/
    LOG(INFO) << "Tar_X_axis_vector.capacity = " << Tar_X_axis_vector.capacity() << ","
              << "Tar_Y_axis_vector.capacity = " << Tar_Y_axis_vector.capacity() << ","
              << "Tar_X_axis_vector.size = " << Tar_X_axis_vector.size() << ","
              << "Tar_Y_axis_vector.size = " << Tar_Y_axis_vector.size() << endl;

    /*--车位姿更新(NAV)--*/
    // last_Latitude = message_Nav2GHB.nav992msg2Human_.latitude;
    // last_Longitude = message_Nav2GHB.nav992msg2Human_.longitude;
    // hangxiang = message_Nav2GHB.nav992msg2Human_.heading;

    /*--车位姿更新(Coppesim)--*/
    // std::lock_guard<std::mutex> lock(global_trace.GPSPOS_flag_mtx);
    GPSPOS_flag_mtx.lock();
    last_Latitude = global_trace.latitude;
    last_Longitude = global_trace.longitude;
    hangxiang = global_trace.heading;
    LOG(INFO) << "last_Latitude=" << setprecision(12) << last_Latitude << ","
              << "  last_Longitude=" << setprecision(12) << last_Longitude << ","
              << "  hangxiang=" << setprecision(12) << hangxiang << endl;
    GPSPOS_flag_mtx.unlock();

    /*--经纬度存储，保持滤波状态--*/
    lon_protect.push_back(last_Longitude);
    lat_protect.push_back(last_Latitude);

    /*--上电位置作为初始原点--*/
    system_latitude = last_Latitude;
    system_longitude = last_Longitude;

    /*--打印所有路点，观察正确与否--*/
    for (int i = 0; i < roadpoint_num; i++)
    {
        LOG(INFO) << "--[CHECK! Berore Running, Print all PathPoints]--"
                  << " road_flag=" << road_flag << ","
                  << " roadpoint_lon[" << i << "] = " << setprecision(12) << roadpoint_lon[i] << ","
                  << " roadpoint_lat[" << i << "] = " << setprecision(12) << roadpoint_lat[i]
                  << endl;
    }
    // 延迟
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LOG(INFO) << "before robottrace while" << endl;
    short datacacu = 0;
    global_trace.LJGH_Flag = true;
    // PlanPathCurves();
    while (true)
    {
        getTime = getsystime();
        /*-----------模式跳转-----------*/
        if (global_trace.Trace_Jump)
        {
            // 这个延迟是等待绘图跳转完事之后，在进行跳转置false
            std::this_thread::sleep_for(std::chrono::seconds(1));
            Trace_Jump_mtx.lock();
            global_trace.Trace_Jump = false; // 跳转一次后,置0
            Trace_Jump_mtx.unlock();
            LOG(INFO) << "=====>>>>>> RobotTrace Initialize= <<<<<<<====" << endl;
            LOG(INFO) << "Trace_Jump = " << global_trace.Trace_Jump << endl;
            goto TraceInitialize;
        }

        /*-----------待机模式-----------*/
        if (global_trace.RobotModel == ROBWAIT)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            Trace_Jump_mtx.lock();
            global_trace.LJGH_Flag = false; // 路径规划未成功
            Trace_Jump_mtx.unlock();
            LOG(INFO) << "<<----ROBWAIT--->>" << endl;
            /*----------这部分管下发----------*/
            // 消息队列-->copplisim
            msg_QT2coppSim.mtype = 1;
            msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = 0.0;
            msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = 0.0;
            // LOG(INFO) << "vehicleAngularVelocity= " << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << endl;
            // LOG(INFO) << "vehicleLineVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity << endl;
        }
        /*-----------遥控模式-----------*/
        else if (global_trace.RobotModel == ROBTElE)
        {
            int printCACU;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            global_trace.LJGH_Flag = false;  // 路径规划未成功
            global_trace.Trace_Jump = false; // 跳转一次后,置0
            dataPLOT++;
            GPSPOS_flag_mtx.lock();
            last_Latitude = global_trace.latitude;
            last_Longitude = global_trace.longitude;
            hangxiang = global_trace.heading;
            if (printCACU++ == 20)
            {
                LOG(INFO) << "\n\n\n <-------------------CHECK! ROBTElE---------------------->" << endl;
                LOG(INFO) << "-------<[YaoKong] Model Start..>>-------" << endl
                          << "yuandianweidu =" << setprecision(12) << last_Latitude
                          << "  yuandianjingdu=" << setprecision(12) << last_Longitude
                          << "  hangxiang=" << setprecision(12) << hangxiang;
                printCACU = 0;
            }
            GPSPOS_flag_mtx.unlock();
            // 实时更新坐标原点
            system_latitude = last_Latitude;
            system_longitude = last_Longitude;
            // LOG(INFO) << "dataPLOT =" << dataPLOT << endl;
            // LOG(INFO) << "traceMax =" << traceMax << ","
            //           << "traceBigger =" << traceBigger << ","
            //           << "traceMin =" << traceMin << ","
            //           << "traceStop =" << traceStop << ","
            //           << "trace_MostZxAngle =" << trace_MostZxAngle << endl;
            // LOG(INFO) << "trace_MostAcc =" << trace_MostAcc << ","
            //           << "roadpointNum_max =" << roadpointNum_max << ","
            //           << "CarAngleLimit =" << CarAngleLimit << endl;
            // LOG(INFO) << "StraightAngle1 =" << StraightAngle1 << ","
            //           << "MoreVel_Angle1 =" << MoreVel_Angle1 << ","
            //           << "StraightAngle2 =" << StraightAngle2 << ","
            //           << "MoreVel_Angle2 =" << MoreVel_Angle2 << endl;

        } /*if(YaoKong update)*/
        /*-----------自主模式-----------*/
        // else if (global_trace.RobotModel == ROBAUTO && global_trace.LJGH_Flag)
        else if (global_trace.RobotModel == ROBNO_START && global_trace.LJGH_Flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            LOG(INFO) << "\n\n<--------------------------<<[ROBAUTO] Model Start..>>-------------------------->" << endl;
            // for (int i = 0; i < roadpoint_num; i++)
            // {
            //     LOG(INFO) << "[CHECK! On Running, Print all PathPoints]:"
            //               << "road_flag=" << road_flag << ","
            //               << "roadpoint_lon[" << i << "] = " << setprecision(12) << roadpoint_lon[i] << ","
            //               << "roadpoint_lat[" << i << "] = " << setprecision(12) << roadpoint_lat[i]
            //               << endl;
            // }

            // 移动机器人参考线速度和角速度
            LOG(INFO) << "Point_line_velocity=" << Point_line_velocity << ","
                      << "Point_angular_velocity" << Point_angular_velocity << endl;

            // COPPESIM实时位姿更新
            GPSPOS_flag_mtx.lock();
            last_Latitude = global_trace.latitude;
            last_Longitude = global_trace.longitude;
            hangxiang = global_trace.heading;
            // 重要信息更新
            LOG(INFO) << "ON Trace Running, road_flag = " << road_flag << ","
                      << " roadpoint_num = " << roadpoint_num << ","
                      << " last_Longitude=" << setprecision(12) << last_Longitude << ","
                      << " last_Latitude = " << setprecision(12) << last_Latitude << ","
                      << " hangxiang=" << hangxiang << endl;
            GPSPOS_flag_mtx.unlock();
            // 目标路点更新（下一个路点）
            Next_Latitude = roadpoint_lat[road_flag];
            Next_Longitude = roadpoint_lon[road_flag];
            aim_hangx = Azimuth_caculate(last_Longitude, last_Latitude, roadpoint_lon[road_flag], roadpoint_lat[road_flag]);
            LOG(INFO) << " Next Longitude = " << setprecision(12) << Next_Longitude << ","
                      << " Next Latitude =" << setprecision(12) << Next_Latitude << ","
                      << " aim_hangx =" << aim_hangx << endl;

            // 目标路点更新（上一个路点）
            if (roadpoint_num >= 2 && road_flag >= 1)
            {
                Next_Latitude_before = roadpoint_lat[road_flag - 1];
                Next_Longitude_before = roadpoint_lon[road_flag - 1];
                LOG(INFO) << " Next_Longitude_before = " << setprecision(12) << Next_Longitude_before << ","
                          << " Next_Latitude_before =" << setprecision(12) << Next_Latitude_before << endl;
            }
            else
            {
                LOG(ERROR) << "Next_Latitude_before and Next_Longitude_before Error!" << endl;
            }

            /*--车实时位置经、纬度微分刻度(米)，分别沿经、纬线圈--*/
            Longitude_kedu = (2.0 * M_PI * R_earth) / (360.0);
            Latitude_kedu = fabs((2.0 * M_PI * R_earth * cos(last_Latitude / RADIANS_TO_DEGREES)) / (360.0));
            LOG(INFO) << " Longitude_kedu = " << setprecision(12) << Longitude_kedu << ","
                      << " Latitude_kedu =" << setprecision(12) << Latitude_kedu << endl;

            /*----目标点转化为笛卡尔坐标系的位置------*/
            convert_next_y = (Next_Latitude - system_latitude) * ((2.0 * M_PI * R_earth) / 360.0);
            convert_next_x = (Next_Longitude - system_longitude) * (fabs((2.0 * M_PI * R_earth * cos(Next_Latitude / RADIANS_TO_DEGREES)) / 360.0)); // system_latitude
            LOG(INFO) << " convert_next_x = " << setprecision(12) << convert_next_x << ","
                      << " convert_next_y =" << setprecision(12) << convert_next_y << endl;

            /*-----车在笛卡尔坐标系下的位置-----*/
            convert_car_y = ((last_Latitude - system_latitude) * ((2.0 * M_PI * R_earth) / 360.0));
            convert_car_x = ((last_Longitude - system_longitude) * (fabs((2.0 * M_PI * R_earth * cos(last_Latitude / RADIANS_TO_DEGREES)) / 360.0)));
            LOG(INFO) << " convert_car_x = " << setprecision(12) << convert_car_x
                      << " convert_car_y =" << setprecision(12) << convert_car_y << endl;

            LOG(INFO) << " car_radious = " << car_radious << ","
                      << " traceZX_gain = " << traceZX_gain << ","
                      << " final_distanerror = " << final_distanerror << endl;
            /*----------圆与直线交点求取----------*/
            if (convert_next_x != 0.0 && convert_next_y != 0.0)
            {
                // 斜率
                k = convert_next_y / convert_next_x;
                LOG(INFO) << " --<-convert_next_x!=0 && convert_next_y!=0->--"
                          << " k = " << k << endl;

                // 求取此时车实时纬度对应的经度值--期望值
                convert_car_y_hope = convert_car_x * k;
                jie_criteria = (2.0 * convert_car_x + 2.0 * k * convert_car_y) * (2.0 * convert_car_x + 2.0 * k * convert_car_y) - 4.0 * (1.0 + k * k) * (convert_car_x * convert_car_x + convert_car_y * convert_car_y - car_radious * car_radious);

                // 笛卡尔坐标系下圆与直线交点的坐标
                if (jie_criteria < 0.0)
                {
                    LOG(INFO) << " No Intersection! ,jie_criteria = " << jie_criteria << endl;
                }
                else
                {
                    x1 = (((2 * convert_car_x + 2 * k * convert_car_y) + sqrt(jie_criteria))) / (2 * (1 + k * k));
                    y1 = x1 * k;
                    x2 = (((2 * convert_car_x + 2 * k * convert_car_y) - sqrt(jie_criteria))) / (2 * (1 + k * k));
                    y2 = x2 * k;
                    LOG(INFO) << "Have Intersection!"
                              << " x1 = " << x1 << ","
                              << " y1 =" << y1 << ","
                              << " x2 =" << x2 << ","
                              << " y2 =" << y2 << ","
                              << " jie_criteria =" << jie_criteria << endl;
                }
                // 圆心到直线垂点的距离求取
                // 直角坐标
                x_chui = (convert_car_x + k * convert_car_y) / (k * k + 1);
                y_chui = (k * convert_car_x + k * k * convert_car_y) / (k * k + 1);
                // 点到直线的距离--验证用
                xy_chui_line1 = (fabs(k * convert_car_x - convert_car_y)) / (sqrt(1 + k * k));
                xy_chui_line2 = sqrt((convert_car_x - x_chui) * (convert_car_x - x_chui) + (convert_car_y - y_chui) * (convert_car_y - y_chui)); // 点到垂点的距离--验证用
                LOG(INFO) << "[Distance from point to line, For verification]"
                          << " x_chui = " << x_chui << ","
                          << " y_chui =" << y_chui << ","
                          << " xy_chui_line1 =" << xy_chui_line1 << ","
                          << " xy_chui_line2 =" << xy_chui_line2 << endl;
                LOG(INFO) << " --<-convert_next_x!=0 && convert_next_y!=0->---OVER!!!--" << endl;
            }
            else if (convert_next_x == 0 && convert_next_y != 0)
            {
                LOG(INFO) << " --<-convert_next_x==0 && convert_next_y!=0,k = wuqiong->--" << endl;
                k_inf_L1 = sqrt(car_radious * car_radious - (convert_car_x * convert_car_x));
                x1 = 0;
                y1 = convert_car_y + k_inf_L1;
                x2 = 0;
                y2 = convert_car_y - k_inf_L1;
                LOG(INFO) << " x1 = " << x1 << ","
                          << " y1 =" << y1 << ","
                          << " x2 =" << x2 << ","
                          << " y2 =" << y2 << endl;
                // 求取此时车实时纬度对应的经度值--期望值
                convert_car_y_hope = convert_next_y;
                // 圆心到直线垂点的距离求取
                x_chui = 0;
                y_chui = convert_car_y;
                // 点到直线的距离
                xy_chui_line1 = (fabs(k * convert_car_x - convert_car_y)) / (sqrt(1 + k * k));
                // 点到垂点的距离
                xy_chui_line2 = sqrt((convert_car_x - x_chui) * (convert_car_x - x_chui) + (convert_car_y - y_chui) * (convert_car_y - y_chui));
                LOG(INFO) << " x_chui = " << x_chui << ","
                          << " y_chui =" << y_chui << ","
                          << " xy_chui_line1 =" << xy_chui_line1 << ","
                          << " xy_chui_line2 =" << xy_chui_line2 << endl;
                LOG(INFO) << " --<--convert_next_x==0 && convert_next_y!=0,k = wuqiong->---OVER!!!--" << endl;
            }
            else if (convert_next_x != 0 && convert_next_y == 0)
            {
                LOG(INFO) << " --<-convert_next_x!=0 && convert_next_y==0,k=0->--" << endl;
                k_inf_L2 = sqrt(car_radious * car_radious - (convert_car_y * convert_car_y));
                x1 = convert_car_x + k_inf_L2;
                y1 = 0;
                x2 = convert_car_x - k_inf_L2;
                y2 = 0;
                LOG(INFO) << " x1 = " << x1 << ","
                          << " y1 =" << y1 << ","
                          << " x2 =" << x2 << ","
                          << " y2 =" << y2 << endl;

                // 求取此时车实时纬度对应的经度值--期望值
                convert_car_y_hope = 0;
                // 圆心到直线垂点的距离求取
                x_chui = convert_car_x;
                y_chui = 0;
                xy_chui_line1 = (fabs(k * convert_car_x - convert_car_y)) / (sqrt(1 + k * k));                                                   // 点到直线的距离
                xy_chui_line2 = sqrt((convert_car_x - x_chui) * (convert_car_x - x_chui) + (convert_car_y - y_chui) * (convert_car_y - y_chui)); // 点到垂点的距离
                LOG(INFO) << " x_chui = " << x_chui << ","
                          << " y_chui =" << y_chui << ","
                          << " xy_chui_line1 =" << xy_chui_line1 << ","
                          << " xy_chui_line2 =" << xy_chui_line2 << endl;
                LOG(INFO) << " --<-convert_next_x!=0 && convert_next_y==0,k=0>---OVER!!!--" << endl;
            }
            else if (convert_next_x == 0 && convert_next_y == 0)
            {
                LOG(INFO) << " --<-convert_next_x==0 && convert_next_y==0->--" << endl;
                x1 = 0;
                y1 = 0;
                x2 = 0;
                y2 = 0;
                LOG(INFO) << "x1 =" << x1 << ","
                          << "y1 =" << y1 << ","
                          << "x2 =" << x2 << ","
                          << "y2 =" << y2 << endl;
                // 圆心到直线垂点的距离求取
                x_chui = 0;
                y_chui = 0;
                xy_chui_line1 = (fabs(k * convert_car_x - convert_car_y)) / (sqrt(1 + k * k));                                                   // 点到直线的距离
                xy_chui_line2 = sqrt((convert_car_x - x_chui) * (convert_car_x - x_chui) + (convert_car_y - y_chui) * (convert_car_y - y_chui)); // 点到垂点的距离
                LOG(INFO) << "Cirle Point distance to chuidian:"
                          << " x_chui = " << x_chui << ","
                          << " y_chui =" << y_chui << ","
                          << " xy_chui_line1 =" << xy_chui_line1 << ","
                          << " xy_chui_line2 =" << xy_chui_line2 << endl;
                LOG(INFO) << " --<-convert_next_x==0 && convert_next_y==0>---OVER!!!--" << endl;
            }

            // 笛卡尔坐标系下两个交点坐标转为经、纬度
            y1_lat = (y1 / ((2 * M_PI * R_earth) / 360)) + system_latitude;
            x1_lon = (x1 / (fabs((2 * M_PI * R_earth * cos(y1_lat / RADIANS_TO_DEGREES)) / 360))) + system_longitude;
            y2_lat = (y2 / ((2 * M_PI * R_earth) / 360)) + system_latitude;
            x2_lon = (x2 / (fabs((2 * M_PI * R_earth * cos(y2_lat / RADIANS_TO_DEGREES)) / 360))) + system_longitude;
            LOG(INFO) << "Point 1、2 transfer Lat and Lon:"
                      << " y1_lat = " << setprecision(12) << y1_lat << ","
                      << " x1_lon =" << setprecision(12) << x1_lon << ","
                      << " y2_lat =" << setprecision(12) << y2_lat << ","
                      << " x2_lon =" << setprecision(12) << x2_lon << endl;
            // 求两个点到目标点的距离
            x1y1_line = Calculation_distance(x1_lon, y1_lat, Next_Longitude, Next_Latitude);
            x2y2_line = Calculation_distance(x2_lon, y2_lat, Next_Longitude, Next_Latitude);
            LOG(INFO) << "Point1、2 distance to Aimpoint:"
                      << " x1y1_line = " << x1y1_line << ","
                      << " x2y2_line =" << x2y2_line << endl;

            // 垂点经、纬度
            y_chui_lat = (y_chui / ((2 * M_PI * R_earth) / 360)) + system_latitude;
            x_chui_lon = (x_chui / (fabs((2 * M_PI * R_earth * cos(y_chui_lat / RADIANS_TO_DEGREES)) / 360))) + system_longitude;
            // 得到车位置到垂点的(交点)距离
            xy_chui_line = Calculation_distance(last_Longitude, last_Latitude, x_chui_lon, y_chui_lat);
            LOG(INFO) << "Car distance to chuidian:"
                      << " x_chui_lon = " << setprecision(12) << x_chui_lon << ","
                      << " y_chui_lat =" << setprecision(12) << y_chui_lat << ","
                      << " xy_chui_line =" << xy_chui_line << endl;

            // 实时经度对应的期望纬度
            convert_car_y_hoplat = convert_car_y_hope / ((2 * M_PI * R_earth) / 360) + system_latitude;
            pos_error = (convert_car_y_hoplat - last_Latitude) * ((2 * M_PI * R_earth) / 360);
            LOG(INFO) << "Longitude Corresponding ideal latitude:"
                      << ","
                      << " convert_car_y_hoplat = " << convert_car_y_hoplat << ","
                      << " last_Longitude =" << last_Longitude << ","
                      << " pos_error =" << pos_error << endl;

            // 车位置与目标点距离
            track_line = Calculation_distance(last_Longitude, last_Latitude, Next_Longitude, Next_Latitude);
            LOG(INFO) << "track_line car_to_aim:"
                      << " track_line = " << track_line << endl;

            // 判断是否有交点
            if (xy_chui_line <= car_radious && track_line > car_radious)
            {
                // 判断下一拍跟踪点
                if (x1y1_line > x2y2_line)
                {
                    lon = x2_lon; // 轨迹交点,包含相切
                    lat = y2_lat;
                }
                else
                {
                    lon = x1_lon;
                    lat = y1_lat;
                }
                // 得到车位置到轨迹上点(交点)距离
                new_track_line = Calculation_distance(last_Longitude, last_Latitude, lon, lat);
                track_Azimuth = Azimuth_caculate(last_Longitude, last_Latitude, lon, lat);
                LOG(INFO) << "Car To jin_jiaodian:"
                          << " track_Azimuth = " << track_Azimuth << ","
                          << " new_track_line = " << new_track_line << endl;
                LOG(INFO) << "Have Intersections,track_line(car to aim)<car_radious:"
                          << " lon = " << setprecision(12) << lon << ","
                          << " lat = " << setprecision(12) << lat << endl;
            }
            else if (xy_chui_line > car_radious && track_line > car_radious)
            {
                lon = x_chui_lon;
                lat = y_chui_lat;
                new_track_line = xy_chui_line;
                // 得到车位置到垂点(交点)方位角
                track_Azimuth = Azimuth_caculate(last_Longitude, last_Latitude, lon, lat);
                LOG(INFO) << "track_Azimuth2 of car_to_chuidian:"
                          << " track_Azimuth = " << track_Azimuth << endl;
                LOG(INFO) << "No Intersection,track_line>car_radious:"
                          << " lon = " << setprecision(12) << lon
                          << " lat = " << setprecision(12) << lat
                          << " line_chuizhi = " << new_track_line << endl;
            }
            else if (track_line <= car_radious)
            {
                lon = Next_Longitude;
                lat = Next_Latitude;
                new_track_line = track_line;
                // 得到车位置到垂点(交点)方位角
                track_Azimuth = Azimuth_caculate(last_Longitude, last_Latitude, lon, lat);
                LOG(INFO) << "track_line<=car_radious:"
                          << " track_Azimuth = " << track_Azimuth << endl;
                LOG(INFO) << "track_line<car_radious,target point is arriving:"
                          << " lon = " << setprecision(12) << lon
                          << " lat = " << setprecision(12) << lat
                          << " new_track_line = " << new_track_line << endl;
            } /* end (track_line <= car_radious)*/

            /*-----控制律输出的是线速度为1-----*/
            Coordinate_System();
            /*-------将控制指令下发-------*/
            LOG(INFO) << "---[Control_MSG] Start Setdown..---" << endl;
            // 左转是+，右转是-
            if (hangxiang >= track_Azimuth) // 航向角>=方位角
            {
                LOG(INFO) << "hangxiang >= track_Azimuth:"
                          << " hangxiang = " << hangxiang << ","
                          << " track_Azimuth = " << track_Azimuth << endl;
                if (hangxiang - track_Azimuth <= CarAngleLimit)
                {
                    LOG(INFO) << "Hangxiang-track_Azimuth<180:"
                              << " hangxiang = " << hangxiang << ","
                              << " track_Azimuth = " << track_Azimuth << endl;
                    // 走直线
                    if ((hangxiang - track_Azimuth) <= StraightAngle1)
                    {
                        traceLineVelocity = traceMax * Linear_velocity_robot; // 这个速度值在控制律中置了1,因此是1！
                        TraceZXAngle = 0.0;
                        LOG(INFO) << "Straight:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << endl;
                    }
                    // 快速转弯 左传
                    else if ((hangxiang - track_Azimuth > StraightAngle1) &&
                             (hangxiang - track_Azimuth <= MoreVel_Angle1))
                    {
                        traceLineVelocity = traceBigger * Linear_velocity_robot;
                        TraceZXAngle = traceZX_gain * fabs(hangxiang - track_Azimuth); // 差最大是30*0.7
                        LOG(INFO) << "Left Quick:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << endl;
                    }
                    // 慢速转弯 左传 角度最大
                    else
                    {
                        traceLineVelocity = traceMin * Linear_velocity_robot;
                        TraceZXAngle = traceZX_gain * fabs(hangxiang - track_Azimuth);
                        LOG(INFO) << "Left Slow:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << endl;
                    }
                } /* end ((hangxiang - track_Azimuth)<180)*/
                else
                {
                    LOG(INFO) << "Hangxiang-track_Azimuth>180:"
                              << " hangxiang = " << hangxiang << ","
                              << " track_Azimuth = " << track_Azimuth << endl;
                    // 直行
                    if (hangxiang - track_Azimuth >= StraightAngle2)
                    {
                        traceLineVelocity = traceMax * Linear_velocity_robot;
                        TraceZXAngle = 0.0;
                        LOG(INFO) << "Straight:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << endl;
                    }
                    // 快速右转弯
                    else if ((hangxiang - track_Azimuth < StraightAngle2) &&
                             (hangxiang - track_Azimuth >= MoreVel_Angle2))
                    {
                        traceLineVelocity = traceBigger * Linear_velocity_robot;
                        TraceZXAngle = -traceZX_gain * fabs(hangxiang - track_Azimuth); // 线速度、角速度求解的转向角
                        LOG(INFO) << "Right Quick:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                    // 慢速右转弯
                    else
                    {
                        traceLineVelocity = traceMin * Linear_velocity_robot;
                        TraceZXAngle = -traceZX_gain * fabs(hangxiang - track_Azimuth); // 线速度、角速度求解的转向角
                        LOG(INFO) << "Right Slow:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                } /* end ((hangxiang - track_Azimuth)>180)*/
            }     /* end (hangxiang >= track_Azimuth)*/
            else  // (hangxiang < track_Azimuth)
            {
                LOG(INFO) << "hangxiang < track_Azimuth:"
                          << " hangxiang = " << hangxiang << ","
                          << " track_Azimuth = " << track_Azimuth << endl;
                // 差小于180
                if (track_Azimuth - hangxiang <= CarAngleLimit)
                {
                    LOG(INFO) << "track_Azimuth-Hangxiang<180:"
                              << " hangxiang = " << hangxiang << ","
                              << " track_Azimuth = " << track_Azimuth << endl;
                    // 走直线
                    if ((track_Azimuth - hangxiang) <= StraightAngle1)
                    {
                        traceLineVelocity = traceMax * Linear_velocity_robot;
                        TraceZXAngle = 0.0;
                        LOG(INFO) << "Straight:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                    // 快速右转
                    else if (((track_Azimuth - hangxiang >= StraightAngle1)) &&
                             (track_Azimuth - hangxiang < MoreVel_Angle1))
                    {
                        traceLineVelocity = traceBigger * Linear_velocity_robot;
                        TraceZXAngle = -traceZX_gain * fabs(hangxiang - track_Azimuth); // 转向角
                        LOG(INFO) << "Right Quick:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                    // 慢速右转
                    else
                    {
                        traceLineVelocity = traceMin * Linear_velocity_robot;
                        TraceZXAngle = -traceZX_gain * fabs(hangxiang - track_Azimuth); // 转向角
                        LOG(INFO) << "Right Slowly:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                }    /*end ((track_Azimuth-hangxiang )<180)*/
                else // ((track_Azimuth-hangxiang )>180)
                {
                    LOG(INFO) << "track_Azimuth-hangxiang>180:"
                              << " hangxiang = " << hangxiang << ","
                              << " track_Azimuth = " << track_Azimuth << endl;
                    // 走直线
                    if ((track_Azimuth - hangxiang) > StraightAngle2)
                    {
                        traceLineVelocity = traceMax * Linear_velocity_robot;
                        TraceZXAngle = 0.0;
                        LOG(INFO) << "Straight:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                    // 快速左转
                    else if (((track_Azimuth - hangxiang) <= StraightAngle2) &&
                             (track_Azimuth - hangxiang > MoreVel_Angle2))
                    {
                        traceLineVelocity = traceBigger * Linear_velocity_robot;
                        TraceZXAngle = traceZX_gain * fabs(hangxiang - track_Azimuth); // 转向角
                        LOG(INFO) << "Left Quick:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                    // 慢速左转
                    else
                    {
                        traceLineVelocity = traceMin * Linear_velocity_robot;
                        TraceZXAngle = traceZX_gain * fabs(hangxiang - track_Azimuth); // 转向角
                        LOG(INFO) << "Left Slow:"
                                  << " hangxiang = " << hangxiang << ","
                                  << " track_Azimuth = " << track_Azimuth << ","
                                  << " traceLineVelocity = " << traceLineVelocity << ","
                                  << " TraceZXAngle = " << TraceZXAngle << endl;
                    }
                } /* end ((track_Azimuth-hangxiang )>180)*/
            }     /* end (hangxiang< track_Azimuth)*/

            /*--速度至最后一个路点缓慢的停下来，仅仅关注最后一个路点--*/
            if (road_flag == roadpoint_num - 1)
            {
                LOG(INFO) << "Car Run to the last PathPoint:"
                          << " road_flag = " << road_flag << ","
                          << " roadpoint_num = " << roadpoint_num << endl;
                // if (track_line <= 5.0 && track_line > 2.0)
                // {
                //     traceLineVelocity = 0.8 * traceMin * Linear_velocity_robot;
                //     LOG(INFO) << "track_line <= 5.0 && track_line > 2.0 :"
                //               << " hangxiang = " << hangxiang << ","
                //               << " track_Azimuth = " << track_Azimuth << ","
                //               << " traceLineVelocity = " << traceLineVelocity << ","
                //               << " TraceZXAngle = " << TraceZXAngle << ","
                //               << " track_line = " << track_line << endl;
                // }
                // if (track_line <= 0.5)
                // {
                //     traceLineVelocity = 0.4 * traceMin * Linear_velocity_robot;
                //     LOG(INFO) << "track_line <= 2 :"
                //               << " hangxiang = " << hangxiang << ","
                //               << " track_Azimuth = " << track_Azimuth << ","
                //               << " traceLineVelocity = " << traceLineVelocity << ","
                //               << " TraceZXAngle = " << TraceZXAngle << ","
                //               << " track_line = " << track_line << endl;
                //     // aimPoints_lenth = Calculation_distance(Next_Longitude_before,Next_Latitude_before,Next_Longitude,Next_Latitude);
                //     // traceLineVelocity=(track_line/aimPoints_lenth)*Linear_velocity_robot;
                //     // printf("3.3 track_line=%lf,traceLineVelocity=%f,road_flag=%d\n",track_line,traceLineVelocity,road_flag);
                // }
                // printf("trace may stop and the last V=TraceLine_V=%f\n",traceLineVelocity);
            } /* end (road_flag == roadpoint_num - 1)*/
            // 观察车距离
            LOG(INFO) << "CHECK!! the car arrived? track_line <=" << track_line << endl;

            /*--判断车是否到达某一目标点，可能会存在问题！！距离到了角度没到，或者相反--*/
            if (track_line <= final_distanerror)
            {
                LOG(INFO) << "one target point arrived, track_line <=  :" << track_line << endl;
                road_flag++;
                LOG(INFO) << "Change road_flag: "
                          << "road_flag= " << road_flag << endl;
                // 更新原点经、纬度
                system_latitude = last_Latitude;
                system_longitude = last_Longitude;
                LOG(INFO) << "Change system_latitude system_longitude : "
                          << "system_longitude= " << setprecision(12) << system_longitude
                          << "system_latitude= " << setprecision(12) << system_latitude << endl;
                if (road_flag > (roadpoint_num - 1)) // 跟踪结束
                {
                    // 仿真时屏蔽,实物时打开
                    printf("LINE:%d, the process of trace is over now!\n\n", __LINE__);
                    global_trace.RobotModel = ROBWAIT;
                    // road_flag=0;//从头重新来
                }
                else
                {
                    LOG(INFO) << "Process of trace isn't over,keep tracing!" << endl;
                }
            } /* end (track_line <= final_distanerror)*/
            else
            {
                LOG(INFO) << "One target point don't arrive!" << endl;
            }

            /*--------------------机器人速度下发--------------------*/
            /**
             * 遥控的控制量是在,信息收发线程中处理和控制的,
             * 自主模式的控制量是在下面进行的约束,
             * 多种量准备下发！
             */
            // 约束
            // 角速度
            if (TraceZXAngle > trace_MostZxAngle)
            {
                TraceZXAngle = trace_MostZxAngle;
            }
            else if (TraceZXAngle < -trace_MostZxAngle)
            {
                TraceZXAngle = -trace_MostZxAngle;
            }

            // 线速度
            if (traceLineVelocity > trace_MostAcc)
            {
                traceLineVelocity = trace_MostAcc;
            }
            else if (traceLineVelocity < -trace_MostAcc)
            {
                traceLineVelocity = -trace_MostAcc;
            }
            LOG(INFO) << "Before Send to Motor:"
                      << " hangxiang = " << hangxiang << ","
                      << " track_Azimuth = " << track_Azimuth << ","
                      << " vehicleAngularVelocity = " << TraceZXAngle << ","
                      << "  Accerlerator = " << traceLineVelocity << endl;

            /*-------------安全保障机制,经纬度不正常车就停下来------------*/
            if (((last_Latitude - 0.0) <= 1e-7) || ((last_Longitude - 0.0) <= 1e-7))
            {
                LOG(ERROR) << "last_Latitude and last_Longitude recv ERROR!" << endl;
                TraceZXAngle = 0.0;
                traceLineVelocity = 0.0;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            /*------------------机器人总体的状态切换-----------------*/
            // 跟踪暂停:只是速度置0 //
            if (global_trace.RobStatusFlag == TracePause)
            {
                if ((GeneralStatusDog++) >= 10)
                {
                    LOG(INFO) << "Trace_Pause..,Then continue!......" << endl;
                    continue;
                } // 10拍之后不再循环
                TraceZXAngle = 0.0;
                traceLineVelocity = 0.0;
                LOG(INFO) << "Trace_Pause........" << endl;
            }
            // 跟踪停止:速度置0 + 控制模式切换 //
            else if (global_trace.RobStatusFlag == TraceStop)
            {
                if ((GeneralStatusDog++) == 10) // 10拍之后跳出自主循环
                {
                    RobotModel_mtx.lock();
                    global_trace.RobotModel = ROBWAIT;
                    RobotModel_mtx.unlock();
                    road_flag = 0;
                }
                TraceZXAngle = 0.0;
                traceLineVelocity = 0.0;
                LOG(INFO) << "Trace_Stop!!!!!!!!!!" << endl;
            }
            else if (global_trace.RobStatusFlag == TraceContinue)
            {
                GeneralStatusDog = 0;
                LOG(INFO) << "Trace_Continue!!!!!!!!!!" << endl;
            }
            else
            {
                GeneralStatusDog = 0;
                LOG(INFO) << "Trace_other staus!!!!!!!!!" << endl;
            }
            /*-----------###-------这部分管下发------###-----------*/
            // 速度拷贝
            VELandANG_mtx.lock();
            Driver_trace.ControllerMsg_.vehicleAngularVelocity = TraceZXAngle;
            Driver_trace.ControllerMsg_.Accerlerator = traceLineVelocity;
            //----*Vrep仿真下发*----*消息队列-->copplisim*----//
            // msg_QT2coppSim.mtype = 1;
            // msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = Driver_trace.ControllerMsg_.vehicleAngularVelocity;
            // msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = Driver_trace.ControllerMsg_.Accerlerator;
            VELandANG_mtx.unlock();
            // LOG(INFO) << "ControlMSG from trace: "
            //           << "msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity= " << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << ","
            //           << "msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity << endl;
            // datacacu++;
            cout << "Driver_trace.ControllerMsg_.Accerlerator" << Driver_trace.ControllerMsg_.Accerlerator
                 << "Driver_trace.ControllerMsg_.vehicleAngularVelocity" << Driver_trace.ControllerMsg_.vehicleAngularVelocity << endl;

            //----*CAN通信--->龚老师小黄车*----//
            // VELandANGsend(Driver_trace.ControllerMsg_.Accerlerator + 1e-5, Driver_trace.ControllerMsg_.vehicleAngularVelocity + 1e-5, 1, datacacu);

            //----*UDP-->中广核*----//
            // msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = Driver_trace.ControllerMsg_.vehicleAngularVelocity;
            // msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = Driver_trace.ControllerMsg_.Accerlerator;
            // LOG(INFO) << "ControlMSG from trace: "
            //           << "msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity= " << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << ","
            //           << "msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity << endl;

            /*--------以下记录数据周期，数据存储---------*/
            // if (dataSAVE++ == 50)
            // {
            //     dataSAVE = 0;
            //     trace_data = fopen("trace_data.txt", "a+");
            //     if (trace_data == NULL)
            //     {
            //         printf("File line cannot open!\n");
            //         exit(0);
            //     }
            //     fprintf(trace_data, "%d/%d/%d %d:%d:%d," // 时间
            //                         "%.10f,%.10f,%.10f," // 车的经度、纬度和航向角
            //                         "%f,%f\n",           // 角速度、小速度
            //             getTime->tm_year + 1900,
            //             getTime->tm_mon + 1,
            //             getTime->tm_mday,
            //             getTime->tm_hour,
            //             getTime->tm_min,
            //             getTime->tm_sec,

            //             last_Latitude,
            //             last_Longitude,
            //             hangxiang,

            //             msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity,
            //             msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity);
            //     fclose(trace_data);
            // }
        } /*else if(auto update)*/
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // LOG(INFO) << " Can't start any Control model!!" << endl;
        }
    } /*while over*/
    // Plot_tarANDactual_POS.join();
    LOG(INFO) << " trace Thread is OVER!" << endl;
} /*trace over*/
