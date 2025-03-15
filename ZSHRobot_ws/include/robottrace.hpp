#ifndef ROBOTTRACE_HPP
#define ROBOTTRACE_HPP
#include <vector>
#include <Eigen/Dense>

class robottrace
{
private:
    /* data */
    float traceMax;    // 轨迹跟踪时最大速度
    float traceBigger; // 轨迹跟踪时比较大速度
    float traceMin;    // 轨迹跟踪时最小速度
    float P_gain;

    float trace_MostZxAngle; // 前轮转向的最大角度
    float trace_MostAcc;     // 油门量的最大百分比
    float trace_minAcc;     // 油门量的最大百分比

    float CarAngleLimit;  // 轨迹跟踪 角度分割界限180度
    float StraightAngle1; // 轨迹跟踪 直线行驶
    float MoreVel_Angle1; // 轨迹跟踪 较大速度转弯
    float StraightAngle2; // 轨迹跟踪 直线行驶
    float MoreVel_Angle2; // 轨迹跟踪较大速度转弯
    double lon, lat;      // 车的经、纬度

    // 经纬度的传递，存储实际的经纬度
    double last_Longitude, last_Latitude, hangxiang;
    double Next_Longitude, Next_Latitude;
    double Next_Longitude_before, Next_Latitude_before;
    double save_Linear_deta_velocity, save_Angular_deta_velocity;
    double car_radious, traceZX_gain, final_distanerror;

    // point_x1为坐标原点0的X轴距离
    double point_x1, point_x2;

    // point_y1为坐标原点0的Y轴距离
    double point_y1, point_y2;
    double error_x, error_y;

    // e1,e2,e3为方位角误差
    double e1, e2, e3;
    double Thit, Thit_r, Thit_r_, Thit_error;

    FILE *fp_RoadPointRead;

    // 移动机器人前进参考的线速度0.4,移动机器人当前的角速度0.3
    double Point_line_velocity, Point_angular_velocity;
    // 为变速度(线速度、角速度)准备
    double Linear_deta_velocity, Angular_deta_velocity;
    // 可能是控制律部分
    double Angular_velocity_robot, Linear_velocity_robot;
    // 绘图
    std::vector<double> x_axis_vector, y_axis_vector, HX_vector;
    // 期望轨迹路点的经、纬度[绘图]
    std::vector<double> Tar_X_axis_vector, Tar_Y_axis_vector;
    // 三点轨迹矩阵法，三个点[绘图]
    std::vector<double> PathPoint_lon_PlanTar, PathPoint_lat_PlanTar;

    /**
     * @brief PathPoint_lon
     *  存储路径规划部分生成的所有路点
     *  三点矩阵法生成的期望路点的经、纬度
     */
    std::vector<double> PathPoint_lon, PathPoint_lat;
    // 等待的轨迹绘制
    std::vector<double> WaitPathPointlon, WaitPathPointlat;
    // kcurve之后的绘图
    std::vector<double> KcurvePlot_X, KcurvePlot_Y;
    // kcurve期望路点
    std::vector<double> KcurvePlot_X_target, KcurvePlot_Y_target;

public:
    robottrace(/* args */);
    ~robottrace();
    double Calculation_distance(double Longitude, double Latitude, double Next_Longitude, double Next_Latitude);
    void Coordinate_System(void);
    double Azimuth_caculate(double Longitude_A, double Latitude_A, double Longitude_M, double Latitude_M);
    void RobotTrace_RUN();
    // 绘制期望位置和航实际的位置
    void Plot_TarANDActual_POS(void);
    void PatrolPathPlan(double lonAndlatFirst[4][2], int VerticalScale, int CrossScale);
    void PlanPathCurves(std::vector<Eigen::Vector2d> input);
};

#endif /* ROBOTTRACE_HPP */