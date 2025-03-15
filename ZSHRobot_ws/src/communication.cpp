#include <iostream>
#include <fstream>
#include <string>
#include <an_packet_protocol.h>
#include "communication.hpp"
#include <glog/logging.h>
#include <netinet/in.h>
#include "XMLParser.hpp"
#include "global.hpp"
#include "Driver.hpp"
#include <stdint.h>
#include <string.h>
#include <define.h>
#include <thread>
#include <sys/msg.h>
#include <chrono>

using namespace std;
MsgHuman2RobFrame MsgHuman2RobFrame_;
QT_msg2Coppeliasim msg_QT2coppSim;

communication::communication(/* args */)
{
}

communication::~communication()
{
}

void communication::Receive_PC_MSG(void)
{
    LOG(INFO) << "Receive_PC_MSG Start.." << endl;
}

void communication::Set2PC_MSG(void)
{
    LOG(INFO) << "Set2PC_MSG Start.." << endl;
}

void CommunicationID_reset(MsgHuman2RobFrame __MsgHuman2RobFrame)
{
    __MsgHuman2RobFrame.MsgHuman2Rob_.msgHeader.msgID = -1;
}

/**
 * @brief
 * 计算txt文件有多少行数据
 * @param filename
 * @return int
 */
int countLines(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return -1;
    }
    int lineCount = 0;
    std::string line;
    while (std::getline(file, line))
    {
        lineCount++;
    }
    // file.close();
    return lineCount;
}

/**
 * @brief
 * 删除txt文件
 * @param filename
 * @return int
 */
int Removefile(const char *filename)
{
    if (std::remove(filename) != 0)
    {
        std::cerr << "Failed to delete the file." << std::endl;
        return -1;
    }
    std::cout << "The file " << filename << " has been deleted." << std::endl;
    return 0;
}

void communication::communication_run(void)
{
    global global_;
    Driver Driver_;
    LOG(INFO) << "communication_run Start.." << endl;
    static int Brake_Data = 5, Mostbrake = 40, lowestbrake = 0; // 油门的变化量,前轮转向角速度的变化量.
    int MostZxAngle = 160;                                      // 前轮转向的最大角度
    int ZXangle_Data = 80;                                      // 前轮转向速度data
    int MostZxSpeed = 260;                                      // 前轮转向最大速度
    int lowestZxSpeed = 0;                                      // 前轮转向最小速度
    int Accelerator_Data = 5;                                   // 驱动速度变化量
    int MostAcc = 100;                                          // 油门量的最大百分比
    int lowestAcc = 0;                                          // 油门最小量的百分比
    static int lineCount_save = 0;

    // 接收UDP
    uint16_t recvcrc16_ret;
    struct timeval timeOut;
    timeOut.tv_sec = 0; // 阻塞时间设置 ,0.1s超时
    timeOut.tv_usec = 40000;
    int ret_joystick = 0;
    int socket_joystick;
    socklen_t addrlen_joystick = sizeof(struct sockaddr_in);
    if ((socket_joystick = socket(PF_INET, SOCK_DGRAM, 0)) == -1)
    {
        LOG(INFO) << "Create socket_joystick failed!" << endl;
        exit(-1);
    }
    struct sockaddr_in saddr_joystick;
    memset(&saddr_joystick, 0, sizeof(saddr_joystick));
    saddr_joystick.sin_family = AF_INET;
    saddr_joystick.sin_port = htons(9009); // 端口
    saddr_joystick.sin_addr.s_addr = INADDR_ANY;
    if (bind(socket_joystick, (struct sockaddr *)&saddr_joystick, sizeof(saddr_joystick)) == -1)
    {
        LOG(INFO) << "bind socket_joystick error!" << endl;
        exit(-1);
    }
    if (setsockopt(socket_joystick, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut)) < 0)
    {
        LOG(INFO) << "socket_joystick time out setting failed!" << endl;
    }
    memset(&MsgHuman2RobFrame_, 0, sizeof(MsgHuman2RobFrame_));
    int16_t Circle_period = 50; // 信息的接收要50ms,尽量快些
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(Circle_period)); // 10ms=100Hz
        // caculate cost time
        namespace sc = std::chrono;
        static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
        sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
        double fps = 0;
        double diff = (now_time - start_time).count();
        if (diff > 0)
        {
            fps = 1 / diff;
            start_time = now_time;
        }
        // LOG(INFO) << "communication_diff=" << diff << " s" << endl;
        // UDP接收
        ret_joystick = recvfrom(socket_joystick, &buff_joystick, sizeof(buff_joystick), 0, (struct sockaddr *)&saddr_joystick, &addrlen_joystick);
        if (ret_joystick < 0 && errno == EWOULDBLOCK)
        {
            // printf("Waiting for UDP's message.....%d\n", ret_joystick);
            // printf("sizeof(MsgHuman2RobFrame_)=%d\n", sizeof(MsgHuman2RobFrame_));
        }
        else
        {
            // printf("recve message succeed, %d\n", ret_joystick);
            memcpy((char *)&MsgHuman2RobFrame_, &buff_joystick, sizeof(MsgHuman2RobFrame_));
            recvcrc16_ret = calculate_crc16((void *)&MsgHuman2RobFrame_.MsgHuman2Rob_, sizeof(MsgHuman2RobFrame_.MsgHuman2Rob_));
            // 校验
            if (MsgHuman2RobFrame_.crc16 == recvcrc16_ret)
            {
                // 任务终止
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.RBZZRW_ID)
                {
                    RobotStatus_mtx.lock();
                    global_.RobotModel = global_.ROBWAIT;
                    global_.RobStatusFlag = TraceStop;
                    LOG(INFO) << "RBZZRW_ID." << endl;
                    RobotStatus_mtx.unlock();
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                // 任务暂停
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.RBZSTZ_ID)
                {
                    RobotStatus_mtx.lock();
                    global_.RobStatusFlag = TracePause;
                    LOG(INFO) << "RBZSTZ_ID." << endl;
                    RobotStatus_mtx.unlock();
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                // 任务继续
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.RBJXZX_ID)
                {
                    RobotStatus_mtx.lock();
                    global_.RobStatusFlag = TraceContinue;
                    LOG(INFO) << "RBJXZX_ID." << endl;
                    RobotStatus_mtx.unlock();
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                // 遥控模式
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.KQYKMS_ID)
                {
                    RobotModel_mtx.lock();
                    global_.RobotModel = global_.ROBTElE;
                    LOG(INFO) << "KQYKMS_ID,global_.RobotModel:" << global_.RobotModel << endl;
                    RobotModel_mtx.unlock();
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                // 自主模式
                if ((MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.KQZZMS_ID))
                {
                    RobotModel_mtx.lock();
                    global_.RobotModel = global_.ROBAUTO;
                    LOG(INFO) << "KQZZMS_ID,global_.RobotModel:" << global_.RobotModel << ","
                              << "global_.Waypoint_flag:" << global_.Waypoint_flag << endl;
                    RobotModel_mtx.unlock();
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                //--路径规划时负责跳转--//
                if ((MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.LJGH_ID))
                {
                    // 蛇形轨迹规划
                    if (MsgHuman2RobFrame_.MsgHuman2Rob_.LDGZ_.LDGZ_mode == 1)
                    {
                        LOG(INFO) << "LJGH_ID,global_.LDGZ_mode : 1" << endl;
                        RobotModel_mtx.lock();
                        global_.RobotModel = -1;
                        global_.Trace_Jump = true;
                        global_.Waypoint_flag = global_.RECTANGLE_XML_PLAN;
                        LOG(INFO) << "LJGH_ID,RECTANGLE_XML_PLAN" << endl;
                        RobotModel_mtx.unlock();
                    }
                    // 离散轨迹规划
                    else if (MsgHuman2RobFrame_.MsgHuman2Rob_.LDGZ_.LDGZ_mode == 2)
                    {
                        LOG(INFO) << "LJGH_ID,global_.LDGZ_mode : 2" << endl;
                        RobotModel_mtx.lock();
                        global_.RobotModel = -1;
                        global_.Trace_Jump = true;
                        global_.Waypoint_flag = global_.TXT_PLAN;
                        LOG(INFO) << "LJGH_ID,TXT_PLAN " << endl;
                        RobotModel_mtx.unlock();
                    }
                    else if (MsgHuman2RobFrame_.MsgHuman2Rob_.LDGZ_.LDGZ_mode == 3)
                    {
                        LOG(INFO) << "LJGH_ID,global_.LDGZ_mode : 3" << endl;
                    }
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                //--路径跟踪--//
                if ((MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.GJZZ_ID))
                {
                    // LOG(INFO) << "LJGH_ID,global_.GJZZ_ID:" << global_.GJZZ_ID << endl;
                }

                // 底盘遥控
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.KQDPYK_ID)
                {
                    LOG(INFO) << "KQDPYK_ID,UpOrDownDPteleFlag." << endl;
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 底盘运动控制
                // 通过RobotModel开关来控制是否是遥控数据和自主数据
                if ((MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.DPYDKZ_ID) && (global_.RobotModel == global_.ROBTElE)) // 遥控模式的数据
                {
                    global_.DPYDKZteleFlag = true;
                    /**
                     * 1.摇杆控制，摇杆向右推是正，摇杆向左推是负值;
                     * 2.但是实际上，机器人规定逆时针为正，顺时针为负；
                     * 3.因此，将摇杆的值加上负号，满足逆时针为正值得,同时将coppeliasim中的左传和右转做了更改
                     * 4.目前满足逆时针转为正，顺时针转为负值得，向前为正值，向后为负值！
                     */
                    VELandANG_mtx.lock();
                    //**----给机器人下发----*//
                    // 横向(-1,1)(-10,10)(-20,20)
                    Driver_.ControllerMsg_.vehicleAngularVelocity = -1 * 2 * MsgHuman2RobFrame_.MsgHuman2Rob_.DPYDKZ_.TeleControlData.vehicleAngularVelocity;
                    // 纵向(-1,1)
                    Driver_.ControllerMsg_.Accerlerator = 1 * MsgHuman2RobFrame_.MsgHuman2Rob_.DPYDKZ_.TeleControlData.Accerlerator;
                    //**----给Vrep下发----*//
                    // 限制旋转量的角度阈值
                    msg_QT2coppSim.mtype = 1;
                    msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = Driver_.ControllerMsg_.vehicleAngularVelocity;
                    msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = Driver_.ControllerMsg_.Accerlerator;
                    // LOG(INFO) << "msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity=" << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << ","
                    //           << " msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity=" << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity;
                    VELandANG_mtx.unlock();
                }

                // 高海拔机器人档位变更
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.DWBG_ID)
                {
                }

                // 抢运机器人构型模式
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.DPGXMS_ID)
                {
                }

                // 抢运机器人越障模式
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.KQYZMS_ID)
                {
                    global_.Trace_Jump = true;
                    global_.RobotModel = -1;
                    global_.Waypoint_flag = global_.RECTANGLE_XML_PLAN;
                    LOG(INFO) << "KQYZMS_ID,global_.RobotModel:" << global_.RobotModel << endl;
                    printf("global_.Trace_Jump=%d \n", global_.Trace_Jump);
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 高海拔机器人接收路点 界面下发Excel
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.FSLD_ID)
                {
                    printf("FSLD_ID,The pathpoints of AutoModel are received!!%d \n\n\n", global_.FSLD_ID);
                    memset(&global_.LatANDLon_, 0, sizeof(global_.LatANDLon_)); // 清0,然后再进行数值的拷贝
                    memcpy(global_.LatANDLon_.latitude, MsgHuman2RobFrame_.MsgHuman2Rob_.FSLD_.latitude,
                           sizeof(MsgHuman2RobFrame_.MsgHuman2Rob_.FSLD_.latitude));
                    memcpy(global_.LatANDLon_.longitude, MsgHuman2RobFrame_.MsgHuman2Rob_.FSLD_.longitude,
                           sizeof(MsgHuman2RobFrame_.MsgHuman2Rob_.FSLD_.longitude));
                    for (int i = 0; i < 50; i++)
                    {
                        printf("LatANDLon_.latitude[%d]=%.8f,LatANDLon_.longitude[%d]=%.8f\n",
                               i, global_.LatANDLon_.latitude[i], i, global_.LatANDLon_.longitude[i]);
                        // 找到路点到哪里停止
                        if (((global_.LatANDLon_.latitude[i] - 0) <= 1.0) || ((global_.LatANDLon_.longitude[i] - 0) <= 1.0))
                        {
                            printf("WayPoint Copy finish!\n");
                            // global_.Waypoint_flag = true;
                            break; // 跳出当前循环
                        }
                    }
                }

                // 高海拔发动机的上电启动与否
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.RBCMZT_ID)
                {
                }

                // 按键A 减小油门
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.YGGX1_ID)
                {
                    // SYStemFlag_.AcceleratorPosition_Gear = SYStemFlag_.AcceleratorPosition_Gear - Accelerator_Data;
                    // if (SYStemFlag_.AcceleratorPosition_Gear <= lowestAcc)
                    // {
                    //     SYStemFlag_.AcceleratorPosition_Gear = lowestAcc;
                    // }
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 按键Y 增大油门
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.YGGX2_ID)
                {
                    // SYStemFlag_.AcceleratorPosition_Gear = SYStemFlag_.AcceleratorPosition_Gear + Accelerator_Data;
                    // if (SYStemFlag_.AcceleratorPosition_Gear >= MostAcc)
                    // {
                    //     SYStemFlag_.AcceleratorPosition_Gear = MostAcc;
                    // }
                    // 继续
                    LOG(INFO) << "继续:global_.RobStatusFlag=" << global_.RobStatusFlag << endl;
                    printf("Y--Y !,msgID=%d\n", MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID);
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 按键X
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.YGGX4_ID)
                {
                    // CanSndMsg_.ACU1_msg_.TargetSteeringSpeed = CanSndMsg_.ACU1_msg_.TargetSteeringSpeed - ZXangle_Data;
                    // if (CanSndMsg_.ACU1_msg_.TargetSteeringSpeed <= lowestZxSpeed)
                    // {
                    //     CanSndMsg_.ACU1_msg_.TargetSteeringSpeed = lowestZxSpeed;
                    // }
                    // printf("3333CanSndMsg_.ACU1_msg_.TargetSteeringSpeed decrease!!,CanSndMsg_.ACU1_msg_.TargetSteeringSpeed=%d\n",
                    //        CanSndMsg_.ACU1_msg_.TargetSteeringSpeed);

                    // 终止
                    LOG(INFO) << "终止:global_.RobStatusFlag=" << global_.RobStatusFlag << endl;
                    printf("X--X !,msgID=%d\n", MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID);
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 按键B
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.YGGX3_ID)
                {
                    // CanSndMsg_.ACU1_msg_.TargetSteeringSpeed = CanSndMsg_.ACU1_msg_.TargetSteeringSpeed + ZXangle_Data;
                    // if (CanSndMsg_.ACU1_msg_.TargetSteeringSpeed >= MostZxSpeed)
                    // {
                    //     CanSndMsg_.ACU1_msg_.TargetSteeringSpeed = MostZxSpeed;
                    // }
                    // printf("4444ACU1_msg_.TargetSteeringSpeed zengjia!!,ACU1_msg_.TargetSteeringSpeed=%d\n",
                    //        CanSndMsg_.ACU1_msg_.TargetSteeringSpeed);
                    // 暂停
                    LOG(INFO) << "暂停:global_.RobStatusFlag=" << global_.RobStatusFlag << endl;
                    printf("B--B !,msgID=%d\n", MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID);
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 刹车变化-->左键为解除刹车，右键为存储路点！
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.SCBG_ID)
                {
                    // printf("!!!SCDWBG MsgHuman2RobFrame.MsgHuman2Rob.SCDWBG_.SCDW_model=%d\n",
                    // MsgHuman2RobFrame.MsgHuman2Rob.SCDWBG_.SCDW_model);
                    // 按键左肩膀键 增大刹车;后期由于刹车电机的损坏，将左侧肩膀按键改成清除刹车！！
                    // if (MsgHuman2RobFrame_.MsgHuman2Rob_.SCDWBG_.SCDW_model == 1)
                    {
                        printf("brake reset,Brake_flag!\n");
                        // control_msg.brakernum = control_msg.brakernum + Brake_Data;
                        // printf("1control_msg.brakernum=%f\n", control_msg.brakernum);
                        // if (control_msg.brakernum >= Mostbrake)
                        // {
                        //     control_msg.brakernum = Mostbrake;
                        //     printf("1.1control_msg.brakernum=%f\n", control_msg.brakernum);
                        // }
                    }
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }
                // 将轨迹跟踪改成路点采集了！！
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.GJZZ_ID)
                {
                    fp_RoadPointSave = fopen("RoadPointSave.txt", "a+"); // 存储采集的路点至文件中
                    if (fp_RoadPointSave == NULL)
                    {
                        printf("RoadPoints Save File cannot open!\n");
                        exit(0);
                    }
                    printf("global_.longitude = %.8lf,global_.latitude = %.8lf\n", global_.longitude, global_.latitude);
                    // 纬度对应的是y的值，经度对应的是x的值；这里的y对应的是slam的Z！
                    // 也就是存储.txt的时候,先存储的Y的值，然后再存储X的值
                    fprintf(fp_RoadPointSave, "%d  %.8lf  %.8lf\n",
                            lineCount_save + 1,
                            global_.longitude, // 经度，先存储的经度！
                            global_.latitude); // 纬度
                    printf("fprintf Road points succeed!\n");
                    fclose(fp_RoadPointSave);
                    int lineCount = countLines("RoadPointSave.txt");
                    printf("lineCount = %d,lineCount_save = %d\n", lineCount, lineCount_save);
                    if (lineCount >= 0)
                    {
                        std::cout << "The file contains " << lineCount << " lines." << std::endl;
                        if ((lineCount - lineCount_save) == 1)
                        {
                            global_.PointCollectFlag = 1;
                            printf("Collect One Pathpoint succeed! \n");
                        }
                        else
                        {
                            LOG(INFO) << "Collect One PathPoint failed!" << endl;
                        }
                    }
                    else
                    {
                        LOG(INFO) << "RoadPointSave.txt countLines failed!" << endl;
                    }
                    global_.PointTXT_lineCount = lineCount;
                    lineCount_save = lineCount;
                    CommunicationID_reset(MsgHuman2RobFrame_);
                }

                // 路点清除
                if (MsgHuman2RobFrame_.MsgHuman2Rob_.msgHeader.msgID == global_.LDGZ_ID)
                {
                    Removefile("RoadPointSave.txt");
                    fp_RoadPointSave = fopen("RoadPointSave.txt", "a+"); // 存储采集的路点至文件中
                    if (fp_RoadPointSave == NULL)
                    {
                        printf("RoadPoints Save File cannot open!\n");
                        exit(0);
                    }
                    fclose(fp_RoadPointSave);
                    int lineCount = countLines("RoadPointSave.txt");
                    global_.PointTXT_lineCount = lineCount;
                    lineCount_save = 0;
                }
            }
            else
            {
                printf("calculate_crc16 Error s%d\n", recvcrc16_ret);
            }
        }
    }
}
