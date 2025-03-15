#include <iostream>
#include "control.hpp"
#include <glog/logging.h>
#include <thread>
#include "XMLParser.hpp"
#include <sys/msg.h>
#include <stdio.h>
#include "Driver.hpp"
#include <string>
#include "global.hpp"
#include <sys/socket.h>
#include <linux/tcp.h>
#include <arpa/inet.h>
#include "Driver.hpp"
extern "C"
{
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
}

#define MSGQUE_H2S2MAIN 406
#define MSGQUE_CH4_to_MAIN 408
#define MSGQUE_VOC_to_MAIN 404

#define Robot_Serial_LINK

using namespace std;
control::control(/* args */)
{
}

control::~control()
{
}

int SerialInit(int SerialNum)
{
    struct termios serialSettings;
    // 先进行初始化
    tcgetattr(SerialNum, &serialSettings); // 初始化，清零
    // 再进行置零
    bzero(&serialSettings, sizeof(serialSettings)); //
    cfsetispeed(&serialSettings, B115200);          // 设置输入波特率
    cfsetospeed(&serialSettings, B115200);          // 设置输出波特率
    serialSettings.c_cflag |= (CLOCAL | CREAD);     // 激活本地连接与接受使能
    serialSettings.c_cflag &= ~PARENB;              // 无奇偶校验
    serialSettings.c_cflag &= ~CSTOPB;              // 1位停止位
    serialSettings.c_cflag &= ~CSIZE;               // 消除数据位设置
    serialSettings.c_cflag |= CS8;                  // 8位数据位
    serialSettings.c_cc[VTIME] = 10;                // 阻塞设置
    serialSettings.c_cc[VMIN] = 1;                  // 阻塞设置
    tcflush(SerialNum, TCIOFLUSH);                  // 刷清未处理的输入和或输出
    if (tcsetattr(SerialNum, TCSANOW, &serialSettings) != 0)
    {
        std::cout << "serialSettings  " << std::endl;
        return -1;
    }
    std::cout << "SerialSettings succeed!" << std::endl;
    return 0;
}

/**
 * @brief
 * 速度下发函数:
 * 1.串口ID,
 * 2.角速度实际值,
 * 3.线速度实际值,
 * 4.角速度最大值,
 * 5.线速度最大值
 */
int SerialVelSend(int _Serial, float Ang, float Vel, float MAX_Ang, float MAX_Vel, int16_t time_circle_int)
{
    static float VelSave, AngSave;
    float VelACC = 1, AngACC = 20;
    float time_circle_f = time_circle_int / 1000.0;
    // std::cout << "11 Ang= " << Ang << "  Vel= " << Vel << std::endl;
    // 角速度
    if (Ang >= 0)
    {
        Ang = (MAX_Ang > Ang) ? Ang : MAX_Ang;
    }
    else
    {
        Ang = (Ang < -MAX_Ang) ? -MAX_Ang : Ang;
    }
    // 线速度
    if (Vel >= 0)
    {
        Vel = (MAX_Vel > Vel) ? Vel : MAX_Vel;
    }
    else
    {
        Vel = (Vel < -MAX_Vel) ? -MAX_Vel : Vel;
    }
    // 滤波与平滑
    // 角速度平滑滤波
    if (fabs(Ang - AngSave) > (time_circle_f * AngACC))
    {
        if (Ang >= AngSave)
        {
            Ang = AngSave + time_circle_f * AngACC;
        }
        else
        {
            Ang = AngSave - time_circle_f * AngACC;
        }
    }
    else
    {
        if (fabs(Ang) < 1e-3)
            AngSave = 0.0;
        Ang = AngSave;
    }
    // 线速度平滑滤波
    if (fabs(Vel - VelSave) > (time_circle_f * VelACC))
    {
        if (Vel >= VelSave)
        {
            Vel = VelSave + time_circle_f * VelACC;
        }
        else
        {
            Vel = VelSave - time_circle_f * VelACC;
        }
    }
    else
    {
        if (fabs(Vel) < 1e-3)
            VelSave = 0.0;
        Vel = VelSave;
    }
    // std::cout << "Ang= " << Ang << "  Vel= " << Vel << std::endl;
    int16_t Velocity, Angular;
    // 190对应的是0.13m/s, 1461.54对应的是1m/s
    // 速度和角速度对应关系: 线速度 100 对应是1m/s; 角速度 10 对应是 1°/s
    int16_t VEL_Confer = 400, ANG_Confer = 10;
    Velocity = abs(int(Vel * VEL_Confer));
    Angular = abs(int(Ang * ANG_Confer));
    std::string Velocity_CC = std::to_string(Velocity);
    std::string Angluar_CC = std::to_string(Angular);
    int Vlenth = Velocity_CC.size();
    int Alenth = Angluar_CC.size();
    std::vector<std::string> Serial_strings;
    if ((Vel >= 0) && (Ang >= 0))
    {
        Serial_strings = {"!", "M", " ", Angluar_CC, " ", Velocity_CC, "\n"};
    }
    if ((Vel >= 0) && (Ang < 0))
    {
        Serial_strings = {"!", "M", " ", "-", Angluar_CC, " ", Velocity_CC, "\n"};
    }
    if ((Vel < 0) && (Ang >= 0))
    {
        Serial_strings = {"!", "M", " ", Angluar_CC, " ", "-", Velocity_CC, "\n"};
    }
    if ((Vel < 0) && (Ang < 0))
    {
        Serial_strings = {"!", "M", " ", "-", Angluar_CC, " ", "-", Velocity_CC, "\n"};
    }
    VelSave = Vel;
    AngSave = Ang;
    std::string SeialSEND_result;
    for (const std::string &str : Serial_strings)
    {
        SeialSEND_result += str;
    }
    // std::cout << SeialSEND_result << std::endl;
    char VEL_buffer[SeialSEND_result.length() + 1];
    memset(VEL_buffer, 0, sizeof(VEL_buffer));
    strcpy(VEL_buffer, SeialSEND_result.c_str());
    // 速度下发
    if (write(_Serial, VEL_buffer, sizeof(VEL_buffer)) <= 0)
        std::cout << "Serial Write failed!" << std::endl;
}

// 接收H2S数据
void RecvH2Sdata()
{
    // msg que to main
    Driver Driver_H2S;
    H2Smsg2Ctrl H2Smsg2Ctrl_;
    char H2S_Buf[128];
    int msgQue_H2S2main = -1;
    memset(&H2Smsg2Ctrl_, 0, sizeof(H2Smsg2Ctrl_));
    int H2S2main_BufSize = sizeof(H2Smsg2Ctrl_);
    msgQue_H2S2main = msgget((key_t)MSGQUE_H2S2MAIN, 0666 | IPC_CREAT);
    // printf("msgQue_Nav2GHB=%d,Nav2GHB_BufSize=%ld\n", msgQue_Nav2GHB, Nav2GHB_BufSize);
    while (msgQue_H2S2main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               msgQue_H2S2main);
        msgQue_H2S2main = msgget((key_t)MSGQUE_H2S2MAIN, 0666 | IPC_CREAT);
        sleep(1);
    }
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz
        int ret = msgrcv(msgQue_H2S2main, (void *)H2S_Buf, H2S2main_BufSize, 0, 0);
        if (ret > 0)
        {
            memcpy(&H2Smsg2Ctrl_, H2S_Buf, H2S2main_BufSize);
            // LOG(INFO) << " H2Smsg2Ctrl_.H2SINFO_.H2S_concentration = " << H2Smsg2Ctrl_.H2SINFO_.H2S_concentration << endl;
            Driver_H2S.GasParameter_.H2S_concentration = H2Smsg2Ctrl_.H2SINFO_.H2S_concentration;
        }
        else
        {
            printf("[WARNING] msgrcv return :%d.\n", ret);
        }
    }
}

// 接收CH4数据
void RecvCH4data()
{
    Driver Driver_CH4;
    CH4msg2Ctrl CH4msg2Ctrl_;
    char CH4_Buffer[128];
    int msgQue_CH4_to_main = -1;
    memset(&CH4msg2Ctrl_, 0, sizeof(CH4msg2Ctrl_));
    int CH4_to_main_Size = sizeof(CH4msg2Ctrl_);
    msgQue_CH4_to_main = msgget((key_t)MSGQUE_CH4_to_MAIN, 0666 | IPC_CREAT);
    while (msgQue_CH4_to_main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               msgQue_CH4_to_main);
        msgQue_CH4_to_main = msgget((key_t)MSGQUE_CH4_to_MAIN, 0666 | IPC_CREAT);
        sleep(1);
    }
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz
        int ret = msgrcv(msgQue_CH4_to_main, (void *)CH4_Buffer, CH4_to_main_Size, 0, 0);
        if (ret > 0)
        {
            memcpy(&CH4msg2Ctrl_, CH4_Buffer, CH4_to_main_Size);
            // LOG(INFO) << " CH4msg2Ctrl_.CH4INFO_.CH4_first = " << CH4msg2Ctrl_.CH4INFO_.CH4_first << endl;
            // LOG(INFO) << " CH4msg2Ctrl_.CH4INFO_.CH4_second = " << CH4msg2Ctrl_.CH4INFO_.CH4_second << endl;
            Driver_CH4.GasParameter_.CH4_first = CH4msg2Ctrl_.CH4INFO_.CH4_first;
            Driver_CH4.GasParameter_.CH4_second = CH4msg2Ctrl_.CH4INFO_.CH4_second;
        }
        else
        {
            printf("[WARNING] msgrcv return :%d.\n", ret);
        }
    }
}

// 接收VOC数据
void RecvVOCdata()
{
    VOCmsg2Ctrl VOCmsg2Ctrl_;
    Driver Driver_VOC;
    char VOC_Buffer[128];
    int msgQue_VOC_to_main = -1;
    memset(&VOCmsg2Ctrl_, 0, sizeof(VOCmsg2Ctrl_));
    int VOC_to_main_Size = sizeof(VOCmsg2Ctrl_);
    msgQue_VOC_to_main = msgget((key_t)MSGQUE_VOC_to_MAIN, 0666 | IPC_CREAT);
    while (msgQue_VOC_to_main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               msgQue_VOC_to_main);
        msgQue_VOC_to_main = msgget((key_t)MSGQUE_VOC_to_MAIN, 0666 | IPC_CREAT);
        sleep(1);
    }
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10ms=100Hz
        int ret = msgrcv(msgQue_VOC_to_main, (void *)VOC_Buffer, VOC_to_main_Size, 0, 0);
        if (ret > 0)
        {
            memcpy(&VOCmsg2Ctrl_, VOC_Buffer, VOC_to_main_Size);
            // LOG(INFO) << " VOCmsg2Ctrl_.VOCINFO_.VOC_firstn = " << VOCmsg2Ctrl_.VOCINFO_.VOC_first << endl;
            // LOG(INFO) << " VOCmsg2Ctrl_.VOCINFO_.VOC_second = " << VOCmsg2Ctrl_.VOCINFO_.VOC_second << endl;
            Driver_VOC.GasParameter_.VOC_first = VOCmsg2Ctrl_.VOCINFO_.VOC_first;
            Driver_VOC.GasParameter_.VOC_second = VOCmsg2Ctrl_.VOCINFO_.VOC_second;
        }
        else
        {
            printf("[WARNING] msgrcv return :%d.\n", ret);
        }
    }
}

void MessageFeedback()
{
    // 发送udp
    ZSHRob2MsgHumanFrame Rob2MsgHumanFrame_;
    global global_feedback;
    Driver Driver_feedBack;
    int sockfd_Rob2Human = 0, ret_Rob2Human = 0;
    int16_t Circle_period = 200;
    int sendflag = 0;
    struct sockaddr_in raddr;
    memset(&raddr, 0, sizeof(raddr));
    int Rob2MsgHumanFrame_len = sizeof(Rob2MsgHumanFrame_);
    raddr.sin_family = AF_INET;
    raddr.sin_port = htons(9004);
    raddr.sin_addr.s_addr = inet_addr("192.168.1.93");
    if ((sockfd_Rob2Human = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        perror("Create sockfd_Rob2Human failed!");
        exit(-1);
    }
    else
    {
        printf("%d\n", sockfd_Rob2Human);
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(Circle_period));
        Rob2MsgHumanFrame_.message_feedback_.CH4_r = Driver_feedBack.GasParameter_.CH4_first;
        Rob2MsgHumanFrame_.message_feedback_.CH4_c = Driver_feedBack.GasParameter_.CH4_second;
        Rob2MsgHumanFrame_.message_feedback_.H2S_a = Driver_feedBack.GasParameter_.H2S_concentration;
        Rob2MsgHumanFrame_.message_feedback_.VOC_a = Driver_feedBack.GasParameter_.VOC_first;
        Rob2MsgHumanFrame_.message_feedback_.VOC_b = Driver_feedBack.GasParameter_.VOC_second;
        // std::cout << "global_feedback.latitude : " << global_feedback.latitude
        //           << "  global_feedback.longitude : " << global_feedback.longitude
        //           << "  global_feedback.heading : " << global_feedback.heading << std::endl;
        Rob2MsgHumanFrame_.message_feedback_.GPS_latitude = global_feedback.latitude;
        Rob2MsgHumanFrame_.message_feedback_.GPS_longitude = global_feedback.longitude;
        Rob2MsgHumanFrame_.message_feedback_.GPS_heading = global_feedback.heading;
        Rob2MsgHumanFrame_.message_feedback_.QX_position_x = global_feedback.latitude;
        Rob2MsgHumanFrame_.message_feedback_.QX_position_y = global_feedback.longitude;
        Rob2MsgHumanFrame_.message_feedback_.QX_position_z = global_feedback.heading;
        Rob2MsgHumanFrame_.message_feedback_.QX_RPY_r = 0.0;
        Rob2MsgHumanFrame_.message_feedback_.QX_RPY_p = 0.0;
        Rob2MsgHumanFrame_.message_feedback_.QX_RPY_y = 0.0;
        Rob2MsgHumanFrame_.message_feedback_.Flag_ludian = global_feedback.PointTXT_lineCount;
        Rob2MsgHumanFrame_.message_feedback_.Flag_num = global_feedback.PointCollectFlag;
        ret_Rob2Human = sendto(sockfd_Rob2Human, &Rob2MsgHumanFrame_, Rob2MsgHumanFrame_len, 0, (struct sockaddr *)&raddr, sizeof(struct sockaddr_in));
        if (ret_Rob2Human < 0)
            printf("ret_Rob2Human send error!\n");
    }
}

// 数据处理线程
void control::ZSH_CtrlThread(void)
{
    Driver Driver_SerialSEND;
    int16_t Circle_period = 200; // 下发的时候100ms
    std::thread RECV_H2S(RecvH2Sdata);
    std::thread RECV_CH4(RecvCH4data);
    std::thread RECV_VOC(RecvVOCdata);
    std::thread Send_feedback(MessageFeedback);

#ifdef Robot_Serial_LINK
    int serialPort = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort <= 0)
        std::cout << "open serial Error!" << std::endl;
    std::cout << "Open serial succeed, serialPort: " << serialPort << std::endl;
    if (SerialInit(serialPort) != 0)
        std::cout << "SerialInit Error!" << std::endl;
#endif

    while (true)
    {
        // std::cout << "YK-VEL = " << Driver_SerialSEND.ControllerMsg_.Accerlerator
        //           << "  YK-ANG = " << Driver_SerialSEND.ControllerMsg_.vehicleAngularVelocity << std::endl;
        // std::cout << "MaxVel = " << Driver_SerialSEND.RobotParameter_.MaxVel
        //           << "  MaxAnguar = " << Driver_SerialSEND.RobotParameter_.MaxAnguar << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(Circle_period));
        VELandANG_mtx.lock();
#ifdef Robot_Serial_LINK
        SerialVelSend(serialPort, Driver_SerialSEND.ControllerMsg_.vehicleAngularVelocity,
                      Driver_SerialSEND.ControllerMsg_.Accerlerator,
                      Driver_SerialSEND.RobotParameter_.MaxAnguar,
                      Driver_SerialSEND.RobotParameter_.MaxVel,
                      Circle_period);
#endif
        VELandANG_mtx.unlock();
    }

#ifdef Robot_Serial_LINK
    if (close(serialPort) == -1)
        perror("Error closing serial port");
#endif

    RECV_H2S.join();
    RECV_CH4.join();
    RECV_VOC.join();
    Send_feedback.join();
}
