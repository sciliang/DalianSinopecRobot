#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <glog/logging.h>
#include <stdlib.h>
#include "H2S_read.hpp"
#include <sys/msg.h>

extern "C"
{
#include "rs232.h"
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
}
#define H2S_PACKET_SIZE 11
#define Buffer_SIZE 12
#define SERIAL_NUM 9600
#define MSGQUE_H2S2MAIN 406

#define H2S_LINK
// #define H2S_NOLINK

/* 计算 CRC16 校验
 * auchMsg: 要进行CRC校验的消息
 * usDataLen: 消息中字节数
 */

uint16_t CRC16_(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF; /* 高CRC字节初始化*/
    uint8_t uchCRCLo = 0xFF; /* 低CRC 字节初始化*/
    uint32_t uIndex;         /* CRC循环中的索引*/
    while (usDataLen--)      /* 传输消息缓冲区*/
    {
        uIndex = uchCRCHi ^ *puchMsg++; /* 计算CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (((uint16_t)uchCRCHi << 8u) | uchCRCLo);
}

int SerialInit(int SerialNum)
{
    struct termios serialSettings;
    // 先进行初始化
    tcgetattr(SerialNum, &serialSettings); // 初始化，清零
    // 再进行置零
    bzero(&serialSettings, sizeof(serialSettings)); //
    cfsetispeed(&serialSettings, B9600);            // 设置输入波特率
    cfsetospeed(&serialSettings, B9600);            // 设置输出波特率
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

int16_t H2Sdecoder(unsigned char *CH4GasCorrectData, unsigned char *CH4GasTransit, unsigned char *CH4GasProcess, int bytes_received)
{
    std::string combinedStr, combinedStr2;
    memcpy(CH4GasCorrectData, CH4GasTransit, bytes_received);
    for (short i = 0; i < 11; i++)
    {
        // printf("[%d]:%x  ", i, CH4GasCorrectData[i]);
    }
    // std::cout << std::endl;
    GasDataConver.buffer[0] = CH4GasCorrectData[6];
    // printf("GasDataConver.buffer[0], DEC: %d , HEX: %x\n", GasDataConver.buffer[0], GasDataConver.buffer[0]);
    GasDataConver.buffer[1] = CH4GasCorrectData[5];
    // printf("GasDataConver.buffer[1], DEC: %d  , HEX: %x\n", GasDataConver.buffer[1], GasDataConver.buffer[1]);
    // std::cout << "GasDataConver.value = " << GasDataConver.value << std::endl;
    return GasDataConver.value;
}

int main(int argc, char **argv)
{
    H2Smsg2Ctrl H2Smsg2Ctrl_;
#ifdef H2S_LINK
    unsigned char CH4GasBuffer[Buffer_SIZE], CH4GasProcess_[Buffer_SIZE];
    unsigned char CH4GasCorrectData_[H2S_PACKET_SIZE], CH4GasTransit_[Buffer_SIZE];
    uint8_t H2S_MIC_SEND[8] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x03, 0x15, 0xCA};
    int serialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    // fcntl(serialPort, F_SETFL, 0);
    if (serialPort <= 0)
    {
        std::cout << "open serial Error!" << std::endl;
        return -1;
    }
    // std::cout << "Open serial succeed, serialPort: " << serialPort << std::endl;
    if (SerialInit(serialPort) != 0)
    {
        std::cout << "SerialInit Error!" << std::endl;
    }
    memset(CH4GasCorrectData_, 0, Buffer_SIZE);
    static int Cacul_clock = 0;
#endif
    // message que
    int msgQue_H2S_main = -1;
    memset(&H2Smsg2Ctrl_, 0, sizeof(H2Smsg2Ctrl_));
    int msgH2S_BufSize = sizeof(H2Smsg2Ctrl_);
    msgQue_H2S_main = msgget((key_t)MSGQUE_H2S2MAIN, 0666 | IPC_CREAT);
    while (msgQue_H2S_main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n", msgQue_H2S_main);
        msgQue_H2S_main = msgget((key_t)MSGQUE_H2S2MAIN, 0666 | IPC_CREAT);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (true)
    {
        // std::cout << "\n<----------------------------------->" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
#ifdef H2S_LINK
        // 写数据
        ssize_t bytes_written = write(serialPort, H2S_MIC_SEND, sizeof(H2S_MIC_SEND));
        if (bytes_written <= 0)
        {
            // std::cout << "Serial Write failed!" << std::endl;
        }
        else
        {
            // std::cout << "Write succeed, bytes_written = " << bytes_written << std::endl;
        }
        Cacul_clock++;
        // 收数据
        memset(CH4GasBuffer, 0, Buffer_SIZE);
        memset(CH4GasTransit_, 0, Buffer_SIZE);
        int bytes_receive = read(serialPort, CH4GasBuffer, sizeof(CH4GasBuffer));
        if (bytes_receive <= 0)
            std::cout << "Serial Receive failed!" << std::endl;
        else
        {
            // std::cout << "Receive succeed, bytes_receive = " << bytes_receive << std::endl;
            CH4GasBuffer[5] = 0x03;
            CH4GasBuffer[6] = 0xE8;
            // 数据收到的数据
            // std::cout << Cacul_clock << " , "
            //           << "recv data: ";
            for (short i = 0; i < Buffer_SIZE - 1; i++)
            {
                // printf("%x ", CH4GasBuffer[i]);
            }
            // std::cout << std::endl;
            // 解析数据
            memcpy(CH4GasTransit_, CH4GasBuffer, bytes_receive);
            int16_t H2S_concentration = H2Sdecoder(CH4GasProcess_, CH4GasTransit_, CH4GasCorrectData_, bytes_receive);

            H2Smsg2Ctrl_.mtype = 1;
            // position
            H2Smsg2Ctrl_.H2SINFO_.H2S_concentration = H2S_concentration;
#endif
#ifdef H2S_NOLINK
            H2Smsg2Ctrl_.mtype = 1;
            // position
            H2Smsg2Ctrl_.H2SINFO_.H2S_concentration = 342.4521;
#endif 
            std::cout << "H2Smsg2Ctrl_.H2SINFO_.H2S_concentration = " << H2Smsg2Ctrl_.H2SINFO_.H2S_concentration << std::endl;
            // send message
            if (msgsnd(msgQue_H2S_main, (void *)&H2Smsg2Ctrl_, sizeof(H2Smsg2Ctrl_.H2SINFO_), IPC_NOWAIT) < 0)
                std::cout << "messge send failed " << std::endl;
#ifdef H2S_LINK
        }
#endif
    }

#ifdef H2S_LINK
    close(serialPort); // 关闭串口
#endif

    return EXIT_SUCCESS;
}