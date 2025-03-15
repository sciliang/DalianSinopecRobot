#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <sys/msg.h>
#include "CH4_read.hpp"
extern "C"
{
#include "rs232.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
}
#define CH4_PACKET_SIZE 24
#define Buffer_SIZE 300
#define SERIAL_NUM 9600
#define MSGQUE_CH4_to_MAIN 408

#define CH4_LINK
// #define CH4_NOLINK

void SerialInit(int SerialNum)
{
    struct termios serialSettings;
    tcgetattr(SerialNum, &serialSettings);
    cfsetispeed(&serialSettings, B9600); // 设置波特率为9600
    cfsetospeed(&serialSettings, B9600);
    serialSettings.c_cflag &= ~PARENB; // 无奇偶校验
    serialSettings.c_cflag &= ~CSTOPB; // 1位停止位
    serialSettings.c_cflag &= ~CSIZE;  // 8位数据位
    serialSettings.c_cflag |= CS8;
    tcsetattr(SerialNum, TCSANOW, &serialSettings);
}

void CH4decoder(unsigned char *CH4GasProcess, unsigned char *CH4GasTransit, unsigned char *CH4GasCorrectData, int bytes_received, float *CH4_finalValue)
{
    static int decode_iterator = 0, decode_unused = 0;
    std::string combinedStr, combinedStr2;
    memcpy(CH4GasProcess + decode_iterator, CH4GasTransit, bytes_received);
    decode_iterator = decode_iterator + bytes_received;
    if (decode_iterator < CH4_PACKET_SIZE)
    {
        // std::cout << "decode_iterator < CH4_PACKET_SIZE " << std::endl;
    }
    else
    {
        for (int i = 0; i < decode_iterator; i++)
        {
            // std::cout << "decode_iterator = " << decode_iterator << std::endl;
            if (CH4GasProcess[i] != 'R')
            {
                decode_unused++;
                // std::cout << "decode_unused = " << decode_unused << std::endl;
            }
            else
            {
                if (CH4GasProcess[i + CH4_PACKET_SIZE - 4] == '%')
                {
                    // std::cout << "Find end succeed !" << std::endl;
                    memcpy(CH4GasCorrectData, CH4GasProcess + i, CH4_PACKET_SIZE - 3);
                    // std::cout << "CH4GasCorrectData[0] = " << CH4GasCorrectData[0] << std::endl;
                    // std::cout << "CH4GasCorrectData[1] = " << CH4GasCorrectData[1] << std::endl;
                    // std::cout << "CH4GasCorrectData[2] = " << CH4GasCorrectData[2]
                    //           << CH4GasCorrectData[3]
                    //           << CH4GasCorrectData[4]
                    //           << CH4GasCorrectData[5]
                    //           << CH4GasCorrectData[6]
                    //           << CH4GasCorrectData[7]
                    //           << CH4GasCorrectData[8] << std::endl;
                    for (int i = 3; i <= 8; i++)
                    {
                        combinedStr += CH4GasCorrectData[i];
                    }
                    float result_1 = std::stof(combinedStr);
                    // std::cout << "result1 = " << result_1 << std::endl;
                    // printf("CH4 : result_1 = %.8f\n", result_1);
                    // std::cout << "CH4GasCorrectData[9] = " << CH4GasCorrectData[9] << std::endl;
                    // std::cout << "CH4GasCorrectData[10] = " << CH4GasCorrectData[10]
                    //           << CH4GasCorrectData[11] << std::endl;
                    // std::cout << "CH4GasCorrectData[12] = " << CH4GasCorrectData[12] << std::endl;
                    // std::cout << "CH4GasCorrectData[13] = " << CH4GasCorrectData[13]
                    //           << CH4GasCorrectData[14]
                    //           << CH4GasCorrectData[15]
                    //           << CH4GasCorrectData[16]
                    //           << CH4GasCorrectData[17]
                    //           << CH4GasCorrectData[18] << std::endl;
                    for (int i = 14; i <= 18; i++)
                    {
                        combinedStr2 += CH4GasCorrectData[i];
                    }
                    float result_2 = std::stof(combinedStr2);
                    // printf("CH4 : result_2 = %.8f\n", result_2);
                    // std::cout << "CH4GasCorrectData[19] = " << CH4GasCorrectData[19] << std::endl;
                    // std::cout << "CH4GasCorrectData[20] = " << CH4GasCorrectData[20] << std::endl;
                    memmove(CH4GasProcess, CH4GasProcess + i + CH4_PACKET_SIZE - 3, sizeof(CH4GasProcess));
                    // std::cout << "CH4GasProcess = " << CH4GasProcess << std::endl;
                    decode_iterator = decode_iterator - (CH4_PACKET_SIZE - 3) - decode_unused;
                    // std::cout << "decode_iterator = " << decode_iterator << std::endl;
                    CH4_finalValue[0] = result_1;
                    CH4_finalValue[1] = result_2;
                }
                break;
            }
        }
        memmove(CH4GasProcess, CH4GasProcess + decode_unused, sizeof(CH4GasProcess));
        decode_iterator = decode_iterator - decode_unused;
        decode_unused = 0;
    }
}

int main(int argc, char **argv)
{
    CH4msg2Ctrl CH4msg2Ctrl_;
#ifdef CH4_LINK
    unsigned char CH4GasBuffer[Buffer_SIZE], CH4GasProcess_[Buffer_SIZE];
    unsigned char CH4GasCorrectData_[CH4_PACKET_SIZE], CH4GasTransit_[Buffer_SIZE];
    int serialPort = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY);
    float CH4FinalValue[2];
    while (serialPort == -1)
    {
        std::cout << "open serial Error!" << std::endl;
    }
    SerialInit(serialPort);
    memset(CH4GasBuffer, 0, Buffer_SIZE);
    memset(CH4GasTransit_, 0, Buffer_SIZE);
    memset(CH4GasCorrectData_, 0, Buffer_SIZE);
#endif
    // message que
    int msgQue_CH4_main = -1;
    memset(&CH4msg2Ctrl_, 0, sizeof(CH4msg2Ctrl_));
    int msgCH4_BufSize = sizeof(CH4msg2Ctrl_);
    msgQue_CH4_main = msgget((key_t)MSGQUE_CH4_to_MAIN, 0666 | IPC_CREAT);
    while (msgQue_CH4_main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n", msgQue_CH4_main);
        msgQue_CH4_main = msgget((key_t)MSGQUE_CH4_to_MAIN, 0666 | IPC_CREAT);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
#ifdef CH4_LINK
        int bytes_received_ = read(serialPort, CH4GasBuffer, sizeof(CH4GasBuffer));
        // std::cout << "bytes_received = " << bytes_received_ << std::endl;
        if (bytes_received_ <= 0)
        {
            // std::cout << "Serial Reacv ERROR!" << std::endl;
        }
        else
        {
            memcpy(CH4GasTransit_, CH4GasBuffer, bytes_received_);
            CH4decoder(CH4GasProcess_, CH4GasTransit_, CH4GasCorrectData_, bytes_received_, CH4FinalValue);
            CH4msg2Ctrl_.mtype = 1;
            CH4msg2Ctrl_.CH4INFO_.CH4_first = CH4FinalValue[0];
            CH4msg2Ctrl_.CH4INFO_.CH4_second = CH4FinalValue[1];
#endif

#ifdef CH4_NOLINK
            CH4msg2Ctrl_.mtype = 1;
            CH4msg2Ctrl_.CH4INFO_.CH4_first = 41.567;
            CH4msg2Ctrl_.CH4INFO_.CH4_second = 51.234;
#endif
            printf("CH4 : CH4msg2Ctrl_.CH4INFO_.CH4_first = %.8f\n", CH4msg2Ctrl_.CH4INFO_.CH4_first);
            printf("CH4 : CH4msg2Ctrl_.CH4INFO_.CH4_second  = %.8f\n", CH4msg2Ctrl_.CH4INFO_.CH4_second);
            if (msgsnd(msgQue_CH4_main, (void *)&CH4msg2Ctrl_, sizeof(CH4msg2Ctrl_.CH4INFO_), IPC_NOWAIT) < 0)
                std::cout << "messge send failed " << std::endl;
#ifdef CH4_LINK
            memset(CH4GasBuffer, 0, Buffer_SIZE);
            memset(CH4GasTransit_, 0, Buffer_SIZE);
        }
#endif
    }
    return EXIT_SUCCESS;
}