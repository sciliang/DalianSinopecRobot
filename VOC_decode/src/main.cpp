#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <sys/msg.h>
#include "VOC_read.hpp"
extern "C"
{
#include "rs232.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
}
#define VOC_PACKET_SIZE 18
#define Buffer_SIZE 300
#define SERIAL_NUM 9600
#define MSGQUE_VOC_to_MAIN 404

#define VOC_LINK
// #define VOC_NOLINK

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

void VOCdecoder(unsigned char *VOC_Process, unsigned char *VOC_Transit, unsigned char *VOC_CorrectData, int bytes_received, float *VOC_finalValue)
{
    static int decode_iterator = 0, decode_unused = 0;
    std::string combinedStr, combinedStr2;
    memcpy(VOC_Process + decode_iterator, VOC_Transit, bytes_received);
    decode_iterator = decode_iterator + bytes_received;
    if (decode_iterator < VOC_PACKET_SIZE)
    {
        // std::cout << "decode_iterator < VOC_PACKET_SIZE " << std::endl;
    }
    else
    {
        for (int i = 0; i < decode_iterator; i++)
        {
            if ((VOC_Process[i] == 'V') && (VOC_Process[i + 8] == 'm'))
            {
                // std::cout << "Find end succeed !" << std::endl;
                memcpy(VOC_CorrectData, VOC_Process + i - 7, VOC_PACKET_SIZE - 2);
                // std::cout << VOC_CorrectData[0]
                //           << VOC_CorrectData[1]
                //           << VOC_CorrectData[2]
                //           << VOC_CorrectData[3]
                //           << VOC_CorrectData[4]
                //           << VOC_CorrectData[5]
                //           << VOC_CorrectData[6]
                //           << VOC_CorrectData[7] << std::endl;
                for (int i = 0; i <= 5; i++)
                {
                    combinedStr += VOC_CorrectData[i];
                }
                float result_1 = std::stof(combinedStr);
                printf("\nresult_1 = %.2f\n", result_1);
                // std::cout << VOC_CorrectData[9]
                //           << VOC_CorrectData[10]
                //           << VOC_CorrectData[11]
                //           << VOC_CorrectData[12]
                //           << VOC_CorrectData[13]
                //           << VOC_CorrectData[14]
                //           << VOC_CorrectData[15] << std::endl;
                for (int i = 9; i <= 12; i++)
                {
                    combinedStr2 += VOC_CorrectData[i];
                }
                float result_2 = std::stof(combinedStr2);
                printf("result_2 = %.2f\n", result_2);
                memset(VOC_Process, 0, sizeof(VOC_Process));
                decode_iterator = 0;
                VOC_finalValue[0] = result_1;
                VOC_finalValue[1] = result_2;
            }
            else
            {
                // std::cout << "Fail to find end!" << std::endl;
            }
        }
    }
}

int main(int argc, char **argv)
{
    VOCmsg2Ctrl VOCmsg2Ctrl_;
#ifdef VOC_LINK
    unsigned char VocBuffer[Buffer_SIZE], VocProcess_[Buffer_SIZE];
    unsigned char VocCorrectData_[VOC_PACKET_SIZE], VocTransit_[Buffer_SIZE];
    int serialPort = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    float VOCFinalValue[2];
    while (serialPort == -1)
    {
        std::cout << "open serial Error!" << std::endl;
    }
    SerialInit(serialPort);
    memset(VocBuffer, 0, Buffer_SIZE);
    memset(VocTransit_, 0, Buffer_SIZE);
    memset(VocCorrectData_, 0, Buffer_SIZE);
#endif
    // message que
    int msgQue_VOC_main = -1;
    memset(&VOCmsg2Ctrl_, 0, sizeof(VOCmsg2Ctrl_));
    int msgVOC_BufSize = sizeof(VOCmsg2Ctrl_);
    msgQue_VOC_main = msgget((key_t)MSGQUE_VOC_to_MAIN, 0666 | IPC_CREAT);
    while (msgQue_VOC_main < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n", msgQue_VOC_main);
        msgQue_VOC_main = msgget((key_t)MSGQUE_VOC_to_MAIN, 0666 | IPC_CREAT);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

#ifdef VOC_LINK
        int bytes_received_ = read(serialPort, VocBuffer, sizeof(VocBuffer));
        if (bytes_received_ <= 0)
        {
            // std::cout << "Serial Reacv ERROR!" << std::endl;
        }
        else
        {
            memcpy(VocTransit_, VocBuffer, bytes_received_);
            VOCdecoder(VocProcess_, VocTransit_, VocCorrectData_, bytes_received_, VOCFinalValue);
            VOCmsg2Ctrl_.mtype = 1;
            VOCmsg2Ctrl_.VOCINFO_.VOC_first = VOCFinalValue[0];
            VOCmsg2Ctrl_.VOCINFO_.VOC_second = VOCFinalValue[1];
#endif

#ifdef VOC_NOLINK
            VOCmsg2Ctrl_.mtype = 1;
            VOCmsg2Ctrl_.VOCINFO_.VOC_first = 47.456;
            VOCmsg2Ctrl_.VOCINFO_.VOC_second = 87.346;
#endif

            std::cout << "VOCmsg2Ctrl_.VOCINFO_.VOC_first = " << VOCmsg2Ctrl_.VOCINFO_.VOC_first << std::endl;
            std::cout << "VOCmsg2Ctrl_.VOCINFO_.VOC_second = " << VOCmsg2Ctrl_.VOCINFO_.VOC_second << std::endl;

            if (msgsnd(msgQue_VOC_main, (void *)&VOCmsg2Ctrl_, sizeof(VOCmsg2Ctrl_.VOCINFO_), IPC_NOWAIT) < 0)
                std::cout << "messge send failed " << std::endl;

#ifdef VOC_LINK
            memset(VocBuffer, 0, Buffer_SIZE);
            memset(VocTransit_, 0, Buffer_SIZE);
        }
#endif
    }
    return EXIT_SUCCESS;
}