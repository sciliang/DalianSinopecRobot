#ifndef CONTROL_HPP
#define CONTROL_HPP
#include "Driver.hpp"
#include <iostream>
//--H2S
struct H2SINFO
{
    int H2S_concentration;
};

typedef struct
{
    __syscall_slong_t mtype;
    H2SINFO H2SINFO_;
} H2Smsg2Ctrl;

//--CH4
struct CH4INFO
{
    float CH4_first;
    float CH4_second;
};

typedef struct
{
    __syscall_slong_t mtype;
    CH4INFO CH4INFO_;
} CH4msg2Ctrl;

//--VOC
struct VOCINFO
{
    float VOC_first;
    float VOC_second;
};

typedef struct
{
    __syscall_slong_t mtype;
    VOCINFO VOCINFO_;
} VOCmsg2Ctrl;

//--
class control
{
private:
    /* data */
public:
    control(/* args */);
    ~control();
    void ZSH_CtrlThread(void); // 主控制程序
};
void RecvH2Sdata();
void RecvCH4data();
void RecvVOCdata();
void MessageFeedback();
#endif /* CONTROL_HPP */