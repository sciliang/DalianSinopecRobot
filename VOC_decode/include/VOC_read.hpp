#pragma once
#include <stdint.h>

//--
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

