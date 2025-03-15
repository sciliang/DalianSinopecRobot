#pragma once
#include <stdint.h>

//--
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

