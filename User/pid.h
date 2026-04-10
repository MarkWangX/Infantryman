#ifndef INFANTRYMAN_PID_H
#define INFANTRYMAN_PID_H

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
} PID_params;

int16_t Chas3508_PID(int8_t ID, float expV, float truV);

#endif