#include "pid.h"
#include "dbus.h"

#define iErrorLimit 0.0f
#define ILimit 16384

PID_params chas = {10.0f, 20.0f, 0.0f};

int16_t Chas3508_PID(int8_t ID, float expV, float truV)
{
    static float iError[4] = {0, 0, 0, 0};

    if (ID < 0 || ID > 3) return -1;
    float pError = expV - truV;
    iError[ID] += pError;
    if (iError[ID] >= +iErrorLimit) {iError[ID] = +iErrorLimit;}
    if (iError[ID] <= -iErrorLimit) {iError[ID] = -iErrorLimit;}

    float output = chas.kp * pError + chas.ki *iError[ID];
    if (output >= +ILimit) {output = +ILimit;}
    if (output <= -ILimit) {output = -ILimit;}

    return (int16_t)output;
}