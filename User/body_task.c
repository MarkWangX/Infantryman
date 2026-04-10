#include "body_task.h"
#include "dbus.h"
#include "pid.h"
#include "motors.h"
#include "pid.h"

const float M3508_Reduction_Ratio = 19.0f;
float chassis_speed_level = M3508_Reduction_Ratio * 10.0f;
float chassis_rotate_level = M3508_Reduction_Ratio * 0.0f;




void BodyTask(void)
{
    int16_t current[4] = {0, 0, 0, 0};
    if (SW_MID == dbus.sw1 || SW_DOWN == dbus.sw1)
    {
        float x = dbus.LX;
        float y = dbus.LY;
        float z = 0.0f;

        x *= chassis_speed_level;
        y *= chassis_speed_level;

        float velocity[4];
        velocity[0] = -z - y + x;
        velocity[1] = -z + y + x;
        velocity[2] = -z + y - x;
        velocity[3] = -z - y - x;

        current[0] = Chas3508_PID(0,velocity[0], Chas3508_Velocity[0]);
        current[1] = Chas3508_PID(1,velocity[1], Chas3508_Velocity[1]);
        current[2] = Chas3508_PID(2,velocity[2], Chas3508_Velocity[2]);
        current[3] = Chas3508_PID(3,velocity[3], Chas3508_Velocity[3]);
    }
    else {
        current[0] = 0; current[1] = 0; current[2] = 0; current[3] = 0;
    }
    Body_M3508_Tx(current);
}
