#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

/*-----------------------extern-----------------------*/
//can2数据：pitch电机&拨弹盘
extern float Pitch6020_Angle;
extern const float pitch_lookup_lim;
extern const float pitch_lookdown_lim;
extern float Load2006_Velocity;//rad/s
extern float Shoot3508_Velocity[2];
//can1数据：yaw电机&麦轮
extern float Chas3508_Velocity[4];
extern float Yaw6020_Angle;//经过特殊处理，为(-pi,pi]

/*函数声名*/
void Enable_Motors(void);
void Head_Motors_Tx(int16_t Pitch_Voltage, int16_t Shooter_Current[2], int16_t Loader_Current);
void Body_M3508_Tx(int16_t Current[4]);
void Neck_GM6020_Tx(int16_t Yaw_Voltage);

#endif //MOTORS_H
