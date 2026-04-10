#include "motors.h"
#include "can.h"
#include "tim.h"
#ifndef PI
#define PI (3.1415927F)
#endif

/**
命令     类型     编号    反馈
0x200	M3508   1234   0x201-204
0x200	m2006   1234   0x201-204

0x1FF   M3508   5678   0x205-208
0x1FF   m2006   5678   0x205-208

0x1FF   GM6020  1234   0x205-208
0x2FF   GM6020  567    0x209,20A,20B
*/

/* ------------------------------ 常量 ------------------------------ */
//can2 头
const uint32_t HEAD_CMD_ID = 0x1ff; //GM6020(id=1) m3508(id=2,3) m2006(id=4)

//can1 身
const uint32_t NECK_CMD_ID = 0x2ff; //Yaw GM6020(id=5) #返回0x209
const uint32_t BODY_CMD_ID = 0x200; //4个m3508(1~4) #返回0x201-204

//进制转化
#define _pi_over_4096_ (PI/4096.0f)
#define _rads_per_rpm_ (PI/30.0f)

/* ------------------------------ 全局变量 ------------------------------ */
//can1数据：pitch电机&拨弹盘
float Pitch6020_Angle=0;
const float pitch_lookup_lim = 700 * _pi_over_4096_; //仰角
const float pitch_lookdown_lim = 2000 * _pi_over_4096_; //俯角
float Load2006_Velocity=0;
float Shoot3508_Velocity[2] = {0,0};
//can2数据：yaw电机&麦轮
float Chas3508_Velocity[4] = {0,0,0,0};
float Yaw6020_Angle = 0.0f;//(-pi,pi]

/* ------------------------------ 函数 ------------------------------ */
static void CAN1_Rx_Handler(CAN_RxHeaderTypeDef RxHeader, const uint8_t RxData[8]);
static void CAN2_Rx_Handler(CAN_RxHeaderTypeDef RxHeader, const uint8_t RxData[8]);

/* ------------------------------ 初始化（配置过滤器）------------------------------ */
void Enable_Motors(void)
{
	//配置CAN1过滤器
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; //CAN1使用FIFO0
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter.FilterBank = 0;
    CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter.SlaveStartFilterBank = 0;
    CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    CAN_Filter.FilterIdHigh = 0x0000;
    CAN_Filter.FilterIdLow = 0x0000;
    CAN_Filter.FilterMaskIdHigh= 0x0000;
    CAN_Filter.FilterMaskIdLow = 0x0000;
    if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
        Error_Handler();
    }
	//复用CAN_Filter, 配置CAN2过滤器
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1; //CAN2使用FIFO1
    CAN_Filter.FilterBank = 14;
    CAN_Filter.SlaveStartFilterBank = 14;
    if(HAL_CAN_ConfigFilter(&hcan2,&CAN_Filter)!= HAL_OK){
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
        Error_Handler();
    }
	//使能CAN总线
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
	//使能CAN接收缓冲区
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // 滑环下侧电机 -> CAN1 -> FIFO0
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // 滑环上侧电机 -> CAN2 -> FIFO1
	//使能CAN发送任务 在stm32f4xx_it.c
	HAL_TIM_Base_Start_IT(&htim4);//1ms 头 Pitch 拨弹盘 摩擦轮
    HAL_TIM_Base_Start_IT(&htim6);//1ms 脖 Yaw
    HAL_TIM_Base_Start_IT(&htim7);//1ms 身 底盘4个电机
}

/* ------------------------------------------ 发送函数：底盘和Yaw轴 ------------------------------------------ */
/*
 * 此函数用于控制底盘四个轮子
 * 轮子由M3508电机驱动 均挂在在CAN1
 */
void Body_M3508_Tx(int16_t Current[4])
{
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(Current[0]>>8);
	TxData[1] = (uint8_t)Current[0];
	TxData[2] = (uint8_t)(Current[1]>>8);
	TxData[3] = (uint8_t)Current[1];
	TxData[4] = (uint8_t)(Current[2]>>8);
	TxData[5] = (uint8_t)Current[2];
	TxData[6] = (uint8_t)(Current[3]>>8);
	TxData[7] = (uint8_t)Current[3];
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 8,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = BODY_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX0;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}

/*
 * 此函数用于控制云台YAW轴
 * 云台YAW轴由底盘上的GM6020电机驱动 因此挂载在CAN1
 */
void Neck_GM6020_Tx(int16_t Yaw_Voltage)
{
	uint8_t TxData[2];
	TxData[0] = (uint8_t)(Yaw_Voltage>>8);
	TxData[1] = (uint8_t)Yaw_Voltage;
	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 2,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = NECK_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX1;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}

/*
 * 此函数用于一次性控制滑环之上的四个电机
 * 包括云台Pitch轴（GM6020）
 * 供弹装置的拨弹盘（M2006）
 * 发射机构的摩擦轮（两个去掉减速箱的M3508）
 */
void Head_Motors_Tx(int16_t Pitch_Voltage, int16_t Shooter_Current[2], int16_t Loader_Current) {
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(Pitch_Voltage>>8);
	TxData[1] = (uint8_t)Pitch_Voltage;
	TxData[2] = (uint8_t)(Shooter_Current[0]>>8);
	TxData[3] = (uint8_t)Shooter_Current[0];
	TxData[4] = (uint8_t)(Shooter_Current[1]>>8);
	TxData[5] = (uint8_t)Shooter_Current[1];
	TxData[6] = (uint8_t)(Loader_Current>>8);
	TxData[7] = (uint8_t)Loader_Current;

	CAN_TxHeaderTypeDef TxHeader = {
		.DLC = 8,
		.IDE = CAN_ID_STD,    // 标准帧
		.RTR = CAN_RTR_DATA,  // 数据帧
		.StdId = HEAD_CMD_ID
	};
	uint32_t TxBox = CAN_TX_MAILBOX2;
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK){
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
	}
}

/* ------------------------------------------ 接收函数 ------------------------------------------ */
/*
 * 处理CAN1总线上的报文数据
 */
int debug = 0;
static void CAN1_Rx_Handler(CAN_RxHeaderTypeDef RxHeader, const uint8_t RxData[8]) {
	debug ++;
	if(RxHeader.StdId == 0x209) //GM6020(id=5) #返回0x209
	{
		int16_t rawAngle = (int16_t)( (RxData[0]<<8) | RxData[1] );
		rawAngle -= 3418; //magic number 取决于Yaw轴GM6020的安装角度
		if(rawAngle>=4096){rawAngle-=8192;}//-> (4095)~(0)~(-4096)
		Yaw6020_Angle = (float)rawAngle * _pi_over_4096_; //-> [-pi,pi)
	}
	else if(RxHeader.StdId>0x200 && RxHeader.StdId<0x205) //4个m3508(1~4) #返回0x201-204
	{
		uint32_t i = RxHeader.StdId-(uint32_t)0x201;
		int16_t rawVelocity = (int16_t)( (RxData[2]<<8) | RxData[3] );
		Chas3508_Velocity[i] = (float)rawVelocity * _rads_per_rpm_;
	}
	else
	{
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
	}
}

/*
 * 处理CAN2的数据
 */
static void CAN2_Rx_Handler(CAN_RxHeaderTypeDef RxHeader, const uint8_t RxData[8]) {
	if (RxHeader.StdId == 0x205)
	{
		int16_t rawAngle = (int16_t)( (RxData[0]<<8) | RxData[1] );
		Pitch6020_Angle = (float)rawAngle * _pi_over_4096_;
	}
	else if (RxHeader.StdId == 0x206)
	{
		int16_t rawVelocity = (int16_t)( (RxData[2]<<8) | RxData[3] );
		Shoot3508_Velocity[0] = (float)rawVelocity * _rads_per_rpm_;
	}
	else if (RxHeader.StdId == 0x207)
	{
		int16_t rawVelocity = (int16_t)( (RxData[2]<<8) | RxData[3] );
		Shoot3508_Velocity[1] = (float)rawVelocity * _rads_per_rpm_;
	}
	else if (RxHeader.StdId == 0x208)
	{
		int16_t rawVelocity = (int16_t)( (RxData[2]<<8) | RxData[3] );
		Load2006_Velocity = (float)rawVelocity * _rads_per_rpm_;
	}
	else
	{
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
	}
}

/*
 * 重定义FIFO0接收回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	debug++;
	if (hcan == &hcan1)
	{
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			CAN1_Rx_Handler(RxHeader,RxData);
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
	}
	else
	{
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
	}
}

/*
 * 重定义FIFO1接收回调函数
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan == &hcan2)
	{
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t RxData[8];
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
			CAN2_Rx_Handler(RxHeader,RxData);
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);  // 再次使能FIFO0接收中断
	}
	else
	{
		HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);
	}
}