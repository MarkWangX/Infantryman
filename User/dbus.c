
#include "dbus.h"
#include "usart.h"
#include "iwdg.h"


/* ------------------------------ Macro Definition ------------------------------ */
#define DBUS_HUART huart3              // 定义遥控器串口句柄
#define DBUS_HDMA_RX (huart3.hdmarx)  // 定义遥控器DMA句柄
#define DBUS_HUART_BASE USART3

#define DBUS_RX_BUF_NUM 36  // 设置DMA接收数组长度为36字节，防止接收越界
#define DBUS_FRAME_LENGTH 18  // 一帧遥控器数据为18字节

#define DBUS_RATIO 660.0f  // 摇杆数值CH归一化比例
#define DBUS_OFFSET 1024   // 摇杆数值CH的偏移量

/* ------------------------------ Global Variable ------------------------------ */
DBUS_Type dbus;                             // 当前和上次键鼠、遥控器数据，上次数据更新为当前数据的周期为1ms
uint32_t dbus_tick;                       // 接收到数据的时刻
uint8_t DbusRxBuf[2][DBUS_RX_BUF_NUM];  // 接收到的原始数据

/* ------------------------------ Function Declaration (only used in this .c file) ------------------------------ */
static void Dbus_Data_Process(DBUS_Type *dbus, const uint8_t *buff);
/* ------------------------------ Function Definition ------------------------------ */

/*
 * 遥控器初始化
 */
void Dbus_Init()
{
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);  // 使能空闲中断

	SET_BIT(DBUS_HUART.Instance->CR3, USART_CR3_DMAR);  // 使能DMA串口接收

	__HAL_DMA_DISABLE(DBUS_HDMA_RX);                // 失效DMA
	while (DBUS_HDMA_RX->Instance->CR & DMA_SxCR_EN)  // 轮询是否已经失效
	{
		__HAL_DMA_DISABLE(DBUS_HDMA_RX);
	}

	DBUS_HDMA_RX->Instance->PAR = (uint32_t) & (DBUS_HUART_BASE->DR);  // 设置传输源地址为串口的数据寄存器
	DBUS_HDMA_RX->Instance->M0AR = (uint32_t)(DbusRxBuf[0]);  // 内存缓冲区1
	DBUS_HDMA_RX->Instance->M1AR = (uint32_t)(DbusRxBuf[1]);  // 内存缓冲区2
	DBUS_HDMA_RX->Instance->NDTR = DBUS_RX_BUF_NUM;           // 设置DMA接收数据长度
	SET_BIT(DBUS_HDMA_RX->Instance->CR, DMA_SxCR_DBM);        // 使能双缓冲区

	__HAL_DMA_ENABLE(DBUS_HDMA_RX);  // 使能DMA
}

/*
 * 串口空闲中断处理
 */
void Dbus_UART_IRQHandler(void)
{
	if (DBUS_HUART.Instance->SR & UART_FLAG_IDLE)  // 如果是串口空闲中断，则进行处理
	{
		static uint16_t this_time_rx_len = 0;                    // 用于设定接收到的长度
		__HAL_UART_CLEAR_PEFLAG(&DBUS_HUART);                    // 先读SR再读DR，清零空闲中断标志
		if ((DBUS_HDMA_RX->Instance->CR & DMA_SxCR_CT) == RESET)  // 当前目标存储器为存储器0
		{
			__HAL_DMA_DISABLE(DBUS_HDMA_RX);                // 失效DMA
			while (DBUS_HDMA_RX->Instance->CR & DMA_SxCR_EN)  // 轮询是否已经失效
			{
				__HAL_DMA_DISABLE(DBUS_HDMA_RX);
			}

			this_time_rx_len = DBUS_RX_BUF_NUM - DBUS_HDMA_RX->Instance->NDTR;  // 本次接收到的数据长度=设定长度-剩余长度
			DBUS_HDMA_RX->Instance->NDTR = DBUS_RX_BUF_NUM;                     // 重新设置传输的数据长度
			DBUS_HDMA_RX->Instance->CR |= DMA_SxCR_CT;                          // 设定缓冲区1

			__HAL_DMA_ENABLE(DBUS_HDMA_RX);  // 使能DMA

			if (this_time_rx_len == DBUS_FRAME_LENGTH)  // 如果接收到的长度符合一帧遥控器数据长度，则进行处理
			{
				Dbus_Data_Process(&dbus, DbusRxBuf[0]);
			}
		} else {
			__HAL_DMA_DISABLE(DBUS_HDMA_RX);                // 失效DMA
			while (DBUS_HDMA_RX->Instance->CR & DMA_SxCR_EN)  // 轮询是否已经失效
			{
				__HAL_DMA_DISABLE(DBUS_HDMA_RX);
			}

			this_time_rx_len = DBUS_RX_BUF_NUM - DBUS_HDMA_RX->Instance->NDTR;  // 本次接收到的数据长度=设定长度-剩余长度
			DBUS_HDMA_RX->Instance->NDTR = DBUS_RX_BUF_NUM;                     // 重新设置传输的数据长度
			DBUS_HDMA_RX->Instance->CR &= ~(DMA_SxCR_CT);                       // 设定缓冲区0

			__HAL_DMA_ENABLE(DBUS_HDMA_RX);  // 使能DMA

			if (this_time_rx_len == DBUS_FRAME_LENGTH)  // 如果接收到的长度符合一帧遥控器数据长度，则进行处理
			{
				Dbus_Data_Process(&dbus, DbusRxBuf[1]);
			}
		}
	}
}

/*
 * 遥控器数据处理
 */
static void Dbus_Data_Process(DBUS_Type *dbus, const uint8_t *buff)
{
	//接收到遥控器数据则喂狗，防止程序重启
	HAL_IWDG_Refresh(&hiwdg);

	//  得到摇杆数据
	int16_t ch1 = (int16_t)( (buff[0] | (buff[1] << 8)) & 0x07FF );
	int16_t ch2 = (int16_t)( ((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF );
	int16_t ch3 = (int16_t)( ((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF );
	int16_t ch4 = (int16_t)( ((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF );
	// 根据摇杆数据的偏置和比例解析出[-1, 1]之间的数值
	ch1 -= DBUS_OFFSET; dbus->RX = (float)ch1 / DBUS_RATIO;
	ch2 -= DBUS_OFFSET; dbus->RY = (float)ch2 / DBUS_RATIO;
	ch3 -= DBUS_OFFSET; dbus->LX = (float)ch3 / DBUS_RATIO;
	ch4 -= DBUS_OFFSET; dbus->LY = (float)ch4 / DBUS_RATIO;
	// 得到三位开关数据
	dbus->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	dbus->sw2 = (buff[5] >> 4) & 0x0003;

	// 得到鼠标数据
	dbus->mouse.x = (int16_t)( buff[6] | (buff[7] << 8) );
	dbus->mouse.y = (int16_t)( buff[8] | (buff[9] << 8) );
	dbus->mouse.z = (int16_t)( buff[10] | (buff[11] << 8) );
	dbus->mouse.l = (int8_t)( buff[12] );
	dbus->mouse.r = (int8_t)( buff[13] );

	// 得到键盘数据
	dbus->kb.key_code = buff[14] | (buff[15] << 8);
	// 拨轮数据解算
	int16_t whl = (int16_t)( buff[16] | (buff[17] << 8) );//364~1024~1684
	whl -= DBUS_OFFSET; dbus->wheel = (float)whl / (DBUS_RATIO);
}

