#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

/*
*   每次进入低功耗模式之前都要调用，
*   关闭不使用的外设，节省功耗
*/
void LowPowerInit(uint8_t mode);

/*
*   系统上电后会调用
*   对系统中从未用到的外设和引脚进行设置
*/
void CloseNotUsedPeriphClock(void);

/*
*   处理控制命令，同步命令返回0
*   异步命令返回1
*/
uint8_t ProcessTheData(void);

/*
*   每次从低功耗模式被唤醒后需要重新初始化串口
*/
void UART_ReInit(UART_HandleTypeDef *huart);

/*
*   异步命令的执行结果处理，通过lora发送resp信息
*/
uint8_t SyncCMDDone(void);

#endif
