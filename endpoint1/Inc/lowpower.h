#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"

/*
*   每次进入低功耗模式之前都要调用，
*   关闭不使用的外设，节省功耗
*   mode:对应不同的指令，非电机运动的命令都是能够即时返回，而电机运动命令
*   需要运行一段时间才能返回。mode = 0 对应即时返回 = 1对应非即时返回
*   flag：对应异常动作标志，产生异常动作不同于命令控制 =1 对应有异常动作产生
*/
void LowPowerInit(uint8_t mode, uint8_t flag);

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
