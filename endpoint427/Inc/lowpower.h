#ifndef __LOWPOWER_H__
#define __LOWPOWER_H__

#include "stdint.h"
#include "stm32f0xx_hal.h"
#include "motor.h"
#include "user_config.h"

/*
*   每次进入低功耗模式之前都要调用，
*   关闭不使用的外设，节省功耗
*   mode:对应不同的指令，非电机运动的命令都是能够即时返回，而电机运动命令
*   需要运行一段时间才能返回。mode = 0 对应即时返回 = 1对应非即时返回
*   flag：对应异常动作标志，产生异常动作不同于命令控制 =1 对应有异常动作产生
*/
//bool LowPowerInit(Device *d);
void EnterLowPower(void);
/*
*   系统上电后会调用
*   对系统中从未用到的外设和引脚进行设置
*/
void CloseNotUsedPeriphClock(void);

/*
*   处理控制命令，同步命令返回0
*   异步命令返回1
*/
void ExecCMD(Device *d);

/*
*   异步命令的执行结果处理，通过lora发送resp信息
*/
void SyncCMDDone(Device *d);

/*
*   按照指定的周期进行超声波检测，并通过lora发送
*/
void doCheckUltraByTimer(uint8_t u8TimeOut);

/*
*   按照指定的周期进行心跳信息发送
*/
void doHeartByTimer(uint8_t u8TimeOut, MOTOR_STATUS mStatus);

#endif
