#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "hardware.h"
#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_hal_gpio.h"
#include "user_config.h"

typedef enum {
    MOTOR_CMD_IDLE    = 0,
    MOTOR_CMD_DOWN    = 1,
    MOTOR_CMD_DOWN_INTERNAL = 2,
    MOTOR_CMD_UP      = 3,
} MOTOR_CMD;

typedef void (* motor_ctrl_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_gpio_callback)(uint16_t pin);
typedef void (* timer_ctrl_timeout_callback)(void);

typedef struct {
    //保存电机执行指令之后的状态
    volatile MOTOR_STATUS status;     //电机当前状态 1=卧倒,2=前倾,3=直立,4=后倾
    volatile MOTOR_CMD action;     //命令
    volatile MOTOR_STATUS realStatus; //保存真实的地锁状态
    
    MOTOR_STATUS can_stop; //开始运动时的位置状态，用来对电机上升时进行停止判断。

    motor_ctrl_callback         ctrl_cb;
    motor_gpio_callback         gpio_cb;
    timer_ctrl_timeout_callback ctrl_timer_cb;
} Motor;

void GPIOMotorSenserInit(void);
void GPIOMotorInit(uint8_t cmd);
//电机控制函数
MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd);
//获取电机位置状态的函数
MOTOR_STATUS MotorGetStatus(void);

void MotorInit(void);

//电机位置检测引脚中断开启和关闭
void setMotorGpioNormal(void);
void setMotorGpioInterrupt(void);

void MotorSenceSwitch(bool b);
//电机异常动作处理
bool MotorAbnormalCheck(Device *d, bool flag);

MOTOR_STATUS getMotorStatus(void); //主循环获取实时电机位置
void resetMotorStatus(void);
void setMotorInterrupt(void);
void cancelMotorInterrupt(void);
MOTOR_STATUS getMotorStatusInt(void); //中断获取实时电机位置

#endif