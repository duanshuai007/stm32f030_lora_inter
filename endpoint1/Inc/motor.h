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
    MOTOR_CMD_UP      = 3,
} MOTOR_CMD;

typedef void (* motor_ctrl_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_check_callback)(MOTOR_CB_TYPE m);
typedef void (* motor_gpio_callback)(uint16_t pin);
typedef void (* timer_gpio_xd_callback)(void);
typedef void (* timer_ctrl_timeout_callback)(void);

typedef struct {
  uint8_t head;
  uint8_t data_h;
  uint8_t data_l;
  uint8_t sum;
} UltraSonicType;

typedef struct {
    volatile MOTOR_STATUS status;     //电机当前状态 1=卧倒,2=前倾,3=直立,4=后倾
    volatile MOTOR_CMD action;     //命令

    volatile MOTOR_STATUS last_status;    //保存上一次的正常位置状态
    MOTOR_STATUS can_stop; //开始运动时的位置状态，用来对电机上升时进行停止判断。

    motor_ctrl_callback         ctrl_cb;
    motor_gpio_callback         gpio_cb;
    timer_ctrl_timeout_callback ctrl_timer_cb;
} Motor;

//电机控制函数
MOTOR_STATUS motor_conctrl(MOTOR_CMD cmd);
//获取电机位置状态的函数
MOTOR_STATUS motor_get_status(void);

void motor_init(void);

//电机位置检测引脚中断开启和关闭
void motor_input_pin_off_interrupt(bool b);

//电机异常动作处理
bool motor_abnormal(void);

#endif
