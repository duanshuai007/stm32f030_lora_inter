#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal_def.h"
#include "lora_paramter.h"

typedef enum {
    false,
    true,
}bool;

enum {
    CMD_STOP,
    CMD_RUN,
};

//必须根据地锁版本来进行配置
#define MODE_NEW

//重要，设置目标的id，必须配置为目标lora模块的地址
#define TARGET_ID       0x0002

#pragma pack(1)
typedef struct {
    uint16_t    u16LocalID;     //本机ID
    uint8_t     u8CHAN;         //通信信道

    uint8_t     u8Cmd;
    uint8_t     u8CmdRunning;   //当前有指令在运行
    uint8_t     u8Resp;
    uint8_t     u8CmdDone;      //异步指令执行完成标志位
    uint8_t     u8ReCMD;        //指令执行过程中接到新指令标志位
} Device;
#pragma pack()

#define GPIO_BEEP       GPIOB
#define GPIO_BEEP_Pin   GPIO_PIN_10
//LORA所占用的引脚
#define GPIO_Lora_M0 	GPIOA
#define GPIO_Lora_M1	GPIOB
#define GPIO_Lora_AUX	GPIOB
#define GPIO_Lora_M0_Pin	GPIO_PIN_8
#define GPIO_Lora_M1_Pin	GPIO_PIN_15
#define GPIO_Lora_AUX_Pin	GPIO_PIN_14
//PA4 方向输入，电机位置检测信号K1
#define GPIO_SENSOR_UP                  GPIOA
#define GPIO_SENSOR_UP_PIN              GPIO_PIN_4
//PA5 方向输入，电机位置检测信号k2
#define GPIO_SENSOR_DOWN                GPIOA
#define GPIO_SENSOR_DOWN_PIN            GPIO_PIN_5
//PB4 方向输出，电机驱动控制信号P
#define GPIO_SENSOR_FORWARE             GPIOB
#define GPIO_SENSOR_FORWARE_PIN         GPIO_PIN_4
//PB5 方向输出，电机驱动控制信号N
#define GPIO_SENSOR_BACKWARD            GPIOB
#define GPIO_SENSOR_BACKWARD_PIN        GPIO_PIN_5
//PA6 方向输出，电机位置传感器发送使能信号K3
#define GPIO_SENSOR_SWITCH              GPIOA
#define GPIO_SENSOR_SWITCH_PIN          GPIO_PIN_6

#endif