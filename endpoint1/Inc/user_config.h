#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"

typedef enum {
  false,
  true,
}bool;

enum {
  CMD_STOP,
  CMD_RUN,
};

enum {
  CMD_EXEC_NONE = 0,
  CMD_EXEC_DOING,
  CMD_EXEC_DONE
};

#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A

//单位厘米，原单位是毫米，值400
#define ULTRASONIC_MIN_SAFE_DISTANCE  40

//接收缓冲
#define SERVER_SEND_CMD_LEN     (12)
//发送缓冲
#define SERVER_REC_RESP_LEN     (13)

//必须根据地锁版本来进行配置
#define MODE_OLD

#define RECMD_MAX_NUMBER         10

#pragma pack(1)

typedef struct {
  uint8_t u8Cmd;
  uint32_t u32Identify;
} reCmd;

typedef struct {
  uint8_t u8Cmd;
  uint8_t u8CmdRunning;   //当前有指令在运行
  volatile uint8_t u8MotorResp;         //专用于电机的异步返回
  volatile uint8_t u8CmdDone;      //异步指令执行完成标志位
  
  volatile bool bInteFlag;   //中断标志
  bool bInterDone;
  volatile uint8_t u8ReCmdNumber;  //接收到的重复指令的个数
  uint8_t u8LPUseRTC;         //在低功耗模式下是否使用rtc唤醒,每有个外设调用它就+1，使用完毕则-1.
  
  volatile bool bHasRtcInter;
  volatile bool bHasLoraInter;
  volatile bool bHasMotorNormalInter;
  volatile bool bHasMotorAbnmalInter;
  
  volatile uint8_t u8ReCmdPos;
  uint16_t u16UltraDistance;
  uint32_t u32Identify;
  uint8_t u8UltraSafeDistance;
  
  reCmd sReCmd[RECMD_MAX_NUMBER];
} Device;

typedef struct {
  uint8_t u8Cmd;
  uint8_t u8Resp;
  uint32_t u32Identify;
} MsgDevice;
#pragma pack()

#define GPIO_BEEP                       GPIOB
#define GPIO_BEEP_Pin                   GPIO_PIN_10
//LORA所占用的引脚
#define GPIO_Lora_M0 	                  GPIOA
#define GPIO_Lora_M1	                  GPIOB
#define GPIO_Lora_AUX	                  GPIOB
#define GPIO_Lora_M0_Pin	              GPIO_PIN_8
#define GPIO_Lora_M1_Pin	              GPIO_PIN_15
#define GPIO_Lora_AUX_Pin	              GPIO_PIN_14
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
//PB13 超声波使能引脚
#define GPIO_ULTRASONIC                 GPIOB
#define GPIO_ULTRASONIC_PIN             GPIO_PIN_13

#endif