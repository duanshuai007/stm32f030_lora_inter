#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include <stdbool.h>
#include "stm32f0xx.h"
#include "lora_paramter.h"

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
#define LORA_MAX_CHANNEL 30 //endpoint channel range : 0-29

#define SYSTEM_RTC_CYC             5 //RTC定时唤醒cpu周期

#define SYSTEM_HEARTBEAT_CYC       60 //主动上报心跳周期
//单位厘米，原单位是毫米，值400
#define ULTRASONIC_MIN_SAFE_DISTANCE  (40)

//接收缓冲
#define SERVER_SEND_CMD_LEN     (12)
//发送缓冲
#define SERVER_REC_RESP_LEN     (13)

#define SERVER_REC_RESP_HEAD_LEN (3)

//必须根据地锁版本来进行配置
#define MODE_OLD

#define RECMD_MAX_NUMBER         (10)

#pragma pack(1)

typedef struct {
  uint8_t u8Cmd;
  uint32_t u32Identify;
} reCmd;

//名字最后带有Inter的都是在中断中有修改值的变量
typedef struct {
  volatile bool bMotorUseRTCInter;         //在低功耗模式下是否使用rtc唤醒,每有个外设调用它就+1，使用完毕则-1.
  volatile bool bHasRtcInter;
  volatile bool bHasLoraInter;
  volatile bool bHasMotorNormalInter;
  
  volatile uint8_t u8Cmd;
  volatile uint8_t u8CmdRunning;   //当前有指令在运行
  volatile uint8_t u8MotorResp;         //专用于电机的异步返回
  volatile uint8_t u8CmdDone;      //异步指令执行完成标志位
  
  volatile uint8_t u8ReCmdNumberInter;  //接收到的重复指令的个数
  volatile uint8_t u8ReCmdPosInter;
  volatile uint8_t u8UltraSafeDistance;      //超声波安全距离
  volatile uint8_t u8UltraCheckTimeinterval; //超声波检测时间间隔

  volatile uint8_t u8MotorRtcTimerInter;
//  volatile uint8_t u8UltraHasDataInter;       //接收到的超声波数据的次数
//  uint16_t u16UltraDistance;    //读取到的超声波距离值
  
  uint32_t u32Identify;
  volatile uint32_t u32GlobalTimeCount; //全局时间计数器
  uint32_t u32LastCheckUltraTime; //超声波检测的上一次时间戳
  uint32_t u32LastHeartTime;      //心跳的上一次时间戳
  
  //  bool bMotorAbnormal;
  //用来判断当前次的循环中是否已经获取过电机的位置信息，如果获取则不需要重复获取
  volatile bool bHasGetMotorStatus;
  
  reCmd sReCmd[RECMD_MAX_NUMBER];
} Device;

typedef struct {
  uint8_t u8Cmd;
  uint8_t u8Resp;
  uint32_t u32Identify;
} MsgDevice;
#pragma pack()

#define GPIO_BEEP                       GPIOB
#define GPIO_BEEP_Pin                   GPIO_PIN_10     //下拉电阻2K
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
#define GPIO_MOTOR_FORWARE             GPIOB
#define GPIO_MOTOR_FORWARE_PIN         GPIO_PIN_4
//PB5 方向输出，电机驱动控制信号N
#define GPIO_MOTOR_BACKWARD            GPIOB
#define GPIO_MOTOR_BACKWARD_PIN        GPIO_PIN_5
//PA6 方向输出，电机位置传感器发送使能信号K3
#define GPIO_SENSOR_SWITCH              GPIOA
#define GPIO_SENSOR_SWITCH_PIN          GPIO_PIN_6
//PB13 超声波使能引脚
#define GPIO_ULTRASONIC                 GPIOB
#define GPIO_ULTRASONIC_PIN             GPIO_PIN_13

#endif
