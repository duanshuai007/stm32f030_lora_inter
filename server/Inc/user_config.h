#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "lora_paramter.h"

typedef enum {
  false,
  true,
}bool;

//#define PRINT(...) printf(__VA_ARGS__)
#define PRINT(...)

#define DEFAULT_CHANNEL 0x1e

#define LORA_MSG_HEAD 0xA5
#define LORA_MSG_TAIL 0x5A
#define POOL_CLEAN    0x00

  //电机运行状态信息
#define MOTOR_RUNING          99   //地锁电机开始运行了
#define CMD_STATUS_IDLE       100   //空闲状态
#define CMD_STATUS_RUN        101   //正在运行
#define CMD_STATUS_DONE       102   //运行完成
#define CMD_STATUS_STOP       103   //超时停止运行
#define CMD_STATUS_TIMEOUT    104   //超出响应时间没有应答
#define CMD_STATUS_HEART      105   //接收到设备的心跳响应
#define CMD_STATUS_STANDBY    106   //有新指令，等待运行
//modify by liyongsheng begin

typedef enum 
{
  CMD_MOTOR_UP = 1,
  CMD_MOTOR_DOWN = 2,
  CMD_MOTOR_STATUS_GET = 3,
  CMD_BEEP_ON = 4,
  CMD_BEEP_OFF = 5,
  CMD_BEEP_STATUS_GET = 6,
  CMD_ADC_GET = 7,  
  CMD_MOTOR_ABNORMAL = 8,
  CMD_DEVICE_REGISTER = 9,
  CMD_DEVICE_HEART = 10,

}CMD_TYPE;

typedef enum
{
  //Normal
  NORMAL_SUCCESS = 101,
  NORMAL_DOWN = 102,
  NORMAL_FORWARD = 103,    
  NORMAL_UP = 104,
  NORMAL_BACK = 105,
  NORMAL_BUSY = 106,
  NORMAL_BEEP_OPEN_FAILED = 107,
  NORMAL_BEEP_CLOSE_FAILED = 108,
  NORMAL_BEEP_STATUS_OPEN = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,

  NODE_ONLINE = 120,
  NODE_OFFLINE = 121,
  
  //interupt
  INTERUPT_DOWN = 201,
  INTERUPT_FORWARD = 202,    
  INTERUPT_UP = 203,
  INTERUPT_BACK = 204,  
  
  ERROR_CMD_TIMEOUT = 210,
  
}RESP_CODE;

//modify by liyongsheng end

//串口DMA数据空间
//服务器发送到节点的命令长度
#define SERVER_CMD_LEN          12
#define SERVER_CMD_HEAD_SIZE    3
//服务器从节点接收到的resp长度
#define SERVER_RESP_LEN         13

//lora receive module
#define UART1_RX_DMA_LEN        64
#define UART1_RX_DATAPOOL_SIZE  128
//lora send module
#define UART3_TX_DMA_LEN        (SERVER_CMD_LEN + SERVER_CMD_HEAD_SIZE) //15

//f405
#define F405_SEND_CMD_LEN       10
#define F405_RECV_CMD_LEN       10
//uart2 send buffer size
#define UART2_TX_DMA_LEN        (F405_SEND_CMD_LEN)
//uart2 receive buffer size
#define UART2_RX_DMA_LEN        64
#define UART2_RX_DATAPOOL_SIZE  128


//目标设备的信息
#pragma pack(1)
typedef struct {
  uint8_t   u8Cmd;        //发送的命令
  uint16_t  u16TargetID;  //目标ID
  uint32_t  u32Identify;
} Device;
#pragma pack()

//LORA Receive所占用的引脚
//PA8 M0
#define GPIO_Lora_M0_R 	        GPIOA
#define GPIO_Lora_M0_R_Pin	GPIO_PIN_8
//PA12 M1
#define GPIO_Lora_M1_R	        GPIOA
#define GPIO_Lora_M1_R_Pin	GPIO_PIN_12
//PA11 AUX
#define GPIO_Lora_AUX_R	        GPIOA
#define GPIO_Lora_AUX_R_Pin	GPIO_PIN_11

//LORA Send所占用的引脚
//PB13 M0
#define GPIO_Lora_M0_S          GPIOB
#define GPIO_Lora_M0_S_Pin      GPIO_PIN_13
//PB12 M1
#define GPIO_Lora_M1_S          GPIOB
#define GPIO_Lora_M1_S_Pin      GPIO_PIN_12
//PA4 AUX
#define GPIO_Lora_AUX_S         GPIOA
#define GPIO_Lora_AUX_S_Pin     GPIO_PIN_4

//PA7 led
#define GPIO_LED                GPIOA
#define GPIO_LED_Pin            GPIO_PIN_7

#endif
