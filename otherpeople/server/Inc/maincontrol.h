#ifndef _MAINCONTROL_H_
#define _MAINCONTROL_H_

#include "stdint.h"
#include "user_config.h"
#include "lora.h"
#include "list.h"

//设备所能识别的cmd
typedef enum 
{
  CMD_NONE              = 0,
  CMD_MOTOR_UP          = 1,
  CMD_MOTOR_DOWN        = 2,
  CMD_MOTOR_STATUS_GET  = 3,
  CMD_BEEP_ON           = 4,
  CMD_BEEP_OFF          = 5,
  CMD_BEEP_STATUS_GET   = 6,
  CMD_ADC_GET           = 7, 
  
  HW_ULTRA_GET          = 8,
  CMD_MODIFY_DISTANCE   = 9,
  CMD_GET_DISTANCE      = 10,
  CMD_MODIFY_SCANCYCLE  = 11,
  CMD_GET_SCANCYCLE     = 12,
  CMD_SYSTEM_RESET      = 13, 
  
  CMD_GET_ALL_NODE      = 31,
  CMD_UPDATE            = 32,     
  CMD_ADD_ENDPOINT      = 33,
  CMD_DEL_ENDPOINT      = 34,
  CMD_MODIFY_CHANNEL    = 35,
  CMD_MODIFY_SERVER_DISTANCE    = 36,
  CMD_GET_SERVER_DISTANCE    = 37,
  

}CMD_TYPE;

//地锁的状态
typedef enum 
{
  EP_STATUS_NONE             = 0,
  EP_STATUS_DOWN,
  EP_STATUS_FORWARD,
  EP_STATUS_UP,  
  EP_STATUS_BACK

}EP_STATUS;

//地锁的状态
typedef enum 
{
  STATUS_NO_HAVE_CAR          = 0,
  STATUS_HAVE_CAR             = 1,
  STATUS_TROUBLE              = 0xe,
  STATUS_VALID                = 0xf
}STATUS_ULTRA;

//server内部与endpoint之间的通信命令，与405 cmd共占用资源
#define DEVICE_REGISTER     100
#define DEVICE_HEART        101
#define DEVICE_ABNORMAL     102

typedef enum
{
  //Normal
  NORMAL_SUCCESS            = 101,
  NORMAL_DOWN               = 102,
  NORMAL_FORWARD            = 103,
  NORMAL_UP                 = 104,
  NORMAL_BACK               = 105,
  NORMAL_BUSY               = 106,
  NORMAL_BEEP_OPEN_FAILED   = 107,
  NORMAL_BEEP_CLOSE_FAILED  = 108,
  NORMAL_BEEP_STATUS_OPEN   = 109,
  NORMAL_BEEP_STATUS_CLOSED = 110,
  NORMAL_MOTOR_RUNNING      = 111,
  NORMAL_HAVE_CAR           = 112,
  NORMAL_UNKNOWN_ERROR      = 113,
  NORMAL_CAR_CHECK_ERROR    = 114,

  NODE_ONLINE               = 120,
  NODE_OFFLINE              = 121,
  NODE_NOTEXIST             = 122,
  NODE_NO_SUPPORT_CMD       = 123,

  //get firmware
  GET_RUN_BIN = 131,  
  GET_RECOVERY_BIN = 132,  
  GET_RUN_BIN_FIN = 133,  
  GET_RECOVERY_BIN_FIN = 134,  
  ENDPOINT_ADD_SUCCESS = 135,  
  ENDPOINT_DEL_SUCCESS = 136,  
  CHANNEL_MODIFY_SUCCESS = 137,  
  SOUND_DIST_SET_SUCCESS = 138, 
  SOUND_DIST_SET_FAILED = 139, 
  SOUND_DIST_GET_SUCCESS = 140, 
  SOUND_SCANCYCLE_SET_SUCCESS = 141, 
  SOUND_SCANCYCLE_SET_FAILED = 142, 
  SOUND_SCANCYCLE_GET_SUCCESS = 143,
    
    
  //通知服务器是否有车
  UPLOAD_HAVE_CAR = 150,
  UPLOAD_NO_HAVE_CAR = 151,
  UPLOAD_SOUNDWAVE_TROUBLE = 152,  
  
  //interupt
  INTERUPT_DOWN             = 201,
  INTERUPT_FORWARD          = 202,    
  INTERUPT_UP               = 203,
  INTERUPT_BACK             = 204,
}RESP_CODE;

/*
*   初始化与F405通信的串口相关结构体
*/
void UARTF405_Init(UART_HandleTypeDef *uart);

void UartSendOnlineMSGToF405(DeviceNode *device);
void UartSendAutoMSGToF405(uint8_t status, uint16_t id, uint32_t identify);
void UartSendVerMSGToF405(uint8_t resp_code, uint8_t *version);


/*
*   向F405发送指令响应信息，成功返回true，失败返回false
*/
bool UartSendRespToF405(DeviceNode *devn);

/*
*   获取uart module结构体
*/
UartModule *GetF405UartModule(UART_HandleTypeDef *huart);

/*
*   中断函数:F405接收函数
*   在串口接收完成中断中被调用
*/
void F405ReveiveHandler(UART_HandleTypeDef *huart);

/*
*   F405命令处理线程
*   F405只能控制由本机发送出去的设备ID，对于其他ID不进行操作
*/
void F405Task(void);

/*
*   停止从F405接收新的数据
*   不影响等待发送中的数据
*/
void closeF405Communication(void);

#endif



