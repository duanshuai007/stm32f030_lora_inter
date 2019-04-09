#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "lora_paramter.h"

typedef enum {
  false,
  true,
}bool;

#if 0

#define INFO_DBG  0
#define WARN_DBG  1
#define ERROR_DBG 1

#define DEBUG_INFO(format, ...)   do{if(INFO_DBG){printf("[INFO]:"format, ##__VA_ARGS__);}}while(0)
#define DEBUG_WARN(format, ...)   do{if(WARN_DBG){printf("[WARN][%s]:"format, __FUNCTION__ , ##__VA_ARGS__);}}while(0)
#define DEBUG_ERROR(format, ...)  do{if(ERROR_DBG){printf("[ERROR][%s]:"format, __FUNCTION__ , ##__VA_ARGS__);}}while(0)
#else

#define DEBUG_INFO(format, ...) 
#define DEBUG_WARN(format, ...)
#define DEBUG_ERROR(format, ...)
#endif

  
//Lora模块与endpoint通信的最小时间间隔，单位10ms
//处于唤醒模式的Lora模块每条消息之间必须保持一定
//的时间间隔才能正常附加唤醒码，否则不能正常通信
#define LORA_SEND_MIN_TIMEINTERVAL              80

//如果没有响应，重试次数
#define CMD_MAX_RETRY_TIMES     3


//CMD没有响应CMD_TIMEOUT秒后就认为超时
#define CMD_TIMEOUT             (3 * 1000)

//等待地锁完成动作抬起或者落下
#define CMD_WAIT_FIN_ACTION_TIMEOUT             (8*1000)

//长时间没有收到endpoint的响应，则认为掉线
#define CMD_ONLINE_TIMEOUT             (210*1000)

//长时间没有收到device的消息，则尝试发送心跳数据给该device
#define CMD_HEART_INTERVAL_TIME        (90*1000)


//串口DMA数据空间
//服务器从节点接收到的resp长度
#define SERVER_RESP_LEN         13

//lora receive module
#define UART1_RX_DMA_LEN        64
#define UART1_RX_DATAPOOL_SIZE  128
//lora send module
#define UART3_TX_DMA_LEN        (sizeof(CmdDataPacket)) //15
#define UART3_TX_DATAPOOL_SIZE  128

//f405 命令长度，固定值不需要修改
//uart2 send buffer size
#define UART2_TX_DMA_LEN        (sizeof(T_Resp_Info))
#define UART2_TX_DATAPOOL_SIZE  240
//uart2 receive buffer size
#define UART2_RX_DMA_LEN        (sizeof(T_Control_Cmd))
#define UART2_RX_DATAPOOL_SIZE  240

//目标设备的信息
#pragma pack(1)
typedef struct {
  uint8_t   u8Cmd;        //发送的命令
  uint16_t  u16TargetID;  //目标ID
  uint32_t  u32Identify;
} Device;
#pragma pack()

#endif
