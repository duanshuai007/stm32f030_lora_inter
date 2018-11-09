#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f1xx.h"
#include "lora_paramter.h"
#include "lora_datapool.h"
#include "list.h"
#include "user_config.h"
#include "crc16.h"

typedef enum { 
  LoraMode_Normal,
  LoraMode_WakeUp,
  LoraMode_LowPower,
  LoraMode_Sleep,
}Lora_Mode;

#pragma pack(1)

typedef struct {
  uint8_t u8Head;
  uint8_t u8LoraMHZ;
  uint8_t u8Version;
  uint8_t u8Nummy;
} LoraVersion;

typedef struct {
  uint16_t u16Addr;    
  //Byte3
  uint8_t u8Parity;   //6-7bit
  uint8_t u8Baud;     //3-5bit        
  uint8_t u8Speedinair; //0-2bit
  //Byte4
  uint8_t u8Channel;  //0-4 or 0-5 bit
  //Byte5
  uint8_t u8TranferMode;      //7bit 
  uint8_t u8IOMode;           //6bit
  uint8_t u8WakeUpTime;       //3-5bit
  uint8_t u8FEC;              //2bit
  uint8_t u8SendDB;           //0-1bit
} LoraPar;

//节点resp结构体
typedef struct {
  uint8_t   u8Head;
  uint8_t   u8Len;
  uint16_t  u16Id;
  uint8_t   u8Cmd;
  uint8_t   u8Resp;
  uint32_t  u32Identify;
  uint16_t  u16Crc;
  uint8_t   u8Tail;
} RespDataPacket;

//服务器cmd结构体
typedef struct {
  uint8_t   u8Head;
  uint8_t   u8Len;
  uint16_t  u16Id;
  uint8_t   u8Cmd;
  uint32_t  u32Identify;
  uint16_t  u16Crc;
  uint8_t   u8Tail;
} CmdDataPacket;

typedef struct {
  GPIO_TypeDef  *GPIOX;
  uint16_t      Pin;
} LoraPin;

typedef struct {
  LoraPin AUX;
  LoraPin M0;
  LoraPin M1;
} LoraGPIO;

//串口结构体
typedef struct {
  
  volatile uint16_t   pos;    //buffer current postion
  UART_HandleTypeDef  *uart;
  DMA_HandleTypeDef   *uart_tx_hdma;
  DMA_HandleTypeDef   *uart_rx_hdma;

  uint16_t    dma_rbuff_size;  //dma接收缓冲区的大小
  uint16_t    dma_sbuff_size;  //dma发送缓冲区的大小
  
  uint8_t *dma_rbuff;         //指向dma接收缓冲区
  uint8_t *dma_sbuff;         //指向dma发送缓冲区
  
  DataPool *dataPool;
  
} UartModule;

//保存本机lora设备相关信息
typedef struct {
  volatile uint8_t    isIdle; //IDLE FLAG
  Lora_Mode           mode;   //work mode
  LoraPar             paramter;
  LoraGPIO            gpio;
  UartModule          muart; 
} LoraModule;

//modify by liyongsheng begin
typedef union
{
  uint32_t identity;
  uint16_t endpointId;  
}U_Resp_Data;

typedef struct 
{
  uint8_t resp_code;
  U_Resp_Data resp_data;
  uint8_t crc;
}T_Resp_Info;

typedef struct 
{
  uint32_t identity;
  uint16_t endpointId;
  uint8_t action;
  uint8_t crc;
}T_Control_Cmd;
//modify by liyongsheng end

#pragma pack()

//初始化结构体内的参数
void LoraModuleGPIOInit(LoraModule *lm, UART_HandleTypeDef *huart);
void LoraModuleDMAInit( LoraModule *lm);

//When setting or reading parameters, the serial port must be no parity.
void LoraSetParamter(LoraModule *lm, 
                     uint16_t addr, 
                     BaudType baud, 
                     CHANType channel, 
                     ParityType parity, 
                     SpeedInAirType sppedinair, 
                     TransferModeType transmode, 
                     WakeUpTimeType wutime);

void LoraReadParamter(LoraModule *lm);
//reset lora module
uint8_t Lora_Reset(LoraModule *lp);
//control endpoint
//bool LoraCtrlEndpoint(LoraModule *lp, Device *d);
bool LoraCtrlEndPoint(LoraModule *lm, uint16_t id, uint8_t channel, uint8_t cmd, uint32_t timestamp);
//move data from dma to pool
void CopyDataFromDMA(UART_HandleTypeDef *huart);
//
void LoraPoolDataProcess(LoraModule *lp);

#endif



