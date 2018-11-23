#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"
#include "user_config.h"
#include "lora_datapool.h"

#define NORMALCMD   1
#define RECMD       0

#pragma pack(1)

typedef enum { 
    LoraMode_Normal,
    LoraMode_WakeUp,
    LoraMode_LowPower,
    LoraMode_Sleep,
}Lora_Mode;

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
    volatile bool     isIdle; //IDLE FLAG
  
    Lora_Mode mode;   //work mode
    LoraPar   paramter;

    uint8_t   dma_sbuff_size;
    uint8_t   dma_rbuff_size;
    
    uint8_t   *dma_sbuff; //dma send buffer
    uint8_t   *dma_rbuff; //dma receive buffer
} LoraPacket;

#pragma pack()


void LoraModuleInit(void);

//When setting or reading parameters, the serial port must be no parity.
void LoraWriteParamter(LoraPar *lp);

void LoraReadParamter(void);

void LoraModuleDMAInit(void);
//reset lora module
uint8_t Lora_Reset(LoraPacket *lp);
//send data
bool LoraTransfer(uint8_t flag);


bool LoraRegister(void);

/*
*   判断模块是否空闲
*/
bool LoraModuleIsIdle(void);

/*
*   AUX引脚中断处理函数
*/
void LoraModuleIsIdleHandler(void);

/*
*   Lora模块所对应的串口数据接收完成中断处理
*/
void LoraModuleReceiveHandler(void);

#endif
