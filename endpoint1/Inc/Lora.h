#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"
#include "user_config.h"

#define USE_INTERRUPT

#define MAX_DMA_LEN     64

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

//new modify start
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
//new modify end 

typedef struct {
    volatile uint8_t    pos;    //buffer current postion
    volatile uint8_t    isIdle; //IDLE FLAG
  
    Lora_Mode           mode;   //work mode
    LoraPar             paramter;

    uint8_t             dma_sbuff[20]; //dma send buffer
    uint8_t             dma_rbuff[MAX_DMA_LEN]; //dma receive buffer
} LoraPacket;

#pragma pack()

//When setting or reading parameters, the serial port must be no parity.
void LoraSetParamter(LoraPacket *lm, 
                     uint16_t addr, 
                     BaudType baud, 
                     CHANType channel, 
                     ParityType parity,
                     SpeedInAirType speedinair, 
                     TransferModeType transmode, 
                     WakeUpTimeType wutime);

void LoraReadParamter(LoraPacket *lp);

void LoraModuleDMAInit(LoraPacket *lp);
//reset lora module
uint8_t Lora_Reset(LoraPacket *lp);
//send data
bool LoraTransfer(Device *d, uint8_t flag);

void CopyDataFromDMA(void);
void UartDataProcess(void);
uint8_t LoraRegister(void);

#endif
