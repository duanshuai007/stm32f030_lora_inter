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

//节点resp结构体 s:13+3
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

//服务器cmd结构体 r:12
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
    Lora_Mode mode;   //work mode
    uint16_t  u16ServerID;
    uint8_t   u8ServerCH;
    LoraPar   paramter;

//    uint8_t   sbuff_size;
    uint8_t   rbuff_size;
    
//    uint8_t   *sbuff; //send buffer
    uint8_t   *rbuff; //receive buffer
} LoraPacket;

#pragma pack()


void LoraModuleInit(void);

void SetServer(uint16_t serverid, uint8_t ch);
//When setting or reading parameters, the serial port must be no parity.
void LoraWriteParamter(LoraPar *lp);

void LoraReadParamter(void);

void LoraModuleITInit(void);
//reset lora module
uint8_t Lora_Reset(LoraPacket *lp);
//generate normal data
uint8_t GetSendData(uint8_t *sendbuff, MsgDevice *d);
//generate recmd data
uint8_t GetRecmdSendData(uint8_t *sendbuff, Device *d);
//send data
bool LoraSend(uint8_t *sbuff, uint8_t len);

/*
*   判断模块是否空闲
*/
bool LoraModuleIsIdle(void);

/*
*   Lora模块所对应的串口数据接收完成中断处理
*/
void LoraModuleReceiveHandler(void);

#endif
