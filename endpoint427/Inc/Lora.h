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
  uint8_t u8Speedinair:3;
  uint8_t u8Baud:3;
  uint8_t u8Parity:2;
} Byte2;

typedef struct {
  uint8_t u8SendDB:2;
  uint8_t u8FEC:1;
  uint8_t u8WakeUpTime:3;
  uint8_t u8IOMode:1;
  uint8_t u8TranferMode:1;
} Byte4;

//5 Bytes
typedef struct {
    //Byte0 Save or not save
    SaveType saveType;
    //Byte0-1
    uint8_t u8AddrH;
    uint8_t u8AddrL;
    //Byte2
//    uint8_t u8Parity;   //6-7bit
//    uint8_t u8Baud;     //3-5bit        
//    uint8_t u8Speedinair; //0-2bit
    Byte2 sParaOne;
    //Byte3
    uint8_t u8Channel;  //0-4 or 0-5 bit
    //Byte4
//    uint8_t u8TranferMode;      //7bit 
//    uint8_t u8IOMode;           //6bit
//    uint8_t u8WakeUpTime;       //3-5bit
//    uint8_t u8FEC;              //2bit
//    uint8_t u8SendDB;           //0-1bit
    Byte4 sParaTwo;
} LoraPar;

//�ڵ�resp�ṹ�� s:13+3
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

//������cmd�ṹ�� r:12
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
    uint8_t   u8ServerCH; //server channel
    LoraPar   paramter;
    uint8_t   rbuff_size;
    uint8_t   *rbuff; //receive buffer
} LoraPacket;

#pragma pack()


void LoraModuleInit(void);

void SetServer(uint16_t serverid, uint8_t ch);
//When setting or reading parameters, the serial port must be no parity.
void LoraWriteParamter(void);

void LoraReadParamter(void);

void LoraModuleITInit(void);

void LoraSendReCmd(void);
void LoraSendResp(uint8_t cmd, uint8_t resp, uint32_t identify);

/*
*   �ж�ģ���Ƿ����
*/
bool LoraModuleIsIdle(void);

/*
*   Loraģ������Ӧ�Ĵ������ݽ�������жϴ���
*/
void LoraModuleReceiveHandler(void);


void LoraUartEnable(void);
void LoraUartDisable(void);

#endif
