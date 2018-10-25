#ifndef _LORA_H_
#define _LORA_H_

#include "stdint.h"
#include "stm32f0xx.h"
#include "lora_paramter.h"
#include "user_config.h"

#define USE_INTERRUPT

#define MAX_DMA_LEN     128

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

typedef struct {
    uint8_t u8Head;
    uint8_t u8Len;
    uint16_t u16Id;
    uint8_t u8Cmd;
    uint8_t u8Resp;
    uint16_t u16Crc;
} RespDataPacket;

typedef struct {
    uint8_t u8Head;
    uint8_t u8Len;
    uint16_t u16Id;
    uint8_t u8Cmd;
    uint16_t u16Crc;
} CmdDataPacket;

typedef struct {
    volatile uint8_t    sflag;  //send flag
    volatile uint8_t    rflag;  //receive flag
    volatile uint8_t    pos;    //buffer current postion
    volatile uint8_t    isIdle; //IDLE FLAG
    volatile uint8_t    setflag; //set and read paramter flag
    
    Lora_Mode           mode;   //work mode
    LoraPar             paramter;
    LoraVersion         version;
    
    uint8_t             dma_sbuff[MAX_DMA_LEN/4]; //dma send buffer
    uint8_t             dma_rbuff[MAX_DMA_LEN]; //dma receive buffer
} LoraPacket;

#pragma pack()

//set work mode
bool Lora_Module_Set_Mode(LoraPacket *lp, Lora_Mode lm);
//check idle
uint8_t Lora_is_idle(void);
//When setting or reading parameters, the serial port must be no parity.
//set paramter
uint8_t Lora_SetPar(uint8_t flag, LoraPacket *lp);
//read paramter
uint8_t Lora_Read(uint8_t no, LoraPacket *lp);

void LoraSetAndReadParamter(UART_HandleTypeDef *huart);

//reset lora module
uint8_t Lora_Reset(LoraPacket *lp);
//send data
bool LoraTransfer(Device *d);
//read data
int8_t LoraReceiveData(DMA_HandleTypeDef *hdma, LoraPacket *lp);

#endif
