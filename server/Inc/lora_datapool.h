#ifndef __LORA_DATAPOOL_H__
#define __LORA_DATAPOOL_H__

#include "stdint.h"
#include "user_config.h"

#pragma pack(1)
typedef struct {
  uint16_t  u16Start;
  volatile uint16_t u16End;
  uint16_t  u16MaxSize;
  uint8_t   *pool;
} DataPool;
#pragma pack()

DataPool *DataPoolInit(uint16_t size);

/*
*   将数据写入到数据池中
*/
uint16_t DataPoolWrite(DataPool *ldp, uint8_t *buf, uint16_t len);

/*
*   从数据池中读取当前的开始处字节数据
*   不改变start的位置
*/
uint8_t DataPoolLookByte(DataPool *ldp, uint8_t *data);
/*
*   从数据池中获取当前start处的数据
*   start指向下一个数据
*/
uint8_t DataPoolGetByte(DataPool *ldp, uint8_t *data);
/*
*   从数据池中取出多个数据
*   return: 返回取出的数据长度
*/
uint8_t DataPoolGetNumByte(DataPool *ldp, uint8_t *buf, uint8_t len);

/*
*   恢复数据池中的数据(移动start的位置)
*/
void DataPoolResume(DataPool *ldp, uint8_t len);

#endif



