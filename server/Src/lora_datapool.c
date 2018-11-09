#include "lora_datapool.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_def.h"
#include "stdlib.h"
#include "string.h"
#include "user_config.h"

DataPool *DataPoolInit(uint16_t size) 
{
  DataPool *ldp = (DataPool *)malloc(sizeof(DataPool));
  if ( ldp == NULL )
  {
    PRINT("ldp null\r\n");
    return NULL;
  }
  memset(ldp, 0, sizeof(DataPool));
  
  ldp->pool = (uint8_t *)malloc(sizeof(uint8_t) * size);
  if ( ldp->pool == NULL ) {
    PRINT("ldp->pool null\r\n");
    free(ldp);
    return NULL;
  }
  memset(ldp->pool, 0, size * sizeof(uint8_t));
  
  ldp->u16End = 0;
  ldp->u16Start = 0;
  ldp->u16MaxSize = size;
  
  return ldp;
}

/*
*   将数据写入到数据池中
*/
uint16_t DataPoolWrite(DataPool *ldp, uint8_t *buf, uint16_t len)
{
  uint16_t i;
  
  for ( i = 0; i < len; i++ ) {
    ldp->pool[ldp->u16End++] = buf[i];
    if ( ldp->u16End == ldp->u16MaxSize ) {
      //到达缓冲池的末尾
      ldp->u16End = 0;
    }
    if ( ldp->u16End == ldp->u16Start ) {
      //数据已满，不能存入新的数据,返回已经存入的数据长度
      i++;
      break;
    }
  }
  
  return i;
}

/*
*   从数据池中读取当前的开始处字节数据
*   不改变start的位置
*   返回值0:未读取到数据
*   返回值1:读取到数据
*/
uint8_t DataPoolLookByte(DataPool *ldp, uint8_t *data)
{   
  if (ldp->u16Start == ldp->u16End)
    return 0;
  
  *data = ldp->pool[ldp->u16Start];
  
  return 1;
}

/*
*   从数据池中获取当前start处的数据
*   start指向下一个数据
*/
uint8_t DataPoolGetByte(DataPool *ldp, uint8_t *data)
{
  if (ldp->u16Start == ldp->u16End)
    return 0;
  
  *data = ldp->pool[ldp->u16Start++];
  
  if (ldp->u16Start == ldp->u16MaxSize) {
    ldp->u16Start = 0;
  }
  
  return 1;
}

/*
*   从数据池中取出多个数据
*   return: 返回取出的数据长度
*/
uint8_t DataPoolGetNumByte(DataPool *ldp, uint8_t *buf, uint8_t len)
{
  uint16_t u16End = ldp->u16End;
  uint8_t i;
  
  if ( ldp->u16Start == u16End ) {
    return 0;
  } else if (ldp->u16Start < u16End) {
    //正常的写入数据，此刻start在end之前
    if (u16End - ldp->u16Start < len) { 
      //数据不满足长度
      return 0;
    }
  } else if (ldp->u16Start > u16End) {
    if ( ldp->u16MaxSize - ldp->u16Start + u16End < len ) {
      //数据长度不够
      return 0;
    }
  }
  
  for(i = 0; i < len; i++) {
    
    buf[i] = ldp->pool[ldp->u16Start++];
    
    if(ldp->u16Start == ldp->u16MaxSize)
      ldp->u16Start = 0;
    
    if(ldp->u16Start == ldp->u16End) {
      i++;
      break;
    }
  }
  
  return i;
}


/*
*   恢复数据池中的数据(移动start的位置)
*/
void DataPoolResume(DataPool *ldp, uint8_t len)
{
  if (ldp->u16Start >= len)
    ldp->u16Start -= len;
  else
    ldp->u16Start = (ldp->u16MaxSize - (len - ldp->u16Start));
}

