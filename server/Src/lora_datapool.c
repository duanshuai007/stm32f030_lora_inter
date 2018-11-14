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
*   ������д�뵽���ݳ���
*/
uint16_t DataPoolWrite(DataPool *ldp, uint8_t *buf, uint16_t len)
{
  uint16_t i;
  
  for ( i = 0; i < len; i++ ) {
    ldp->pool[ldp->u16End++] = buf[i];
    if ( ldp->u16End == ldp->u16MaxSize ) {
      //���ﻺ��ص�ĩβ
      ldp->u16End = 0;
    }
    if ( ldp->u16End == ldp->u16Start ) {
      //�������������ܴ����µ�����,�����Ѿ���������ݳ���
      i++;
      break;
    }
  }
  
  return i;
}

/*
*   �����ݳ��ж�ȡ��ǰ�Ŀ�ʼ���ֽ�����
*   ���ı�start��λ��
*   ����ֵ0:δ��ȡ������
*   ����ֵ1:��ȡ������
*/
uint8_t DataPoolLookByte(DataPool *ldp, uint8_t *data)
{   
  if (ldp->u16Start == ldp->u16End)
    return 0;
  
  *data = ldp->pool[ldp->u16Start];
  
  return 1;
}

/*
*   �����ݳ��л�ȡ��ǰstart��������
*   startָ����һ������
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
*   �����ݳ���ȡ���������
*   return: ����ȡ�������ݳ���
*/
uint8_t DataPoolGetNumByte(DataPool *ldp, uint8_t *buf, uint8_t len)
{
  uint16_t u16End = ldp->u16End;
  uint8_t i;
  
  if ( ldp->u16Start == u16End ) {
    return 0;
  } else if (ldp->u16Start < u16End) {
    //������д�����ݣ��˿�start��end֮ǰ
    if (u16End - ldp->u16Start < len) { 
      //���ݲ����㳤��
      return 0;
    }
  } else if (ldp->u16Start > u16End) {
    if ( ldp->u16MaxSize - ldp->u16Start + u16End < len ) {
      //���ݳ��Ȳ���
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
*   �ָ����ݳ��е�����(�ƶ�start��λ��)
*/
void DataPoolResume(DataPool *ldp, uint8_t len)
{
  if (ldp->u16Start >= len)
    ldp->u16Start -= len;
  else
    ldp->u16Start = (ldp->u16MaxSize - (len - ldp->u16Start));
}
