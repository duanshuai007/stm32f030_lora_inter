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
*   ������д�뵽���ݳ���
*/
uint16_t DataPoolWrite(DataPool *ldp, uint8_t *buf, uint16_t len);

/*
*   �����ݳ��ж�ȡ��ǰ�Ŀ�ʼ���ֽ�����
*   ���ı�start��λ��
*/
uint8_t DataPoolLookByte(DataPool *ldp, uint8_t *data);
/*
*   �����ݳ��л�ȡ��ǰstart��������
*   startָ����һ������
*/
uint8_t DataPoolGetByte(DataPool *ldp, uint8_t *data);
/*
*   �����ݳ���ȡ���������
*   return: ����ȡ�������ݳ���
*/
uint8_t DataPoolGetNumByte(DataPool *ldp, uint8_t *buf, uint8_t len);

/*
*   �ָ����ݳ��е�����(�ƶ�start��λ��)
*/
void DataPoolResume(DataPool *ldp, uint8_t len);

#endif



