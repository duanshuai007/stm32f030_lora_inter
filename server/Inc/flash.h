#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_

#include "stdint.h"
#include "user_config.h"

#define FLASH_SERVERADDR_START  0x0800f000
#define FLASH_DATABASE_START    0x0800f030
#define SAVE_MESSAGE_HEAD       0x474f5246        //"FROG"
#define DEVICE_MAX_NUMBER       200

/*
[                 flash                    ]
[start]                        [message]
[0-3] [4-7]
[HEAD][NUM][device1][device2]...[device[n]]
*/

#define MSG_SIZE  4

#pragma pack(1)
typedef struct SaveMSG{
  uint16_t u16DeviceID;   //�豸ID
  uint16_t u16LastTime;   //�豸���һ����Ӧ��ʱ��
  struct SaveMSG *next;
} SaveMSG;

typedef struct {
  uint8_t u8Total; //���豸����
  SaveMSG *Head;  //head������ͷ������ʾ�κ��豸
} FLASHDeviceList;
#pragma pack()

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

/*
*   Flashl�����ʼ��
*   �Ӵ洢������ȡ���е��豸��Ϣд��flash������
*/
bool FLASH_Init(uint16_t *sid, uint8_t *sch, uint8_t *rch);

/*
*   ���豸��ӵ������б���
*/
bool FlashAddDeviceToList(uint16_t id, uint16_t time); 

/*
*   ��Flash������ɾ���ڵ�
*/
void FlashDelDeviceFromList(uint16_t id);

/*
*   Flash�����豸��Ϣ��ÿ�� FLASH_SAVE_TIMEINTERVAL ��
*   ��flash����gFDList������д�뵽flash�ڴ��С�
*/
void FlashTask(void);

/*
*   ��F405���������豸������Ϣ
*/
void SendALLDeviceNodeToF405(void);

/*
*   Debug Function
*/
#if 0
void LookList(void);
void FLASH_CLEAN(void);
#endif

#endif