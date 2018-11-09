#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_

#include "stdint.h"
#include "list.h"

#define FLASH_DATABASE_START  0x0800f000
#define SAVE_MESSAGE_HEAD     0xA5B4C3D2
#define DEVICE_MAX_NUMBER     200

/*
[                 flash                    ]
[start]                        [message]
[0-3] [4-7]
[HEAD][NUM][device1][device2]...[device[n]]
*/

#define MSG_SIZE  4

typedef struct SaveMSG{
  uint16_t u16DeviceID;   //�豸ID
  uint16_t u16LastTime;   //�豸���һ����Ӧ��ʱ��
  struct SaveMSG *next;
  uint32_t u32RegStatus;
} SaveMSG;

typedef struct {
  uint32_t u32Total; //���豸����
  SaveMSG *Head;
} FLASHDeviceList;

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

FLASHDeviceList *FLASH_Init(List *list);

/*
*   ���豸��ӵ������б���
*/
uint8_t FlashAddDeviceToList(FLASHDeviceList *list, uint16_t id, uint16_t time); 

/*
*   ��������ȡ������д��flash
*/
uint8_t FlashWriteDevie(FLASHDeviceList *list);

/*
*   ��������flash�ڵ��豸��f405����ע�ᣬע��ɹ����Ӧ�ı�־λ��1
*   �豸����������Ҳ��Ҫ����ע��
*/
void DeviceOnlineStatusProcess(FLASHDeviceList *list);

/**
*   ����flash���豸��ע��״̬
*/
void FlashSetDeviceOnline(FLASHDeviceList *list, uint16_t id);

/*
*   Debug Function
*/
void printFlashTest(uint32_t addr);
void LookList(FLASHDeviceList *list);
void FLASH_CLEAN(void);

#endif