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
  uint16_t u16DeviceID;   //设备ID
  uint16_t u16LastTime;   //设备最后一次响应的时间
  struct SaveMSG *next;
  uint32_t u32RegStatus;
} SaveMSG;

typedef struct {
  uint32_t u32Total; //总设备数量
  SaveMSG *Head;
} FLASHDeviceList;

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

FLASHDeviceList *FLASH_Init(List *list);

/*
*   将设备添加到保存列表中
*/
uint8_t FlashAddDeviceToList(FLASHDeviceList *list, uint16_t id, uint16_t time); 

/*
*   从链表中取出数据写入flash
*/
uint8_t FlashWriteDevie(FLASHDeviceList *list);

/*
*   将保存在flash内的设备向f405进行注册，注册成功后对应的标志位置1
*   设备重新启动后也需要重新注册
*/
void DeviceOnlineStatusProcess(FLASHDeviceList *list);

/**
*   设置flash内设备的注册状态
*/
void FlashSetDeviceOnline(FLASHDeviceList *list, uint16_t id);

/*
*   Debug Function
*/
void printFlashTest(uint32_t addr);
void LookList(FLASHDeviceList *list);
void FLASH_CLEAN(void);

#endif