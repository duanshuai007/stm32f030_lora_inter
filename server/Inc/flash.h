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
  uint16_t u16DeviceID;   //设备ID
  uint16_t u16LastTime;   //设备最后一次响应的时间
  struct SaveMSG *next;
} SaveMSG;

typedef struct {
  uint8_t u8Total; //总设备数量
  SaveMSG *Head;  //head是链表头，不表示任何设备
} FLASHDeviceList;
#pragma pack()

#define READ_FLASH_WORD(addr) (*(volatile uint32_t *)(addr))

/*
*   Flashl链表初始化
*   从存储地区读取已有的设备信息写入flash链表内
*/
bool FLASH_Init(uint16_t *sid, uint8_t *sch, uint8_t *rch);

/*
*   将设备添加到保存列表中
*/
bool FlashAddDeviceToList(uint16_t id, uint16_t time); 

/*
*   从Flash链表中删除节点
*/
void FlashDelDeviceFromList(uint16_t id);

/*
*   Flash保存设备信息，每隔 FLASH_SAVE_TIMEINTERVAL 秒
*   将flash链表gFDList的内容写入到flash内存中。
*/
void FlashTask(void);

/*
*   向F405发送所有设备上线信息
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