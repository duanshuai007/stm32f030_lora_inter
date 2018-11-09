#include "flash.h"
#include "stm32f1xx.h"
#include "stdlib.h"
#include "lora.h"
#include "maincontrol.h"
#include "rtc.h"
#include "list.h"

extern UartModule gUartMx;

/*
*   FLASH初始化函数：每个设备信息结构体占用4个字节，最大200个设备，共800字节
*   因为stm32f103c8每个扇区有1024字节，所以一共需要使用1个扇区。
*
*   如果在flash中发现有效的数据头，则将设备信息读出加入到链表中
*   如果没发现有效的数据头，则清空整个扇区，并写入数据头和设备个数0
*/
FLASHDeviceList *FLASH_Init(List *list)
{
  FLASHDeviceList *fdl = (FLASHDeviceList *)malloc(sizeof(FLASHDeviceList));
  if ( NULL == fdl ) {
    PRINT("FLASH_Init:fdl == null\r\n");
    return NULL;
  }
  
  fdl->Head = (SaveMSG *)malloc(sizeof(SaveMSG));
  if( NULL == fdl->Head ) {
    PRINT("FLASH_Init:Head == NULL\r\n");
    free(fdl);
    return NULL;
  }
  
  fdl->Head->next = NULL;
  fdl->Head->u16DeviceID = 0xffff;
  fdl->Head->u16LastTime = 0;
  fdl->Head->u32RegStatus = 0;
  fdl->u32Total = 0;
  
  uint32_t dat;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  if( SAVE_MESSAGE_HEAD == dat) {
    //如果flash内已经有了有效的数据
    uint32_t num = READ_FLASH_WORD(FLASH_DATABASE_START + 4);
    
    if ( num > 0 ) {
      //有设备信息
      uint32_t i;
      uint32_t data;
      
      for( i = 0; i < num; i++) {
        data = READ_FLASH_WORD(FLASH_DATABASE_START + 8 + (i * MSG_SIZE));
        FlashAddDeviceToList(fdl, (uint16_t)((data & 0xffff0000) >> 16), (uint16_t)(data & 0x0000ffff));
        add_device_to_list(list, (uint16_t)((data & 0xffff0000) >> 16));
      }
    }
    
    if ( fdl->u32Total != num ) 
      PRINT("FLASH:init error,%d %d\r\n", num, fdl->u32Total);
    
    return fdl;
    
  } else {
    //如果flash内还是空空的
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = FLASH_DATABASE_START;
    f.NbPages = 1;
    
    uint32_t pageerror = 0;
    
    if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
      PRINT("FLASH:erase %08x failed\r\n", pageerror);
      free(fdl->Head);
      free(fdl);
      return NULL;
    }
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, 0);
    
    HAL_FLASH_Lock();
    
    return fdl; 
  }
}

/*
*   将设备添加到保存列表中
*/
uint8_t FlashAddDeviceToList(FLASHDeviceList *list, uint16_t id, uint16_t time) 
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head;
  //寻找最后一个节点
  while ( sm->next ) {
    
    if(sm->u16DeviceID == id) {
      sm->u16LastTime = time;
      return 1;
    }
    sm = sm->next;
  }
  //上面循环有缺陷，最后一个sm结构体不会判断id，所以需要在这里处理一下
  if(sm->u16DeviceID == id) {
    sm->u16LastTime = time;
    return 1;
  }
  
  sm->next = (SaveMSG *)malloc(sizeof(SaveMSG));
  if ( NULL == sm->next ) {
    PRINT("FLASHLIST:add malloc failed\r\n");
    return 0;
  }
  
  fdl->u32Total++;
  sm->next->u16DeviceID = id;
  sm->next->u16LastTime = time;
  sm->next->u32RegStatus = 0;
  sm->next->next = NULL;
  
  return 1;
}

void FlashSetDeviceOnline(FLASHDeviceList *list, uint16_t id)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  
  while ( sm ) {
    if ( sm->u16DeviceID == id ) {
      sm->u32RegStatus = 1;
    }
    sm = sm->next;
  }
}

/*
*   从链表中读取第一个元素
*/
SaveMSG *FlashGetDeviceFromList(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;  //Head是空节点，没有有效数据，它的下一个节点是实际的开始节点，但是要判断是否为空 
  
  //读取一个节点出来，将读取的节点从链表中删除
  if ( sm ) {
    SaveMSG *nextsm = sm->next;
    fdl->Head->next = nextsm;   //从链表中删除该节点
    
    return sm;
  }
  
  return NULL;
}

/*
*   从链表中取出数据写入flash
*/
uint8_t FlashWriteDevie(FLASHDeviceList *list)
{
  uint32_t pageerror = 0;
  uint32_t dat;
  FLASH_EraseInitTypeDef f;
  
  dat = READ_FLASH_WORD(FLASH_DATABASE_START);
  
  if ( SAVE_MESSAGE_HEAD != dat ) {
    PRINT("FLASH:cant't read flash head\r\n");
    return 0;
  }
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  //因为在初始化的时候已经把设备信息读取出来保存到链表中了，所以这里可以直接
  //进行覆盖式写入。
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    PRINT("FLASH:erase %08x failed\r\n", pageerror);
    return 0;
  }
  //写入验证头
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START, SAVE_MESSAGE_HEAD);
  //写入总设备数
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_DATABASE_START + 4, list->u32Total);
  //写入所有设备信息
  SaveMSG *sm = list->Head->next;
  uint8_t i = 0;
  
  while ( sm ) {
    uint32_t data = (((uint32_t)sm->u16DeviceID) << 16) | sm->u16LastTime;
    PRINT("FLASH:write D:%04x T:%04x\r\n", sm->u16DeviceID, sm->u16LastTime);
    HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FLASH_DATABASE_START + 8 + (i * MSG_SIZE)), data);
    i++;
    //写入的信息不删除，因为下一次写入还需要，在整个设备执行的过程中要维护一个设备链表
    //每次会修改链表内对应的设备信息状态
    //设备状态的改变包括：执行了指令，心跳包
    //      free(sm);
    sm = sm->next;
  }
  
  PRINT("FLASH:write done\r\n");
  HAL_FLASH_Lock();
  
  return 1;
}

void DeviceOnlineStatusProcess(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  static uint32_t time = 0;
  static uint32_t lasttime = 0;
  
  while ( sm ) {
  
    if ( sm->u32RegStatus == 0 ) {
      //UartSendOnLineToF405(&gUartMx, sm->u16DeviceID);
    }
    
    sm = sm->next;
  }
  //每隔60秒保存一次状态信息，写入到flash中
  time = GetRTCTime();
  if ( time - lasttime > 60 ) {
    PRINT("FLASH:write device info to flash\r\n");
    lasttime = time;
    FlashWriteDevie(list);
  }
}

void LookList(FLASHDeviceList *list)
{
  FLASHDeviceList *fdl = list;
  SaveMSG *sm = fdl->Head->next;
  
  while ( sm ) {
    
    PRINT("LOOK:D:%04x, T:%04x, R:%d\r\n", sm->u16DeviceID, sm->u16LastTime, sm->u32RegStatus);
    
    sm = sm->next;
  }
}

void printFlashTest(uint32_t addr)
{
  uint32_t temp = *(__IO uint32_t*)(addr);
  
  PRINT("addr:0x%x, data:0x%x\r\n", addr, temp);
}


void FLASH_CLEAN(void)
{
  FLASH_EraseInitTypeDef f;
  uint32_t pageerror;
  
  HAL_FLASH_Unlock();
  
  f.TypeErase = FLASH_TYPEERASE_PAGES;
  f.PageAddress = FLASH_DATABASE_START;
  f.NbPages = 1;
  
  if ( HAL_OK != HAL_FLASHEx_Erase(&f, &pageerror)) {
    PRINT("FLASH:erase %08x failed\r\n", pageerror);
    return;
  }
 
  HAL_FLASH_Lock();
}

